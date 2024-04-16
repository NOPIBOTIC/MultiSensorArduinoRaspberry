/***************************************************************************************
  Programme de la carte B021-012 indice B
  
  Arduino MKR ZERO

  by Pierrick SERRES
  Copyright SUBC-MARINE
 
  ------------------------------------------------------------
  v1.0
  19/05/2022
  Pierrick SERRES
  
  Lecture des différents capteurs.
  Envoie des données par liaisons série (USB) et par Ethernet (module W5500).
  Récupération des commandes de l'alimentation externe (pour cellules de chloration).
  Affectation des commandes aux différents actionneurs (relais, sortie PWM 0/10V, etc.).

  ------------------------------------------------------------
  v1.1
  22/06/2022
  Pierrick SERRES
  
  Ajout d’une partie du mode autonome de fonctionnement de la cellule de chloration.
  Le courant dans la cellule est maintenant proportionnel au débit.
  Un débit minimum de fonctionnement est nécessaire.
  Au-delà de ce débit minimum, le courant dans la cellule suit une fonction affine y=ax+b.
  
  ------------------------------------------------------------
  v1.2
  27/06/2022
  Pierrick SERRES

  Détection de débit nul (pas d'impulsions du débitmètre).
  Inversion automatique de la polarité dans la cellule de chloration, avec une rampe sur la commande en courant.
  Filtrage de la mesure de débit pour adoucir la commande en courant.
  Réglage des paramètres de fonctionnement en entrant les valeurs extrêmes (et non directement a et b).
  Refonte de la communication pour adaptation avec l'interface PC de production.
  Test préalable au calcul de débit pour éviter une division par zéro.

  ------------------------------------------------------------
  v2.0
  01/11/2022
  Pierrick SERRES

  Adaptation à la nouvelle version de l'électronique B021-012-B.
  Sur Arduino MKR Zero.
  - Utilisation de la RTC pour dater les informations.
  - mesure directe du courant de sortie de l'alimentation.
  - mesure de la résistance interne de la charge, après arrêt du courant.
  - sortie liaison série RS485 pour communiquer avec un dispositif d'inteface homme-machine.
  - on retire la communication réseau, qui sera assurée par le dispositif d'IHM.
  A développer et tester pour vérifier la faisabilité.
  
  
***************************************************************************************/

 
#include <FlashAsEEPROM.h>
//#include <SPI.h>
#include <SD.h>
#include <Ethernet2.h>
#include <DS3231.h>


//#define W5500_OK
//#define SD_OK

const int cell_volt_mes_PIN = A2; // Mesure de la tension dans la cellule de chloration 
const int current_mes_PIN = A3; // Mesure du courant de sortie de l'alimentation
const int set_cell_res_PIN = A4; // Activation de la mesure de la résistance interne (via relais)
const int alimSens1_PIN = A5; // Sélection de l'alimentation à la borne 1 de la cellule
const int alimSens2_PIN = A6; // Sélection de l'alimentation à la borne 2 de la cellule
const int ON_OFF_POWER_PIN = 0; // Pilotage relais ON:OFF de l'alimentation
const int voyant_PIN = 1; // Pilotage voyant externe 12V
const int PWM_POWER_PIN = 2; // Commande PWM 10V de l'alimentation
const int DEBITMETRE_PIN = 3; // Entrée impulsion du débitmère
const int SS_SD = 4; // Pour carte SD
const int SS_W5500 = 5; // Pour module Ethernet optionnel
const int DIO1_PIN = 7; // IO générique
const int DIO2_PIN = 6; // IO générique
const int TX485_PIN = A1; // Sens du driver RS485
const int ANALOG_PIN = A0; // Analog pin spare


#ifdef W5500_OK
// ETHERNET
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0x62, 0xA7, 0xD1, 0x21, 0xDB, 0x7B}; // Généré aléatoirement via https://miniwebtool.com/fr/mac-address-generator/
byte ipadd[] = {192, 168, 2, 3};
int port = 1470;
IPAddress ip(ipadd[0], ipadd[1], ipadd[2], ipadd[3]);
unsigned int localPort = port; // local port to listen on
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char replyBuffer[UDP_TX_PACKET_MAX_SIZE];   // a string to send back
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
#endif

// Autres variables globales
unsigned long tBoucle, tMinute;

// Serial
#define SERIAL_TAB_MAX_LENTH 100
char rxTab[SERIAL_TAB_MAX_LENTH], txTab[SERIAL_TAB_MAX_LENTH];
int nbBytes;
#define NB_VALUES_IN_STRING   7 // La trame type NMEA attendue contient 7 valeurs (voir protocole)
#define LaisonDebug Serial // Serial = USB | Serial1 = RS485
#define LaisonRPi Serial1 // Serial = USB | Serial1 = RS485

// RTC DS3231
DS3231 rtc;
bool century = false;
bool h12Flag;
bool pmFlag;
byte alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits;
bool alarmDy, alarmH12Flag, alarmPmFlag;
byte Year;
byte Month;
byte Date;
byte DoW;
byte Hour;
byte Minute;
byte Second;
byte Temp1, Temp2, Temp3, Temp4;
const char jour[8][10] = {"UNDEFINED", "DIMANCHE", "LUNDI", "MARDI", "MERCREDI", "JEUDI", "VENDREDI", "SAMEDI"};

// Variables commandes et capteurs
double currentCommand, current, currentFiltered; // de 0 à 30 A
#define COEFF_FILTRAGE_CURRENT 10
double cellVoltage; // Tension dans la cellule
double a, b;
int commandePWM; // de 0 à 255
int powerOn;
int alimSens1 = 0;
int alimSens2 = 0;
double debit, debitFiltered; // de 0 à 4000 l/h
#define COEFF_FILTRAGE_DEBIT  5
int timerDebitNul; // pour détecter l'absence d'impulsions si débit nul
int etapeInversion;
int timerInversion;
int timerSD;
int timerEtapeInversion;

/* Variables EEPROM */
#define PARAM_VALID_VALUE 51
union u_eeprom
{
  struct s_eeprom
  {
    byte valid;
    double periodeInversion;
    double deadtime;
    double facteurKdebitmetre;
    double debitMin;
    double debitMax;
    double courantMin;
    double courantMax;
  } val;
  byte tab[sizeof(val)];
} params;

// Variables diverses
signed long t0debit;


//*************************************************************************************
void setup()
{
  //-------------------------------------------------
  // Init PINS
  pinMode(set_cell_res_PIN, OUTPUT);
  pinMode(alimSens1_PIN, OUTPUT);
  pinMode(alimSens2_PIN, OUTPUT);
  pinMode(ON_OFF_POWER_PIN, OUTPUT);
  pinMode(voyant_PIN, OUTPUT);
  pinMode(PWM_POWER_PIN, OUTPUT);
  pinMode(DEBITMETRE_PIN, INPUT);
  pinMode(DIO1_PIN, INPUT);
  pinMode(DIO2_PIN, INPUT);
  pinMode(cell_volt_mes_PIN, INPUT);
  pinMode(ANALOG_PIN, INPUT);
  pinMode(TX485_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(TX485_PIN, 0); // Mode RX
    
  Wire.begin(); // Init I2C pour RTC
  
  //-------------------------------------------------
  // Init SERIAL console
  LaisonRPi.begin(57600);
  LaisonDebug.begin(57600);
  delay(1000);

  LaisonDebug.println("*********************************");
  LaisonDebug.println("*****  MULTI SENSOR CONTROL *****");
  LaisonDebug.println("*****         v2.0          *****");
  LaisonDebug.println("*****                       *****");
  LaisonDebug.println("*****   by Pierrick SERRES  *****");
  LaisonDebug.println("***** Copyright SUBC-MARINE *****");
  LaisonDebug.println("*********************************");
  GetClock();
  DisplayClock();
  LaisonDebug.println("");
  
#ifdef W5500_OK  
  //-------------------------------------------------
  // Init ETHERNET communication
  LaisonDebug.print("Init W5500... ");
  // start the Ethernet and UDP:
  Ethernet.init(SS_W5500);
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  LaisonDebug.println("IP adress: " + String(ipadd[0]) + "." + String(ipadd[1]) + "." + String(ipadd[2]) + "." + String(ipadd[3]));
  LaisonDebug.println("Port: " + String(port));
  LaisonDebug.println("");
#endif  

  //-------------------------------------------------
  // Init PWM constant current supply control
  digitalWrite(ON_OFF_POWER_PIN, 0);
  currentCommand = 0;
  powerOn = 1;
  analogWrite(PWM_POWER_PIN, 255);
  LaisonDebug.println("PWM constant current supply control init done...");
  LaisonDebug.println("");

#ifdef SD_OK  
  //-------------------------------------------------
  // Init SD card
  if (!SD.begin(SS_SD))
  {
    LaisonDebug.println("SD Card failed, or not present");
    delay(2000);
  } 
  else LaisonDebug.println("SD card initialized.");
  LaisonDebug.println("");
#endif  

  //-------------------------------------------------
  // Mesure impulsions du débitmètre sur D3
  attachInterrupt(digitalPinToInterrupt(DEBITMETRE_PIN), PulsesCount, FALLING);

  //-------------------------------------------------
  // Lecture EEPROM
  delay(1000);
  ReadParams();
  
  CalculFonctionAffine(params.val.debitMin, params.val.courantMin, params.val.debitMax, params.val.courantMax, &a, &b);

  // Position de départ relais sens courant
  alimSens1 = 1;
  alimSens2 = 0;

}


//*************************************************************************************
void loop()
{ 
  //-------------------------------------------------
  // Boucle 100ms
  if(millis() > tBoucle + 100)
  {
    tBoucle = millis(); // RAZ boucle

    //-------------------------------------------------
    // Inversion de la circulation du courant
    switch(etapeInversion)
    {
      case 0: // NOP
      break;
      
      case 1: // désactivation du courant
        LaisonDebug.println("Desactivation Alim");
        powerOn = 0;
        timerEtapeInversion = 0;
        etapeInversion++;
        LaisonDebug.println("Deadtime 1/2");
      break;
      
      case 2: // Attente 1/2 deadtime
        if(timerEtapeInversion++ >= (int)(params.val.deadtime * 5)) etapeInversion++;
      break;
      
      case 3: // inversion relais
        LaisonDebug.println("inversion relais");
        if(alimSens1 == 1) alimSens1 = 0; else alimSens1 = 1;
        if(alimSens2 == 1) alimSens2 = 0; else alimSens2 = 1;
        etapeInversion++;
        timerEtapeInversion = 0;
        LaisonDebug.println("Deadtime 2/2");
      break;
      
      case 4: // Attente 1/2 deadtime
        if(timerEtapeInversion++ >= (int)(params.val.deadtime * 5)) etapeInversion++;
      break;
      
      case 5: // reactivation du courant
        LaisonDebug.println("Reactivation Alim");
        powerOn = 1;
        etapeInversion = 0;
       break;
    }

    //-------------------------------------------------
    // Affectations automatiques des commandes en fonction des capteurs
    if(timerDebitNul > 0) timerDebitNul--;
    else debit = 0;
    //else debit = analogRead(ANALOG_PIN)/3;

    LectureCapteurs();
    
    AffectationCommandes(powerOn);
    
#ifdef SD_OK
    //-------------------------------------------------
    // Ecriture carte SD
    if(++timerSD >= 10)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      timerSD = 0;
      
      // Ecriture dans la carte SD
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile)
      {
        // Préparation du tableau de données à écrire sur la carte SD
        // A FAIRE

        
        dataFile.println((const char*)txTab);
        dataFile.close();
      }
    }
#endif

  }

  //-------------------------------------------------
  // Boucle 1 minute
  // Pour l'inversion de la circulation du courant
  if(millis() > tMinute + 60000)
  //if(millis() > tMinute + 1000) // 1s pour le debug
  {
    tMinute = millis(); // RAZ boucle

    GetClock();
    DisplayClock();

    // timer de demande d'inversion du courant
    if(++timerInversion >= params.val.periodeInversion)
    {
      timerInversion = 0;
      etapeInversion = 1;
      LaisonDebug.println(" Inversion");
    }
    else
    {
      LaisonDebug.print("Inversion dans ");
      LaisonDebug.print(params.val.periodeInversion - timerInversion, 0);
      LaisonDebug.println(" min");
    }
  }

#ifdef W5500_OK
  //-------------------------------------------------
  // Reception Ethernet UDP
  int packetSize = Udp.parsePacket();
  if(packetSize)
  {
    IPAddress remote = Udp.remoteIP();
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);

    LectureTrameUDP(packetBuffer, NB_VALUES_IN_STRING);
  }
#endif

  //-------------------------------------------------
  // Reception liaison Série
  if(LaisonRPi.available())
  {
    char c = LaisonRPi.read();
    rxTab[nbBytes++] = c;
    if(c == '\n')
    {
      LectureTrame(rxTab, NB_VALUES_IN_STRING);

      nbBytes = 0;
    }
    if(nbBytes > SERIAL_TAB_MAX_LENTH) nbBytes = 0;
  }

}


//*************************************************************************************
// Interruption de comptage des impulsions du débitmètre
void PulsesCount()
{
  unsigned long tdebit = micros() - t0debit; // en période entre les impulsions, en µs
  if(tdebit > 0) debit = (params.val.facteurKdebitmetre*1000000) / (double)tdebit; // passage en l/h
  t0debit = micros();

  timerDebitNul = 10; // pour détecter un débit nul
}


//*************************************************************************************
// Lecture des information de commande venant de l'exterieur
// Renvoie 1 sur une demande de paramètre
/*
  -----------------------------------------------------
  Trames reçues:

  DATA-C1<LF>
  Cette trame ne possède que deux champs :
  DATA-C1  : Entête. DATA pour indiquer que l’on demande trame de données.
             C1 pour indiquer que l’on s’adresse à la cellule d’adresse 1.
             Les adresses possibles sont C1 à C4.
  <LF>    : Caractère de fin de trame, 0x0A.

  PARAM-C1,60.0,3.0,7.467,50.0,300.0,2.0,25.0<LF>
  Cette trame possède 9 champs :
  PARAM-C1  : Entête. PARAM pour indiquer que c’est une trame de paramètres.
              C1 pour indiquer que l’on s’adresse à la cellule d’adresse 1.
              Les adresses possibles sont C1 à C4.
  60.0      : Période d’inversion du sens du courant dans les cellules en minutes.
  3.0       : Temps de repos lors de l’inversion du courant dans les cellules, en secondes.
  7.467     : Facteur K du débitmètre en l/h/Hz.
  50.0      : Débit minimum de la fonction affine Courant = f(Débit), en l/h.
  300.0     : Débit maximum de la fonction affine Courant = f(Débit), en l/h.
  2.0       : Courant minimum de la fonction affine Courant = f(Débit), en A.
  25.0      : Courant maximum de la fonction affine Courant = f(Débit), en A.
  <LF>      : Caractère de fin de trame, 0x0A.

  -----------------------------------------------------
  Trame envoyée:

  DATA-C1,123.4,12.0,11.9,24.0,1,1,0,55.0,60.0,3.0,7.467,50.0,300.0,2.0,25.0<LF>
  Cette trame possède 16 champs :
  DATA-C1  : Entête. DATA pour indiquer que c’est une trame de données.
              C1 pour indiquer que l’on s’adresse à la cellule d’adresse 1.
              Les adresses possibles sont C1 à C4.
  01 04 24  : Date (JJ MM AA)
  15 25 00  : Heure (hh mm ss)
  123.4     : Débit mesuré en l/h.
  12.0      : Commande du régulateur de courant en A.
  11.9      : Courant dans la cellule mesuré en A.
  24.0      : Tension de la cellule mesurée en V.
  1         : Etat du régulateur de courant, 1 : ON | 0 : OFF.
  1         : Etat du premier relais de sens du courant, 1 : ON | 0 : OFF.
  0         : Etat du second relais de sens du courant, 1 : ON | 0 : OFF.
  55.0      : Temps avant inversion du sens du courant, en minutes.
  60.0      : Période d’inversion du sens du courant dans les cellules en minutes.
  3.0       : Temps de repos lors de l’inversion du courant dans les cellules, en secondes.
  7.467     : Facteur K du débitmètre en l/h/Hz.
  50.0      : Débit minimum de la fonction affine Courant = f(Débit), en l/h.
  300.0     : Débit maximum de la fonction affine Courant = f(Débit), en l/h.
  2.0       : Courant minimum de la fonction affine Courant = f(Débit), en A.
  25.0      : Courant maximum de la fonction affine Courant = f(Débit), en A.
  <LF>      : Caractère de fin de trame, 0x0A.

 */

void LectureTrame(String str, int nb_values)
{
  double valeur[nb_values];
  int i;
  char tempStr[50];

  /***************************************************************************/
  if(str.startsWith("DATA-C1")) // Demande de données
  {
    GetClock();
    digitalWrite(TX485_PIN, 1); // Mode TX
    LaisonRPi.print("DATA-C1");
    LaisonRPi.print("," + String(Date));
    LaisonRPi.print(" " + String(Month));
    LaisonRPi.print(" " + String(Year));
    LaisonRPi.print("," + String(Hour));
    LaisonRPi.print(" " + String(Minute));
    LaisonRPi.print(" " + String(Second));
    LaisonRPi.print("," + String(debitFiltered, 0));
    LaisonRPi.print("," + String(currentCommand, 1));
    LaisonRPi.print("," + String(current, 1));
    LaisonRPi.print("," + String(cellVoltage, 1));
    LaisonRPi.print("," + String(powerOn));
    LaisonRPi.print("," + String(alimSens1));
    LaisonRPi.print("," + String(alimSens2));
    LaisonRPi.print("," + String(params.val.periodeInversion - timerInversion, 0));    
    LaisonRPi.print("," + String(params.val.periodeInversion, 0));
    LaisonRPi.print("," + String(params.val.deadtime, 1));
    LaisonRPi.print("," + String(params.val.facteurKdebitmetre, 3));
    LaisonRPi.print("," + String(params.val.debitMin, 0));
    LaisonRPi.print("," + String(params.val.debitMax, 0));
    LaisonRPi.print("," + String(params.val.courantMin, 1));
    LaisonRPi.print("," + String(params.val.courantMax, 1));
    LaisonRPi.print("\n");
    delay(10);
    digitalWrite(TX485_PIN, 0); // Mode RX
  }
  /***************************************************************************/
  else if(str.startsWith("PARAM-C1,")) // Réception de paramètres
  {
    // Réception de nouveaux params
    LaisonDebug.println("Reception de params");
    for(i=0; i<nb_values; i++) // On lit les 7 valeurs de la chaine
    {
      str = strchr(str.c_str(), ','); // recherche de la virgule
      if(str.length() == 0) break; // Si fin de la trame on stoppe
      str[0] = ' '; // On supprime la virgule pour pouvoir lire la valeur qui suit la virgule
      valeur[i] = str.toDouble(); // Lecture du premier double de la chaine, et donc la valeur qui suit la virgule
    }

    // Affectation des valeurs lues aux variables correspondantes (suivant le protocole)
    if(i == nb_values)
    {
      i = 0;
      params.val.periodeInversion = valeur[i++];
      params.val.deadtime = valeur[i++];
      params.val.facteurKdebitmetre = valeur[i++];
      params.val.debitMin = valeur[i++];
      params.val.debitMax = valeur[i++];
      params.val.courantMin = valeur[i++];
      params.val.courantMax = valeur[i++];
      LaisonDebug.println("Params update");

      SaveParams();
  
      CalculFonctionAffine(params.val.debitMin, params.val.courantMin, params.val.debitMax, params.val.courantMax, &a, &b);
    }
  }
  /***************************************************************************/
  else if(str.startsWith("SETDATE")) // Réglage de la RTC
  //SETDATEYYMMDDdHHMMSS\n
  {
    LaisonDebug.println("Reglages DATE & HEURE");
    Temp1 = (byte)rxTab[7+0] - 48;
    Temp2 = (byte)rxTab[7+1] - 48;
    Year = Temp1*10 + Temp2;
    // now Month
    Temp1 = (byte)rxTab[7+2] - 48;
    Temp2 = (byte)rxTab[7+3] - 48;
    Month = Temp1*10 + Temp2;
    // now Date
    Temp1 = (byte)rxTab[7+4] - 48;
    Temp2 = (byte)rxTab[7+5] - 48;
    Date = Temp1*10 + Temp2;
    // now Day of Week
    DoW = (byte)rxTab[7+6] - 48;   
    // now Hour
    Temp1 = (byte)rxTab[7+7] - 48;
    Temp2 = (byte)rxTab[7+8] - 48;
    Hour = Temp1*10 + Temp2;
    // now Minute
    Temp1 = (byte)rxTab[7+9] - 48;
    Temp2 = (byte)rxTab[7+10] - 48;
    Minute = Temp1*10 + Temp2;
    // now Second
    Temp1 = (byte)rxTab[7+11] - 48;
    Temp2 = (byte)rxTab[7+12] - 48;
    Second = Temp1*10 + Temp2;
    rtc.setClockMode(false);  // set to 24h
    rtc.setYear(Year);
    rtc.setMonth(Month);
    rtc.setDate(Date);
    rtc.setDoW(DoW);
    rtc.setHour(Hour);
    rtc.setMinute(Minute);
    rtc.setSecond(Second);
    // Affichage date et heure
    LaisonDebug.print("Le nouveau reglage est : ");
    GetClock();
    DisplayClock();
    LaisonDebug.print("\n");
    
  }
}


#ifdef W5500_OK
//*************************************************************************************
// Lecture des information de commande venant de l'exterieur
// Renvoie 1 sur une demande de paramètre
void LectureTrameUDP(String str, int nb_values)
{
  
}
#endif

//*************************************************************************************
// Lectures des données du système
// S'execute dans la boucle principale de 100ms
void LectureCapteurs(void)
{
  // Lecture débit
  // Le débit est calculé par lecture de la fréquance des impulsions sur l'entrée interruption
  // ici on filtre simplement cette valeur pour la mettre en forme
  debitFiltered = debitFiltered*COEFF_FILTRAGE_DEBIT - debitFiltered + debit;
  debitFiltered /= COEFF_FILTRAGE_DEBIT;

  // Lecture courant cellule
  // Valeur lue sur la sortie du capteur de courant ACS712ELCTR-30A-T, après un pont diviseur 10k/10k
  // Sensibilité de 66mv/A en sortie du capteur, donc 33mV/A sur l'entrée AD
  current = (double)analogRead(current_mes_PIN);
  current = current * 3.3 / 1023; // tension sur l'entrée AD
  current = current * 2; // tension en sortie de capteur (avant pont diviseur)
  current = (current - 2.5) / 0.066; // passage en A (offset de 2.5V et sensibilité de 66mV/A)
  currentFiltered = currentFiltered*COEFF_FILTRAGE_CURRENT - currentFiltered + current;
  currentFiltered /= COEFF_FILTRAGE_CURRENT;

  // Lecture tension cellule
  // Valeur lue après un pont diviseur 10k/1k
  cellVoltage = (double)analogRead(cell_volt_mes_PIN);
  cellVoltage = cellVoltage * 3.3 / 1023; // tension sur l'entrée AD
  cellVoltage = cellVoltage * 11; // tension avant pont diviseur

}


//*************************************************************************************
// Affectation des commandes aux actionneurs
// S'execute dans la boucle principale de 100ms
void AffectationCommandes(int on)
{
  // Calcule de la commande en courant en fonction du débit :
  // Si le débit n'est pas suffisant, l'alimentation est OFF
  if(debitFiltered < params.val.debitMin) currentCommand = 0;
  else currentCommand = (a * debitFiltered) + b;
  
  // on borne la commande en courant
  if(currentCommand > params.val.courantMax) currentCommand = params.val.courantMax;
  else if(currentCommand < 0) currentCommand = 0;
  // Affectation de la commande PWM en fonction de la commande en courant
  // le courant est en A, de 0 à 30.
  // La commande PWM de 0 à 255 (en logique inversée)
  commandePWM = map((int)(currentCommand*10), 0, (int)(params.val.courantMax*10), 0, 255);
  analogWrite(PWM_POWER_PIN, 255 - commandePWM);

  // Activation de l'alimentation
  digitalWrite(ON_OFF_POWER_PIN, on);

  // Commandes TOR
  digitalWrite(alimSens1_PIN, alimSens1);
  digitalWrite(alimSens2_PIN, alimSens2);

}


//*************************************************************************************
void SaveParams(void)
{
  params.val.valid = PARAM_VALID_VALUE;
  for(int i = 0; i < sizeof(params.tab); i++) EEPROM.write(i, params.tab[i]);
  EEPROM.commit();

  timerInversion = 0;
  LaisonDebug.print("Params saved\n");
}


//*************************************************************************************
void ReadParams(void)
{
  // Lecture eeprom
  for(int i = 0; i < sizeof(params.tab); i++) params.tab[i] = EEPROM.read(i);
  LaisonDebug.print("Params load\n");
  if(params.val.valid != PARAM_VALID_VALUE)
  {
    LaisonDebug.print("No valid params, default used\n");
    params.val.periodeInversion = 60;
    params.val.deadtime = 2;
    params.val.facteurKdebitmetre = 7.467;
    params.val.debitMin = 50;
    params.val.debitMax = 300;
    params.val.courantMin = 2.0;
    params.val.courantMax = 25.0;

    SaveParams();
  }
}


//*************************************************************************************
void CalculFonctionAffine(double x1, double y1, double x2, double y2, double *a, double *b)
{
  double d1,d2;
  
  d1 = (y2-y1)/(x2-x1);
  d2 = y1 - (d1*x1);
  *a = d1;
  *b = d2;
}


//*************************************************************************************
// Mise à jour des variables de temps
void GetClock(void)
{
  Year = rtc.getYear();
  Month = rtc.getMonth(century);
  Date = rtc.getDate();
  DoW = rtc.getDoW();
  Hour = rtc.getHour(h12Flag, pmFlag);
  Minute = rtc.getMinute();
  Second = rtc.getSecond();
}


// Affichage des variables de temps
void DisplayClock(void)
{
  sprintf((char *)txTab, "%s %02d/%02d/20%02d %02d:%02d:%02d ", jour[DoW], Date, Month, Year, Hour, Minute, Second);
  LaisonDebug.print((const char *)txTab);
}


//*************************************************************************************
int Compare(String str1, String str2)
{
  int match = 1;
  for(int i=0; (str1[i] != 0) && (str2[i] != 0); i++)
  {
    if(str1[i] != str2[i]) match = 0;
  }
  return match;
}
