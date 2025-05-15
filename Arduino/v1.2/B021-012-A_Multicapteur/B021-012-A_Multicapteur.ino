/***************************************************************************************
  Programme de la carte B021-012 indice A prototype
  
  Arduino Mega 2560

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

  A FAIRE:
  Activation du journal de bord dans la carte SD (boite noire).
  Ajouter commmande ON/OFF alim en courant via l'interface PC.
    
  
***************************************************************************************/

 
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Ethernet2.h>


#define W5500_OK
#define SD_OK

const int tempCelcius_PIN = A2;
const int pot_PIN = A3;
const int sensor1_PIN = A4;
const int sensor2_PIN = A5;
const int sensor3_PIN = A6;
const int sensor4_PIN = A7;
const int DEBITMETRE_PIN = 3;
const int IN2_PIN = 4;
const int IN3_PIN = 5;
const int D1_PIN = 34;
const int D2_PIN = 36;
const int D3_PIN = 38;

const int relais_PIN = 47;
const int voyant_PIN = 41;
const int alimSens1_PIN = 23;
const int alimSens2_PIN = 25;
const int tx485_PIN = 2;

const int ON_OFF_POWER_PIN = 45;
const int PWM_POWER_PIN = 6;
const int SS_W5500 = 10;
const int SS_SD = 9;

const int dummy1_PIN = 11; // à mettre entrée et ne pas changer (erreur routage carte)
const int dummy2_PIN = 12; // à mettre entrée et ne pas changer (erreur routage carte)
const int dummy3_PIN = 13; // à mettre entrée et ne pas changer (erreur routage carte)
const int dummy4_PIN = 33; // à mettre entrée et ne pas changer (erreur routage carte)


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
unsigned char rxTab[SERIAL_TAB_MAX_LENTH], txTab[SERIAL_TAB_MAX_LENTH];
int nbBytes;

// Variables commandes et capteurs
double currentCommand; // de 0 à 30 A
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
int timerConsole;
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
int pot; // pour les tests sur table


//*************************************************************************************
void setup()
{

  //-------------------------------------------------
  // Init PINS
  pinMode(DEBITMETRE_PIN, INPUT);
  pinMode(alimSens1_PIN, OUTPUT);
  pinMode(alimSens2_PIN, OUTPUT);
  pinMode(pot_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // For prototype board errors
  pinMode(dummy1_PIN, INPUT);
  pinMode(dummy2_PIN, INPUT);
  pinMode(dummy3_PIN, INPUT);
  pinMode(dummy4_PIN, INPUT);

  
  //-------------------------------------------------
  // Init SERIAL console (USB)
  Serial.begin(57600);
  delay(1000);

  Serial.println("*********************************");
  Serial.println("*****  MULTI SENSOR CONTROL *****");
  Serial.println("*****         v1.2          *****");
  Serial.println("*****                       *****");
  Serial.println("*****   by Pierrick SERRES  *****");
  Serial.println("***** Copyright SUBC-MARINE *****");
  Serial.println("*********************************");
  Serial.println("");
  
#ifdef W5500_OK  
  //-------------------------------------------------
  // Init ETHERNET communication
  Serial.print("Init W5500... ");
  // start the Ethernet and UDP:
  Ethernet.init(SS_W5500);
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  Serial.println("IP adress: " + String(ipadd[0]) + "." + String(ipadd[1]) + "." + String(ipadd[2]) + "." + String(ipadd[3]));
  Serial.println("Port: " + String(port));
  Serial.println("");
#endif  

  //-------------------------------------------------
  // Init PWM constant current supply control
  pinMode(ON_OFF_POWER_PIN, OUTPUT);
  digitalWrite(ON_OFF_POWER_PIN, 0);
  pinMode(PWM_POWER_PIN, OUTPUT);
  currentCommand = 0;
  powerOn = 1;
  analogWrite(PWM_POWER_PIN, 255);
  Serial.println("PWM constant current supply control init done...");
  Serial.println("");

#ifdef SD_OK  
  //-------------------------------------------------
  // Init SD card
  if (!SD.begin(SS_SD))
  {
    Serial.println("SD Card failed, or not present");
    delay(2000);
  } 
  else Serial.println("SD card initialized.");
  Serial.println("");
#endif  

  //-------------------------------------------------
  // Mesure impulsions du débitmètre sur D3
  attachInterrupt(digitalPinToInterrupt(DEBITMETRE_PIN), PulsesCount, FALLING);

  //-------------------------------------------------
  // Lecture EEPROM
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
        powerOn = 0;
        timerEtapeInversion = 0;
        etapeInversion++;
      break;
      
      case 2: // Attente 1/2 deadtime
        if(timerEtapeInversion++ >= (int)(params.val.deadtime * 5)) etapeInversion++;
      break;
      
      case 3: // inversion relais
        if(alimSens1 == 1) alimSens1 = 0; else alimSens1 = 1;
        if(alimSens2 == 1) alimSens2 = 0; else alimSens2 = 1;
        etapeInversion++;
        timerEtapeInversion = 0;
        Serial.println("inversion relais");
      break;
      
      case 4: // Attente 1/2 deadtime
        if(timerEtapeInversion++ >= (int)(params.val.deadtime * 5)) etapeInversion++;
      break;
      
      case 5: // reactivation du courant
        powerOn = 1;
        etapeInversion = 0;
       break;
    }

    //-------------------------------------------------
    // Affectations automatiques des commandes en fonction des capteurs
    if(timerDebitNul > 0) timerDebitNul--;
    else debit = 0;

    LectureCapteurs();
    
    AffectationCommandes(powerOn);
    
    //-------------------------------------------------
    // Ecriture carte SD
    if(++timerConsole >= 10)
    {
      timerConsole = 0;
#ifdef SD_OK
      // Ecriture dans la carte SD
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile)
      {
        // Préparation du tableau de données à écrire sur la carte SD
        // A FAIRE

        
        dataFile.println((const char*)txTab);
        dataFile.close();
      }
#endif
    }

  }

  //-------------------------------------------------
  // Boucle 1 minute
  // Pour l'inversion de la circulation du courant
  if(millis() > tMinute + 1000)
  {
    tMinute = millis(); // RAZ boucle

    // timer de demande d'inversion du courant
    if(++timerInversion >= params.val.periodeInversion)
    {
      timerInversion = 0;
      etapeInversion = 1;
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

    LectureTrameUDP(packetBuffer);
  }
#endif

  //-------------------------------------------------
  // Reception liaison Série
  if(Serial.available())
  {
    char c = Serial.read();
    rxTab[nbBytes++] = c;
    if(c == '\n')
    {
      LectureTrame(rxTab);

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
void LectureTrame(String str)
{
  int nb_values = 7;
  double valeur[nb_values];
  int i;
  
  if(str.startsWith("SENSORS?"))
  {
    // Demande valeur capteurs
    Serial.print("SENSORS");
    Serial.print("," + String(debitFiltered, 0));
    Serial.print("," + String(currentCommand, 1));
    Serial.print("," + String(powerOn));
    Serial.print("," + String(alimSens1));
    Serial.print("," + String(alimSens2));
    Serial.print("," + String(params.val.periodeInversion - timerInversion));    
    Serial.print("\n");
  }
  else if(str.startsWith("PARAMS?"))
  {
    // Demande valeur params, on les envoie
    Serial.print("PARAMS");
    Serial.print("," + String(params.val.periodeInversion, 0));
    Serial.print("," + String(params.val.deadtime, 1));
    Serial.print("," + String(params.val.facteurKdebitmetre, 3));
    Serial.print("," + String(params.val.debitMin, 0));
    Serial.print("," + String(params.val.debitMax, 0));
    Serial.print("," + String(params.val.courantMin, 1));
    Serial.print("," + String(params.val.courantMax, 1));
    Serial.print("\n");

  }
  else if(str.startsWith("PARAMS,"))
  {
    // Réception de nouveaux params
    Serial.println("Reception de params");
    for(i=0; i<nb_values; i++)
    {
      str = strchr(str.c_str(), ','); // recherche de la virgule
      if(str.length() == 0) break;
      str[0] = ' '; // On supprime la virgule
      valeur[i] = str.toDouble(); // Lecture du premier double
    }

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
      Serial.println("PARAMS RECEIVED");

      SaveParams();
  
      CalculFonctionAffine(params.val.debitMin, params.val.courantMin, params.val.debitMax, params.val.courantMax, &a, &b);
    }
  }
}


//*************************************************************************************
// Lecture des information de commande venant de l'exterieur
// Renvoie 1 sur une demande de paramètre
void LectureTrameUDP(String str)
{
  int nb_values = 7;
  double valeur[nb_values];
  int i;
  
  if(str.startsWith("SENSORS?"))
  {
    // Demande valeur capteurs
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.print("SENSORS");
    Udp.print("," + String(debitFiltered, 0));
    Udp.print("," + String(currentCommand, 1));
    Udp.print("," + String(powerOn));
    Udp.print("," + String(alimSens1));
    Udp.print("," + String(alimSens2));
    Udp.print("," + String(params.val.periodeInversion - timerInversion));    
    Udp.print("\n");
    Udp.endPacket();

  }
  else if(str.startsWith("PARAMS?"))
  {
    // Demande valeur params, on les envoie
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.print("PARAMS");
    Udp.print("," + String(params.val.periodeInversion, 0));
    Udp.print("," + String(params.val.deadtime, 1));
    Udp.print("," + String(params.val.facteurKdebitmetre, 3));
    Udp.print("," + String(params.val.debitMin, 0));
    Udp.print("," + String(params.val.debitMax, 0));
    Udp.print("," + String(params.val.courantMin, 1));
    Udp.print("," + String(params.val.courantMax, 1));
    Udp.print("\n");
    Udp.endPacket();


  }
  else if(str.startsWith("PARAMS,"))
  {
    // Réception de nouveaux params
    Serial.println("Reception de params");
    for(i=0; i<nb_values; i++)
    {
      str = strchr(str.c_str(), ','); // recherche de la virgule
      if(str.length() == 0) break;
      str[0] = ' '; // On supprime la virgule
      valeur[i] = str.toDouble(); // Lecture du premier double
    }

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
      Serial.println("PARAMS RECEIVED");

      SaveParams();
  
      CalculFonctionAffine(params.val.debitMin, params.val.courantMin, params.val.debitMax, params.val.courantMax, &a, &b);
    }
  }
}


//*************************************************************************************
// Lectures des données du système
// S'execute dans la boucle principale de 100ms
void LectureCapteurs(void)
{
  debitFiltered = debitFiltered*COEFF_FILTRAGE_DEBIT - debitFiltered + debit;
  debitFiltered /= COEFF_FILTRAGE_DEBIT;

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
  digitalWrite(ON_OFF_POWER_PIN, 0);

  // Commandes TOR
  digitalWrite(alimSens1_PIN, alimSens1);
  digitalWrite(alimSens2_PIN, alimSens2);

}


//*************************************************************************************
void SaveParams(void)
{
  params.val.valid = PARAM_VALID_VALUE;
  for(int i = 0; i < sizeof(params.tab); i++) EEPROM.write(i, params.tab[i]);

  timerInversion = 0;
}


//*************************************************************************************
void ReadParams(void)
{
  // Lecture eeprom
  for(int i = 0; i < sizeof(params.tab); i++) params.tab[i] = EEPROM.read(i);
  if(params.val.valid != PARAM_VALID_VALUE)
  {
    params.val.periodeInversion = 60;
    params.val.deadtime = 1.5;
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
int Compare(String str1, String str2)
{
  int match = 1;
  for(int i=0; (str1[i] != 0) && (str2[i] != 0); i++)
  {
    if(str1[i] != str2[i]) match = 0;
  }
  return match;
}
