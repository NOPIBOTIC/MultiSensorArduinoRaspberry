
// Serial
#define SERIAL_TAB_MAX_LENTH 100
char rxTab[SERIAL_TAB_MAX_LENTH], txTab[SERIAL_TAB_MAX_LENTH];
int nbBytes;
#define LaisonRPi Serial // Serial = USB | Serial1 = RS485

// Variables commandes et capteurs
double currentCommand, current; // de 0 à 30 A
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

// Autres variables globales
unsigned long tMinute;

//*************************************************************************************
void setup()
{
  //-------------------------------------------------
  // Init PINS
  pinMode(LED_BUILTIN, OUTPUT);
  
  //-------------------------------------------------
  // Init SERIAL console
  LaisonRPi.begin(57600);
  delay(1000);

  LaisonRPi.println("*********************************");
  LaisonRPi.println("*****  MULTI SENSOR CONTROL *****");
  LaisonRPi.println("*****         v2.0          *****");
  LaisonRPi.println("*****                       *****");
  LaisonRPi.println("*****   by Pierrick SERRES  *****");
  LaisonRPi.println("***** Copyright SUBC-MARINE *****");
  LaisonRPi.println("*********************************");
  LaisonRPi.println("");

  params.val.periodeInversion = 20;
  params.val.deadtime = 2;
  params.val.facteurKdebitmetre = 7.467;
  params.val.debitMin = 50;
  params.val.debitMax = 300;
  params.val.courantMin = 2.0;
  params.val.courantMax = 25.0;

  debitFiltered = 120;
  currentCommand = 12;
  alimSens1 = 1;

}


//*************************************************************************************
void loop()
{ 
  //-------------------------------------------------
  // Boucle 1 minute
  // Pour l'inversion de la circulation du courant
  //if(millis() > tMinute + 60000)
  if(millis() > tMinute + 1000) // 1s pour le debug
  {
    tMinute = millis(); // RAZ boucle

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // timer de demande d'inversion du courant
    if(++timerInversion >= params.val.periodeInversion)
    {
      timerInversion = 0;
      alimSens1 = !alimSens1;
      alimSens2 = !alimSens2;
    }

  }

  //-------------------------------------------------
  // Reception liaison Série
  if(LaisonRPi.available())
  {
    char c = LaisonRPi.read();
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

  DATA-C1,123.4,12.0,11.9,1,1,0,55.0,60.0,3.0,7.467,50.0,300.0,2.0,25.0<LF>
  Cette trame possède 16 champs :
  DATA -C1  : Entête. DATA pour indiquer que c’est une trame de données.
              C1 pour indiquer que l’on s’adresse à la cellule d’adresse 1.
              Les adresses possibles sont C1 à C4.
  123.4     : Débit mesuré en l/h.
  12.0      : Commande du régulateur de courant en A.
  11.9      : Courant mesuré en A.
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

void LectureTrame(String str)
{
  int nb_values = 7;
  double valeur[nb_values];
  int i;

  debitFiltered += (double)random(-50,51)/10;
  if(debitFiltered < 0) debitFiltered = -debitFiltered;
  if(debitFiltered > 300) debitFiltered = 300;  
  currentCommand += (double)random(-10,11)/10;
  if(currentCommand < 0) currentCommand = -currentCommand;
  if(currentCommand > 25) currentCommand = 25;  
  current = currentCommand + ((double)random(-1,2)/10);
  if(current < 0) current = -current;
  powerOn = random(0,2);
  
  if(str.startsWith("DATA-C1")) // Demande de données
  {
    LaisonRPi.print("DATA-C1");
    LaisonRPi.print("," + String(debitFiltered, 0));
    LaisonRPi.print("," + String(currentCommand, 1));
    LaisonRPi.print("," + String(current, 1));
    LaisonRPi.print("," + String(powerOn));
    LaisonRPi.print("," + String(alimSens1));
    LaisonRPi.print("," + String(alimSens2));
    LaisonRPi.print("," + String(params.val.periodeInversion - timerInversion));    
    LaisonRPi.print("," + String(params.val.periodeInversion, 0));
    LaisonRPi.print("," + String(params.val.deadtime, 1));
    LaisonRPi.print("," + String(params.val.facteurKdebitmetre, 3));
    LaisonRPi.print("," + String(params.val.debitMin, 0));
    LaisonRPi.print("," + String(params.val.debitMax, 0));
    LaisonRPi.print("," + String(params.val.courantMin, 1));
    LaisonRPi.print("," + String(params.val.courantMax, 1));
    LaisonRPi.print("\n");

  }
  else if(str.startsWith("PARAM-C1,"))
  {
    // Réception de nouveaux params
    LaisonRPi.println("Reception de params");
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
      LaisonRPi.println("PARAMS RECEIVED");
    }
  }
}
