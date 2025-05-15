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
    
  
***************************************************************************************/


#include <SPI.h>
#include <SD.h>
#include <Ethernet2.h>


#define PERIODE_BOUCLE_SERIE 500

const int OPR_sensor_PIN = A9; // A0;
const int PH_sensor_PIN = A8; // A1;
const int tempCelcius_PIN = A2;
const int pot_PIN = A3;
const int sensor1_PIN = A4;
const int sensor2_PIN = A5;
const int sensor3_PIN = A6;
const int sensor4_PIN = A7;
const int IN1_PIN = 3;
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

const int dummy1_PIN = 11; // à mettre entrée et ne pas changer (erreur carte)
const int dummy2_PIN = 12; // à mettre entrée et ne pas changer (erreur carte)
const int dummy3_PIN = 13; // à mettre entrée et ne pas changer (erreur carte)
const int dummy4_PIN = 33; // à mettre entrée et ne pas changer (erreur carte)


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
char replyBuffer[100];   // a string to send back
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
// Autres variables globales
unsigned long t0;

// SD card
Sd2Card card;
SdVolume volume;
SdFile root;

// Serial
#define SERIAL_TAB_MAX_LENTH 100
unsigned char rxTab[SERIAL_TAB_MAX_LENTH], txTab[SERIAL_TAB_MAX_LENTH];
int nbBytes;

// Variables commandes et capteurs
int currentCommand;
int powerOn;
int OPR_sensor;
int PH_sensor;
int tempCelcius;
int pot;
int sensor1;
int sensor2;
int sensor3;
int sensor4;
int IN1;
int IN2;
int IN3;
int D1;
int D2;
int D3;
int relais = 0;
int voyant = 0;
int alimSens1 = 0;
int alimSens2 = 0;
int debit;
int K_PULSES_DEBIT = 1;

// Variables diverses
int tempint;
signed long t0debit;


void setup()
{

  // Init PINS
  pinMode(OPR_sensor_PIN, INPUT);
  pinMode(PH_sensor_PIN, INPUT);
  pinMode(tempCelcius_PIN, INPUT);
  pinMode(pot_PIN, INPUT);
  pinMode(sensor1_PIN, INPUT);
  pinMode(sensor2_PIN, INPUT);
  pinMode(sensor3_PIN, INPUT);
  pinMode(sensor4_PIN, INPUT);
  pinMode(IN1_PIN, INPUT);
  pinMode(IN2_PIN, INPUT);
  pinMode(IN3_PIN, INPUT);
  pinMode(D1_PIN, INPUT);
  pinMode(D2_PIN, INPUT);
  pinMode(D3_PIN, INPUT);
  pinMode(relais_PIN, OUTPUT);
  pinMode(voyant_PIN, OUTPUT);
  pinMode(alimSens1_PIN, OUTPUT);
  pinMode(alimSens2_PIN, OUTPUT);
  pinMode(tx485_PIN, OUTPUT);

  // For prototype board errors
  pinMode(dummy1_PIN, INPUT);
  pinMode(dummy2_PIN, INPUT);
  pinMode(dummy3_PIN, INPUT);
  pinMode(dummy4_PIN, INPUT);

  
  // Init SERIAL console (USB)
  Serial.begin(57600);
  delay(1000);
  if(Serial) Serial.println("Serial Init OK");
  else Serial.println("Serial init failed...");

  Serial.println("*********************************");
  Serial.println("*****  MULTI SENSOR CONTROL *****");
  Serial.println("*****                       *****");
  Serial.println("*****         v1.0          *****");
  Serial.println("*****      18/01/2022       *****");
  Serial.println("*****                       *****");
  Serial.println("*****   by Pierrick SERRES  *****");
  Serial.println("***** Copyright SUBC-MARINE *****");
  Serial.println("*********************************");
  Serial.println("\n\n");
  
  // Init SERIAL communication
  //Serial1.begin(57600);
  
  // Init ETHERNET communication
  Serial.print("Init W5500... ");
  // start the Ethernet and UDP:
  Ethernet.init(SS_W5500);
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  Serial.print("IP adress ");
  Serial.print(ipadd[0]);
  Serial.print(".");
  Serial.print(ipadd[1]);
  Serial.print(".");
  Serial.print(ipadd[2]);
  Serial.print(".");
  Serial.print(ipadd[3]);
  Serial.print(" and port ");
  Serial.print(port);
  Serial.println("");

  // Init PWM constant current supply control
  pinMode(ON_OFF_POWER_PIN, OUTPUT);
  digitalWrite(ON_OFF_POWER_PIN, 0);
  pinMode(PWM_POWER_PIN, OUTPUT);
  currentCommand = 0;
  powerOn = 0;
  analogWrite(PWM_POWER_PIN, 255 - currentCommand);
  Serial.println("\nPWM constant current supply control init done...");

  // Init SD card
  if (!card.init(SPI_HALF_SPEED, SS_SD)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    delay(2000);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    delay(2000);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
/**/

  // Mesure impulsions sur D3
  attachInterrupt(digitalPinToInterrupt(IN1_PIN), PulsesCount, FALLING);

}

void PulsesCount()
{
  debit = millis() - t0debit;
  t0debit = millis();
}

void loop()
{
  // Boucle temps réel
  if(millis() > t0 + PERIODE_BOUCLE_SERIE)
  {
    t0 = millis(); // RAZ boucle
  }
  
  // Reception Ethernet UDP
  int packetSize = Udp.parsePacket();
  if(packetSize)
  {
    IPAddress remote = Udp.remoteIP();
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);

    LectureTrame(packetBuffer);
    AffectationCommandes();
    LectureCapteurs();
    ConstructionTrame(replyBuffer);

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(replyBuffer);
    Udp.endPacket();
  }

  // Reception liaison Série
  {
    if(Serial.available())
    {
      char c = Serial.read();
      rxTab[nbBytes++] = c;
      if(c == '\n')
      {
        if((rxTab[0] == 'G') && (rxTab[1] == 'U') && (rxTab[2] == 'I'))
        {
          LectureTrame(rxTab);
          AffectationCommandes();
          LectureCapteurs();
          ConstructionTrame(txTab);
          Serial.print((const char*)txTab);
        }
        nbBytes = 0;
      }
      if(nbBytes > SERIAL_TAB_MAX_LENTH) nbBytes = 0;
    }
  }
}


void LectureTrame(char *str)
{
  sscanf(str, "GUI,%d,%d,%d,%d,%d,%d\n", &currentCommand, &powerOn, &relais, &voyant, &alimSens1, &alimSens2);
}

void ConstructionTrame(char *str)
{
  unsigned char checksum = OPR_sensor+PH_sensor+tempCelcius+pot+sensor1+sensor2+sensor3+sensor4+IN1+IN2+IN3+D1+D2+D3;
  sprintf(str, "MULTISENSOR,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%02X\n",
    OPR_sensor, PH_sensor, tempCelcius, pot, sensor1, sensor2, sensor3, sensor4, debit, IN2, IN3, D1, D2, D3,
    currentCommand, powerOn, relais, voyant, alimSens1, alimSens2, checksum);
}

void LectureCapteurs(void)
{
  OPR_sensor = analogRead(OPR_sensor_PIN);
  PH_sensor = analogRead(PH_sensor_PIN);
  tempCelcius = analogRead(tempCelcius_PIN);
  pot = analogRead(pot_PIN);
  sensor1 = analogRead(sensor1_PIN);
  sensor2 = analogRead(sensor2_PIN);
  sensor3 = analogRead(sensor3_PIN);
  sensor4 = analogRead(sensor4_PIN);
  //IN1 = digitalRead(IN1_PIN);
  IN2 = digitalRead(IN2_PIN);
  IN3 = digitalRead(IN3_PIN);
  D1 = digitalRead(D1_PIN);
  D2 = digitalRead(D2_PIN);
  D3 = digitalRead(D3_PIN);
}

void AffectationCommandes(void)
{
  digitalWrite(relais_PIN, relais);
  digitalWrite(voyant_PIN, voyant);
  digitalWrite(alimSens1_PIN, alimSens1);
  digitalWrite(alimSens2_PIN, alimSens2);
  digitalWrite(tx485_PIN, 0);
  digitalWrite(ON_OFF_POWER_PIN, powerOn);
  if(currentCommand > 100) currentCommand = 100;
  else if(currentCommand < 0) currentCommand = 0;
  analogWrite(PWM_POWER_PIN, 255 - map(currentCommand, 0, 100, 0, 255));
}
