 /*
***************************************************************************************************************************************************
                                                      CAN_Flight_Data_Recorder_AvionicsDuino_V1.1
***************************************************************************************************************************************************
    
  CAN_Flight_Data_Recorder_AvionicsDuino is free software    
  MIT License (MIT)
  
  Copyright (c) 2024 AvionicsDuino - benjamin.fremond@avionicsduino.com
  https://avionicsduino.com/index.php/en/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

  Downloading/Uploading files over WiFi from/to an Arduino Nano ESP32 Board + MicroSD card adapter : ESP32_SD_Webserver
  
  ESP32_SD_Webserver : the ideas and concepts is Copyright (c) David Bird 2018. All rights to this software are reserved.
 
  Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
  3. You may not, except with my express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

  The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
  software use is visible to an end-user.
 
  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY 
  OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  See more at http://www.dsbird.org.uk and https://github.com/G6EJD/ESP32-8266-File-Upload

  Adaptation and simplification of David Birds original code by "My Circuits", 2022. See https://www.youtube.com/watch?v=zoYMU1tA3nI
  Adaptation of My Circuit's code by Benjamin Fremond

 ****************************************************************************************************************************
 */

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques de la carte Arduino Nano ESP32
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

//                   Module lecteur de carte micro-SD
// ---------------- SD module ------------- Arduino Nano ESP32 ------------------------
//                   GND ------------------->   GND 
//                   VCC ------------------->   + 5 Volts
//                    CS ------------------->   D10
//                   MISO ------------------>   D12
//                   MOSI ------------------>   D11
//                   SCK ------------------->   D13

//         Adafruit CAN Bus Transceiver - TJA1051T/3 Product ID: 5708
// -------------- Transceiver ------------ Arduino Nano ESP32 ------------------------
//                   GND ------------------->   GND 
//                   VCC ------------------->   + 3.3 Volts
//                    RX ------------------->   A3 - GPIO 4
//                    TX ------------------->   D2 - GPIO 5
//                  CANH -> CAN H network
//                  CANL -> CAN L network

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <driver/twai.h>
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>           
#include <ESP32WebServer.h>    //https://github.com/Pedroalbuquerque/ESP32WebServer download and place in your Libraries folder
#include <ESPmDNS.h>
#include "CSS.h"               //Includes headers of the web and de style file

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                    Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

SPIClass spi = SPIClass(HSPI);
File file;                                    // file est un objet fichier global 
ESP32WebServer server(80);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                           Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// ************************************************************** Broches **********************************************************************************************

#define SCK  D13 // broches pour le lecteur de cartes SD
#define MISO D12
#define MOSI D11
#define CS   D10

#define RX_GPIO_PIN GPIO_NUM_4 // broches pour le transceiver CAN
#define TX_GPIO_PIN GPIO_NUM_5

// ************************************************************** Variables liées au buffer circulaire **********************************************************************************************

uint8_t ringBuffer[3000];                // Buffer circulaire : il reçoit les données du CAN bus converties au format texte à un rythme régulier de 5 hertz.
                                         // Ce buffer est "vidé" vers la carte SD par paquets de 512 octets, dès qu'il contient plus de 512 octets.
                                         // Sa capacité assez large de 3000 octets a été choisie arbitrairement (plus de 2 secondes de données CAN converties).
char sprtfBuffer[75];                    // Un petit buffer utilitaire pour la fonction sprintf qui convertit au format texte les données numériques binaires contenues dans la variable structure canDataSnapshot.
char canDataTxt[300];                    // Un buffer pouvant contenir un instantané CAN complet après sa conversion au format texte, et avant envoi au buffer circulaire.
uint8_t outBuffer[512];                  // Un buffer pour recueillir exactement 512 octets de données du buffer circulaire et les écrire sur la carte SD.
uint16_t writeIndex = 0, readIndex =0;   // Les index d'écriture et de lecture.
uint16_t nbBytesInBuffer = 0;            // Nombre d'octets dans le buffer circulaire à un instant donné.

// ************************************************************** Structure pour stockage temporaire des données du CAN bus **********************************************************************************************

typedef struct  // Structure permettant d'enregistrer les données lues sur le CAN bus
{
  float oat;
  float rh;
  float magHeading;
  uint8_t lowerLimit;
  uint8_t upperLimit;
  uint16_t rpmInt;
  float rpm;
  float vBat;
  float fuelFlow;
  float calcFuelLevel;
  uint16_t totalNbSec;
  float fuelUsed;
  float roll;
  float pitch;
  float AccY;
  float AccZ;
  float Vz;
  float trk;
  uint16_t yearGNSS;
  uint8_t monthGNSS;
  uint8_t dayGNSS;
  uint8_t hourGNSS;
  uint8_t minuteGNSS;
  uint8_t secondGNSS;
  uint8_t satInView;
  int32_t nanoSecGNSS;
  uint32_t timeAccuracyGNSS;
  float altitudeGNSS;
  float groundSpeedGNSS;
  double latitude;
  double longitude;
  uint64_t inc;
  int16_t QNH;
  int16_t qnhAltitudeInt;
  int16_t densityAltitudeInt;
  int16_t baroVarioInt;
  int16_t indicatedAirSpeed;
  int16_t trueAirSpeed;
  int16_t windSpeedInt;
  int16_t windDirectionInt;
  float pressionAdmission;
  float CHT2;
  float CHT3;
  float Thuile;
  float Phuile;
  float EGT3;
  float EGT4;
  float AFR;
  float Vbus14volts;
  float Ibus;
  float Ibat;
}CanDataStructure;

CanDataStructure canDataRealTime, canDataSnapshot; // canDataRealTime est mis à jour en temps réel au fur et à mesure de l'arrivée de données sur le bus CAN
                                                   // canDataSnapshot est une copie instantanée de canDataRealTime, ses données sont à convertir en texte et à écrire dans le buffer circulaire.

// ************************************************************** Variables diverses **********************************************************************************************

bool   SD_present = false; // Flag to control if the SD card is present or not
uint32_t periode200msStartTime;      // Deux variables utilisées pour le timing des envois à 5 Hz
uint16_t periode200msTimeOut = 200;

static bool canControllerReady = false;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                            SETUP()
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  // ******************************************** Initialisation de la voie série USB, du bus SPI, et de la carte microSD *************************************************************************************
  Serial.begin(115200);
  spi.begin(SCK, MISO, MOSI, CS);
  if(!SD.begin(CS,spi,80000000))
  {
    Serial.println("Card Mount Failed");
    while(true){}
  }
  SD_present = true; 
  delay(1000);
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC)
  {
    Serial.println("MMC");
  } else if(cardType == CARD_SD)
  {
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  } else 
  {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

// On va maintenant déterminer le nom du fichier dans lequel les données vont être enregistrées
// Le but est, au moment du setup, de chercher un fichier index.txt sur la carte SD. Ce fichier mémorise sur un seul octet les deux premières lettres du non du fichier d'enregistrement utilisé lors de la dernière session.
// Un octet de valeur 0 peut être converti avec sprintf en une chaine de deux lettres "00", par exemple une valeur 9 donnera "09"... etc.
// On peut ainsi, avec un seul octet, composer 100 chaines différentes de "00" à "99".
// Si le fichier index.txt est trouvé, son premier octet est lu, incrémenté de +1, puis converti en une chaine de 2 caractères.
// Ces 2 caractères seront les deux premières lettres du fichier d'enregistrement qui va être utilisé pour la session qui commence. Ce nom de fichier a le format suivant : "xxrecord.txt".
// Si le premier octet du fichier index.txt a par exemple la valeur 6, alors le nom du fichier d'enregistrement à ouvrir sera 07record.txt. Et le précédent fichier utilisé, le plus récent, soit 06record.txt, ne sera pas écrasé.
// Par contre, le précédent fichier "07record.txt" va être écrasé, mais avec ce système de type FIFO, c'était le plus ancien des fichiers d'enregistrement de la carte SD. 
// On peut facilement imaginer conserver au moins 20 fichiers sur la carte SD, numérotés de 00 à 19
// Après avoir lu le fichier "index.txt", on écrase l'octet unique qu'il contient en le remplaçant par la même valeur incrémentée de 1. Si la valeur 20 est atteinte après incrémentation, alors on repart à 0.
// Donc par exemple, lors d'une session qui commence, si le fichier d'enregistrement est "14record.txt", alors le fichier "index.txt" contient un octet de valeur 14.
// Si le fichier "index.txt" n'est pas trouvé sur la carte SD, il est créé, et on y place un octet de valeur 0.
// Une fois qu'on a déterminer le nom du fichier, il ne reste plus qu'à ouvrir ce fichier en écriture pour pouvoir commencer à enregistrer les données de vol.


  uint8_t fileNameIndex = 0;                    // La valeur de cette variable permettra de déterminer les deux premiers caractères (deux chiffres) du nom du fichier d'enregistrement à utiliser
  const uint8_t maxNbRecordFiles = 3;           // Le nombre maximum de fichiers d'enregistrement autorisé sur la carte SD avant d'écraser le plus ancien

// ---------------- Dans cette étape, on va déterminer la valeur à écrire dans le fichier "index.txt" ----------------------
  file = SD.open("/index.txt");   // On essaye d'ouvrir un fichier "index.txt" en lecture seule
  if(file)                        // si l'ouverture est possible, c'est que le fichier existe sur la carte SD
  {
    fileNameIndex = file.read();  // fileNameIndex contient maintenant l'index du fichier xxrecord.txt utilisé lors de la précédente session d'enregistrement.
    fileNameIndex ++;             // fileNameIndex contient maintenant l'index du fichier xxrecord.txt qui va être utilisé lors de la session qui commence. A ce stade, on ne peut pas encore écrire cette valeur dans "index.txt"
    if (fileNameIndex >= maxNbRecordFiles) fileNameIndex = 0; // Si la valeur maxNbRecordFiles est atteinte après incrémentation, alors on repart à 0.
    file.close();                 // L'étape de lecture seule du fichier "index.txt" est terminée, le fichier peut être fermé
  }
  else                            // si le fichier "index.txt" n'existe pas, il va falloir le créer à l'étape suivante et y mettre la valeur 0
  {
    fileNameIndex = 0;            // On attribue la valeur 0 à fileNameIndex, le nom du fichier d'enregistrement de la session qui commence sera donc "00record.txt"
  }

// --------------------- Etape suivante, on écrit cette bonne valeur dans le fichier "index.txt" -------------------------------
  file = SD.open("/index.txt",FILE_WRITE); // On ouvre maintenant en écriture (ou on crée pour écriture) le fichier "index.txt
  file.write(fileNameIndex);               // On place la valeur fileNameIndex dans ce fichier
  file.close();                            // Puis on le ferme

// ----------------- Dernière étape, ouvrir en écriture le fichier d'enregistrement avec le nom qui va bien pour pouvoir commencer à enregistrer les données de vol. -------------------
  char fileName[20];
  sprintf(fileName,"%c%02d%s", '/', fileNameIndex, "record.txt");
  file = SD.open(fileName,FILE_WRITE); // ce fichier ne sera jamais fermé pendant toute la session et jusqu'à la mise hors tension du système. 
                                       // Des file.flush() successifs assureront des écritures effectives de chaque paquet. Seul le dernier sera perdu.
  delay(1000);
  // Ecriture des noms de champs sur la première ligne du fichier.
  file.print("Date;Hour;NanoS;GndSpeed;IAS;TAS;TRK;MagHead;WindSpeed;WindDir;AltGPS;AltQNH;AltDens;GPSVz;BaroVz;OAT;RH;QNH;roll;pitch;AccY;AccZ;FuelLowLim;FuelUpLim;FuelFlow;CalcFLevel;FuelUsed;NbSec;RPM;RPMint;PA;CHT2;CHT3;EGT3;EGT4;AFR;Thuile;Phuile;VBat;VBus;IBus;IBat;Latitude;Longitude;TimeAcc;Inc;SatInView\n");
  file.flush();

  // **************************************************************** Initialisation du WEB server ********************************************************************************************

  WiFi.softAP("ESP32_AccessPoint", "123456789"); // SSID and password for the Access Point generated by the Nano ESP32 board
  delay(1000);

  Serial.println("Set Access Point named ESP32_AccessPoint");
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(myIP);
  
  // With the server named "esp32ap" the URL is http://esp32ap.local/
  if (!MDNS.begin("esp32ap")) // Define the name of the server
  {          
    Serial.println(F("Error setting up MDNS responder!")); 
    ESP.restart(); 
  } 

  /*********  Server Commands  **********/
  server.on("/",         SD_dir);
  server.on("/upload",   File_Upload);
  server.on("/fupload",  HTTP_POST,[](){ server.send(200);}, handleFileUpload);
  server.begin();
  Serial.println("HTTP server started");

  // **************************************************************** Initialisation du contrôleur CAN Bus *************************************************************************************

  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_PIN, RX_GPIO_PIN, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 0;
  general_config.rx_queue_len = 3000; //Number of messages RX queue can hold
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK)
  {
    Serial.println("Driver installed");
    if (twai_start() == ESP_OK)
    {
      Serial.println("Driver started");
      uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
      if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
      {
        Serial.println("CAN alerts reconfigured");
        canControllerReady = true;
      }
      else Serial.println("Failed to reconfigure alerts");
    }
    else Serial.println("Failed to start driver");
  }
  else Serial.println("Failed to install driver");
   
  // **************************************************************** Initialisations diverses *************************************************************************************
  pinMode(D4, OUTPUT);
  pinMode(D6, OUTPUT);
  periode200msStartTime = millis();

}
// ************************************************************************ Fin du setup() ************************************************************************************************

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                   LOOP()
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{
  digitalWrite(D4, !digitalRead(D4)); // suivi oscilloscope
  
  // *********************************************************** Prise en charge d'une éventuelle connexion d'un client du serveur *************************************************************************
  server.handleClient(); //Listen for client connections
  
  // **************************************************************** Interrogation de la file d'attente de réception du CAN Bus *************************************************************************************
  if (canControllerReady)
  {
    uint32_t alerts_triggered;
    twai_status_info_t status_info;

    // Check if alert triggered
    twai_read_alerts(&alerts_triggered, 0);
    twai_get_status_info(&status_info);

    // Handle the alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
      Serial.println("Alert: TWAI controller has become error passive.");
    else if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
      Serial.println("Alert: a Bit, Stuff, CRC, Form, or ACK error has occurred on the bus.");
      Serial.printf("Bus error count: %d\n", status_info.bus_error_count);
    }
    else if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
    {
      Serial.println("Alert: the RX queue is full causing received frames to be lost.");
      Serial.printf("RX buffered: %d\t", status_info.msgs_to_rx);
      Serial.printf("RX miussed: %d\n", status_info.rx_missed_count);
      Serial.printf("RX overrun %d\n", status_info.rx_overrun_count);
    }
    else if (alerts_triggered & TWAI_ALERT_RX_DATA)
    {
       // One or more message received ? --> handle all
       twai_message_t message;
       while (twai_receive(&message, 0) == ESP_OK) 
        handle_rx_message(message); // Cette fonction met à jour en temps réel la structure canDataRealTime dès qu'une donnée nouvelle arrive sur le bus CAN
    }
  }

  // **************************** Envoi à intervalles réguliers des données du bus CAN au buffer circulaire, puis éventuellement à la carte SD si on a assez de données accumulées *******************************************
  // Le bus CAN véhicule de nombreuses données, certaines émises à 20 Hz d'autres à 10 Hz et enfin d'autres à 5 Hz.
  // On a choisi de faire du data logging à 5 Hz ce qui suffit pour toutes les analyses qu'on souhaite pouvoir faire.
  if ((millis()- periode200msStartTime)>= periode200msTimeOut) // L'intervalle choisi est de 200 ms, soit une fréquence de datalogging de 5 Hz
  {
    periode200msStartTime = millis();
    digitalWrite(D6, !digitalRead(D6)); // suivi oscilloscope
  // ============= Création d'un instantané des données ==================== // Cette étape est probablement inutile...
    noInterrupts();
    canDataSnapshot = canDataRealTime; 
    interrupts();
  // ============ Conversion des données binaires en format texte ==========
  // Les données sont traitées dans l'ordre où elle devront apparaitre à chaque ligne du fichier texte enregistré sur la carte SD
    canDataTxt[0] = '\0'; // On va accumuler toutes les données individuelles dans le buffer canDataTxt que l'on commence par "vider" de la chaine précédente en donnant à son premier octet la valeur NULL.
                          // Ainsi la fonction strcat ci-dessous ajoutera les petits bouts de chaine successifs en commençant à l'index 0 de canDataTxt.

    sprintf(sprtfBuffer,"%4d%c%02d%c%02d%c%02d%c%02d%c%02d%c%ld%c", canDataSnapshot.yearGNSS,'/',canDataSnapshot.monthGNSS,'/',canDataSnapshot.dayGNSS,';',canDataSnapshot.hourGNSS,':',canDataSnapshot.minuteGNSS,':',canDataSnapshot.secondGNSS,';',canDataSnapshot.nanoSecGNSS,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%d%c%d%c%.1f%c%.1f%c%d%c%d%c",canDataSnapshot.groundSpeedGNSS,';', canDataSnapshot.indicatedAirSpeed,';',canDataSnapshot.trueAirSpeed,';',canDataSnapshot.trk,';',canDataSnapshot.magHeading,';',canDataSnapshot.windSpeedInt,';',canDataSnapshot.windDirectionInt,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.0f%c%d%c%d%c%0.f%c%d%c%.1f%c%.1f%c%d%c",canDataSnapshot.altitudeGNSS,';',canDataSnapshot.qnhAltitudeInt,';',canDataSnapshot.densityAltitudeInt,';',canDataSnapshot.Vz,';',canDataSnapshot.baroVarioInt,';',canDataSnapshot.oat,';',canDataSnapshot.rh,';',canDataSnapshot.QNH,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%.1f%c%.1f%c%.1f%c%d%c%d%c%.1f%c%.1f%c%.1f%c",canDataSnapshot.roll,';',canDataSnapshot.pitch,';',canDataSnapshot.AccY,';',canDataSnapshot.AccZ,';',canDataSnapshot.lowerLimit,';',canDataSnapshot.upperLimit,';',canDataSnapshot.fuelFlow,';',canDataSnapshot.calcFuelLevel,';',canDataSnapshot.fuelUsed,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%d%c%.0f%c%d%c%.1f%c%.0f%c%.0f%c%.0f%c%.0f%c%.1f%c",canDataSnapshot.totalNbSec,';',canDataSnapshot.rpm,';',canDataSnapshot.rpmInt,';',canDataSnapshot.pressionAdmission,';',canDataSnapshot.CHT2,';',canDataSnapshot.CHT3,';',canDataSnapshot.EGT3,';',canDataSnapshot.EGT4,';',canDataSnapshot.AFR,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%.1f%c", canDataSnapshot.Thuile,';',canDataSnapshot.Phuile,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%.1f%c%.1f%c%.1f%c%.6f%c%.6f%c%d%c%lld%c%d%c",canDataSnapshot.vBat,';',canDataSnapshot.Vbus14volts,';',canDataSnapshot.Ibus,';',canDataSnapshot.Ibat,';',canDataSnapshot.latitude,';',canDataSnapshot.longitude,';',canDataSnapshot.timeAccuracyGNSS,';',canDataSnapshot.inc,';',canDataSnapshot.satInView,'\n');
    strcat(canDataTxt, sprtfBuffer);
    
    // sprintf ajoute systématiquement un null byte. Mais strcat l'ôte avant d'ajouter des caractères supplémentaires, puis rajoute un nouveau null byte à la fin.
    // Donc à l'issue de toutes ces opérations avec sprintf et strcat, le buffer canDataTxt contient une vraie C String Variable terminée par un '\n', puis par un null byte.
    // strlen(canDataTxt) retourne exactement le nombre de caractères ajoutés, en comptant le '\n', mais pas le null byte terminal.

    // ============ Copie des données texte formattées dans le buffer circulaire ==========
    ringBufferWrite(); //Cette fonction effectue la copie de la string variable contenue dans canDataTxt dans le buffer circulaire, à l'index writeIndex
    
    // ============ Ecriture éventuelle d'un paquet de données du buffer circulaire sur la carte SD ==========
    if (nbBytesInBuffer >= sizeof(outBuffer)) //On teste si le buffer circulaire contient plus d'octets que la capacité de outBuffer (un multiple exact de 512 octets). 
    {
      ringBufferRead ();          // Si la condition est vraie, alors cette fonction rempli intégralement outBuffer à partir du buffer circulaire et met à jour les variables globales readIndex et nbBytesInBuffer,
      file.write(outBuffer, 512); // puis écriture de outBuffer sur la carte SD.
      file.flush();               // On force une écriture effective sur la carte SD
    }
  }
}
//************************************************************************** Fin de la boucle infinie Loop *************************************************************************************

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                        Fonctions de gestion du buffer circulaire
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void ringBufferWrite() // Cette fonction effectue la copie de la chaine variable contenue dans canDataTxt (données issues du CAN bus converties au format texte) dans le buffer circulaire, à l'index writeIndex
                       // et met à jour les variables globales writeIndex et nbBytesInBuffer
{
  if ((strlen(canDataTxt) + writeIndex)<=sizeof(ringBuffer))
  {
    memcpy(ringBuffer+writeIndex, canDataTxt, strlen(canDataTxt));
  }
  else
  {
    memcpy(ringBuffer+writeIndex, canDataTxt, sizeof(ringBuffer)-writeIndex);
    memcpy(ringBuffer, canDataTxt+sizeof(ringBuffer)-writeIndex, writeIndex+strlen(canDataTxt)-sizeof(ringBuffer));
  }
  writeIndex = (writeIndex+strlen(canDataTxt))%sizeof(ringBuffer);
  nbBytesInBuffer = nbBytesInBuffer + strlen(canDataTxt);
}

void ringBufferRead()
{
  if ((sizeof(ringBuffer) - readIndex) >= sizeof(outBuffer))
  {
    memcpy(outBuffer, ringBuffer+readIndex, sizeof(outBuffer));
    readIndex = readIndex + sizeof(outBuffer);
  }
  else
  {
    memcpy(outBuffer, ringBuffer+readIndex, sizeof(ringBuffer)-readIndex);
    memcpy(outBuffer+(sizeof(ringBuffer)-readIndex), ringBuffer, sizeof(outBuffer)-(sizeof(ringBuffer)-readIndex));
    readIndex = sizeof(outBuffer)-(sizeof(ringBuffer)-readIndex);
  }
  nbBytesInBuffer = nbBytesInBuffer - sizeof(outBuffer);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                        Fonction de gestion des messages CAN
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static void handle_rx_message(twai_message_t& message) // Cette fonction met à jour en temps réel la structure canDataRealTime dès qu'une donnée nouvelle arrive sur le bus CAN
                                                       // On commence par les données transmises avec la fréquence la plus élevée (20 Hz), puis les données à 10 Hz, et enfin les données à 5 Hz.
{
  switch (message.identifier)
  {
    case 10: canDataRealTime.inc = *(uint64_t*)(message.data+0);
             break;
             
    case 20: canDataRealTime.roll  = (*(float*)(message.data+0))*(180 / PI); // donnée reçue en radians de l'AHRS, non filtrée, et convertie en degrés.
             canDataRealTime.pitch =  *(float*)(message.data+4);             // donnée reçue en degrés de l'AHRS, non filtrée.
             break;
             
    case 22: canDataRealTime.AccY = *(float*)(message.data+0); 
             canDataRealTime.AccZ = *(float*)(message.data+4);
             break;

    case 24: canDataRealTime.Vz  = (*(float*)(message.data+0))* 196.8504;                  // donnée reçue de l'AHRS en m/s, non filtrée, on la convertit en ft/min.
             canDataRealTime.trk = *(float*)(message.data+4);                              // L'AHRS transmet au CAN Bus une valeur de trk float non filtrée, en degrés, de -180° à +180°.
             if (canDataRealTime.trk < 0) canDataRealTime.trk = 360 + canDataRealTime.trk; // Conversion de 0 à 360°
             break;
             
    case 26: canDataRealTime.lowerLimit =  *(uint8_t*)(message.data+0); 
             canDataRealTime.upperLimit =  *(uint8_t*)(message.data+1);
             canDataRealTime.rpmInt     = *(uint16_t*)(message.data+2); 
             canDataRealTime.vBat       =    *(float*)(message.data+4);
             break;
    
    case 28: canDataRealTime.magHeading = *(float*)(message.data+0);
             break;
    
    case 30: canDataRealTime.altitudeGNSS    = *(float*)(message.data+0);
             canDataRealTime.groundSpeedGNSS = *(float*)(message.data+4);
             break;
    
    case 32: canDataRealTime.fuelFlow      = *(float*)(message.data+0);
             canDataRealTime.calcFuelLevel = *(float*)(message.data+4);
             break;

    case 33: canDataRealTime.totalNbSec = *(int16_t*)(message.data+0); 
             canDataRealTime.fuelUsed   =   *(float*)(message.data+2);
             break;
    
    case 34: canDataRealTime.yearGNSS =   *(uint16_t*)(message.data+0);
             canDataRealTime.monthGNSS =  *(uint8_t*) (message.data+2);
             canDataRealTime.dayGNSS =    *(uint8_t*) (message.data+3);
             canDataRealTime.hourGNSS =   *(uint8_t*) (message.data+4);
             canDataRealTime.minuteGNSS = *(uint8_t*) (message.data+5);
             canDataRealTime.secondGNSS = *(uint8_t*) (message.data+6);
             canDataRealTime.satInView =  *(uint8_t*) (message.data+7);
             break;
             
    case 36: canDataRealTime.latitude  = *(double*)(message.data+0);
             break;
    
    case 37: canDataRealTime.longitude = *(double*)(message.data+0);
             break;

    case 38: canDataRealTime.nanoSecGNSS      =  *(int32_t*)(message.data+0);
             canDataRealTime.timeAccuracyGNSS = *(uint32_t*)(message.data+4);
             break;
    
    case 42: canDataRealTime.oat = *(float*)(message.data+0);
             break;
             
    case 44: canDataRealTime.rh  = *(float*)(message.data+0);
             break;
             
    case 46: canDataRealTime.QNH                =  *(int16_t*)(message.data+0);
             canDataRealTime.qnhAltitudeInt     =  *(int16_t*)(message.data+2);
             canDataRealTime.densityAltitudeInt =  *(int16_t*)(message.data+4);
             canDataRealTime.baroVarioInt       =  *(int16_t*)(message.data+6);
             break;
             
    case 48: canDataRealTime.indicatedAirSpeed =  *(int16_t*)(message.data+0);
             canDataRealTime.trueAirSpeed      =  *(int16_t*)(message.data+2);
             canDataRealTime.windSpeedInt      =  *(int16_t*)(message.data+4);
             canDataRealTime.windDirectionInt  =  *(int16_t*)(message.data+6);
             break;
             
    case 50: canDataRealTime.pressionAdmission  = *(float*)(message.data+0);
             break;

    case 51: canDataRealTime.rpm  = *(float*)(message.data+0);
             break;
             
    case 52: canDataRealTime.CHT2  = *(float*)(message.data+0);
             canDataRealTime.CHT3  = *(float*)(message.data+4);
             break;

    case 54: canDataRealTime.Thuile  = *(float*)(message.data+0);
             canDataRealTime.Phuile  = *(float*)(message.data+4);
             break;            

    case 56: canDataRealTime.EGT3  = *(float*)(message.data+0);
             canDataRealTime.EGT4  = *(float*)(message.data+4);
             break;
             
    case 58: canDataRealTime.AFR          = *(float*)(message.data+0);
             canDataRealTime.Vbus14volts  = *(float*)(message.data+4);
             break;
                      
    case 60: canDataRealTime.Ibus  = *(float*)(message.data+0);
             canDataRealTime.Ibat  = *(float*)(message.data+4);
             break;
                   
    default: break;
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                        Fonction de gestion du WEB server
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Initial page of the server web, list directory and give you the chance of deleting and uploading
void SD_dir()
{
  if (SD_present) 
  {
    //Action acording to post, dowload or delete, by MC 2022
    if (server.args() > 0 ) //Arguments were received, ignored if there are not arguments
    { 
      Serial.println(server.arg(0));
  
      String Order = server.arg(0);
      Serial.println(Order);
      
      if (Order.indexOf("download_")>=0)
      {
        Order.remove(0,9);
        SD_file_download(Order);
        Serial.println(Order);
      }
  
      if ((server.arg(0)).indexOf("delete_")>=0)
      {
        Order.remove(0,7);
        SD_file_delete(Order);
        Serial.println(Order);
      }
    }

    File root = SD.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();    
      webpage += F("<table align='center'>");
      webpage += F("<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>");
      printDirectory("/",0);
      webpage += F("</table>");
      SendHTML_Content();
      root.close();
    }
    else 
    {
      SendHTML_Header();
      webpage += F("<h3>No Files Found</h3>");
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   //Stop is needed because no content length was sent
  } else ReportSDNotPresent();
}

//Upload a file to the SD
void File_Upload()
{
  append_page_header();
  webpage += F("<h3>Select file to upload</h3>"); 
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:50%' type='file' name='fupload' id = 'fupload' value=''>");
  webpage += F("<button class='buttons' style='width:50%' type='submit'>Upload File</button><br><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html",webpage);
}

//Prints the directory, it is called in void SD_dir() 
void printDirectory(const char * dirname, uint8_t levels)
{
  
  File root = SD.open(dirname);

  if(!root){
    return;
  }
  if(!root.isDirectory()){
    return;
  }
  File file = root.openNextFile();

  int i = 0;
  while(file){
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if(file.isDirectory()){
      webpage += "<tr><td>"+String(file.isDirectory()?"Dir":"File")+"</td><td>"+String(file.name())+"</td><td></td></tr>";
      printDirectory(file.name(), levels-1);
    }
    else
    {
      webpage += "<tr><td>"+String(file.name())+"</td>";
      webpage += "<td>"+String(file.isDirectory()?"Dir":"File")+"</td>";
      int bytes = file.size();
      String fsize = "";
      if (bytes < 1024)                     fsize = String(bytes)+" B";
      else if(bytes < (1024 * 1024))        fsize = String(bytes/1024.0,3)+" KB";
      else if(bytes < (1024 * 1024 * 1024)) fsize = String(bytes/1024.0/1024.0,3)+" MB";
      else                                  fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
      webpage += "<td>"+fsize+"</td>";
      webpage += "<td>";
      webpage += F("<FORM action='/' method='post'>"); 
      webpage += F("<button type='submit' name='download'"); 
      webpage += F("' value='"); webpage +="download_"+String(file.name()); webpage +=F("'>Download</button>");
      webpage += "</td>";
      webpage += "<td>";
      webpage += F("<FORM action='/' method='post'>"); 
      webpage += F("<button type='submit' name='delete'"); 
      webpage += F("' value='"); webpage +="delete_"+String(file.name()); webpage +=F("'>Delete</button>");
      webpage += "</td>";
      webpage += "</tr>";

    }
    file = root.openNextFile();
    i++;
  }
  file.close();

 
}

//Download a file from the SD, it is called in void SD_dir()
void SD_file_download(String filename)
{
  if (SD_present) 
  { 
    File download = SD.open("/"+filename);
    if (download) 
    {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename="+filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download"); 
  } else ReportSDNotPresent();
}

//Handles the file upload a file to the SD
File UploadFile;
//Upload a new file to the Filing system
void handleFileUpload()
{ 
  HTTPUpload& uploadfile = server.upload(); //See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
                                            //For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if(uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("Upload File Name: "); Serial.println(filename);
    SD.remove(filename);                         //Remove a previous version, otherwise data is appended the file again
    UploadFile = SD.open(filename, FILE_WRITE);  //Open the file for writing in SD (create it, if doesn't exist)
    filename = String();
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if(UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  } 
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if(UploadFile)          //If the file was successfully created
    {                                    
      UploadFile.close();   //Close the file again
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<h3>File was successfully uploaded</h3>"); 
      webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename+"</h2>";
      webpage += F("<h2>File Size: "); webpage += file_size(uploadfile.totalSize) + "</h2><br><br>"; 
      webpage += F("<a href='/'>[Back]</a><br><br>");
      append_page_footer();
      server.send(200,"text/html",webpage);
    } 
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}

//Delete a file from the SD, it is called in void SD_dir()
void SD_file_delete(String filename) 
{ 
  if (SD_present) { 
    SendHTML_Header();
    File dataFile = SD.open("/"+filename, FILE_READ); //Now read data from SD Card 
    if (dataFile)
    {
      if (SD.remove("/"+filename)) {
        Serial.println(F("File deleted successfully"));
        webpage += "<h3>File '"+filename+"' has been erased</h3>"; 
        webpage += F("<a href='/'>[Back]</a><br><br>");
      }
      else
      { 
        webpage += F("<h3>File was not deleted - error</h3>");
        webpage += F("<a href='/'>[Back]</a><br><br>");
      }
    } else ReportFileNotPresent("delete");
    append_page_footer(); 
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSDNotPresent();
} 

//SendHTML_Header
void SendHTML_Header()
{
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate"); 
  server.sendHeader("Pragma", "no-cache"); 
  server.sendHeader("Expires", "-1"); 
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); 
  server.send(200, "text/html", ""); //Empty content inhibits Content-length header so we have to close the socket ourselves. 
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Content
void SendHTML_Content()
{
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Stop
void SendHTML_Stop()
{
  server.sendContent("");
  server.client().stop(); //Stop is needed because no content length was sent
}

//ReportSDNotPresent
void ReportSDNotPresent()
{
  SendHTML_Header();
  webpage += F("<h3>No SD Card present</h3>"); 
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//ReportFileNotPresent
void ReportFileNotPresent(String target)
{
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//ReportCouldNotCreateFile
void ReportCouldNotCreateFile(String target)
{
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//File size conversion
String file_size(int bytes)
{
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes)+" B";
  else if(bytes < (1024*1024))      fsize = String(bytes/1024.0,3)+" KB";
  else if(bytes < (1024*1024*1024)) fsize = String(bytes/1024.0/1024.0,3)+" MB";
  else                              fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
  return fsize;
}
