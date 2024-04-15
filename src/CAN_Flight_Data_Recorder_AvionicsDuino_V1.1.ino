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
//                                                Physical connections with Arduino Nano ESP32 board
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

//                       Micro-SD card reader module
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
//                                                                  Libraries and external files inclusions
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// Don't forget to install the Nano ESP32 boards package from the board manager of the Arduino IDE
#include <driver/twai.h>      // Espressif CAN library : https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>           
#include <ESP32WebServer.h>    //https://github.com/Pedroalbuquerque/ESP32WebServer download and place in your Libraries folder
#include <ESPmDNS.h>
#include "CSS.h"               //Includes headers of the web and style file

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                    Objects instantiations
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

SPIClass spi = SPIClass(HSPI);
File file;                                    
ESP32WebServer server(80);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                           Declarations of global variables and constants
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// ************************************************************** Pins **********************************************************************************************

#define SCK  D13 // SPI microSD card reader pins
#define MISO D12
#define MOSI D11
#define CS   D10

#define RX_GPIO_PIN GPIO_NUM_4 // CAN transceiver pins
#define TX_GPIO_PIN GPIO_NUM_5

// ************************************************************** Circular buffer variables **********************************************************************************************

uint8_t ringBuffer[3000];                // The circular buffer : it receives data from the CAN bus (after it has been converted from binary to text format) at a steady rate of 5 hertz.
                                         // As soon as it contains more than 512 bytes, this ring buffer is "emptied" to the SD card in packets of 512 bytes.
                                         // Its rather large capacity of 3000 bytes was chosen arbitrarily (more than 2 seconds of CAN binary data converted to text format).
char sprtfBuffer[75];                    // A small utility buffer for the sprintf function which converts the binary data contained in the canDataSnapshot structure variable to text format.
char canDataTxt[300];                    // A buffer that can contain a complete CAN snapshot after its conversion to text format, and before sending to the circular buffer.
uint8_t outBuffer[512];                  // A buffer to collect exactly 512 bytes of data from the circular buffer and write them to the SD card.
uint16_t writeIndex = 0, readIndex =0;   // Writing and reading indexes of the ring buffer
uint16_t nbBytesInBuffer = 0;            // Number of bytes in the circular buffer at a given time.

// ************************************************************** Structure for temporary storage of CAN bus data **********************************************************************************************

typedef struct 
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
  float manifoldPressure;
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

CanDataStructure canDataRealTime, canDataSnapshot; // canDataRealTime is updated in real time as data arrives on the CAN bus at 20, 10 or 5 Hz
                                                   // canDataSnapshot is an instantaneous copy made every 200 ms of canDataRealTime, its data is to be converted into text then written into the circular buffer.

// ************************************************************** Various variables **********************************************************************************************

bool   SD_present = false; // Flag to control if the SD card is present or not
uint32_t periode200msStartTime;      // Two variables used for the 200 ms timing (5 Hz)
uint16_t periode200msTimeOut = 200;

static bool canControllerReady = false;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                            SETUP()
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  // ******************************************************** USB serial UART and SPI bus initializations *************************************************************************************
  Serial.begin(115200);
  spi.begin(SCK, MISO, MOSI, CS);

  // ************************************************************** microSD card initialization *********************************************************************
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

// We will now determine the name of the file in which the data will be saved
// The goal is, during setup, to look for an "index.txt" file on the SD card. This file stores on a single byte the first two letters of the recording file name used during the previous session.
// A byte with value 0 can be converted with sprintf into a string of two characters "00", and for example, a value 9 will give "09", and 32 will give "32"... etc.
// We can thus, with a single byte, compose 100 different strings from "00" to "99".
// If the index.txt file is found, its first byte is read, incremented by +1, then converted into a 2-character string.
// These 2 characters will be the first two letters of the name of the recording file, which will be used for the session that is starting. This file name has the following format: "xxrecord.txt".
// If the first byte of the index.txt file has, for example, the value 6, then the name of the recording file to open will be 07record.txt. And the previous file used (therefore the most recent), i.e., 06record.txt, will not be overwritten.
// On the other hand, the previous file named "07record.txt" on the SD will be overwritten, but with this FIFO type system, it was the oldest recording file on the SD card.
// We can easily imagine keeping 100 files on a microSD card of several GB, numbered from 00 to 99
// After reading the "index.txt" file, we overwrite the single byte it contains by replacing it with the same value incremented by 1. If the value 100 is reached after incrementation, then we start again at 0.
// So, for example, when a session begins, if the recording file is "14record.txt", then the file "index.txt" now contains a byte with value 14.
// If the "index.txt" file is not found on the SD card, it is created, and a byte of value 0 is placed there.
// Once we have determined the name of the file, all that remains is to open this file for writing to be able to start recording the flight data.

  uint8_t fileNameIndex = 0;                    // The value of this variable will determine the first two characters (two digits) of the name of the recording file to use
  const uint8_t maxNbRecordFiles = 100;         // The maximum number of recording files allowed on the SD card before overwriting the oldest

// ---------------- In this step, we will determine the value to write in the “index.txt” file. ----------------------
  file = SD.open("/index.txt");   // We are trying to open an "index.txt" file on the SD card, in read-only mode
  if(file)                        // if opening is possible, the file exists on the SD card
  {
    fileNameIndex = file.read();  // fileNameIndex now contains the index of the xxrecord.txt file used in the previous recording session.
    fileNameIndex ++;             // fileNameIndex now contains the index of the xxrecord.txt file that will be used during the session that begins. We cannot yet write this value in "index.txt"
    if (fileNameIndex >= maxNbRecordFiles) fileNameIndex = 0; // If the maxNbRecordFiles value is reached after incrementation, then we start again at 0.
    file.close();                 // The read-only stage of the "index.txt" file process is completed, the file can be closed
  }
  else                            // If the "index.txt" file does not exist, it will be created during the next step and a zero will be written to its first byte.
  {
    fileNameIndex = 0;            // We assign the value 0 to fileNameIndex, the name of the recording file of the session which begins will therefore be "00record.txt"
  }

// --------------------- Next step, we write this value in the “index.txt” file -------------------------------
  file = SD.open("/index.txt",FILE_WRITE); // We now open for writing (or create for writing) the file "index.txt"
  file.write(fileNameIndex);               // We place the fileNameIndex value in this file
  file.close();                            // Then we close it

// ----------------- Last step, open for writing the recording file with the correct name to be able to start recording the flight data. -------------------
  char fileName[20];
  sprintf(fileName,"%c%02d%s", '/', fileNameIndex, "record.txt");
  file = SD.open(fileName,FILE_WRITE); // this file will never be closed during the entire session and until the system is powered off. 
                                       // Successive file.flush() will ensure effective writes of each packet. Only the last packet will be lost.
  delay(1000);
  // Writing field names on the first line of the file.
  file.print("Date;Hour;NanoS;GndSpeed;IAS;TAS;TRK;MagHead;WindSpeed;WindDir;AltGPS;AltQNH;AltDens;GPSVz;BaroVz;OAT;RH;QNH;roll;pitch;AccY;AccZ;FuelLowLim;FuelUpLim;FuelFlow;CalcFLevel;FuelUsed;NbSec;RPM;RPMint;ManPress;CHT2;CHT3;EGT3;EGT4;AFR;Thuile;Phuile;VBat;VBus;IBus;IBat;Latitude;Longitude;TimeAcc;Inc;SatInView\n");
  file.flush();

  // **************************************************************** WEB server initialization ********************************************************************************************

  WiFi.softAP("ESP32_AccessPoint", "123456789"); // SSID (ESP32_AccessPoint) and password (123456789) for the Access Point generated by the Nano ESP32 board
  delay(1000);

  Serial.println("Set Access Point named ESP32_AccessPoint");
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(myIP);
  
  // With the server named "esp32ap" the URL is http://esp32ap.local/
  // You can enter this URL in your browser's address bar, or more simply the above IP address myIP. See the exact IP value in the serial monitor.
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

  // **************************************************************** CAN initialization *************************************************************************************

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
   
  // **************************************************************** Various initializations *************************************************************************************
  pinMode(D4, OUTPUT);
  pinMode(D6, OUTPUT);
  periode200msStartTime = millis();

}
// ************************************************************************ End of setup() ************************************************************************************************

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                   LOOP()
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{
  digitalWrite(D4, !digitalRead(D4)); // for oscilloscope tracking
  
  // *********************************************************** Is there a client trying to connect to the server? *************************************************************************
  server.handleClient(); //Listen for client connections
  
  // **************************************************************** Querying the CAN Bus receive queue *************************************************************************************
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
        handle_rx_message(message); // This function updates the canDataRealTime structure in real time as soon as new data arrives on the CAN bus
    }
  }

  // **************************** Sending data from the CAN bus to the circular buffer at regular intervals, then possibly to the SD card if there is enough accumulated data *******************************************
  // The CAN bus conveys a lot of data, some transmitted at 20 Hz, others at 10 Hz, and finally, others at 5 Hz.
  // We chose to log data at 5 Hz, which is sufficient for all the analyses we might like to perform.
  if ((millis()- periode200msStartTime)>= periode200msTimeOut) // The chosen interval is 200 ms, i.e. a datalogging frequency of 5 Hz
  {
    periode200msStartTime = millis();
    digitalWrite(D6, !digitalRead(D6)); // for oscilloscope tracking
  // ============= Creating a Data Snapshot ==================== // This step is possibly unnecessary...
    noInterrupts();
    canDataSnapshot = canDataRealTime; 
    interrupts();
  // ============ Converting binary data to text format ==========
  // The data is processed in the order in which it should appear on each line of the text file saved on the SD card
    canDataTxt[0] = '\0'; // We will accumulate all the individual data in the canDataTxt buffer. We start by "deleting" the previous string by setting the first byte of the buffer to NULL.
                          // So the strcat function below will add the successive small pieces of string starting at index 0 of canDataTxt.

    sprintf(sprtfBuffer,"%4d%c%02d%c%02d%c%02d%c%02d%c%02d%c%ld%c", canDataSnapshot.yearGNSS,'/',canDataSnapshot.monthGNSS,'/',canDataSnapshot.dayGNSS,';',canDataSnapshot.hourGNSS,':',canDataSnapshot.minuteGNSS,':',canDataSnapshot.secondGNSS,';',canDataSnapshot.nanoSecGNSS,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%d%c%d%c%.1f%c%.1f%c%d%c%d%c",canDataSnapshot.groundSpeedGNSS,';', canDataSnapshot.indicatedAirSpeed,';',canDataSnapshot.trueAirSpeed,';',canDataSnapshot.trk,';',canDataSnapshot.magHeading,';',canDataSnapshot.windSpeedInt,';',canDataSnapshot.windDirectionInt,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.0f%c%d%c%d%c%0.f%c%d%c%.1f%c%.1f%c%d%c",canDataSnapshot.altitudeGNSS,';',canDataSnapshot.qnhAltitudeInt,';',canDataSnapshot.densityAltitudeInt,';',canDataSnapshot.Vz,';',canDataSnapshot.baroVarioInt,';',canDataSnapshot.oat,';',canDataSnapshot.rh,';',canDataSnapshot.QNH,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%.1f%c%.1f%c%.1f%c%d%c%d%c%.1f%c%.1f%c%.1f%c",canDataSnapshot.roll,';',canDataSnapshot.pitch,';',canDataSnapshot.AccY,';',canDataSnapshot.AccZ,';',canDataSnapshot.lowerLimit,';',canDataSnapshot.upperLimit,';',canDataSnapshot.fuelFlow,';',canDataSnapshot.calcFuelLevel,';',canDataSnapshot.fuelUsed,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%d%c%.0f%c%d%c%.1f%c%.0f%c%.0f%c%.0f%c%.0f%c%.1f%c",canDataSnapshot.totalNbSec,';',canDataSnapshot.rpm,';',canDataSnapshot.rpmInt,';',canDataSnapshot.manifoldPressure,';',canDataSnapshot.CHT2,';',canDataSnapshot.CHT3,';',canDataSnapshot.EGT3,';',canDataSnapshot.EGT4,';',canDataSnapshot.AFR,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%.1f%c", canDataSnapshot.Thuile,';',canDataSnapshot.Phuile,';');
    strcat(canDataTxt, sprtfBuffer);

    sprintf(sprtfBuffer,"%.1f%c%.1f%c%.1f%c%.1f%c%.6f%c%.6f%c%d%c%lld%c%d%c",canDataSnapshot.vBat,';',canDataSnapshot.Vbus14volts,';',canDataSnapshot.Ibus,';',canDataSnapshot.Ibat,';',canDataSnapshot.latitude,';',canDataSnapshot.longitude,';',canDataSnapshot.timeAccuracyGNSS,';',canDataSnapshot.inc,';',canDataSnapshot.satInView,'\n');
    strcat(canDataTxt, sprtfBuffer);
    
    // sprintf systematically adds a null byte at the end of the string. But strcat removes it before adding additional characters, then adds a new null byte at the end.
    // So at the end of all these operations with sprintf and strcat, the canDataTxt buffer contains a real C String Variable terminated by a '\n', then by a null byte.
    // strlen(canDataTxt) returns exactly the number of characters added, counting the '\n', but not the terminal null byte.

    // ============ Copying formatted text data into the circular buffer ==========
    ringBufferWrite(); //This function copies the string variable contained in canDataTxt into the circular buffer, at index writeIndex
    
    // ============ Possible writing of a data packet from the circular buffer to the SD card ==========
    if (nbBytesInBuffer >= sizeof(outBuffer)) //We are testing whether the circular buffer contains more bytes than the capacity of outBuffer (an exact multiple of 512 bytes for SD card writing operations optimization).
    {
      ringBufferRead ();          // If the condition is true, then this function fully fills outBuffer from the circular buffer and updates the global variables readIndex and nbBytesInBuffer.
      file.write(outBuffer, 512); // Then write outBuffer to SD card.
      file.flush();               // We force an effective write to the SD card
    }
  }
}
//************************************************************************** End of Loop() *************************************************************************************

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                        Circular buffer management functions
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void ringBufferWrite() // This function copies the variable string contained in canDataTxt (binary data from the CAN bus converted to text format) into the circular buffer at the writeIndex position
                       // and updates the global variables writeIndex and nbBytesInBuffer
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

void ringBufferRead() // This function fully fills outBuffer from the circular buffer and updates the global variables readIndex and nbBytesInBuffer
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
//                                                                        CAN messages management function
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static void handle_rx_message(twai_message_t& message) // This function updates the canDataRealTime structure in real time as soon as new data arrives on the CAN bus
                                                       // We start with the data transmitted with the highest frequency (20 Hz), then the data at 10 Hz, and finally the data at 5 Hz.
{
  switch (message.identifier)
  {
    case 10: canDataRealTime.inc = *(uint64_t*)(message.data+0);
             break;
             
    case 20: canDataRealTime.roll  = (*(float*)(message.data+0))*(180 / PI); // data received in radians from the AHRS, unfiltered, and converted to degrees.
             canDataRealTime.pitch =  *(float*)(message.data+4);             // data received in degrees from the AHRS, unfiltered.
             break;
             
    case 22: canDataRealTime.AccY = *(float*)(message.data+0); 
             canDataRealTime.AccZ = *(float*)(message.data+4);
             break;

    case 24: canDataRealTime.Vz  = (*(float*)(message.data+0))* 196.8504;                  // data received from the AHRS in m/s, unfiltered, it is converted to ft/min.
             canDataRealTime.trk = *(float*)(message.data+4);                              // The AHRS transmits to the CAN Bus an unfiltered trk float value, in degrees, from -180° to +180°.
             if (canDataRealTime.trk < 0) canDataRealTime.trk = 360 + canDataRealTime.trk; // Conversion from 0 to 360°
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
             
    case 50: canDataRealTime.manifoldPressure  = *(float*)(message.data+0);
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
//                                                                        WEB server management function
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
