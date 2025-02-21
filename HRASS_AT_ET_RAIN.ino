/* 
*****************************************************************************************************************
High Resolution Analogue Sensor Simulator (HRASS) 
by Nick Owen 2025

// working code with postional CSV parser
//postional command :

Rain,AT,ET
Rain is in frequency
AT and ET are in a int value 0-255 

todo:
  - implement key value pair parser
  - add pulse width to rain

Change Log:
  Change Number - Date -         - Engineer -    Description   - Note
*****************************************************************************************************************
    001          6/02/2025       N.O            Added, WD and PTB330B 
    002         10/02/2025       N.O            Added, Simulation mode using the HRASS on board barometer 
    003         17/02/2025       N.O            Added, temperature lookup table so that temperature can be commanded with signed value
    004         21/02/2025       N.O            Started work on WD look up table
    
*/

#include "Arduino.h"
#include <AD5254_asukiaaa.h>
#include "MCP4728.h"
#include <BME280I2C.h>
#include "PT100.h"
BME280I2C bme;  // Default : forced mode, standby time = 1000 ms

MCP4728 dac;
//#include "NVP_Parser.h"

void uartTask(void *pvParameters);
void blinkTask(void *pvParameters);
void TBRGTask(void *pvParameters);
void ATTask(void *pvParameters);
void ETTask(void *pvParameters);
void WDTask(void *pvParameters);
void RHTask(void *pvParameters);


QueueHandle_t LEDinputQueueHandle;
QueueHandle_t TBRGinputQueueHandle;
QueueHandle_t ATinputQueueHandle;
QueueHandle_t ETinputQueueHandle;
QueueHandle_t WDinputQueueHandle;
QueueHandle_t RHinputQueueHandle;
QueueHandle_t BPinputQueueHandle;  //barometric pressure Queue



enum BlinkType {
  NO_BLINK,
  SLOW_BLINK,
  FAST_BLINK
};

// Define TX and RX pins for UART (RS232 port DB9)
#define TXD1 7
#define RXD1 6

// Use Serial1 for UART communication
HardwareSerial mySerial(2);

signed int FREQ;
signed int AT;
signed int ET;
signed int WD;
signed int RH;
signed int BP;  // default start with internal barometer data


/*
char FREQ = 0;
char AT = 0;
char ET = 0;
*/




// Function to generate a mod256 checksum from a buffer
char *GenerateCheckSum(char *buf, long bufLen) {
  static char tmpBuf[255];  // Buffer to store the checksum as a string
  long idx;
  unsigned int cks = 0;

  // Iterate through the buffer and sum up the ASCII values of the characters
  for (idx = 0; idx < bufLen; idx++) {
    cks += (unsigned char)buf[idx];
  }
  //Serial.println(cks);
  // Store the checksum modulo 256, formatted to 3 digits
  //Serial.println(cks % 256, HEX);
  sprintf(tmpBuf, "%02X", (unsigned int)(cks % 256));
  return tmpBuf;
}

void setup() {
  mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  Serial.begin(115200);
  Serial.println("Starting HRASS");
  Wire.begin(39, 40);

  while (!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  LEDinputQueueHandle = xQueueCreate(1, sizeof(BlinkType));
  if (LEDinputQueueHandle == 0) {
    Serial.println("Failed to create LED Queue");
    vTaskDelay(1000);
    ESP.restart();
  }

  TBRGinputQueueHandle = xQueueCreate(1, sizeof(FREQ));
  if (TBRGinputQueueHandle == 0) {
    Serial.println("Failed to create TBRG Queue");
    vTaskDelay(1000);
    ESP.restart();
  }

  ATinputQueueHandle = xQueueCreate(1, sizeof(AT));
  if (ATinputQueueHandle == 0) {
    Serial.println("Failed to create AT Queue");
    vTaskDelay(1000);
    ESP.restart();
  }

  ETinputQueueHandle = xQueueCreate(1, sizeof(ET));
  if (ATinputQueueHandle == 0) {
    Serial.println("Failed to create AT Queue");
    vTaskDelay(1000);
    ESP.restart();
  }

  WDinputQueueHandle = xQueueCreate(1, sizeof(WD));
  if (ATinputQueueHandle == 0) {
    Serial.println("Failed to create AT Queue");
    vTaskDelay(1000);
    ESP.restart();
  }

  RHinputQueueHandle = xQueueCreate(1, sizeof(RH));
  if (RHinputQueueHandle == 0) {
    Serial.println("Failed to create RH Queue");
    vTaskDelay(1000);
    ESP.restart();
  }

  BPinputQueueHandle = xQueueCreate(1, sizeof(BP));
  if (BPinputQueueHandle == 0) {
    Serial.println("Failed to create BP Queue");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode = xTaskCreatePinnedToCore(blinkTask, "Blink Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create Blink Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode1 = xTaskCreatePinnedToCore(uartTask, "UART Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create UART Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode2 = xTaskCreatePinnedToCore(TBRGTask, "TBRG Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create TBRG Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode3 = xTaskCreatePinnedToCore(ATTask, "AT Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create AT Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode4 = xTaskCreatePinnedToCore(ETTask, "ET Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create ET Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode5 = xTaskCreatePinnedToCore(WDTask, "WD Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create WD Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode6 = xTaskCreatePinnedToCore(RHTask, "RH Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create RH Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  BaseType_t returnCode7 = xTaskCreatePinnedToCore(BPTask, "BP Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create BP Task");
    vTaskDelay(1000);
    ESP.restart();
  }
}

void uartTask(void *pvParameters) {
  BlinkType blinkMode;
  int receivedChar;
  String Serialin;
  for (;;) {

    while (Serial.available() > 0) {

      // look for the next valid integer in the incoming serial stream:
      FREQ = Serial.parseInt();
      AT = Serial.parseInt();
      ET = Serial.parseInt();
      WD = Serial.parseInt();
      RH = Serial.parseInt();
      BP = Serial.parseInt();

      //AT= map(AT, -40, 60, 105, 212);
      //AT = AT + 40;
      //AT = AT_TempLUT[AT][1];
      //Serial.println(AT);
      ET = TempConvert(ET, BufLen);
      AT = TempConvert(AT, BufLen);
      //Serial.println(AT);

      if (Serial.read() == '\n') {
        //Serial.println("TBRG_FREQ");
        xQueueOverwrite(TBRGinputQueueHandle, &FREQ);
        //Serial.println("AT DEG C");
        //float AT1 = 0.94008 * AT - 132.6537 + 19;
        //AT = static_cast<int>(round(AT1));
        xQueueOverwrite(ATinputQueueHandle, &AT);
        xQueueOverwrite(ETinputQueueHandle, &ET);
        xQueueOverwrite(WDinputQueueHandle, &WD);
        xQueueOverwrite(RHinputQueueHandle, &RH);
        xQueueOverwrite(BPinputQueueHandle, &BP);
      }
    }
    //Serial.print("RFQ = ");
    //Serial.println(FREQ);
    //Serial.print("RPW = ");
    //Serial.println(RPW);
    //Serial.print("AT = ");
    //Serial.println(AT);
    //Serial.print("ET = ");
    //Serial.println(ET);

    vTaskDelay(100);
  }
}


void ATTask(void *pvParameters) {
  AD5254_asukiaaa potentio(AD5254_ASUKIAAA_ADDR_A0_VDD_A1_GND);
  potentio.begin();
  uint8_t targetChannel = 0;
  //  unsigned int currentAT = atoi(AT);
  // unsigned int newAT = atoi(AT);
  unsigned int currentAT = 0;
  unsigned int newAT = 0;
  BaseType_t result3;
  //char Temp;
  signed int Temp;
  for (;;) {
    /*
    currentAT = atoi(AT);
    Serial.print("Selected TEMP: ");
    Serial.println(currentAT);
*/
    if (currentAT == 0) {
      result3 = xQueueReceive(ATinputQueueHandle, &Temp, portMAX_DELAY);
      //currentAT = atoi(&Temp);
      newAT = Temp;
    }

    if (currentAT != newAT) {
      String channelStr = String(targetChannel);
      uint8_t value;
      for (uint8_t i = 0; i < 4; ++i) {
        String indexStr = String(i);
        if (potentio.readRDAC(i, &value) == 0) {
          // Serial.println("RDAC of channel " + indexStr + " is " + String(value));
        } else {
          Serial.println("Cannot read RDAC of channel " + indexStr + ".");
        }
        for (uint8_t x = 0; x < 4; ++x) {
          String channelStr = String(x);
          if (potentio.writeRDAC(x, newAT) == 0) {
            Serial.println("Update RDAC of AT channel " + channelStr + " to " + String(newAT));
          } else {
            Serial.println("Cannot update RDAC of channel " + channelStr + ".");
          }
        }
        currentAT = newAT;
      }
      vTaskDelay(100);
    }

    result3 = xQueueReceive(ATinputQueueHandle, &Temp, 0);
    //currentAT = atoi(&Temp);
    //currentAT = Temp;
    newAT = Temp;
    if (result3 == pdTRUE) {
      //currentAT = newAT;
      Serial.println("AT value changed");
    }
  }
}

void ETTask(void *pvParameters) {
  AD5254_asukiaaa potentio(AD5254_ASUKIAAA_ADDR_A0_GND_A1_VDD);
  potentio.begin();
  uint8_t targetChannel = 0;
  //  unsigned int currentAT = atoi(AT);
  // unsigned int newAT = atoi(AT);
  unsigned int currentET = 0;
  unsigned int newET = 0;
  BaseType_t result4;
  //char Temp;
  signed int Temp;
  for (;;) {
    /*
    currentAT = atoi(AT);
    Serial.print("Selected TEMP: ");
    Serial.println(currentAT);
*/
    if (currentET == 0) {
      result4 = xQueueReceive(ETinputQueueHandle, &Temp, portMAX_DELAY);
      //currentAT = atoi(&Temp);
      newET = Temp;
    }
    if (currentET != newET) {
      String channelStr = String(targetChannel);
      uint8_t value;
      for (uint8_t i = 0; i < 4; ++i) {
        String indexStr = String(i);
        if (potentio.readRDAC(i, &value) == 0) {
          // Serial.println("RDAC of channel " + indexStr + " is " + String(value));
        } else {
          Serial.println("Cannot read RDAC of ET channel " + indexStr + ".");
        }
        for (uint8_t x = 0; x < 4; ++x) {
          String channelStr = String(x);


          if (potentio.writeRDAC(x, newET) == 0) {
            Serial.println("Update RDAC of ET channel " + channelStr + " to " + String(newET));
          } else {
            Serial.println("Cannot update RDAC of ET channel " + channelStr + ".");
          }
        }
      }
      currentET = newET;
    }

    vTaskDelay(100);
    result4 = xQueueReceive(ETinputQueueHandle, &Temp, 0);
    //currentAT = atoi(&Temp);
    //currentET = Temp;
    newET = Temp;
    if (result4 == pdTRUE) {
      //currentET = newET;
      Serial.println("ET value changed");
    }
  }
}

void WDTask(void *pvParameters) {
  AD5254_asukiaaa potentio3(AD5254_ASUKIAAA_ADDR_A0_GND_A1_GND);
  potentio3.begin();
  uint8_t targetChannel = 0;
  //  unsigned int currentAT = atoi(AT);
  // unsigned int newAT = atoi(AT);
  unsigned int currentWD = 0;
  unsigned int newWD = 0;
  BaseType_t result5;
  //char Temp;
  signed int Winddir;
  for (;;) {
    if (currentWD == 0) {
      result5 = xQueueReceive(WDinputQueueHandle, &Winddir, portMAX_DELAY);
      //currentAT = atoi(&Temp);
      newWD = Winddir;
    }
    if (currentWD != newWD) {
      String channelStr = String(targetChannel);
      uint8_t value;
      for (uint8_t i = 0; i < 4; ++i) {
        String indexStr = String(i);
        if (potentio3.readRDAC(i, &value) == 0) {
          // Serial.println("RDAC of channel " + indexStr + " is " + String(value));
        } else {
          Serial.println("Cannot read RDAC of WD channel " + indexStr + ".");
        }
        for (uint8_t x = 0; x < 4; ++x) {
          String channelStr = String(x);
          if (potentio3.writeRDAC(x, newWD) == 0) {
            Serial.println("Update RDAC of WD channel " + channelStr + " to " + String(newWD));
          } else {
            Serial.println("Cannot update RDAC of WD channel " + channelStr + ".");
          }
        }
      }
      currentWD = newWD;
    }
    vTaskDelay(100);
    result5 = xQueueReceive(WDinputQueueHandle, &Winddir, 0);
    //currentAT = atoi(&Temp);
    //currentET = Temp;
    newWD = Winddir;
    if (result5 == pdTRUE) {
      //currentET = newET;
      Serial.println("WD value changed");
    }
  }
}


void RHTask(void *pvParameters) {
  //Wire.begin(39, 40);
  dac.attach(Wire, 14);
  dac.readRegisters();
  // dac.selectVref(MCP4728::VREF::INTERNAL_2_8V, MCP4728::VREF::INTERNAL_2_8V, MCP4728::VREF::INTERNAL_2_8V, MCP4728::VREF::INTERNAL_2_8V);
  dac.selectVref(MCP4728::VREF::VDD, MCP4728::VREF::INTERNAL_2_8V, MCP4728::VREF::INTERNAL_2_8V, MCP4728::VREF::INTERNAL_2_8V);
  dac.selectPowerDown(MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_500KOHM, MCP4728::PWR_DOWN::GND_500KOHM);
  dac.selectGain(MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1);
  dac.analogWrite(MCP4728::DAC_CH::A, 4000);
  dac.analogWrite(MCP4728::DAC_CH::B, 4000);
  dac.analogWrite(MCP4728::DAC_CH::C, 4000);
  dac.analogWrite(MCP4728::DAC_CH::D, 4000);
  dac.enable(true);
  unsigned int currentRH = 0;
  unsigned int newRH = 0;
  BaseType_t result6;
  signed int RH;
  for (;;) {
    if (currentRH == 0) {
      result6 = xQueueReceive(RHinputQueueHandle, &RH, portMAX_DELAY);
      //currentAT = atoi(&Temp);
      newRH = RH;
    }
    if (currentRH != newRH) {
      //dac.analogWrite(0, newRH, 0, 0);
      dac.analogWrite(0, map(newRH, 0, 100, 0, 2000), 0, 0);
      //Serial.println("0");
      delay(500);
      currentRH = newRH;
    }
    vTaskDelay(100);
    result6 = xQueueReceive(RHinputQueueHandle, &RH, 0);
    //currentAT = atoi(&Temp);
    //currentET = Temp;
    newRH = RH;
    if (result6 == pdTRUE) {
      //currentET = newET;
      Serial.println("RH value changed");
    }
  }
}


void BPTask(void *pvParameters) {
  unsigned int currentBP = 0;
  unsigned int newBP = 0;
  BaseType_t result7;
  signed int BP;
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  String PTBstring;
  char PTBbuffer[20];
  String PTBpresstring;
  for (;;) {
    if (currentBP == 0) {
      result7 = xQueueReceive(BPinputQueueHandle, &BP, portMAX_DELAY);
      newBP = BP;
    }
    if ((currentBP != newBP) || (currentBP != 97)) {
      //mySerial.print("testing ");  // read it and send it out Serial1 (pins 0 & 1)
      //mySerial.print("PTB330A_U4430842|P=");
      //mySerial.print("PTB330A_U1234567|P=");
      //mySerial.print(newBP, DEC);  // read it and send it out Serial1 (pins 0 & 1)
      //mySerial.println(".00|P1=1004.48|P2=1004.47|P3=1004.46|TP1= 26.00|TP2= 25.88|TP3= 25.90|ERR=000|CS=94\r\n");
      bme.read(pres, temp, hum, tempUnit, presUnit);
      pres = (pres / 100);  //convert PA to KPA

      Serial.print("Temp: ");
      Serial.print(temp);
      Serial.print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
      Serial.print("\t\tHumidity: ");
      Serial.print(hum);
      Serial.print("% RH");
      Serial.print("\t\tPressure: ");
      Serial.print(pres);
      Serial.println("KPa");

      //dtostrf(pres, 5, 2, PTBbuffer);  //convert float to string
      PTBpresstring = ("PTB330A_U1234567|P=");
      if (newBP < 1000) { PTBpresstring += "0"; }
      PTBpresstring += newBP;
      PTBpresstring += ".00|P1=";
      if (newBP < 1000) { PTBpresstring += "0"; }
      PTBpresstring += newBP;
      PTBpresstring += ".00|P2=";
      if (newBP < 1000) { PTBpresstring += "0"; }
      PTBpresstring += newBP;
      PTBpresstring += ".00|P3=";
      if (newBP < 1000) { PTBpresstring += "0"; }
      PTBpresstring += newBP;
      PTBpresstring += ".00|TP1=0";
      PTBpresstring += temp;
      PTBpresstring += "|TP2=0";
      PTBpresstring += temp;
      PTBpresstring += "|TP3=0";
      PTBpresstring += temp;
      PTBpresstring += "|ERR=000";
      PTBpresstring += "|CS=";
      // Call the checksum function and print the result
      int n = PTBpresstring.length();
      // declaring character array (+1 for null
      // character)
      char arr[n + 1];
      strcpy(arr, PTBpresstring.c_str());
      char *checksum = GenerateCheckSum(arr, strlen(arr));

      PTBpresstring += checksum;

      Serial.println(PTBpresstring);
      //Serial.println(checksum);
      mySerial.println(PTBpresstring);
      //mySerial.println(checksum);
      vTaskDelay(15000);  //delay 15 seconds, ie send sensor data every 15 seconds.
      currentBP = newBP;
    }

    if (newBP == 97) {  // special test that uses the HRASS internal barometer sensor
      Serial.println("Pressure Automode a");
      bme.read(pres, temp, hum, tempUnit, presUnit);
      pres = (pres / 100);  //convert PA to KPA
      Serial.print("Temp: ");
      Serial.print(temp);
      Serial.print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
      Serial.print("\t\tHumidity: ");
      Serial.print(hum);
      Serial.print("% RH");
      Serial.print("\t\tPressure: ");
      Serial.print(pres);
      Serial.println("KPa");

      dtostrf(pres, 5, 2, PTBbuffer);  //convert float to string
      PTBpresstring = ("PTB330A_U1234567|P=");
      PTBpresstring += PTBbuffer;
      PTBpresstring += "|P1=";
      PTBpresstring += PTBbuffer;
      PTBpresstring += "|P2=";
      PTBpresstring += PTBbuffer;
      PTBpresstring += "|P3=";
      PTBpresstring += PTBbuffer;
      PTBpresstring += "|TP1=0";
      PTBpresstring += temp;
      PTBpresstring += "|TP2=0";
      PTBpresstring += temp;
      PTBpresstring += "|TP3=0";
      PTBpresstring += temp;
      PTBpresstring += "|ERR=000";
      PTBpresstring += "|CS=";
      // Call the checksum function and print the result
      int n = PTBpresstring.length();
      // declaring character array (+1 for null
      // character)
      char arr[n + 1];
      strcpy(arr, PTBpresstring.c_str());
      char *checksum = GenerateCheckSum(arr, strlen(arr));

      Serial.print(PTBpresstring);
      Serial.println(checksum);
      mySerial.print(PTBpresstring);
      mySerial.println(checksum);


      //  input= "PTB330A_U1234567|P=1011.58|P1=1011.58|P2=1011.58|P3=1011.58|TP1=27.15|TP2=27.15|TP3=27.15|ERR=000|CS=";

      vTaskDelay(15000);  //delay 15 seconds, ie send sensor data every 15 seconds.
    }


    vTaskDelay(100);
    result7 = xQueueReceive(BPinputQueueHandle, &BP, 0);
    //currentAT = atoi(&Temp);
    //currentET = Temp;
    newBP = BP;
    if (result7 == pdTRUE) {
      //currentET = newET;
      Serial.println("BP value changed");
    }
  }
}




void blinkTask(void *pvParameters) {
  const uint8_t ledPin = 8;
  pinMode(ledPin, OUTPUT);
  BlinkType currentBlinkMode = NO_BLINK;
  BlinkType newBlinkMode = NO_BLINK;
  BaseType_t result;
  for (;;) {
    if (currentBlinkMode == NO_BLINK) {
      digitalWrite(ledPin, LOW);
      result = xQueueReceive(LEDinputQueueHandle, &currentBlinkMode, portMAX_DELAY);
    }
    switch (currentBlinkMode) {
      case FAST_BLINK:
        digitalWrite(ledPin, HIGH);
        vTaskDelay(100);
        digitalWrite(ledPin, LOW);
        vTaskDelay(100);
        break;
      case SLOW_BLINK:
        digitalWrite(ledPin, HIGH);
        vTaskDelay(500);
        digitalWrite(ledPin, LOW);
        vTaskDelay(500);
        break;
      case NO_BLINK:
        break;
      default:
        break;
    }
    result = xQueueReceive(LEDinputQueueHandle, &newBlinkMode, 0);
    if (result == pdTRUE) {
      currentBlinkMode = newBlinkMode;
    }
  }
}

void TBRGTask(void *pvParameters) {
  const uint8_t TBRGPin = 18;
  pinMode(TBRGPin, OUTPUT);
  unsigned int currentTBRG_FREQ = 0;
  ;
  unsigned int newTBRG_FREQ = 0;
  BaseType_t result;
  unsigned rxchar = 0;
  result = xQueueReceive(TBRGinputQueueHandle, &rxchar, portMAX_DELAY);
  currentTBRG_FREQ = rxchar;

  for (;;) {

    if (currentTBRG_FREQ == 0) {
      digitalWrite(TBRGPin, LOW);
      result = xQueueReceive(TBRGinputQueueHandle, &rxchar, portMAX_DELAY);
      //Serial.print(RFQ);
      //currentTBRG_FREQ = atoi(RFQ);
      //currentTBRG_FREQ = &rxchar;
      currentTBRG_FREQ = rxchar;
      //Serial.print("FREQ_zero = ");
      //Serial.println(currentTBRG_FREQ);
    } else {
      // currentTBRG_FREQ = atoi(RFQ);
      if (currentTBRG_FREQ != 0) {
        digitalWrite(TBRGPin, HIGH);
        //vTaskDelay(atoi(RPW));
        vTaskDelay(10);
        digitalWrite(TBRGPin, LOW);
        vTaskDelay(currentTBRG_FREQ);
        //Serial.print("FREQ = ");
        //Serial.println(currentTBRG_FREQ);
      }
      result = xQueueReceive(TBRGinputQueueHandle, &rxchar, 0);
      //newTBRG_FREQ=atoi(RFQ);
      //newTBRG_FREQ = atoi(&rxchar);
      newTBRG_FREQ = rxchar;
      if (result == pdTRUE) {
        currentTBRG_FREQ = newTBRG_FREQ;
      }
    }
  }
}




void loop() {
}