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
  - convert the AT and ET to signed temperature values
  - add pulse width to rain

Change Log:
  Change Number - Date - Description - Engineer
*****************************************************************************************************************
*/

#include "Arduino.h"
#include <AD5254_asukiaaa.h>
//#include "NVP_Parser.h"

void uartTask(void *pvParameters);
void blinkTask(void *pvParameters);
void TBRGTask(void *pvParameters);
void ATTask(void *pvParameters);
void ETTask(void *pvParameters);
void WDTask(void *pvParameters);


QueueHandle_t LEDinputQueueHandle;
QueueHandle_t TBRGinputQueueHandle;
QueueHandle_t ATinputQueueHandle;
QueueHandle_t ETinputQueueHandle;
QueueHandle_t WDinputQueueHandle;


enum BlinkType {
  NO_BLINK,
  SLOW_BLINK,
  FAST_BLINK
};

signed int FREQ;
signed int AT;
signed int ET;
signed int WD;

/*
char FREQ = 0;
char AT = 0;
char ET = 0;
*/

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting");

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
      if (Serial.read() == '\n') {
        //Serial.println("TBRG_FREQ");
        xQueueOverwrite(TBRGinputQueueHandle, &FREQ);
        //Serial.println("AT DEG C");
        //float AT1 = 0.94008 * AT - 132.6537 + 19;
        //AT = static_cast<int>(round(AT1));
        xQueueOverwrite(ATinputQueueHandle, &AT);
        xQueueOverwrite(ETinputQueueHandle, &ET);
        xQueueOverwrite(WDinputQueueHandle, &WD);
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
        Serial.print("FREQ = ");
        Serial.println(currentTBRG_FREQ);
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