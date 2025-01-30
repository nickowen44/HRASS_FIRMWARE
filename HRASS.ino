#include "Arduino.h"

void uartTask(void *pvParameters);
void blinkTask(void *pvParameters);
void TBRGTask(void *pvParameters);

QueueHandle_t LEDinputQueueHandle;
QueueHandle_t TBRGinputQueueHandle;

enum BlinkType {
  NO_BLINK,
  SLOW_BLINK,
  FAST_BLINK
};


unsigned int FREQ = 0;

void setup() {
  Serial.begin(115200);
  delay(4000);
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

  BaseType_t returnCode = xTaskCreatePinnedToCore(blinkTask, "Blink Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create Blink Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  returnCode = xTaskCreatePinnedToCore(uartTask, "UART Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create UART Task");
    vTaskDelay(1000);
    ESP.restart();
  }

  returnCode = xTaskCreatePinnedToCore(TBRGTask, "TBRG Task", 2000, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  if (returnCode != pdPASS) {
    log_e("Failed to create TBRG Task");
    vTaskDelay(1000);
    ESP.restart();
  }
}

void uartTask(void *pvParameters) {
  BlinkType blinkMode;

  int receivedChar;

  for (;;) {
    while ((receivedChar = Serial.read()) > 0) {
      switch (receivedChar) {
        case 's':
          blinkMode = NO_BLINK;
          break;

        case '1':
          blinkMode = SLOW_BLINK;
          break;

        case '2':
          blinkMode = FAST_BLINK;
          break;

        case 't':
          Serial.println("TBRG_FREQ");
          FREQ = FREQ + 10;
          xQueueOverwrite(TBRGinputQueueHandle, &FREQ);
          break;


        default:
          continue;
      }
      xQueueOverwrite(LEDinputQueueHandle, &blinkMode);
    }
    vTaskDelay(10);
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
  unsigned int currentTBRG_FREQ = FREQ;
  unsigned int newTBRG_FREQ = FREQ;
  BaseType_t result;

  for (;;) {
    if (currentTBRG_FREQ == 0) {
      digitalWrite(TBRGPin, LOW);
      result = xQueueReceive(TBRGinputQueueHandle, &currentTBRG_FREQ, portMAX_DELAY);
    }

    else {
      digitalWrite(TBRGPin, HIGH);
      vTaskDelay(currentTBRG_FREQ);
      digitalWrite(TBRGPin, LOW);
      vTaskDelay(currentTBRG_FREQ);
    }

    Serial.print("FREQ = ");
    Serial.println(currentTBRG_FREQ);


    result = xQueueReceive(TBRGinputQueueHandle, &newTBRG_FREQ, 0);
    if (result == pdTRUE) {
      currentTBRG_FREQ = newTBRG_FREQ;
    }
  }
}








void loop() {
}