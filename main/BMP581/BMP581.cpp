#include <stdio.h>
#include "Arduino.h"
#include "SPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Pins.h"
#include "BMP581.h"
#include <Adafruit_BMP5xx.h>

#define DEBUG

static Adafruit_BMP5xx BMP;

static bool BMP_init() {
  if (!BMP.begin(BMP581_CS, &SPI)) {   
    return false;
  }
  return true;
}

void BMP_task(void *pvParameter) {
    while (1) {
        if(!BMP_init() || !BMP.dataReady() || !BMP.performReading()) {
            taskYIELD();
        }

        float bmp_temp = BMP.temperature;
        float bmp_press = BMP.pressure;
        float bmp_alt = BMP.readAltitude(1013.25);
        
        TickType_t upTime = xTaskGetTickCount();

        #ifdef DEBUG
            printf("BMP Runtime: %lu\n", upTime);
            printf("Pressure: %f\n", bmp_temp);
            printf("Temperature: %f\n", bmp_temp);
            printf("Altitude: %f\n\n", bmp_alt);
        #endif
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

