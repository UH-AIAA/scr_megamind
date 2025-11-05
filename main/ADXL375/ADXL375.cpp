#include <stdio.h>
#include "Arduino.h"
#include "SPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Pins.h"
#include "ADXL375.h"
#include "Adafruit_ADXL375.h"
#include "Adafruit_Sensor.h"

#define DEBUG

static Adafruit_ADXL375 ADXL(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, ADXL375_CS);

static bool ADXL_init() {
  if (!ADXL.begin()) { 
    return false;
  }
  return true;
}

void ADXL_task(void *pvParameter) {
    if (!ADXL_init()) {
      printf("[ADXL] not present; yield\n");
      taskYIELD();            
    }
    while(1) {
        uint32_t startTime = millis();
        //Taking the time since activation of sensor
        TickType_t uptime = xTaskGetTickCount();

        sensors_event_t event;
        //stores data from the sensor into the sensor event variable "accelration"
        if(!ADXL.getEvent(&event)) {
            taskYIELD();
        }

        uint32_t endTime = millis();

        #ifdef DEBUG
            printf("ADXL Uptime: %lu [ms]\n",uptime); //Display the results (acceleration is measured in m/s^2)
            printf("X: %f [m/s^2]\n",event.acceleration.x);
            printf("Y: %f [m/s^2]\n",event.acceleration.y);
            printf("Z: %f [m/s^2]\n",event.acceleration.z);
            printf("Elapsed Time: %li\n\n", endTime - startTime);
        #endif
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

