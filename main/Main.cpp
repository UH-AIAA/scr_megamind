// ///////////////////////////////////////////////////////////////////////
// /*                   SCR ESP-32 Brute Sensor Test                    */
// ///////////////////////////////////////////////////////////////////////
// /*                           N. Samuell                              */
// /*                      FreeRTOS/ESP-IDF test                        */
// /*                          MIT License                              */
// ///////////////////////////////////////////////////////////////////////


// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include <utility/imumaths.h>

// #include "esp_system.h"
// #include "driver/gpio.h"

// // SPI imports, I^2C, and UART Imports
// #include "SPI.h"
// #include "Arduino.h"
// #include "Adafruit_BMP5xx.h"
// #include "Adafruit_ADXL375.h"
// #include "Adafruit_LSM6DSO32.h"
// #include "Adafruit_BNO055.h"
// #include "Adafruit_GPS.h"
// #include "LoRa.h"
// #include "Pins.h"

// // SRAD Imports
// #include "SRAD_PHX.h"

// // Debug control definitions
// #define DEBUG

// // Chip Object Instantiation
// Adafruit_BMP5xx BMP;
// Adafruit_ADXL375 ADXL(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, ADXL375_CS);
// Adafruit_LSM6DSO32 LSM;
// Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
// Adafruit_GPS GPS(&Wire);

// void init_spi() {
//     // set outputs/inputs for software spi
//     gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
//     gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
//     gpio_set_direction(GPIO_NUM_11, GPIO_MODE_INPUT);

//     // start SPI bus
//     BMP.begin(BMP581_CS, &SPI);
//     ADXL.begin();
//     LSM.begin_SPI(LSM6DSO32_CS, SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
// }

// void init_I2C() {
//     Wire.begin(I2C_SDA, I2C_SCL);
//     BNO.begin();
// }

// void BMP_task(void *pvParameter);
// void ADXL_task(void *pvParameter);
// void BNO_task(void *pvParameter);
// void LSM_task(void *pvParameter);
// void GPS_task(void *pvParameter);

// extern "C" void app_main()
// {
//     // init Arduino Framework from ESP HAL
//     initArduino();

//     // init SPI buses
//     init_spi();

//     // init I^2C bus
//     init_I2C();

//     // dump GPIO config
//     gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
    
//     xTaskCreatePinnedToCore(BMP_task, "BMP_task", 5000, NULL, 5, NULL, 0);
//     xTaskCreatePinnedToCore(ADXL_task, "ADXL_task", 5000, NULL, 4, NULL, 0);
//     xTaskCreatePinnedToCore(LSM_task, "LSM_task", 5000, NULL, 3, NULL, 0);
//     xTaskCreatePinnedToCore(BNO_task, "BNO_task", 5000, NULL, 2, NULL, 0);
//     xTaskCreatePinnedToCore(GPS_task, "GPS_task", 5000, NULL, 1, NULL, 1);
// }

// void BMP_task(void *pvParameter) {
//     while (1) {
//         if(!BMP.performReading()) {
//           taskYIELD();
//         }
//         float bmp_temp = BMP.temperature;
//         float bmp_press = BMP.pressure;
//         float bmp_alt = BMP.readAltitude(1013.25);
        
//         TickType_t upTime = xTaskGetTickCount();

//         #ifdef DEBUG
//             printf("BMP Runtime: %lu\n", upTime);
//             printf("Pressure: %f\n", bmp_press);
//             printf("Temperature: %f\n", bmp_temp);
//             printf("Altitude: %f\n\n", bmp_alt);
//         #endif
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }

// void ADXL_task(void *pvParameter) {

//     while(1) {
//     //ADXL already initialized in previous code?
//         //check ADXL connection and yield to other tasks if connection bad
//         uint32_t startTime = millis();
//         //Taking the time since activation of sensor
//         TickType_t uptime = xTaskGetTickCount();

//         sensors_event_t event;
//         //stores data from the sensor into the sensor event variable "accelration"
//         if(!ADXL.getEvent(&event)) {
//             taskYIELD();
//         }

//         uint32_t endTime = millis();

//         #ifdef DEBUG
//             printf("ADXL Uptime: %lu [ms]\n",uptime);
//             //Display the results (acceleration is measured in m/s^2)

//             printf("X: %f [m/s^2]\n",event.acceleration.x);
//             printf("Y: %f [m/s^2]\n",event.acceleration.y);
//             printf("Z: %f [m/s^2]\n",event.acceleration.z);
//             printf("Elapsed Time: %li\n\n", endTime - startTime);
//         #endif

//         //I believe the delay function in comment below is for arduino
//         //delay(500);
//         //delay 1 tick
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

// void BNO_task(void *pvParameter) {
//     while (1) {
//         // printf("BNO Task Called!\n");
//         sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;

//         if (!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
//             taskYIELD();
//         }
//         if (!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
//             taskYIELD();
//         }
//         if (!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER)) {
//             taskYIELD();
//         }
//         if (!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
//             taskYIELD();
//         }
        
//         uint32_t startTime = millis();
//         TickType_t uptime = xTaskGetTickCount();
        
//         imu::Quaternion quat = BNO.getQuat();

//         uint32_t endTime = millis();

//         #ifdef DEBUG
//             printf("BNO Uptime: %lu\n", uptime);
//             printf("Quaternion:\n");
//             printf("W: %f\n", quat.w());
//             printf("X: %f\n", quat.x());
//             printf("Y: %f\n", quat.y());
//             printf("Z: %f\n\n", quat.z());

//             printf("Euler Orientation:\n");
//             printf("X: %f\n", angVelocityData.gyro.x);
//             printf("Y: %f\n", angVelocityData.gyro.y);
//             printf("Z: %f\n", angVelocityData.gyro.z);
//             printf("Elapsed Time: %li\n\n\n", endTime - startTime);
//             // other data has been left out to avoid slowing down printing

//         #endif

//         vTaskDelay(pdMS_TO_TICKS(20 - (endTime - startTime)));
//     }
// }

// void LSM_task(void *pvParameter) {
//     while (1) {
//         TickType_t uptime = xTaskGetTickCount();

//         uint32_t startTime = millis();

//         //Sensor events
//         sensors_event_t accel;
//         sensors_event_t gyro;
//         sensors_event_t temp;

//         //event to get data
//         if(!LSM.getEvent(&accel, &gyro, &temp)) {
//             taskYIELD();
//         }

//         //data printing 

//         #ifdef DEBUG
//             printf("LSM Ticktime: %lu [ms]\n", uptime);
//             printf("X Acceleration: %f [m/s^2]\n", accel.acceleration.x);
//             printf("Y Acceleration: %f [m/s^2]\n", accel.acceleration.y);
//             printf("Z Acceleration: %f [m/s^2]\n", accel.acceleration.z);
//             printf("X Gyro: %f [idk]\n",gyro.gyro.x);
//             printf("Y Gyro: %f [idk]\n",gyro.gyro.y);
//             printf("Z Gyro: %f [idk]\n",gyro.gyro.z);
//             // printf("Temperature: %f [deg C]\n",temp);
//             uint32_t endTime = millis();
//             printf("Elapsed Time: %li\n\n", endTime - startTime);

//         #endif
        
//         //delay funct 
//         //delay (500), 1 tick
//         vTaskDelay(pdMS_TO_TICKS(5));
//     }
// }

// void GPS_task(void *pvParameter) {
//     while(true){
//         uint32_t startms = millis();
//         uint32_t timeout = startms + 200;

//         while (millis() < timeout) {
//             while (GPS.available()) {
//                 GPS.read();
//                 //choke point is from GPS.read();
//                 //can only read 1 byte at a time
//                 if (GPS.newNMEAreceived()) {
//                     if (!GPS.parse(GPS.lastNMEA())) {
//                         continue;
//                     }
//                     if (GPS.fix && GPS.satellites > 0) {
//                         printf("Satellites: %i\n", GPS.satellites);
//                         printf("Latitude: %f, %c\n", GPS.latitude,GPS.lat);
//                         printf("Longitude: %f, %c\n", GPS.longitude, GPS.lon);
//                         printf("Altitude: %f [meters]\n", GPS.altitude);
//                         //Collects speed over the ground, not sure how useful it'll be
//                         //printf("Speed (knots): %f\n" GPS.speed);

//                         // if we found data, go to end of function
//                         // we don't want to print out the same data multiple times
//                         goto end;
//                     }
//                 }
//             }
//         }

//         end:
//             uint32_t endms = millis();
//             uint32_t spentms = endms - startms;
//             printf("Millis: %lu\n", endms);
//             printf("Task took %lu ms to complete.\n\n", spentms);
//             uint32_t delayms = std::min(200 - spentms, static_cast<uint32_t>(10));
//             vTaskDelay(delayms);
//     }
// }




// Main.cpp — Diagnostics ON, ordered loop, BMP fixed (no manual CS), I2C auto-detect

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <utility/imumaths.h>

#include "esp_system.h"
#include "driver/gpio.h"

#include "SPI.h"
#include "Arduino.h"
#include "Wire.h"

#include "Adafruit_BMP5xx.h"
#include "Adafruit_ADXL375.h"
#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_GPS.h"

#include "Pins.h"
#include "SRAD_PHX.h"

#define DEBUG

// ---------- Tunables ----------
#define SENSOR_LOOP_MS   50
#define STACK_SZ_BYTES   8192   // 8 KB per task
#define PRIO_SENSORS     4
#define PRIO_GPS         3
#define PRIO_HEARTBEAT   2

// ---------- Devices ----------
Adafruit_BMP5xx BMP;
Adafruit_ADXL375 ADXL(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, ADXL375_CS); // declared but disabled for now
Adafruit_LSM6DSO32 LSM;                                                     // declared but disabled for now
Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS GPS(&Wire);

// ---------- SPI mutex (optional) ----------
static SemaphoreHandle_t spiMutex = nullptr;

// ---------- Availability flags ----------
static bool hasBMP = false;
static bool hasADXL = false; // disabled to isolate BMP first
static bool hasLSM = false;  // disabled to isolate BMP first
static bool hasBNO = false;
static bool hasGPS = false;

// ---------- Re-probe cadence ----------
static const uint32_t REPROBE_MS = 5000;
static uint32_t nextProbeBMP = 0, nextProbeBNO = 0, nextProbeGPS = 0;

// ---------- Prototypes ----------
static void Sensors_task(void *pvParameter);
static void GPS_task(void *pvParameter);
static void Heartbeat_task(void *pvParameter);

static inline void poll_BMP();
static inline void poll_ADXL();
static inline void poll_LSM();
static inline void poll_BNO();

static inline void reprobe_if_needed();

static void init_spi();
static void init_i2c_if_present();
static bool i2c_present(uint8_t addr);

// =======================================================
// SPI init: force CS HIGH, start HW SPI, very slow freq, enable BMP only.
// =======================================================
static void init_spi() {
  pinMode(BMP581_CS,    OUTPUT); digitalWrite(BMP581_CS,    HIGH);
  pinMode(ADXL375_CS,   OUTPUT); digitalWrite(ADXL375_CS,   HIGH);
  pinMode(LSM6DSO32_CS, OUTPUT); digitalWrite(LSM6DSO32_CS, HIGH);

  SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  SPI.setFrequency(125000UL); // 125 kHz ultra-safe

  hasBMP = BMP.begin(BMP581_CS, &SPI); // only BMP enabled now

  uint32_t now = millis();
  nextProbeBMP = now + REPROBE_MS;

  #ifdef DEBUG
  printf("[init_spi] BMP=%d (ADXL/LSM temporarily disabled)\n", hasBMP);
  #endif
}

// =======================================================
// I2C init: only start devices that ACK to avoid NACK spam.
// =======================================================
static bool i2c_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  uint8_t e = Wire.endTransmission(true);
  return (e == 0);
}

static void init_i2c_if_present() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(10);

  bool bno_ack = i2c_present(0x28) || i2c_present(0x29);
  hasBNO = bno_ack ? BNO.begin() : false;

  bool gps_ack = i2c_present(0x10);
  hasGPS = gps_ack ? GPS.begin(0x10) : false;

  uint32_t now = millis();
  nextProbeBNO = now + REPROBE_MS;
  nextProbeGPS = now + REPROBE_MS;

  #ifdef DEBUG
  printf("[init_i2c] BNO=%d  GPS=%d\n", hasBNO, hasGPS);
  #endif
}

// =======================================================
// Re-probe missing devices occasionally (ADXL/LSM off for now).
// =======================================================
static inline void reprobe_if_needed() {
  uint32_t now = millis();

  if (!hasBMP && now >= nextProbeBMP) {
    hasBMP = BMP.begin(BMP581_CS, &SPI);
    nextProbeBMP = now + REPROBE_MS;
    #ifdef DEBUG
    if (hasBMP) printf("[probe] BMP online\n");
    #endif
  }
  if (!hasBNO && now >= nextProbeBNO) {
    bool bno_ack = i2c_present(0x28) || i2c_present(0x29);
    hasBNO = bno_ack ? BNO.begin() : false;
    nextProbeBNO = now + REPROBE_MS;
    #ifdef DEBUG
    if (hasBNO) printf("[probe] BNO online\n");
    #endif
  }
  if (!hasGPS && now >= nextProbeGPS) {
    bool gps_ack = i2c_present(0x10);
    hasGPS = gps_ack ? GPS.begin(0x10) : false;
    nextProbeGPS = now + REPROBE_MS;
    #ifdef DEBUG
    if (hasGPS) printf("[probe] GPS online\n");
    #endif
  }
}

// =======================================================
// BMP poll: ***no manual CS/transaction*** — let Adafruit driver manage CS.
// =======================================================
static inline void poll_BMP() {
  if (!hasBMP) { vTaskDelay(pdMS_TO_TICKS(1)); return; }

  if (spiMutex && xSemaphoreTake(spiMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  bool ok = BMP.performReading();  // Adafruit_BusIO selects/deselects CS internally

  if (spiMutex) xSemaphoreGive(spiMutex);

  if (!ok) { vTaskDelay(pdMS_TO_TICKS(5)); return; }

  #ifdef DEBUG
    float alt = BMP.readAltitude(1013.25f);
    printf("[BMP] P=%.2f Pa  T=%.2f C  Alt=%.2f m\n", BMP.pressure, BMP.temperature, alt);
  #endif
}

// =======================================================
// ADXL/LSM (skipped for now to isolate BMP).
// =======================================================
static inline void poll_ADXL() {
  if (!hasADXL) { vTaskDelay(pdMS_TO_TICKS(1)); return; }
}

static inline void poll_LSM() {
  if (!hasLSM) { vTaskDelay(pdMS_TO_TICKS(1)); return; }
}

// =======================================================
// BNO (I2C) — only if present.
// =======================================================
static inline void poll_BNO() {
  if (!hasBNO) { vTaskDelay(pdMS_TO_TICKS(1)); return; }

  sensors_event_t e,g,m,acc;
  bool ok = true;
  ok &= BNO.getEvent(&e,   Adafruit_BNO055::VECTOR_EULER);
  ok &= BNO.getEvent(&g,   Adafruit_BNO055::VECTOR_GYROSCOPE);
  ok &= BNO.getEvent(&m,   Adafruit_BNO055::VECTOR_MAGNETOMETER);
  ok &= BNO.getEvent(&acc, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  if (!ok) { vTaskDelay(pdMS_TO_TICKS(5)); return; }

  imu::Quaternion q = BNO.getQuat();
  #ifdef DEBUG
  printf("[BNO] q=(%.3f, %.3f, %.3f, %.3f)  gyro=(%.3f, %.3f, %.3f)\n",
         q.w(), q.x(), q.y(), q.z(),
         g.gyro.x, g.gyro.y, g.gyro.z);
  #endif
}

// =======================================================
// Core 1: Sensor orchestrator (BMP → ADXL → LSM → BNO)
// Prints a "started" banner, a loop tick, and stack watermark.
// =======================================================
static void Sensors_task(void *pvParameter) {
  printf("[Sensors_task] started on core %d\n", xPortGetCoreID());

  TickType_t lastWatermark = xTaskGetTickCount();
  TickType_t lastTickPrint = xTaskGetTickCount();

  for (;;) {
    uint32_t start = millis();

    reprobe_if_needed();

    // Strict order preserved
    poll_BMP();   // 1
    poll_ADXL();  // 2 (skipped)
    poll_LSM();   // 3 (skipped)
    poll_BNO();   // 4 (if present)

    // Every ~1 s, print a tick so we know the loop isn't stuck silently
    if ((xTaskGetTickCount() - lastTickPrint) > pdMS_TO_TICKS(1000)) {
      printf("[Sensors_task] tick\n");
      lastTickPrint = xTaskGetTickCount();
    }

    // Every ~5 s, print watermark
    if ((xTaskGetTickCount() - lastWatermark) > pdMS_TO_TICKS(5000)) {
      UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
      printf("[Sensors_task] stack high-water: %u words (~%u bytes free)\n",
             (unsigned)hw, (unsigned)(hw * sizeof(StackType_t)));
      lastWatermark = xTaskGetTickCount();
    }

    uint32_t spent = millis() - start;
    uint32_t rest  = (spent < SENSOR_LOOP_MS) ? (SENSOR_LOOP_MS - spent) : 1;
    vTaskDelay(pdMS_TO_TICKS(rest));
  }
}

// =======================================================
// Core 0: GPS — only if present; prints started banner.
// =======================================================
static void GPS_task(void *pvParameter) {
  printf("[GPS_task] started on core %d\n", xPortGetCoreID());

  TickType_t lastWatermark = xTaskGetTickCount();

  for (;;) {
    if (!hasGPS) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

    const uint32_t start = millis();
    const uint32_t deadline = start + 200;

    while (millis() < deadline) {
      if (GPS.available()) {
        GPS.read();
        if (GPS.newNMEAreceived()) {
          if (GPS.parse(GPS.lastNMEA())) {
            if (GPS.fix && GPS.satellites > 0) {
              #ifdef DEBUG
              printf("[GPS] sats=%d  lat=%f %c  lon=%f %c  alt=%.2f m\n",
                     GPS.satellites, GPS.latitude, GPS.lat, GPS.longitude, GPS.lon, GPS.altitude);
              #endif
              break;
            }
          }
        }
      } else {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }

    if ((xTaskGetTickCount() - lastWatermark) > pdMS_TO_TICKS(5000)) {
      UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
      printf("[GPS_task] stack high-water: %u words (~%u bytes free)\n",
             (unsigned)hw, (unsigned)(hw * sizeof(StackType_t)));
      lastWatermark = xTaskGetTickCount();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =======================================================
// Core 0: Heartbeat (1 Hz) — proves scheduler is alive even if sensors crash.
// =======================================================
static void Heartbeat_task(void *pvParameter) {
  printf("[Heartbeat_task] started on core %d\n", xPortGetCoreID());
  for (;;) {
    printf("[Heartbeat] alive\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =======================================================
extern "C" void app_main() {
  initArduino();

  // SPI mutex (optional)
  spiMutex = xSemaphoreCreateMutex();

  init_spi();
  init_i2c_if_present();

  printf("[app_main] creating tasks...\n");

  BaseType_t rc1 = xTaskCreatePinnedToCore(
    Sensors_task, "Sensors_task", STACK_SZ_BYTES, nullptr, PRIO_SENSORS, nullptr, 1);
  printf("[app_main] Sensors_task create rc=%ld\n", (long)rc1);

  BaseType_t rc2 = xTaskCreatePinnedToCore(
    GPS_task, "GPS_task", STACK_SZ_BYTES, nullptr, PRIO_GPS, nullptr, 0);
  printf("[app_main] GPS_task create rc=%ld\n", (long)rc2);

  BaseType_t rc3 = xTaskCreatePinnedToCore(
    Heartbeat_task, "Heartbeat_task", 4096, nullptr, PRIO_HEARTBEAT, nullptr, 0);
  printf("[app_main] Heartbeat_task create rc=%ld\n", (long)rc3);

  fflush(stdout);
}
