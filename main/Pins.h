#ifndef PINS_H
#define PINS_H

/// Sensor SPI init
#define SPI_SCLK_PIN 12 /// CLK
#define SPI_MISO_PIN 13 /// SDO
#define SPI_MOSI_PIN 11 /// SDA
#define SPI_MAX_TRSZ 4096

/// SD+LoRa SPI Init
#define VSPI_SCLK_PIN 18 /// CLK
#define VSPI_MISO_PIN 17 /// SDO
#define VSPI_MOSI_PIN 16 /// SDO

/// I2C Init
#define I2C_SDA 8
#define I2C_SCL 9

/// CS definitions
#define BMP581_CS 10
#define ADXL375_CS 5
#define LSM6DSO32_CS 4
#define LORA_CS 7
#define SD_CS 20

/// Lo-Ra Control Pins
#define LORA_RST 21
#define LORA_G0_INT 19

#endif