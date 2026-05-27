// Sensor SPI init
#define SPI_SCLK_PIN 12
#define SPI_MISO_PIN 13
#define SPI_MOSI_PIN 11
#define SPI_MAX_TRSZ 4096

// used for hardware prototype that's flipped
// #define SPI_SCLK_PIN 18
// #define SPI_MISO_PIN 16
// #define SPI_MOSI_PIN 17

// #define VSPI_SCLK_PIN 12
// #define VSPI_MISO_PIN 11
// #define VSPI_MOSI_PIN 13

// SD+LoRa SPI Init
#define VSPI_SCLK_PIN 18
#define VSPI_MISO_PIN 17
#define VSPI_MOSI_PIN 16
#define VSPI_MAX_TRSZ 4092

// I^2C Init
#define I2C_SDA 8
#define I2C_SCL 9

// RTC I2C pins
#define RTC_SDA 14
#define RTC_SCL 15

// CS definitions
#define BMP390_CS 10
#define ADXL375_CS 5
#define LSM6DSO32_CS 4
#define LORA_CS 7
#define SD_CS 20

// Lo-Ra Control Pins
#define LORA_RST 21
#define LORA_IRQ 19
