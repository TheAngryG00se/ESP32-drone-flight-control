#include "stdlib.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22        // GPIO pin for I2C clock
#define I2C_MASTER_SDA_IO 21        // GPIO pin for I2C data
#define I2C_MASTER_FREQ_HZ 100000   // I2C master clock frequency
#define I2C_MASTER_PORT I2C_NUM_0   // I2C port number

#define BMP280_ADDR 0x76            // BMP280 I2C address
#define MPU6050_ADDR 0x68           // MPU6050 I2C address


// BMP280 calibration data

esp_err_t i2c_master_init();


void bmp280_init();
void bmp280_read_calibration_data();
void bmp280_read_data();

void mpu6050_init();
void mpu6050_read_data();