#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp280.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "i2cdev.h"

// Конфигурация I2C
#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_PORT I2C_NUM_0

// Глобальные переменные для MPU6050
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

// Функция для чтения данных с обоих датчиков
void read_sensors(void *pvParameters) {
    // Инициализация i2cdev
    esp_err_t ret = i2cdev_init();
    if (ret != ESP_OK) {
        printf("Failed to initialize i2cdev: %d\n", ret);
        vTaskDelete(NULL);
        return;
    }

    // Инициализация BMP280
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t bmp_dev;
    memset(&bmp_dev, 0, sizeof(bmp280_t));

    // Инициализация BMP280
    ret = bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, I2C_PORT, GPIO_NUM_21, GPIO_NUM_22);
    if (ret != ESP_OK) {
        printf("Failed to initialize BMP280: %d\n", ret);
        vTaskDelete(NULL);
        return;
    }

    ret = bmp280_init(&bmp_dev, &params);
    if (ret != ESP_OK) {
        printf("Failed to configure BMP280: %d\n", ret);
        vTaskDelete(NULL);
        return;
    }

    bool bme280p = bmp_dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    // Инициализация MPU6050
    MPU6050 mpu;
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    printf("MPU6050 initialized and calibrated.\n");

    // Переменные для хранения данных
    float pressure, temperature, humidity;

    while (1) {
        // Чтение данных с BMP280
        if (bmp280_read_float(&bmp_dev, &temperature, &pressure, &humidity) == ESP_OK) {
            if (bme280p) {
                printf(", Humidity: %.2f\n", humidity);
            } else {
                // printf("\n");
            }
        } else {
            printf("BMP280: Failed to read data\n");
        }

        // Чтение данных с MPU6050
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        // ESP_LOGI("MPU", "%d %d\n", mpuIntStatus, fifoCount); 

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
        } else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // printf("MPU6050: YAW: %3.1f, PITCH: %3.1f, ROLL: %3.1f \n", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
        }

        printf("YAW: %3.1f, PITCH: %3.1f, ROLL: %3.1f Pressure: %.2f Pa, Temperature: %.2f C\n", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI, pressure, temperature);

        // Задержка между чтениями
        vTaskDelay(pdMS_TO_TICKS(100)); // Reduced delay to prevent FIFO overflow
    }
}

extern "C" void app_main(void) {
    
    // Создание задачи для чтения данных с датчиков
    xTaskCreate(read_sensors, "read_sensors", 8192 * 2, NULL, 5, NULL);
}