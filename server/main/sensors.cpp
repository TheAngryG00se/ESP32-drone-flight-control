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

#include "sensors.h"

void bmp280_handler::print_data(){
    printf("pressure: %.2f, temperature: %.2f\n", pressure, temperature);
}

void bmp280_handler::read_data(){
    if (bmp280_read_float(&bmp_dev, &temperature, &pressure, &humidity) != ESP_OK) {
        printf("\n\nBMP280: Failed to read data\n\n");
    }
}

void mpu6050_handler::read_data(){
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

}

void mpu6050_handler::print_data(){
    printf("YAW: %3.1f, PITCH: %3.1f, ROLL: %3.1f  g: [%.2f, %.2f, %.2f]", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI, gravity.x, gravity.y, gravity.z);
}


esp_err_t init_sensors(mpu6050_handler* mpu_handler, bmp280_handler* bmp_handler){
    esp_err_t ret = i2cdev_init();
    if (ret != ESP_OK) {
        printf("Failed to initialize i2cdev: %d\n", ret);
        vTaskDelete(NULL);
        return ret;
    }

    // Инициализация BMP280
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    // bmp280_t bmp_dev;
    memset(&(bmp_handler->bmp_dev), 0, sizeof(bmp280_t));

    // Инициализация BMP280
    ret = bmp280_init_desc(&(bmp_handler->bmp_dev), BMP280_I2C_ADDRESS_0, I2C_PORT, GPIO_NUM_21, GPIO_NUM_22);
    if (ret != ESP_OK) {
        printf("Failed to initialize BMP280: %d\n", ret);
        vTaskDelete(NULL);
        return ret;
    }

    ret = bmp280_init(&(bmp_handler->bmp_dev), &params);
    if (ret != ESP_OK) {
        printf("Failed to configure BMP280: %d\n", ret);
        vTaskDelete(NULL);
        return ret;
    }

    bool bme280p = (bmp_handler->bmp_dev).id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");


    // Инициализация MPU6050
    mpu_handler->mpu.initialize();
    mpu_handler->mpu.dmpInitialize();
    mpu_handler->mpu.CalibrateAccel(6);
    mpu_handler->mpu.CalibrateGyro(6);
    mpu_handler->mpu.setDMPEnabled(true);
    printf("MPU6050 initialized and calibrated.\n");

    return ESP_OK;
}

void read_sensors(mpu6050_handler* mpu_handler, bmp280_handler* bmp_handler) {
    mpu_handler->read_data();
    bmp_handler->read_data();
}