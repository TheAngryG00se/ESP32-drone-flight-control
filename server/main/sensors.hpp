#pragma once

#include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h" //хз почему, но конкретно эта строка мешает линковке (multiple def errors)
#include <bmp280.h>

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_PORT I2C_NUM_0

struct mpu6050_handler{
private:
    uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;          // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];      // FIFO storage buffer
    uint8_t mpuIntStatus;        // holds actual interrupt status byte from MPU
    public:
    MPU6050 mpu;                 // mpu6050 instance
    Quaternion q;                // [w, x, y, z]         quaternion container
    VectorFloat gravity;         // [x, y, z]            gravity vector
    float ypr[3];                // [yaw, pitch, roll]   yaw/pitch/roll container
    

    void read_data();
    void print_data();
};

struct bmp280_handler{
    float pressure;
    float temperature; 
    float humidity;
    bmp280_t bmp_dev;

    void read_data();
    void print_data();
};

esp_err_t init_sensors(mpu6050_handler* mpu_handler, bmp280_handler* bmp_handler);
void read_sensors(mpu6050_handler* mpu_handler, bmp280_handler* bmp_handler);