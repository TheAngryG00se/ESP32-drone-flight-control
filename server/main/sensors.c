#include "sensors.h"

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_EXAMPLE";

int32_t t_fine;
int16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;


void mpu6050_init() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); // PWR_MGMT_1 register address
    i2c_master_write_byte(cmd, 0x00, true);  // Wake up the sensor
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
    }
}

void bmp280_init() {
    // Configure the BMP280
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xF4, true); // Ctrl_meas register address
    i2c_master_write_byte(cmd, 0x27, true);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
    }

    // Add a delay to allow the sensor to stabilize
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

// Function to initialize I2C
esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

void bmp280_read_calibration_data() {
    uint8_t data[24];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x88, true); // Start address of calibration data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 24, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        dig_T1 = (data[1] << 8) | data[0];
        dig_T2 = (data[3] << 8) | data[2];
        dig_T3 = (data[5] << 8) | data[4];
        dig_P1 = (data[7] << 8) | data[6];
        dig_P2 = (data[9] << 8) | data[8];
        dig_P3 = (data[11] << 8) | data[10];
        dig_P4 = (data[13] << 8) | data[12];
        dig_P5 = (data[15] << 8) | data[14];
        dig_P6 = (data[17] << 8) | data[16];
        dig_P7 = (data[19] << 8) | data[18];
        dig_P8 = (data[21] << 8) | data[20];
        dig_P9 = (data[23] << 8) | data[22];
        ESP_LOGI(TAG, "BMP280 - Calibration Data: T1=%d, T2=%d, T3=%d, P1=%d, P2=%d, P3=%d, P4=%d, P5=%d, P6=%d, P7=%d, P8=%d, P9=%d",
                 dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
    } else {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration data");
    }
}

// Function to compensate temperature (returns temperature in °C)
float bmp280_compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return (float)T / 100.0;
}

float bmp280_compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) {
        return 0; // Avoid division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256.0; // Convert to Pa
}

void bmp280_read_data(char* msg) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xF7, true); // Start address of pressure and temperature data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        // ESP_LOGI(TAG, "BMP280 - Raw Pressure: %ld, Raw Temperature: %ld", adc_P, adc_T);
        float temperature = bmp280_compensate_T(adc_T);
        float pressure = bmp280_compensate_P(adc_P);
        // ESP_LOGI(TAG, "BMP280 - Temperature: %.2f °C, Pressure: %.3f Pa", temperature, pressure);
        sprintf(msg, "BMP280 - Temperature: %.2f °C, Pressure: %.3f Pa", temperature, pressure);
    } else {
        ESP_LOGE(TAG, "Failed to read from BMP280");
    }
}

void calculate_euler_angles(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, char* msg) {
    // Convert raw accelerometer data to g (assuming ±2g range)
    float accel_x = ax / 16384.0;
    float accel_y = ay / 16384.0;
    float accel_z = az / 16384.0;

    // Calculate roll and pitch from accelerometer data
    float roll = atan2(accel_y, accel_z) * 180 / M_PI;
    float pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / M_PI;

    // Convert raw gyroscope data to degrees per second (assuming ±250°/s range)
    float gyro_x = gx / 131.0;
    float gyro_y = gy / 131.0;
    float gyro_z = gz / 131.0;

    // ESP_LOGI(TAG, "MPU6050 - Raw Accel: X=%d, Y=%d, Z=%d, Raw Gyro: X=%d, Y=%d, Z=%d", ax, ay, az, gx, gy, gz);
    // ESP_LOGI(TAG, "MPU6050 - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", roll, pitch, gyro_z);
    sprintf(msg, "MPU6050 - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", roll, pitch, gyro_z);
}

// Function to read data from MPU6050
void mpu6050_read_data(char* msg) {
    uint8_t data[14];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true); // Start address of accelerometer and gyroscope data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];
        int16_t gx = (data[8] << 8) | data[9];
        int16_t gy = (data[10] << 8) | data[11];
        int16_t gz = (data[12] << 8) | data[13];
        calculate_euler_angles(ax, ay, az, gx, gy, gz, msg);
    } else {
        ESP_LOGE(TAG, "Failed to read from MPU6050");
    }
}


// void app_main() {
//     // Initialize I2C
//     esp_err_t ret = i2c_master_init();
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "I2C initialization failed");
//         return;
//     }

//     // Initialize BMP280
//     bmp280_init();
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//     bmp280_read_calibration_data();

//     // Initialize MPU6050
//     mpu6050_init();

//     while (1) {
//         // Read data from BMP280
//         bmp280_read_data();

//         // Read data from MPU6050
//         mpu6050_read_data();

//         // Delay for 1 second
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }