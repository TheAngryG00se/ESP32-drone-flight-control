#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (32) // GPIO для вывода PWM
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Разрешение ШИМ
#define LEDC_FREQUENCY          (50) // Частота ШИМ в Гц (стандартная для ESC)

static const char *TAG = "BLDC_MOTOR_CONTROL";

// Функция для установки duty cycle в микросекундах
void set_duty_microseconds(uint32_t duty_us) {
    // Преобразуем микросекунды в значение duty cycle
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1; // Максимальное значение duty cycle
    uint32_t duty = (duty_us * LEDC_FREQUENCY * max_duty) / 1000000;
    // printf("max: %ld, current: %ld\n", max_duty, duty);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void app_motor(void) {
    // Настройка таймера ШИМ
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Настройка канала ШИМ
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Начальная скважность
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Инициализация ESC (отправка max сигнала на 2 секунды)
    ESP_LOGI(TAG, "Initializing ESC...");
    uint16_t max_fullf = 2000;
    set_duty_microseconds(max_fullf); // 1000 мкс (минимальная скорость)
    vTaskDelay(pdMS_TO_TICKS(1000)); // Ждем 2 секунды

    // Инициализация ESC (отправка min сигнала на 2 секунды)
    ESP_LOGI(TAG, "Initializing ESC...");
    uint16_t min_fullf = 1000;
    set_duty_microseconds(min_fullf); // 1000 мкс (минимальная скорость)
    vTaskDelay(pdMS_TO_TICKS(1000)); // Ждем 2 секунды

    // Запуск мотора на 25% скорости
    for(int i = 0; i < (max_fullf-min_fullf)/25; i++){
        uint32_t duty_percent = min_fullf + i*25;
        set_duty_microseconds(duty_percent);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    set_duty_microseconds(1000); // 1000 мкс (минимальная скорость)
    vTaskDelay(pdMS_TO_TICKS(2000)); // Ждем 2 секунды
}