#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include "bmp280.h"

extern "C" {
    void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);
extern void bmp280_test(void*);

void app_main(void)
{
    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL); 

    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(bmp280_test, "bmp280_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}