#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "test_rtos_tasks.h"


#define LED_GPIO 23

void blink_task(void *arg){
    gpio_config_t led_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LED_GPIO,
        .pull_down_en = false,
        .pull_up_en = false,
    };

    gpio_config(&led_cfg);
    
    bool enabled = true;
    
    while(1){
        ESP_LOGI("test-freertos", "Toggling blink!");
        gpio_set_level(LED_GPIO, enabled);
        vTaskDelay(pdMS_TO_TICKS(1000));

        enabled = !enabled;
    }
}