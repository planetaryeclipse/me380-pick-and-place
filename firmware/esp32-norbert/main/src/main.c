#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "pinouts.h"

#define BLINK_LOG_TAG "blink-task"
#define BLINK_DELAY_MS 1000

void blink_task(void *){
    gpio_config_t led_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(LED_STATUS_PIN),
        .pull_down_en = true,
        .pull_up_en = false
    };
    gpio_config(&led_gpio_cfg);

    bool enabled = true;
    while (1){
        ESP_LOGI(BLINK_LOG_TAG, "Setting status LED to %s", enabled ? "ON" : "OFF");
        gpio_set_level(LED_STATUS_PIN, enabled);
        enabled = !enabled;

        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100)); // Board setup

    // Initializes tasks
    xTaskCreate(blink_task, "Blink", 4096, NULL, 0, NULL);
}