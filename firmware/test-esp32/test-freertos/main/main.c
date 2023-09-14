/* FreeRTOS Real Time Stats Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"

#define LED_PIN 23

static void blink_task(void *arg){
    bool enabled = true;
    
    while(1){
        ESP_LOGI("test-freertos", "Toggling blink!");
        gpio_set_level(LED_PIN, enabled);
        vTaskDelay(pdMS_TO_TICKS(1000));

        enabled = !enabled;
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_config_t led_cfg;
    led_cfg.intr_type = GPIO_INTR_DISABLE;
    led_cfg.mode = GPIO_MODE_OUTPUT;
    led_cfg.pin_bit_mask = (1 << 23);
    led_cfg.pull_down_en = false;
    led_cfg.pull_up_en = false;

    gpio_config(&led_cfg);

    xTaskCreate(blink_task, "blink", 4096, NULL, 0, NULL);

    //Create and start stats task
    // xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, STATS_TASK_PRIO, NULL, tskNO_AFFINITY);
}
