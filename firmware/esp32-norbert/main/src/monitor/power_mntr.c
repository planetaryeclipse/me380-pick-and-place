#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

#include "pinouts.h"
#include "monitor/power_mntr.h"
#include "sys_state.h"

#define POWER_MNTR_DISABLED_FOR_DEBUG 1

#define POWER_MNTR_TAG_NAME "power-mntr"
#define POWER_MNTR_TASK_UPDATE_MS 50

static void switch_power_state(bool power_on){
    if (power_on){
        // Board reset to re-initialize setup of connected peripherals
        ESP_LOGI(POWER_MNTR_TAG_NAME, "Power has been restored, resetting power for peripheral re-initialization!");
        esp_restart();
    } else{
        // Terminates all non-essential system tasks given that power has been lost
        ESP_LOGI(POWER_MNTR_TAG_NAME, "Power has been lost, placing system into error state!");
        set_sys_state(FAULT);
    }
}

void power_mntr_task(void *){
    // Configures the GPIO to to monitor for external power
    gpio_config_t power_mtnr_cfg = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(POWER_MNTR_PIN),
        .pull_up_en = false,
        .pull_down_en = true
    };
    gpio_config(&power_mtnr_cfg);

    bool power_on = gpio_get_level(POWER_MNTR_PIN);
    if (!power_on){
        ESP_LOGW(POWER_MNTR_TAG_NAME, "No external power detected at startup!");
        switch_power_state(false);
    }

    bool detected_change = false;
    uint32_t prev_upd_tick_count = xTaskGetTickCount();

    while(true){
        bool curr_power_on = gpio_get_level(POWER_MNTR_PIN);
        if (curr_power_on != power_on){
            if (detected_change){
                // Update persisted over a task update cycle, update actually occurred
                power_on = curr_power_on;
                detected_change = false;
                switch_power_state(power_on);
            } else
                detected_change = true; // Marks a potential update
        }

        // Task is guaranteed to update every 50 ms
        xTaskDelayUntil(&prev_upd_tick_count, pdMS_TO_TICKS(POWER_MNTR_TASK_UPDATE_MS));
    }
}