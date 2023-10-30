#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"

void app_main(void)
{
    gpio_config_t led_cfg;
    led_cfg.intr_type = GPIO_INTR_DISABLE;
    led_cfg.mode = GPIO_MODE_OUTPUT;
    led_cfg.pin_bit_mask = (1 << 23);
    led_cfg.pull_down_en = false;
    led_cfg.pull_up_en = false;

    gpio_config(&led_cfg);
    gpio_set_level(23, 1);

    // esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI("test_firmware_bare_metal", "Hello there!");

    esp_rom_delay_us(1000000);

    ESP_LOGI("test_firmware_bare_metal", "Just finished waiting up!");
}
