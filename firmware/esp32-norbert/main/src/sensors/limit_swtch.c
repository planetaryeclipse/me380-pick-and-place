#include "driver/gpio.h"
#include "esp_log.h"

#include "pinouts.h"
#include "sensors/limit_swtch.h"

void isr_homing_limit_switch(void *arg)
{
    ESP_EARLY_LOGI(LIMIT_SWTCH_TAG_NAME, "Homing limit switch triggered");
}

void isr_arm_limit_switch(void *arg)
{
    arm_limit_switch_loc loc = (arm_limit_switch_loc)arg;
    switch (loc)
    {
    case ARM1_INNER:
    {
        ESP_EARLY_LOGI(LIMIT_SWTCH_TAG_NAME, "ARM1_INNER limit switch triggered");
        break;
    }
    case ARM1_OUTER:
    {
        ESP_EARLY_LOGI(LIMIT_SWTCH_TAG_NAME, "ARM1_OUTER limit switch triggered");
        break;
    }
    case ARM2_INNER:
    {
        ESP_EARLY_LOGI(LIMIT_SWTCH_TAG_NAME, "ARM2_INNER limit switch triggered");
        break;
    }
    case ARM2_OUTER:
    {
        ESP_EARLY_LOGI(LIMIT_SWTCH_TAG_NAME, "ARM2_OUTER limit switch triggered");
        break;
    }
    case ARM3_INNER:
    {
        ESP_EARLY_LOGI(LIMIT_SWTCH_TAG_NAME, "ARM3_INNER limit switch triggered");
        break;
    }
    case ARM3_OUTER:
    {
        ESP_EARLY_LOGI(LIMIT_SWTCH_TAG_NAME, "ARM3_OUTER limit switch triggered");
        break;
    }
    }
}

static inline void setup_arm_switch(uint8_t pin, arm_limit_switch_loc loc)
{
    gpio_config_t arm_lim_swch = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(pin),
        .pull_up_en = false,
        .pull_down_en = true};
    gpio_config(&arm_lim_swch);
    gpio_isr_handler_add(pin, isr_arm_limit_switch, (void *)loc);
}

void setup_limit_switches()
{
    gpio_config_t homing_lim_swch = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(HOMING_LIM_SWCH_PIN),
        .pull_up_en = false,
        .pull_down_en = true};
    gpio_config(&homing_lim_swch);
    gpio_isr_handler_add(HOMING_LIM_SWCH_PIN, isr_homing_limit_switch, NULL);

    setup_arm_switch(ARM1_INNER_LIM_SWCH_PIN, ARM1_INNER);
    setup_arm_switch(ARM1_OUTER_LIM_SWCH_PIN, ARM1_OUTER);

    setup_arm_switch(ARM2_INNER_LIM_SWCH_PIN, ARM2_INNER);
    setup_arm_switch(ARM2_OUTER_LIM_SWCH_PIN, ARM2_OUTER);

    setup_arm_switch(ARM3_INNER_LIM_SWCH_PIN, ARM3_INNER);
    setup_arm_switch(ARM3_OUTER_LIM_SWCH_PIN, ARM3_OUTER);
}