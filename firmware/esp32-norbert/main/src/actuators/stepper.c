#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_log.h"

#include "rom/ets_sys.h"

#include "pinouts.h"
#include "actuators/stepper.h"

void setup_stepper(microstep_resol_t resol)
{
    // Sets up all the GPIO pins necessary to communicate with the IC
    gpio_config_t direction_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_DIRECTION_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&direction_gpio_cfg);

    gpio_config_t step_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_STEP_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&step_gpio_cfg);

    // gpio_config_t sleep_gpio_cfg = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = PIN_BITMASK(STEPPER_SLEEP_PIN),
    //     .pull_down_en = true,
    //     .pull_up_en = false};
    // gpio_config(&sleep_gpio_cfg);

    // gpio_config_t reset_gpio_cfg = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = PIN_BITMASK(STEPPER_RESET_PIN),
    //     .pull_down_en = true,
    //     .pull_up_en = false};
    // gpio_config(&reset_gpio_cfg);

    // gpio_config_t ms3_gpio_cfg = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = PIN_BITMASK(STEPPER_MS3_PIN),
    //     .pull_down_en = true,
    //     .pull_up_en = false};
    // gpio_config(&ms3_gpio_cfg);

    // gpio_config_t ms2_gpio_cfg = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = PIN_BITMASK(STEPPER_MS2_PIN),
    //     .pull_down_en = true,
    //     .pull_up_en = false};
    // gpio_config(&ms2_gpio_cfg);

    // gpio_config_t ms1_gpio_cfg = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = PIN_BITMASK(STEPPER_MS1_PIN),
    //     .pull_down_en = true,
    //     .pull_up_en = false};
    // gpio_config(&ms1_gpio_cfg);

    // gpio_config_t enable_gpio_cfg = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = PIN_BITMASK(STEPPER_ENABLE_PIN),
    //     .pull_down_en = true,
    //     .pull_up_en = false};
    // gpio_config(&enable_gpio_cfg);

    // Sets the stepper pins to be ready to accept commands
    // gpio_set_level(STEPPER_SLEEP_PIN, true);
    // gpio_set_level(STEPPER_RESET_PIN, true);
    // switch (resol)
    // {
    // case FULL_STEP:
    // {
    //     gpio_set_level(STEPPER_MS1_PIN, false);
    //     gpio_set_level(STEPPER_MS2_PIN, false);
    //     gpio_set_level(STEPPER_MS3_PIN, false);
    //     break;
    // }
    // case HALF_STEP:
    // {
    //     gpio_set_level(STEPPER_MS1_PIN, true);
    //     gpio_set_level(STEPPER_MS2_PIN, false);
    //     gpio_set_level(STEPPER_MS3_PIN, false);
    //     break;
    // }
    // case QUARTER_STEP:
    // {
    //     gpio_set_level(STEPPER_MS1_PIN, false);
    //     gpio_set_level(STEPPER_MS2_PIN, true);
    //     gpio_set_level(STEPPER_MS3_PIN, false);
    //     break;
    // }
    // case EIGHTH_STEP:
    // {
    //     gpio_set_level(STEPPER_MS1_PIN, true);
    //     gpio_set_level(STEPPER_MS2_PIN, true);
    //     gpio_set_level(STEPPER_MS3_PIN, false);
    //     break;
    // }
    // case SIXTEENTH_STEP:
    // {
    //     gpio_set_level(STEPPER_MS1_PIN, true);
    //     gpio_set_level(STEPPER_MS2_PIN, true);
    //     gpio_set_level(STEPPER_MS3_PIN, true);
    //     break;
    // }
    // }
    // gpio_set_level(STEPPER_ENABLE_PIN, false);

    gpio_set_level(STEPPER_STEP_PIN, false); // Sets voltage level explictly
}

void rotate_stepper(uint16_t steps, bool fw_dir)
{
    int biased_steps = steps;
    if (fw_dir)
        biased_steps *= 1.13;
    else
        biased_steps *= 1.05;

    gpio_set_level(STEPPER_DIRECTION_PIN, fw_dir);
    ESP_LOGI("test-tag", "Sending stepper pulses!!");
    for (int i = 0; i < biased_steps; i++)
    {
        gpio_set_level(STEPPER_STEP_PIN, true);
        vTaskDelay(pdMS_TO_TICKS(STEPPER_HIGH_PULSE_WIDTH_MS));
        // ets_delay_us(STEPPER_HIGH_PULSE_WIDTH_US);
        gpio_set_level(STEPPER_STEP_PIN, false);
        // ets_delay_us(STEPPER_LOW_PULSE_WIDTH_US);
        vTaskDelay(pdMS_TO_TICKS(STEPPER_LOW_PULSE_WIDTH_MS));
    }
}

