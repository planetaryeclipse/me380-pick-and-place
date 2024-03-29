#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/queue.h"

#include "pinouts.h"
#include "tasks/test_tasks.h"
#include "sys_state.h"

#include "sensors/color.h"
#include "sensors/limit_swtch.h"
#include "sensors/ultrasonic.h"
#include "actuators/servo.h"
#include "actuators/stepper.h"

#define TEST_BLINK_TAG_NAME "blink-test"
#define TEST_BLINK_DELAY_MS 1000

void test_blink_task(void *)
{
    gpio_config_t led_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(LED_STATUS_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&led_gpio_cfg);

    bool enabled = true;
    while (1)
    {
        if (sys_in_fault())
            vTaskSuspend(NULL);

        // ESP_LOGI(BLINK_LOG_TAG, "Setting status LED to %s", enabled ? "ON" : "OFF");
        gpio_set_level(LED_STATUS_PIN, enabled);
        enabled = !enabled;

        vTaskDelay(pdMS_TO_TICKS(TEST_BLINK_DELAY_MS));
    }
}

#define TEST_READ_SENSORS_TAG_NAME "read-sensors-test"

void test_read_sensors_task(void *)
{
    // Setup sensors
    setup_color_sensor();
    setup_limit_switches();
    setup_ultrasonic();

    // Color sensor readings
    uint16_t c, r, g, b;

    float ultrasonic_dist;

    int adc_raw = 0;
    int voltage = 0;

    while (1)
    {
        if (sys_in_fault())
            vTaskSuspend(NULL);

        // Periodically read sensors
        read_color_sensor(&c, &r, &g, &b);
        ESP_LOGI(TEST_READ_SENSORS_TAG_NAME, "Color reading: c=%i r=%i g=%i b=%i", c, r, g, b);

        if (read_ultrasonic_distance(&ultrasonic_dist))
            ESP_LOGI(TEST_READ_SENSORS_TAG_NAME, "Ultrasonic distance: %f", ultrasonic_dist);
        else
            ESP_LOGI(TEST_READ_SENSORS_TAG_NAME, "Failed to read ultrasonic distance!");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#define TEST_SERVO_TAG_NAME "servo-test"
#define TEST_SERVO_CHNL 3
#define TEST_SERVO_FREQ 30

#define TEST_ARM1_SERVO_CHNL 0
#define TEST_ARM2_SERVO_CHNL 1
#define TEST_ARM3_SERVO_CHNL 15

#define TEST_ARM_SERVO_LOW_MS 2.5
#define TEST_ARM_SERVO_HIGH_MS 1

#define TEST_ARM_SERVO_SYNC_POS 2.2 // with max of .8


void test_servo_task(void *)
{
    ESP_LOGI(SERVO_LOG_TAG_NAME, "Setting up servos!");

    setup_servo_driver(TEST_SERVO_FREQ);

    set_servo_channel_pulse_width(TEST_ARM1_SERVO_CHNL, TEST_SERVO_FREQ, TEST_ARM_SERVO_HIGH_MS);
    set_servo_channel_pulse_width(TEST_ARM2_SERVO_CHNL, TEST_SERVO_FREQ, TEST_ARM_SERVO_HIGH_MS);
    set_servo_channel_pulse_width(TEST_ARM3_SERVO_CHNL, TEST_SERVO_FREQ, TEST_ARM_SERVO_HIGH_MS);

    vTaskDelay(pdMS_TO_TICKS(500));

    set_servo_channel_pulse_width(TEST_ARM1_SERVO_CHNL, TEST_SERVO_FREQ, TEST_ARM_SERVO_LOW_MS);
    set_servo_channel_pulse_width(TEST_ARM2_SERVO_CHNL, TEST_SERVO_FREQ, TEST_ARM_SERVO_LOW_MS);
    set_servo_channel_pulse_width(TEST_ARM3_SERVO_CHNL, TEST_SERVO_FREQ, TEST_ARM_SERVO_LOW_MS);

    bool min_pos = false;
    while (1)
    {
        if (sys_in_fault())
            vTaskSuspend(NULL);

        // ESP_LOGI(TEST_SERVO_TAG_NAME, "Setting servo channel at %s position", min_pos ? "minimum" : "maximum");

        // set_servo_channel_pulse_width(TEST_SERVO_CHNL, TEST_SERVO_FREQ, min_pos ? 1.5 : 2);
        
        // set_servo_channel_pulse_width(TEST_ARM1_SERVO_CHNL, TEST_SERVO_FREQ, min_pos ? TEST_ARM_SERVO_LOW_MS : TEST_ARM_SERVO_HIGH_MS);
        // set_servo_channel_pulse_width(TEST_ARM2_SERVO_CHNL, TEST_SERVO_FREQ, min_pos ? TEST_ARM_SERVO_LOW_MS : TEST_ARM_SERVO_HIGH_MS);
        // set_servo_channel_pulse_width(TEST_ARM3_SERVO_CHNL, TEST_SERVO_FREQ, min_pos ? TEST_ARM_SERVO_LOW_MS : TEST_ARM_SERVO_HIGH_MS);
        
        min_pos = !min_pos;

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#define TEST_STEPPER_TAG_NAME "stepper-test"
#define TEST_STEPPER_STEPS 200
#define TEST_STEPPER_MS_BETWEEN_STEPS 5000

void test_stepper_task(void *)
{
    setup_stepper(FULL_STEP);
    bool dir = true;

    while (1)
    {
        if (sys_in_fault())
            vTaskSuspend(NULL);

        // ESP_LOGI(STEPPER_LOG_TAG_NAME, "Sending steps!");
        rotate_stepper(TEST_STEPPER_STEPS, dir);
        dir = !dir;
        vTaskDelay(pdMS_TO_TICKS(TEST_STEPPER_MS_BETWEEN_STEPS));
    }
}