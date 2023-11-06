#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

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
        if(sys_in_fault())
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
        if(sys_in_fault())
            vTaskSuspend(NULL);

        // Periodically read sensors
        read_color_sensor(&c, &r, &g, &b);
        ESP_LOGI(TEST_READ_SENSORS_TAG_NAME, "Color reading: c=%i r=%i g=%i b=%i", c, r, g, b);

        ultrasonic_dist = read_ultrasonic_distance();
        ESP_LOGI(TEST_READ_SENSORS_TAG_NAME, "Ultrasonic distance: %f", ultrasonic_dist);

        /*
        Alternative polling code

        trig_ultrasonic_scan();
        while(xQueueReceive(ultrasonic_distances, &ultrasonic_dist, 0)){
            ESP_LOGI(SENSOR_TEST_TAG_NAME, "Ultrasonic distance: %f", ultrasonic_dist);
        }
        */

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#define TEST_SERVO_TAG_NAME "servo-test"
#define TEST_SERVO_CHNL 3
#define TEST_SERVO_FREQ 30

void test_servo_task(void *)
{
    setup_servo_driver(TEST_SERVO_FREQ);
    set_servo_channel_pulse_width(TEST_SERVO_CHNL, TEST_SERVO_FREQ, 1.5);

    bool min_pos = false;
    while (1)
    {
        if(sys_in_fault())
            vTaskSuspend(NULL);

        ESP_LOGI(TEST_SERVO_TAG_NAME, "Setting servo channel at %s position", min_pos ? "minimum" : "maximum");

        set_servo_channel_pulse_width(TEST_SERVO_CHNL, TEST_SERVO_FREQ, min_pos ? 1.5 : 2);
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
        if(sys_in_fault())
            vTaskSuspend(NULL);

        ESP_LOGI(STEPPER_LOG_TAG_NAME, "Sending steps!");
        rotate_stepper(TEST_STEPPER_STEPS, dir);
        dir = !dir;
        vTaskDelay(pdMS_TO_TICKS(TEST_STEPPER_MS_BETWEEN_STEPS));
    }
}