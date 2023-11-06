#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include "pinouts.h"
#include "sensors/ultrasonic.h"

static int64_t ultrasonic_echo_start_us;
static QueueHandle_t ultrasonic_distances;

void isr_ultrasonic_echo(void *arg)
{
    // Due to decreased allowable stack sizes in ISRs there is no possible way to output
    // the distance value in this function. This is handled by the reading location.

    if (gpio_get_level(ULTRASONIC_ECHO_PIN))
    {
        // Record time at the start of the echo pulse
        ultrasonic_echo_start_us = esp_timer_get_time();
    }
    else if (ultrasonic_echo_start_us != -1)
    {
        // Only calculates the distance if the positive edge was previously detected
        // indicating a valid response (and the positive edge was not missed)

        int64_t fly_time_us = esp_timer_get_time() - ultrasonic_echo_start_us;
        float dist = (float)fly_time_us / 1e6 * ULTRASONIC_GROUND_LEVEL_SOUND_VEL / 2.0;

        xQueueSendFromISR(ultrasonic_distances, (void *)&dist, NULL);

        ultrasonic_echo_start_us = -1; // Sets this interrupt to handled
    }
}

void setup_ultrasonic()
{
    ultrasonic_echo_start_us = -1;
    ultrasonic_distances = xQueueCreate(ULTRASONIC_QUEUE_SIZE, sizeof(float));

    gpio_config_t trig_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(ULTRASONIC_TRIG_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&trig_gpio_cfg);

    gpio_config_t echo_gpio_cfg = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(ULTRASONIC_ECHO_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&echo_gpio_cfg);
    gpio_isr_handler_add(ULTRASONIC_ECHO_PIN, isr_ultrasonic_echo, NULL);
}

void trig_ultrasonic_scan()
{
    gpio_set_level(ULTRASONIC_TRIG_PIN, true);
    ets_delay_us(ULTRASONIC_TRIG_DURATION_US);
    gpio_set_level(ULTRASONIC_TRIG_PIN, false);
}

float read_ultrasonic_distance()
{
    trig_ultrasonic_scan();

    float dist = -1;
    xQueueReceive(ultrasonic_distances, &dist, pdMS_TO_TICKS(ULTRASONIC_ECHO_TIMEOUT_MS));
    return dist;
}