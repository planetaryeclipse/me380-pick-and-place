#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "pinouts.h"
#include "sensors/ultrasonic.h"

#define ULTRASONIC_LOG_TAG_NAME "sensor-ultrasonic"

#define ULTRASONIC_TRIG_DURATION_US 10
#define ULTRASONIC_DIST_TIMER_MAX 100000
#define ULTRASONIC_GROUND_LEVEL_SOUND_VEL 336.1

#define ULTRASONIC_QUEUE_SIZE 10
#define ULTRASONIC_ECHO_TIMEOUT_MS 20

static bool echo_up;
static int64_t ultrasonic_echo_start_us;
static QueueHandle_t ultrasonic_distances;

void isr_ultrasonic_echo(void *arg)
{
    if (gpio_get_level(ULTRASONIC_ECHO_PIN))
    {
        echo_up = true;
        ultrasonic_echo_start_us = esp_timer_get_time();
    }
    else
    {
        if (echo_up)
        {
            int64_t echo_duration_us = esp_timer_get_time() - ultrasonic_echo_start_us;
            float dist = (float)echo_duration_us / 1e6 * (float)ULTRASONIC_GROUND_LEVEL_SOUND_VEL / 2.0;

            xQueueSendFromISR(ultrasonic_distances, &dist, NULL);
        }
        echo_up = false;
    }
}

void setup_ultrasonic()
{
    echo_up = false;
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
    ESP_LOGI(ULTRASONIC_LOG_TAG_NAME, "Sending trigger pulse to ultrasonic sensor");
    gpio_set_level(ULTRASONIC_TRIG_PIN, true);
    ets_delay_us(ULTRASONIC_TRIG_DURATION_US);
    gpio_set_level(ULTRASONIC_TRIG_PIN, false);
}

bool read_ultrasonic_distance(float *dist)
{
    trig_ultrasonic_scan();

    bool success_recv = false;
    while (xQueueReceive(ultrasonic_distances, dist, pdMS_TO_TICKS(ULTRASONIC_ECHO_TIMEOUT_MS)))
        success_recv = true;

    return success_recv;
}