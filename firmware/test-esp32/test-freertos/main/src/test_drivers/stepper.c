#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "math.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "rom/ets_sys.h"

#include "test_rtos_tasks.h"


#define STEPPER_PULSE_DURATION_US 30
#define STEPPER_STEP_ANG_DEG 1.8
#define STEPPER_USTEP_DIVIDER 8

#define STEPPER_REVERSE_CCW_DIR 1

// If high then allows free rotation (unused as of now as not required)
// #define STEPPER_EN_GPIO 12

#define STEPPER_DIR_GPIO 14
#define STEPPER_PUL_GPIO 27

#define TEST_STEPPER_ANG_ADVANCE 90
#define TEST_STEPPER_NUM_STEPS_BEFORE_REVERSE 100


static inline void send_pulse(gpio_num_t gpio_num){
    gpio_set_level(gpio_num, true);
    ets_delay_us(STEPPER_PULSE_DURATION_US);
    gpio_set_level(gpio_num, false);
    ets_delay_us(STEPPER_PULSE_DURATION_US);
}

void stepper_test_task(void *args){

    gpio_config_t stepper_pulse = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << STEPPER_PUL_GPIO,
        .pull_down_en = true,
        .pull_up_en = false,
    };
    gpio_config(&stepper_pulse);

    gpio_config_t stepper_direction = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << STEPPER_DIR_GPIO,
        .pull_down_en = true,
        .pull_up_en = false,
    };
    gpio_config(&stepper_direction);

    bool ccw = false;

    int tot_steps = 0;
    double curr_angle = 0;

    while(1){

        int steps_to_take = (float)TEST_STEPPER_ANG_ADVANCE / STEPPER_STEP_ANG_DEG * STEPPER_USTEP_DIVIDER;
#if STEPPER_REVERSE_CCW_DIR
        steps_to_take *= ccw ? 1 : -1;
#else
        steps_to_take *= ccw ? -1 : 1;
#endif
        tot_steps += steps_to_take;

        if (ccw){
            ESP_LOGI("test-stepper", "Advancing stepper CW by %i steps", steps_to_take);
            gpio_set_level(STEPPER_DIR_GPIO, false);
        }
        else{
            ESP_LOGI("test-stepper", "Advancing stepper CCW by %i steps", steps_to_take);
            gpio_set_level(STEPPER_DIR_GPIO, true);
        }
        for(int i = 0; i > steps_to_take; i--)
                send_pulse(STEPPER_PUL_GPIO);

        curr_angle = fmod(tot_steps * ((float)STEPPER_STEP_ANG_DEG / STEPPER_USTEP_DIVIDER) + 360.0, 360.0);
        ESP_LOGI("test-stepper", "Stepper is at angle of %.03f degrees (%i total steps)", curr_angle, tot_steps);

        vTaskDelay(1000);
    }
}