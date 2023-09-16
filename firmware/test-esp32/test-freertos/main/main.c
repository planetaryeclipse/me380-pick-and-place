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

#include "driver/mcpwm_prelude.h"

#include "driver/gpio.h"

#include "driver/pulse_cnt.h"

// #include "esp_intr_alloc.h"

#define LED_GPIO 23

static void blink_task(void *arg){
    gpio_config_t led_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LED_GPIO,
        .pull_down_en = false,
        .pull_up_en = false,
    };

    gpio_config(&led_cfg);
    
    bool enabled = true;
    
    while(1){
        ESP_LOGI("test-freertos", "Toggling blink!");
        gpio_set_level(LED_GPIO, enabled);
        vTaskDelay(pdMS_TO_TICKS(1000));

        enabled = !enabled;
    }
}

#define ENCODER_CH_A_PULLED_UP 1 // if 1 starts as high, goes low on clock pulse
#define ENCODER_CH_B_PULLED_UP 0 // if 1 starts as high, goes low on clock pulse

#define ENCODER_CCW_A_LEADS_B 1 // if ch A leads ch B then is moving CCW

#define ENCODER_CPS 1024 // cycle pulses per rotation

#define PCNT_HIGH_LIMIT 1024*4
#define PCNT_LOW_LIMIT -1024*4

#define ENCODER_CH_A_GPIO 34
#define ENCODER_CH_B_GPIO 35

static void encoder_read_task(void *arg){
    // This basically places a limit on the number of pulse counters we can use in our design
    ESP_LOGI("encoder-test", "Maximum number of pulse counter (PCNT) units: %i", SOC_PCNT_UNITS_PER_GROUP);

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_new_unit(&unit_config, &pcnt_unit);

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_CH_A_GPIO,
        .level_gpio_num = ENCODER_CH_B_GPIO,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_CH_B_GPIO,
        .level_gpio_num = ENCODER_CH_A_GPIO,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);

#if ENCODER_CH_A_PULLED_UP
    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
#else
    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
#endif

#if ENCODER_CH_B_PULLED_UP
    pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
#else
    pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
#endif

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);

    int pulse_count = 0;
    float encoder_ang = 0;

    while(1){
        pcnt_unit_get_count(pcnt_unit, &pulse_count);
        encoder_ang = (float)pulse_count / 4 / ENCODER_CPS * 360.0;

        ESP_LOGI("encoder-test", "Encoder count: %i, angle: %f", pulse_count, encoder_ang);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// NOTE: most servos accept duty cycle durations from 1 ms to 2 ms where 1 ms
// represents the minimum position/speed and 2 ms is the maximum position/speed

// for these servos it appears that 1000 - 2500 gives use the full 120 degrees
// as presented in the data sheet (this is for the MG995s)

// for the S51s it appears that the range 600 to 2500 gives use the full 180
// degrees as described in the datasheet

// WARNING: make sure you configure the correct pulse width before attempting
// to connect the GPIOs, you can potentially damage some of the servos

#define SERVO_MIN_PULSEWIDTH_US 600 // minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500 // maximum pulse width in microseconds

#define SERVO_MIN_ANG -60 // minmimum angle of the servos
#define SERVO_MAX_ANG 60 // maximum angle of the servos

#define SERVO_ANG_STEP_1 15
#define SERVO_ANG_STEP_2 60
#define SERVO_ANG_STEP_DELAY_MS 1000

#define SERVO_PULSE_GPIO_1 18 // generator_1 will output PWM waveform on this pin
#define SERVO_PULSE_GPIO_2 19 // generator_2 will output PWM waveform on this pin

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // configures the timer at 1 MHz
#define SERVO_TIMEBASE_PERIOD 20000 // this equates to a PWM waveform with pulse width of 20 ms total

static inline uint32_t pwm_duty_cycle_duration_from_angle(int angle){
    // computes the duty cycle duration from the angle (linear interpolation)
    return (angle - SERVO_MIN_ANG) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_ANG - SERVO_MIN_ANG) + SERVO_MIN_PULSEWIDTH_US;
}

static void servo_test_task(void *arg){
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ, // handles clock prescaling internally
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    mcpwm_new_operator(&operator_config, &oper);
    mcpwm_operator_connect_timer(oper, timer);
    
    // Sets up the comparators, the specific values that these comparators test
    // for are the configured counter values needed to achieve the PWM duty
    // cycle time before switching the PWM signals to low.

    mcpwm_cmpr_handle_t comparator_1 = NULL;
    mcpwm_cmpr_handle_t comparator_2 = NULL;

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(oper, &comparator_config, &comparator_1);
    mcpwm_new_comparator(oper, &comparator_config, &comparator_2);

    // Sets the servos to the initial position
    mcpwm_comparator_set_compare_value(comparator_1, pwm_duty_cycle_duration_from_angle(0));
    mcpwm_comparator_set_compare_value(comparator_2, pwm_duty_cycle_duration_from_angle(0));

    // Sets up the generators. These are ultimately responsible for generating
    // the PWM waveform. Basically they start at high, given the action to set
    // the pwm signal as high when the timer event is triggered. This exists
    // until the comparator reaches its set value and the event sets the signal
    // to low unti lthe timer resets again (at the end of the 20 ms window)

    mcpwm_gen_handle_t generator_1 = NULL;
    mcpwm_gen_handle_t generator_2 = NULL;

    mcpwm_generator_config_t generator_1_config = { .gen_gpio_num = SERVO_PULSE_GPIO_1 };
    mcpwm_generator_config_t generator_2_config = { .gen_gpio_num = SERVO_PULSE_GPIO_2 };

    mcpwm_new_generator(oper, &generator_1_config, &generator_1);
    mcpwm_new_generator(oper, &generator_2_config, &generator_2);

    mcpwm_generator_set_action_on_timer_event(generator_1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator_1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_1, MCPWM_GEN_ACTION_LOW));

    mcpwm_generator_set_action_on_timer_event(generator_2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator_2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_2, MCPWM_GEN_ACTION_LOW));

    // Now that everything has been configured to be able to generate the PWM
    // waveforms, the timer is enabled and the PWM generation begins

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);  
    
    int servo1_ang = 0;
    bool servo1_up = true;

    int servo2_ang = 0;
    bool servo2_up = false;

    while (1) {
        mcpwm_comparator_set_compare_value(comparator_1, pwm_duty_cycle_duration_from_angle(servo1_ang));
        mcpwm_comparator_set_compare_value(comparator_2, pwm_duty_cycle_duration_from_angle(servo2_ang));

        servo1_ang += SERVO_ANG_STEP_1 * (servo1_up ? 1 : -1);
        servo2_ang += SERVO_ANG_STEP_2 * (servo2_up ? 1 : -1);

        if (servo1_ang >= SERVO_MAX_ANG || servo1_ang <= SERVO_MIN_ANG)
            servo1_up = !servo1_up;
        if (servo2_ang >= SERVO_MAX_ANG || servo2_ang <= SERVO_MIN_ANG)
            servo2_up = !servo2_up;

        ESP_LOGI("servo-test", "Updating servo positions!");

        vTaskDelay(pdMS_TO_TICKS(SERVO_ANG_STEP_DELAY_MS));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    
    xTaskCreate(blink_task, "blink", 4096, NULL, 0, NULL);
    xTaskCreate(servo_test_task, "servo_test", 4096, NULL, 0, NULL);
    xTaskCreate(encoder_read_task, "encoder", 4096, NULL, 0, NULL);

    //Create and start stats task
    // xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, STATS_TASK_PRIO, NULL, tskNO_AFFINITY);
}
