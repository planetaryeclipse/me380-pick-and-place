#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

#include "test_rtos_tasks.h"

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

inline uint32_t pwm_duty_cycle_duration_from_angle(int angle){
    // computes the duty cycle duration from the angle (linear interpolation)
    return (angle - SERVO_MIN_ANG) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_ANG - SERVO_MIN_ANG) + SERVO_MIN_PULSEWIDTH_US;
}

void servo_test_task(void *arg){
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