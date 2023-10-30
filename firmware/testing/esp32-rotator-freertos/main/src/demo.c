#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

#include "demo.h"

#define SERVO_0_OFFSET 5
#define SERVO_1_OFFSET 10
#define SERVO_2_OFFSET 0

const char * lifting_log_name = "demo-lifting";
const char * rotating_log_name = "rotating-lifting";

inline static uint32_t pwm_duty_cycle_duration_from_angle(int angle){
    return (angle - SERVO_MIN_ANG) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_ANG - SERVO_MIN_ANG) + SERVO_MIN_PULSEWIDTH_US;
}

void demo_lifting_task(void *arg){
    mcpwm_timer_handle_t timer_0 = NULL;
    mcpwm_timer_config_t timer_0_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    mcpwm_new_timer(&timer_0_config, &timer_0);

    mcpwm_timer_handle_t timer_1 = NULL;
    mcpwm_timer_config_t timer_1_config = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    mcpwm_new_timer(&timer_1_config, &timer_1);

    // Sets up the maximum number of 2 operators (max 2 servos per operator)

    mcpwm_oper_handle_t servo_oper_0 = NULL;
    mcpwm_operator_config_t servo_oper_0_cfg = {
        .group_id = 0
    };
    mcpwm_new_operator(&servo_oper_0_cfg, &servo_oper_0);
    mcpwm_operator_connect_timer(servo_oper_0, timer_0);

    mcpwm_oper_handle_t servo_oper_1 = NULL;
    mcpwm_operator_config_t servo_oper_1_cfg = {
        .group_id = 1
    };
    mcpwm_new_operator(&servo_oper_1_cfg, &servo_oper_1);
    mcpwm_operator_connect_timer(servo_oper_1, timer_1);

    // Sets up the comparator

    mcpwm_cmpr_handle_t comparator_0 = NULL;
    mcpwm_cmpr_handle_t comparator_1 = NULL;
    mcpwm_cmpr_handle_t comparator_2 = NULL;

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(servo_oper_0, &comparator_config, &comparator_0);
    mcpwm_new_comparator(servo_oper_0, &comparator_config, &comparator_1);
    mcpwm_new_comparator(servo_oper_1, &comparator_config, &comparator_2);

    // mcpwm_comparator_set_compare_value(comparator_0, pwm_duty_cycle_duration_from_angle(0));
    // mcpwm_comparator_set_compare_value(comparator_1, pwm_duty_cycle_duration_from_angle(0));
    // mcpwm_comparator_set_compare_value(comparator_2, pwm_duty_cycle_duration_from_angle(0));

    mcpwm_gen_handle_t generator_0 = NULL;
    mcpwm_gen_handle_t generator_1 = NULL;
    mcpwm_gen_handle_t generator_2 = NULL;

    mcpwm_generator_config_t generator_0_config = { .gen_gpio_num = SERVO_PULSE_GPIO_0 };
    mcpwm_generator_config_t generator_1_config = { .gen_gpio_num = SERVO_PULSE_GPIO_1 };
    mcpwm_generator_config_t generator_2_config = { .gen_gpio_num = SERVO_PULSE_GPIO_2 };

    mcpwm_new_generator(servo_oper_0, &generator_0_config, &generator_0);
    mcpwm_new_generator(servo_oper_0, &generator_1_config, &generator_1);
    mcpwm_new_generator(servo_oper_1, &generator_2_config, &generator_2);

    mcpwm_generator_set_action_on_timer_event(generator_0, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator_0, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_0, MCPWM_GEN_ACTION_LOW));

    mcpwm_generator_set_action_on_timer_event(generator_1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator_1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_1, MCPWM_GEN_ACTION_LOW));

    mcpwm_generator_set_action_on_timer_event(generator_2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator_2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_2, MCPWM_GEN_ACTION_LOW));

    // Timer 0
    mcpwm_timer_enable(timer_0);
    mcpwm_timer_start_stop(timer_0, MCPWM_TIMER_START_NO_STOP); 

    mcpwm_timer_enable(timer_1);
    mcpwm_timer_start_stop(timer_1, MCPWM_TIMER_START_NO_STOP);
    
    bool down_position = false;

    int high_pos = -40;
    int low_pos = high_pos + (80-15);

    while (1) {
        
        float ang = down_position ? low_pos : low_pos - 85;
        

        // ang = low_pos; // enable when testing out/configuring the offsets to allow attachment of servo horns

        // ang = low_pos; // Sets at a test value

        ESP_LOGI(lifting_log_name, "Entered into %s position", down_position ? "down" : "up");

        mcpwm_comparator_set_compare_value(comparator_0, pwm_duty_cycle_duration_from_angle(ang+SERVO_0_OFFSET));
        mcpwm_comparator_set_compare_value(comparator_1, pwm_duty_cycle_duration_from_angle(ang+SERVO_1_OFFSET));
        mcpwm_comparator_set_compare_value(comparator_2, pwm_duty_cycle_duration_from_angle(ang+SERVO_2_OFFSET));

        down_position = !down_position;

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}