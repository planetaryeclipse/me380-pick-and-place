#include "drivers/stepper_tb6600.h"

#include "stdbool.h"
#include "stdlib.h"
#include "math.h"

#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"

static const char *log_tag = "stepper-tb6600";

static inline void send_pulse(gpio_num_t gpio_num, uint32_t us_delay)
{
    // Sends a pulse with specified high and low signal delay
    gpio_set_level(gpio_num, true);
    ets_delay_us(us_delay);
    gpio_set_level(gpio_num, false);
    ets_delay_us(us_delay);
}

bool cfg_tb6600(stepper_tb6600_cfg_t *cfg, stepper_tb6600_t *stepper)
{
    if (cfg == NULL)
    {
        ESP_LOGE(log_tag, "Stepper configuration cannot be NULL!");
        return false;
    } else if (stepper == NULL){
        ESP_LOGE(log_tag, "Stepper cannot be NULL!");
        return false;
    }

    // Sets up the appropriate GPIO ports for the stepper driver

    gpio_config_t stepper_pulse = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << cfg->pul_gpio,
        .pull_down_en = true,
        .pull_up_en = false,
    };
    gpio_config(&stepper_pulse);

    gpio_config_t stepper_direction = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << cfg->dir_gpio,
        .pull_down_en = true,
        .pull_up_en = false,
    };
    gpio_config(&stepper_direction);

    // Sets up the stepper definition
    stepper->cfg = cfg;
    stepper->curr_ang = 0;
    stepper->curr_steps = 0;

    return true;
}

bool step_tb6600(stepper_tb6600_t *stepper, int32_t ccw_steps, bool *stop_early)
{
    if(stepper == NULL){
        ESP_LOGE(log_tag, "Stepper cannot be NULL");
        return false;
    }
    
    gpio_set_level(stepper->cfg->dir_gpio, ccw_steps >= 0);
    for(int i = 0; i < abs(ccw_steps); i++){
        if (*stop_early){ // Access of boolean is atomic
            stepper->curr_steps += i * (ccw_steps >= 0 ? 1 : -1);
            ESP_LOGW(log_tag, "Stepper terminated after %i out of %i requested steps", i, abs(ccw_steps));
            break;
        }
        send_pulse(stepper->cfg->pul_gpio, stepper->cfg->pulse_duration_us);
    }

    stepper->curr_steps += ccw_steps;
    stepper->curr_ang = fmod(stepper->curr_steps * (stepper->cfg->nondivided_step_ang / stepper->cfg->ustep_divider) + 360.0, 360.0);

    return true;
}

uint32_t steps_for_angle_tb6600(stepper_tb6600_cfg_t* cfg, float ang){
    return ang / cfg->nondivided_step_ang * cfg->ustep_divider;
}