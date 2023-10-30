#ifndef STEPPER_TB6600_H
#define STEPPER_TB6600_H

#include "stdbool.h"
#include "stdint.h"

/*
The interface provided below allows interfacing with a TB6600 stepper motor
driver and provides common methods for 
*/

typedef struct {
    uint16_t pulse_duration_us;
    uint8_t dir_gpio, pul_gpio;

    uint8_t ustep_divider;
    float nondivided_step_ang;
    
    bool reverse_ccw_dir;
} stepper_tb6600_cfg_t;

typedef struct {
    stepper_tb6600_cfg_t* cfg;
    
    float curr_ang;
    uint32_t curr_steps;
} stepper_tb6600_t;

bool cfg_tb6600(stepper_tb6600_cfg_t* cfg, stepper_tb6600_t* stepper);
bool step_tb6600(stepper_tb6600_t* stepper, int32_t steps, bool* stop_early);

uint32_t steps_for_angle_tb6600(stepper_tb6600_cfg_t* cfg, float ang);

#endif