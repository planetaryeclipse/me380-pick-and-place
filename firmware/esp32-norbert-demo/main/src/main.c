#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pinouts.h"
#include "drivers/stepper_tb6600.h"

#include "drivers/stepper_tb6600.h"

#include "demo.h"

/*
Configuration variables are defined here so they do not deallocate
due to falling out of scope if they were defined in the main function
*/

stepper_tb6600_cfg_t stepper_cfg = {
        .pulse_duration_us = 30,
        .dir_gpio = MAIN_STEPPER_DIR_GPIO,
        .pul_gpio = MAIN_STEPPER_PUL_GPIO,
        .ustep_divider = 8,
        .nondivided_step_ang = 1.8,
        .reverse_ccw_dir = false};
stepper_tb6600_t stepper;

demo_rotation_params_t demo_rotation_params = {
        .stepper = &stepper};

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100)); // Board setup

    // Performs configuration of stepper motor
    cfg_tb6600(&stepper_cfg, &stepper);

    // Initializes tasks
    xTaskCreate(demo_rotation_task, "RotationDemo", 4096, (void *)&demo_rotation_params, 0, NULL);
}