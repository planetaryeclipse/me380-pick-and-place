#ifndef AUTOHOME_H
#define AUTOHOME_H

#include "drivers/stepper_tb6600.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    stepper_tb6600_t* stepper;
    TickType_t task_exec_ms;
} autohome_params_t;

void autohome_task(void* args);

#endif