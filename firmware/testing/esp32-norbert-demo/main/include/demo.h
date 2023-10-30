#ifndef DEMO_H
#define DEMO_H

#include "drivers/stepper_tb6600.h"

typedef struct {
    stepper_tb6600_t* stepper;
} demo_rotation_params_t;

void demo_rotation_task(void *args);



void demo_lifting_task(void *args);

#endif