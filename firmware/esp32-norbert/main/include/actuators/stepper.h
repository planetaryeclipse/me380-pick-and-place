#ifndef STEPPER_H
#define STEPPER_H

#define STEPPER_LOG_TAG_NAME "act-stepper"

#define STEPPER_HIGH_PULSE_WIDTH_MS 7
#define STEPPER_LOW_PULSE_WIDTH_MS 7

#include "stdint.h"

typedef enum microstep_resol_t
{
    FULL_STEP,
    HALF_STEP,
    QUARTER_STEP,
    EIGHTH_STEP,
    SIXTEENTH_STEP
} microstep_resol_t;

void setup_stepper(microstep_resol_t resol);

void rotate_stepper(uint16_t steps, bool fw_dir);

#endif // STEPPER_H