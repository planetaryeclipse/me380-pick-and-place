#ifndef LIMIT_SWTCH_H
#define LIMIT_SWTCH_H

#define LIMIT_SWTCH_TAG_NAME "sensor-limit-swtch"

typedef enum arm_limit_switch_loc
{
    ARM1_INNER,
    ARM1_OUTER,
    ARM2_INNER,
    ARM2_OUTER,
    ARM3_INNER,
    ARM3_OUTER
} arm_limit_switch_loc;

void isr_homing_limit_switch(void * arg);
void isr_arm_limit_switch(void * arg);

void setup_limit_switches();

#endif // LIMIT_SWTCH_H