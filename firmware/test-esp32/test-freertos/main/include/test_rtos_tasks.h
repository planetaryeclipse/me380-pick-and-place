#ifndef TEST_RTOS_TASKS_H
#define TEST_RTOS_TASKS_H

void blink_task(void *arg);
void encoder_read_task(void *arg);
void servo_test_task(void *arg);
void stepper_test_task(void *args);

#endif