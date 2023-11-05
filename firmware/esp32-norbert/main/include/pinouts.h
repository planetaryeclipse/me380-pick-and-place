#ifndef PINOUTS_H
#define PINOUTS_H

#define PIN_BITMASK(pin_num) (1ULL << pin_num)

#define LED_STATUS_PIN 2

#define ULTRASONIC_TRIG_PIN 19
#define ULTRASONIC_ECHO_PIN 18

#define HOMING_LIM_SWCH_PIN 17
#define ARM1_INNER_LIM_SWCH_PIN 36
#define ARM1_OUTER_LIM_SWCH_PIN 39
#define ARM2_INNER_LIM_SWCH_PIN 34
#define ARM2_OUTER_LIM_SWCH_PIN 35
#define ARM3_INNER_LIM_SWCH_PIN 16
#define ARM3_OUTER_LIM_SWCH_PIN 0

#define I2C_MASTER_SDA_PIN 21
#define I2C_MASTER_SCL_PIN 22
#define I2C_MASTER_FREQ_HZ 100000

#define STEPPER_DIRECTION_PIN 26
#define STEPPER_STEP_PIN  25
#define STEPPER_SLEEP_PIN 33
#define STEPPER_RESET_PIN 32
#define STEPPER_MS3_PIN 13
#define STEPPER_MS2_PIN 12
#define STEPPER_MS1_PIN 14
#define STEPPER_ENABLE_PIN 27

#define STEPPER_OUTPUT_ENABLED_PIN 5

#endif