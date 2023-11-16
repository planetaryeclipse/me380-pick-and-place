#ifndef PINOUTS_H
#define PINOUTS_H

// Generates the bitmask from the pin number
#define PIN_BITMASK(pin_num) (1ULL << pin_num)

// I2C communications
#define I2C_MASTER_SDA_PIN 21
#define I2C_MASTER_SCL_PIN 22

// Status LED on development board
#define LED_STATUS_PIN 2

// Power pin from external 3v3 supply
#define POWER_MNTR_PIN 15

// Ultrasonic sensor - HC-SR04
#define ULTRASONIC_TRIG_PIN 19
#define ULTRASONIC_ECHO_PIN 18

// Limit switches
#define HOMING_LIM_SWCH_PIN 17
#define ARM1_INNER_LIM_SWCH_PIN 23
#define ARM1_OUTER_LIM_SWCH_PIN 39
#define ARM2_INNER_LIM_SWCH_PIN 34
#define ARM2_OUTER_LIM_SWCH_PIN 35
#define ARM3_INNER_LIM_SWCH_PIN 16
#define ARM3_OUTER_LIM_SWCH_PIN 4

// Servo driver - PCA9685
#define SERVO_OUTPUT_ENABLED_PIN 5

// Stepper motor driver - A4988
#define STEPPER_DIRECTION_PIN 26
#define STEPPER_STEP_PIN  25
#define STEPPER_SLEEP_PIN 33
#define STEPPER_RESET_PIN 32
#define STEPPER_MS3_PIN 13
#define STEPPER_MS2_PIN 12
#define STEPPER_MS1_PIN 14
#define STEPPER_ENABLE_PIN 27

#endif // PINOUTS_H