#ifndef SERVO_H
#define SERVO_H

#define SERVO_LOG_TAG_NAME "act-servo"

#define SERVO_I2C_ADDR 0x40

#define SERVO_MODE1_REG 0x00
#define SERVO_MODE2_REG 0x01

#define SERVO_PRESCALE_REG 0xFE

#define SERVO_LED0_ON_L_REG 0x06

#define SERVO_INT_CLK_FREQ 25e6

#include "stdint.h"

void setup_servo_driver(uint16_t freq);
void set_servo_channel_pulse(uint8_t chnl, uint16_t on_count, uint16_t off_count);
void set_servo_channel_pulse_width(uint8_t chnl, uint16_t servo_freq, double up_ms);



#endif // SERVO_H