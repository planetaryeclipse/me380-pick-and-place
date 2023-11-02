#ifndef PINOUTS_H
#define PINOUTS_H

#define PIN_BITMASK(pin_num) (1ULL << pin_num)

#define LED_STATUS_PIN 2

#define I2C_MASTER_SDA_PIN 21
#define I2C_MASTER_SCL_PIN 22
#define I2C_MASTER_FREQ_HZ 100000

#define PCA9685_OUTPUT_ENABLED_PIN 5

#endif