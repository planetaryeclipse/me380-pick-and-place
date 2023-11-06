#ifndef COLOR_H
#define COLOR_H

/**
 * @file color.h
 * @brief Implements the APDS-9960 color sensor.
 * 
 * THis contains the necessary functions to implement and read from the the I2C-compatible
 * APDS-9960 color sensor. The configuration and gains have been hard coded due to a need to
 * implement the minimal use case.
 *  
 * @author Samuel Street
 */

#define COLOR_LOG_TAG_NAME "sensor-color"

#define COLOR_I2C_ADDR 0x39

#define COLOR_ENABLE 0x80  // Power ON
#define COLOR_CONTROL 0x8f // ALS Gain Control
#define COLOR_ATIME 0x82   // ALS ADC Integration Time
#define COLOR_CDATAL 0x94  // Clear Data, Low Byte
#define COLOR_CDATAH 0x95  // Clear Data, High Byte
#define COLOR_RDATAL 0x96  // Red Data, Low Byte
#define COLOR_RDATAH 0x97  // Red Data, High Byte
#define COLOR_GDATAL 0x98  // Green Data, Low Byte
#define COLOR_GDATAH 0x99  // Green Data, High Byte
#define COLOR_BDATAL 0x9a  // Blue Data, Low Byte
#define COLOR_BDATAH 0x9b  // Blue Data, High Byte

#include "stdint.h"

/**
 * @brief Sets up the APDS-9960 color/gesture sensor to read colors
 *
 * Configures the appropriate I2C-addressable registers within the color sensor to produce RGBC
 * data. The color and ambient-light sensor (ALS) is started through setting of specific bits in
 * the register at COLOR_ENABLE. Signals from the photodiode arrary accumulate for a time
 * specified in the register at COLOR_ATIME while the gains are set within the register at
 * COLOR_CONTROL. It is advisable to avoid modifying these configuration registers if possible.
 * The clear data (light level) can be retrieved from CDATA registers. RGB data can be retrieved
 * from the appropriate registers RDATA, GDATA, and BDATA, respectively.
 */
void setup_color_sensor();

/**
 * @brief Reads the current sensor values from the APDS-9960
 *
 * @param c Clear color reading (ambient light)
 * @param r Red color reading
 * @param g Green color reading
 * @param b Blue color reading
 *
 * Reads from the color registers on the APDS-9960 through I2C and places the readings in the
 * appropriate variables pointed to by the function inputs.
 */
void read_color_sensor(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b);

#endif // COLOR_H