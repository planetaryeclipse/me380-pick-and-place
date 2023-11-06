#include "driver/i2c.h"

#include "comms.h"
#include "sensors/color.h"

void setup_color_sensor()
{
    uint8_t write_buf[] = {
        COLOR_ENABLE,
        MAKE_BYTE(
            I2C_REG_BIT_CLEAR, // reserved, set as 0
            I2C_REG_BIT_CLEAR, // keep gestures off
            I2C_REG_BIT_CLEAR, // keep proximity interrupt off
            I2C_REG_BIT_CLEAR, // keep ALS interrupt off
            I2C_REG_BIT_CLEAR, // disable wait timer
            I2C_REG_BIT_CLEAR, // disable proximity detection
            I2C_REG_BIT_SET,   // enable the ALS
            I2C_REG_BIT_SET    // power on the device
            )};
    i2c_master_write_to_device(I2C_MASTER_NUM, COLOR_I2C_ADDR, write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = COLOR_CONTROL;
    write_buf[1] = MAKE_BYTE(
        I2C_REG_BIT_CLEAR, // Sets LED drive strength at minimum
        I2C_REG_BIT_CLEAR,
        I2C_REG_BIT_CLEAR, // reserved
        I2C_REG_BIT_CLEAR, // reserved
        I2C_REG_BIT_CLEAR, // Sets proximity gain control at minimum
        I2C_REG_BIT_CLEAR,
        I2C_REG_BIT_SET, // Sets ALS and color gain control at maximum of 64x
        I2C_REG_BIT_SET);
    i2c_master_write_to_device(I2C_MASTER_NUM, COLOR_I2C_ADDR, write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

void read_color_sensor(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b)
{
    uint8_t als_readings[8];
    uint8_t start_read_reg_addr = COLOR_CDATAL;

    // Reads all the values sequentially starting at CDATAL given the color registers are placed
    // sequentially in the memory of the APDS-9960. Address to read from must be written first.
    i2c_master_write_read_device(I2C_MASTER_NUM, COLOR_I2C_ADDR, &start_read_reg_addr, 1, als_readings, 8, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    // Reconstructs the sensor readings from the high and low bytes
    *c = (als_readings[1] << 8) | als_readings[0];
    *r = (als_readings[3] << 8) | als_readings[2];
    *g = (als_readings[5] << 8) | als_readings[4];
    *b = (als_readings[7] << 8) | als_readings[6];
}