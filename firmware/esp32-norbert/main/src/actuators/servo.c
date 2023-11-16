#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "math.h"

#include "comms.h"
#include "pinouts.h"
#include "actuators/servo.h"

#include "esp_log.h"

void setup_servo_driver(uint16_t freq)
{
    // Disables output on the servo motor until desired to prevent weird glitches
    gpio_config_t output_enabled_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_OUTPUT_ENABLED_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&output_enabled_gpio_cfg);
    gpio_set_level(STEPPER_OUTPUT_ENABLED_PIN, true); // Active low

    // Enables sleep mode to change the prescale
    uint8_t mode1_write_buf[] = {
        SERVO_MODE1_REG,
        MAKE_BYTE(
            I2C_REG_BIT_CLEAR, // disables restart logic
            I2C_REG_BIT_CLEAR, // uses the internal clock
            I2C_REG_BIT_SET,   // uses register auto-increment
            I2C_REG_BIT_SET,   // sets to sleep mode (to change prescaler)
            I2C_REG_BIT_CLEAR, // does not respond to I2C subaddress 1
            I2C_REG_BIT_CLEAR, // does not respond to I2C subaddress 2
            I2C_REG_BIT_CLEAR, // does not respond to I2C subaddress 3,
            I2C_REG_BIT_CLEAR  // does not respond to I2C all call address
            )};
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, mode1_write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    // Changes the total period for a full PWM cycle (essentially remaps the 4096 counter
    // region onto a small time duration set by the frequency)
    uint8_t prescale_write_buf[] = {
        SERVO_PRESCALE_REG,
        round((double)SERVO_INT_CLK_FREQ / (4096.0 * freq)) - 1};
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, &prescale_write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    // Disables sleep mode to be able to generate outputs
    mode1_write_buf[1] ^= I2C_REG_BIT_SET << 4;
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, mode1_write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    vTaskDelay(pdMS_TO_TICKS(1)); // Requires 500 us for the oscillator to startup
}

void enable_servo_driver(){
    gpio_set_level(STEPPER_OUTPUT_ENABLED_PIN, false); // Active low
}

#define ARM_SERVO_1_BIAS 0
#define ARM_SERVO_2_BIAS 0
#define ARM_SERVO_3_BIAS 0

void set_servo_channel_pulse(uint8_t chnl, uint16_t on_count, uint16_t off_count)
{
    // Accepts the first 12 bits for the on and off counts and thus only the first 4
    // bits are taken of the high byte.

    uint8_t servo_pulse_cfg_write_buf[] = {
        SERVO_LED0_ON_L_REG + 4 * chnl, // Determines starting register dynamically
        on_count & 0xFF,                // off low byte
        (on_count & 0xF00) >> 8,        // off high byte
        off_count & 0xFF,               // on low byte
        (off_count & 0xF00) >> 8,       // on high byte
    };
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, &servo_pulse_cfg_write_buf, 5, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

void set_servo_channel_pulse_width(uint8_t chnl, uint16_t servo_freq, double up_ms)
{
    // Subtracts 1 as the counts are on a range of 0-4095 (total of 4096)
    uint16_t off_counts = (up_ms / (1.0 / (double)servo_freq * 1e3)) * 4096 - 1;
    set_servo_channel_pulse(chnl, 0, off_counts);
}