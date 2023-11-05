#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "math.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "driver/uart.h"
#include "driver/i2c.h"

#include "rom/ets_sys.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// #include "driver/adc.h"

#include "pinouts.h"

#define BLINK_LOG_TAG "blink-task"
#define BLINK_DELAY_MS 1000

void blink_task(void *)
{
    gpio_config_t led_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(LED_STATUS_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&led_gpio_cfg);

    bool enabled = true;
    while (1)
    {
        // ESP_LOGI(BLINK_LOG_TAG, "Setting status LED to %s", enabled ? "ON" : "OFF");
        gpio_set_level(LED_STATUS_PIN, enabled);
        enabled = !enabled;

        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));
    }
}

#define PERIPHERAL_SETUP_TAG_NAME "peripheral-setup"

#define I2C_MASTER_NUM 0

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define I2C_MASTER_TIMEOUT_MS 1000

#define ESP_INTR_FLAG_DEFAULT 0

adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc2_cali_chan0_handle;

void setup_peripheral_comms()
{
    // Sets up I2C communication on port 0
    ESP_LOGI(PERIPHERAL_SETUP_TAG_NAME, "Setting up I2C communication");
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t i2c_master_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0};
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &i2c_master_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, i2c_master_cfg.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    // Setup individual ISRs for different GPIO pins
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));

    // Setup the ADC
    adc_oneshot_unit_init_cfg_t adc2_init_cfg = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_init_cfg, &adc2_handle));

    adc_oneshot_chan_cfg_t adc2_ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_0, &adc2_ch_cfg));

    adc2_cali_chan0_handle = NULL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_2,
        .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_cali_create_scheme_linefitting(&cali_cfg, &adc2_cali_chan0_handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_cfg, &adc2_cali_chan0_handle));
#endif

    // Sets measurable input voltage range from 150 mV to 2450 mV
    // FOR SOME REASON GPIO4 CORRESPONDS TO ADC_CHANNEL_0
    // int gpio_num = -1;
    // adc2_pad_get_io_num(ADC_CHANNEL_0, &gpio_num);
    // ESP_LOGI(PERIPHERAL_SETUP_TAG_NAME, "**** CORRESPONDING ADC GPIO: %i", gpio_num);

    // adc2_config_channel_atten(ADC_CHANNEL_0, ADC_ATTEN_DB_11);
}

#define REG_SET 1
#define REG_CLEAR 0

#define MAKE_BYTE(x7, x6, x5, x4, x3, x2, x1, x0) ((x7 << 7) | (x6 << 6) | (x5 << 5) | (x4 << 4) | (x3 << 3) | (x2 << 2) | (x1 << 1) | x0)

#define SENSOR_TEST_TAG_NAME "sensor-test"

// Color sensor - APDS-9960

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
void setup_color_sensor()
{
    uint8_t write_buf[] = {
        COLOR_ENABLE,
        MAKE_BYTE(
            REG_CLEAR, // reserved, set as 0
            REG_CLEAR, // keep gestures off
            REG_CLEAR, // keep proximity interrupt off
            REG_CLEAR, // keep ALS interrupt off
            REG_CLEAR, // disable wait timer
            REG_CLEAR, // disable proximity detection
            REG_SET,   // enable the ALS
            REG_SET    // power on the device
            )};
    i2c_master_write_to_device(I2C_MASTER_NUM, COLOR_I2C_ADDR, write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = COLOR_CONTROL;
    write_buf[1] = MAKE_BYTE(
        REG_CLEAR, // Sets LED drive strength at minimum
        REG_CLEAR,
        REG_CLEAR, // reserved
        REG_CLEAR, // reserved
        REG_CLEAR, // Sets proximity gain control at minimum
        REG_CLEAR,
        REG_SET, // Sets ALS and color gain control at maximum of 64x
        REG_SET);
    i2c_master_write_to_device(I2C_MASTER_NUM, COLOR_I2C_ADDR, write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

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

#define DEVICE_SAFTEY_TAG_NAME "device-safety"
#define AUTOHOMING_TAG_NAME "autohoming"

typedef enum arm_limit_switch_loc
{
    ARM1_INNER,
    ARM1_OUTER,
    ARM2_INNER,
    ARM2_OUTER,
    ARM3_INNER,
    ARM3_OUTER
} arm_limit_switch_loc;

void isr_homing_limit_switch(void *arg)
{
    ESP_EARLY_LOGI(AUTOHOMING_TAG_NAME, "Homing limit switch triggered");
}

void isr_arm_limit_switch(void *arg)
{
    arm_limit_switch_loc loc = (arm_limit_switch_loc)arg;
    switch (loc)
    {
    case ARM1_INNER:
    {
        ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM1_INNER limit switch triggered");
        break;
    }
    case ARM1_OUTER:
    {
        ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM1_OUTER limit switch triggered");
        break;
    }
    case ARM2_INNER:
    {
        ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM2_INNER limit switch triggered");
        break;
    }
    case ARM2_OUTER:
    {
        ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM2_OUTER limit switch triggered");
        break;
    }
    case ARM3_INNER:
    {
        ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM3_INNER limit switch triggered");
        break;
    }
    case ARM3_OUTER:
    {
        ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM3_OUTER limit switch triggered");
        break;
    }
    }
}

static inline void setup_arm_switch(uint8_t pin, arm_limit_switch_loc loc)
{
    gpio_config_t arm_lim_swch = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(pin),
        .pull_up_en = true,
        .pull_down_en = false};
    gpio_config(&arm_lim_swch);
    gpio_isr_handler_add(pin, isr_arm_limit_switch, (void *)loc);
}

void setup_limit_switches()
{
    gpio_config_t homing_lim_swch = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(HOMING_LIM_SWCH_PIN),
        .pull_up_en = true,
        .pull_down_en = false};
    gpio_config(&homing_lim_swch);
    gpio_isr_handler_add(HOMING_LIM_SWCH_PIN, isr_homing_limit_switch, NULL);

    setup_arm_switch(ARM1_INNER_LIM_SWCH_PIN, ARM1_INNER);
    setup_arm_switch(ARM1_OUTER_LIM_SWCH_PIN, ARM1_OUTER);

    setup_arm_switch(ARM2_INNER_LIM_SWCH_PIN, ARM2_INNER);
    setup_arm_switch(ARM2_OUTER_LIM_SWCH_PIN, ARM2_OUTER);

    setup_arm_switch(ARM3_INNER_LIM_SWCH_PIN, ARM3_INNER);
    setup_arm_switch(ARM3_OUTER_LIM_SWCH_PIN, ARM3_OUTER);
}

#define PICKUP_TAG_NAME "pickup"

#define ULTRASONIC_TRIG_DURATION_US 10
#define ULTRASONIC_DIST_TIMER_MAX 100000
#define GROUND_LEVEL_SOUND_VEL 336.1

#define ULTRASONIC_QUEUE_SIZE 10
#define ULTRASONIC_ECHO_TIMEOUT_MS 50

int64_t ultrasonic_echo_start_us;
QueueHandle_t ultrasonic_distances;

void isr_ultrasonic_echo(void *arg)
{
    // Due to decreased allowable stack sizes in ISRs there is no possible way to output
    // the distance value in this function. This is handled by the reading location.

    if (gpio_get_level(ULTRASONIC_ECHO_PIN))
    {
        // Record time at the start of the echo pulse
        ultrasonic_echo_start_us = esp_timer_get_time();
    }
    else
    {
        int64_t fly_time_us = esp_timer_get_time() - ultrasonic_echo_start_us;
        float dist = (float)fly_time_us / 1e6 * GROUND_LEVEL_SOUND_VEL / 2.0;

        xQueueSendFromISR(ultrasonic_distances, (void *)&dist, NULL);
    }
}

void setup_ultrasonic()
{
    gpio_config_t trig_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(ULTRASONIC_TRIG_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&trig_gpio_cfg);

    gpio_config_t echo_gpio_cfg = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(ULTRASONIC_ECHO_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&echo_gpio_cfg);
    gpio_isr_handler_add(ULTRASONIC_ECHO_PIN, isr_ultrasonic_echo, NULL);

    ultrasonic_distances = xQueueCreate(ULTRASONIC_QUEUE_SIZE, sizeof(float));
}

void trig_ultrasonic_scan()
{
    gpio_set_level(ULTRASONIC_TRIG_PIN, true);
    ets_delay_us(ULTRASONIC_TRIG_DURATION_US);
    gpio_set_level(ULTRASONIC_TRIG_PIN, false);
}

float read_distance()
{
    trig_ultrasonic_scan();

    float dist = -1;
    xQueueReceive(ultrasonic_distances, &dist, pdMS_TO_TICKS(ULTRASONIC_ECHO_TIMEOUT_MS));
    return dist;
}

void test_read_sensors_task(void *)
{
    // Setup sensors
    setup_color_sensor();
    setup_limit_switches();
    setup_ultrasonic();

    // Color sensor readings
    uint16_t c, r, g, b;

    float ultrasonic_dist;

    int adc_raw = 0;
    int voltage = 0;

    while (1)
    {
        // Periodically read sensors
        read_color_sensor(&c, &r, &g, &b);
        ESP_LOGI(SENSOR_TEST_TAG_NAME, "Color reading: c=%i r=%i g=%i b=%i", c, r, g, b);

        ultrasonic_dist = read_distance();
        ESP_LOGI(SENSOR_TEST_TAG_NAME, "Ultrasonic distance: %f", ultrasonic_dist);

        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_0, &adc_raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_chan0_handle, adc_raw, &voltage));
        ESP_LOGI(SENSOR_TEST_TAG_NAME, "ADC voltage reading: %i mV", voltage);

        // ESP_ERROR_CHECK(adc2_get_raw(ADC_CHANNEL_0, ADC_WIDTH_BIT_12, &adc_reading));
        // ESP_LOGI(SENSOR_TEST_TAG_NAME, "ADC reading: %i", adc_reading);

        /*
        Alternative polling code

        trig_ultrasonic_scan();
        while(xQueueReceive(ultrasonic_distances, &ultrasonic_dist, 0)){
            ESP_LOGI(SENSOR_TEST_TAG_NAME, "Ultrasonic distance: %f", ultrasonic_dist);
        }
        */

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Microstep driver

typedef enum microstep_resol_t
{
    FULL_STEP,
    HALF_STEP,
    QUARTER_STEP,
    EIGHTH_STEP,
    SIXTEENTH_STEP
} microstep_resol_t;

// Minimum pulse width for both high and low are 1 us
#define STEPPER_HIGH_PULSE_WIDTH_US 1000
#define STEPPER_LOW_PULSE_WIDTH_US 1000

void setup_stepper(microstep_resol_t resol)
{
    // Sets up all the GPIO pins necesary to communicate with the board
    gpio_config_t direction_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_DIRECTION_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&direction_gpio_cfg);

    gpio_config_t step_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_STEP_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&step_gpio_cfg);

    gpio_config_t sleep_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_SLEEP_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&sleep_gpio_cfg);

    gpio_config_t reset_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_RESET_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&reset_gpio_cfg);

    gpio_config_t ms3_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_MS3_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&ms3_gpio_cfg);

    gpio_config_t ms2_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_MS2_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&ms2_gpio_cfg);

    gpio_config_t ms1_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_MS1_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&ms1_gpio_cfg);

    gpio_config_t enable_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_ENABLE_PIN),
        .pull_down_en = true,
        .pull_up_en = false};
    gpio_config(&enable_gpio_cfg);

    // Sets the stepper pins to be ready to accept commands
    gpio_set_level(STEPPER_SLEEP_PIN, true);
    gpio_set_level(STEPPER_RESET_PIN, true);
    switch (resol)
    {
    case FULL_STEP:
    {
        gpio_set_level(STEPPER_MS1_PIN, false);
        gpio_set_level(STEPPER_MS2_PIN, false);
        gpio_set_level(STEPPER_MS3_PIN, false);
        break;
    }
    case HALF_STEP:
    {
        gpio_set_level(STEPPER_MS1_PIN, true);
        gpio_set_level(STEPPER_MS2_PIN, false);
        gpio_set_level(STEPPER_MS3_PIN, false);
        break;
    }
    case QUARTER_STEP:
    {
        gpio_set_level(STEPPER_MS1_PIN, false);
        gpio_set_level(STEPPER_MS2_PIN, true);
        gpio_set_level(STEPPER_MS3_PIN, false);
        break;
    }
    case EIGHTH_STEP:
    {
        gpio_set_level(STEPPER_MS1_PIN, true);
        gpio_set_level(STEPPER_MS2_PIN, true);
        gpio_set_level(STEPPER_MS3_PIN, false);
        break;
    }
    case SIXTEENTH_STEP:
    {
        gpio_set_level(STEPPER_MS1_PIN, true);
        gpio_set_level(STEPPER_MS2_PIN, true);
        gpio_set_level(STEPPER_MS3_PIN, true);
        break;
    }
    }
    gpio_set_level(STEPPER_ENABLE_PIN, false);
}

void rotate_stepper(uint16_t steps, bool fw_dir)
{
    gpio_set_level(STEPPER_DIRECTION_PIN, fw_dir);
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEPPER_STEP_PIN, true);
        ets_delay_us(STEPPER_HIGH_PULSE_WIDTH_US);
        gpio_set_level(STEPPER_STEP_PIN, false);
        ets_delay_us(STEPPER_LOW_PULSE_WIDTH_US);
    }
}

void test_stepper_task(void *)
{
    setup_stepper(FULL_STEP);

    while (1)
    {
        ESP_LOGI(SENSOR_TEST_TAG_NAME, "Sending steps!");
        rotate_stepper(200, true);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ACS723 Current sensor
// Device operation is passive but input is read through the ADC

// Servo driver board (PCA9685)

#define SERVO_I2C_ADDR 0x40

#define SERVO_MODE1_REG 0x00
#define SERVO_MODE2_REG 0x01

#define SERVO_PRESCALE_REG 0xFE

#define SERVO_LED0_ON_L_REG 0x06

#define SERVO_INT_CLK_FREQ 25e6

void setup_servo_driver(int freq)
{
    /*

     uint8_t write_buf[] = {
            COLOR_ENABLE,
            MAKE_BYTE(
                REG_CLEAR, // reserved, set as 0
                REG_CLEAR, // keep gestures off
                REG_CLEAR, // keep proximity interrupt off
                REG_CLEAR, // keep ALS interrupt off
                REG_CLEAR, // disable wait timer
                REG_CLEAR, // disable proximity detection
                REG_SET,   // enable the ALS
                REG_SET    // power on the device
                )};
        i2c_master_write_to_device(I2C_MASTER_NUM, COLOR_I2C_ADDR, write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

        write_buf[0] = COLOR_CONTROL;
        write_buf[1] = MAKE_BYTE(
            REG_CLEAR, // Sets LED drive strength at minimum
            REG_CLEAR,
            REG_CLEAR, // reserved
            REG_CLEAR, // reserved
            REG_CLEAR, // Sets proximity gain control at minimum
            REG_CLEAR,
            REG_SET, // Sets ALS and color gain control at maximum of 64x
            REG_SET);
        i2c_master_write_to_device(I2C_MASTER_NUM, COLOR_I2C_ADDR, write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    */

    // Enables sleep mode to change the prescale
    uint8_t mode1_write_buf[] = {
        SERVO_MODE1_REG,
        MAKE_BYTE(
            REG_CLEAR, // disables restart logic
            REG_CLEAR, // uses the internal clock
            REG_SET,   // uses register auto-increment
            REG_SET,   // sets to sleep mode (to change prescaler)
            REG_CLEAR, // does not respond to I2C subaddress 1
            REG_CLEAR, // does not respond to I2C subaddress 2
            REG_CLEAR, // does not respond to I2C subaddress 3,
            REG_CLEAR  // does not respond to I2C all call address
            )};
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, mode1_write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    // Changes the total period for a full PWM cycle (essentially remaps the 4096 counter
    // region onto a small time duration set by the frequency)
    uint8_t prescale_write_buf[] = {
        SERVO_PRESCALE_REG,
        round((double)SERVO_INT_CLK_FREQ / (4096.0 * freq)) - 1};
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, &prescale_write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    // Disables sleep mode to be able to generate outputs
    mode1_write_buf[1] ^= REG_SET << 4;
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, mode1_write_buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    vTaskDelay(pdMS_TO_TICKS(1)); // Requires 500 us for the oscillator to startup

    // Enable output on the servo driver
    gpio_config_t output_enabled_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(STEPPER_OUTPUT_ENABLED_PIN),
        .pull_down_en = true,
        .pull_up_en = false
    };
    gpio_config(&output_enabled_gpio_cfg);
    gpio_set_level(STEPPER_OUTPUT_ENABLED_PIN, false); // Active low
}

void set_servo_channel_pulse(uint8_t chnl, uint16_t on_count, uint16_t off_count){
    // Accepts the first 12 bits for the on and off counts and thus only the first 4
    // bits are taken of the high byte.

    uint8_t servo_pulse_cfg_write_buf[] = {
        SERVO_LED0_ON_L_REG + 4 * chnl, // Determines starting register dynamically
        on_count & 0xFF, // off low byte
        (on_count & 0xF00) >> 8, // off high byte
        off_count & 0xFF, // on low byte
        (off_count & 0xF00) >> 8, // on high byte
    };
    i2c_master_write_to_device(I2C_MASTER_NUM, SERVO_I2C_ADDR, &servo_pulse_cfg_write_buf, 5, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

void set_servo_channel_pulse_width(uint8_t chnl, uint16_t servo_freq, double up_ms){
    // Subtracts 1 as the counts are on a range of 0-4095 (total of 4096)
    uint16_t off_counts = (up_ms / (1.0 / (double)servo_freq * 1e3)) * 4096 - 1;
    set_servo_channel_pulse(chnl, 0, off_counts);
}

void test_servo_task(void *){
   uint8_t servo_chnl = 3;
   uint16_t servo_freq = 30;

   setup_servo_driver(servo_freq);

   set_servo_channel_pulse_width(servo_chnl, servo_freq, 1.5);
   
   bool min_pos = false;
   while(1){
        ESP_LOGI(SENSOR_TEST_TAG_NAME, "Setting servo channel at %s position", min_pos ? "minimum" : "maximum");

        set_servo_channel_pulse_width(servo_chnl, servo_freq, min_pos ? 1.5 : 2);
        min_pos = !min_pos;

        vTaskDelay(pdMS_TO_TICKS(2000));
   }

}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100)); // Board setup

    // Perform peripheral setup code
    setup_peripheral_comms();

    // Initializes tasks
    xTaskCreate(blink_task, "Blink", 4096, NULL, 0, NULL);
    xTaskCreate(test_read_sensors_task, "ReadSensorsTest", 4096, NULL, 0, NULL);
    xTaskCreate(test_stepper_task, "StepperTest", 4096, NULL, 0, NULL);
    xTaskCreate(test_servo_task, "ServoTest", 4096, NULL, 0, NULL);
}