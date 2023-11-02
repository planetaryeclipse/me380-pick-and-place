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

typedef enum arm_limit_switch_loc {
    ARM1_INNER,
    ARM1_OUTER,
    ARM2_INNER,
    ARM2_OUTER,
    ARM3_INNER,
    ARM3_OUTER
} arm_limit_switch_loc;

void isr_homing_limit_switch(void* arg){
    ESP_EARLY_LOGI(AUTOHOMING_TAG_NAME, "Homing limit switch triggered");
}

void isr_arm_limit_switch(void* arg){
    arm_limit_switch_loc loc = (arm_limit_switch_loc)arg;
    switch(loc){
        case ARM1_INNER: {
            ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM1_INNER limit switch triggered");
            break;
        }
        case ARM1_OUTER: {
            ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM1_OUTER limit switch triggered");
            break;
        }
        case ARM2_INNER: {
            ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM2_INNER limit switch triggered");
            break;
        }
        case ARM2_OUTER: {
            ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM2_OUTER limit switch triggered");
            break;
        }
        case ARM3_INNER: {
            ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM3_INNER limit switch triggered");
            break;
        }
        case ARM3_OUTER: {
            ESP_EARLY_LOGI(DEVICE_SAFTEY_TAG_NAME, "ARM3_OUTER limit switch triggered");
            break;
        }
    }
}

static inline void setup_arm_switch(uint8_t pin, arm_limit_switch_loc loc){
    gpio_config_t arm_lim_swch = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(pin),
        .pull_up_en = true,
        .pull_down_en = false};
    gpio_config(&arm_lim_swch);
    gpio_isr_handler_add(pin, isr_arm_limit_switch, (void*)loc);
}

void setup_limit_switches(){
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

void isr_ultrasonic_echo(void* arg){
    // Due to decreased allowable stack sizes in ISRs there is no possible way to output
    // the distance value in this function. This is handled by the reading location.

    if (gpio_get_level(ULTRASONIC_ECHO_PIN)){
        // Record time at the start of the echo pulse
        ultrasonic_echo_start_us = esp_timer_get_time();
    } else {
        int64_t fly_time_us = esp_timer_get_time() - ultrasonic_echo_start_us;
        float dist = (float)fly_time_us / 1e6 * GROUND_LEVEL_SOUND_VEL / 2.0;

        xQueueSendFromISR(ultrasonic_distances, (void*)&dist, NULL);
    }
}

void setup_ultrasonic(){
    gpio_config_t trig_gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_BITMASK(ULTRASONIC_TRIG_PIN),
        .pull_down_en = true,
        .pull_up_en = false
    };
    gpio_config(&trig_gpio_cfg);

    gpio_config_t echo_gpio_cfg = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIN_BITMASK(ULTRASONIC_ECHO_PIN),
        .pull_down_en = true,
        .pull_up_en = false
    };
    gpio_config(&echo_gpio_cfg);
    gpio_isr_handler_add(ULTRASONIC_ECHO_PIN, isr_ultrasonic_echo, NULL);

    ultrasonic_distances = xQueueCreate(ULTRASONIC_QUEUE_SIZE, sizeof(float));
}

void trig_ultrasonic_scan(){
    gpio_set_level(ULTRASONIC_TRIG_PIN, true);
    ets_delay_us(ULTRASONIC_TRIG_DURATION_US);
    gpio_set_level(ULTRASONIC_TRIG_PIN, false);
}

float read_distance(){
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

    while (1)
    {
        // Periodically read sensors
        read_color_sensor(&c, &r, &g, &b);
        ESP_LOGI(SENSOR_TEST_TAG_NAME, "Color reading: c=%i r=%i g=%i b=%i", c, r, g, b);
        
        ultrasonic_dist = read_distance();
        ESP_LOGI(SENSOR_TEST_TAG_NAME, "Ultrasonic distance: %f", ultrasonic_dist);

        /* 
        Alternative polling code

        trig_ultrasonic_scan();
        while(xQueueReceive(ultrasonic_distances, &ultrasonic_dist, 0)){
            ESP_LOGI(SENSOR_TEST_TAG_NAME, "Ultrasonic distance: %f", ultrasonic_dist);
        }
        */

        vTaskDelay(pdMS_TO_TICKS(500));
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
}