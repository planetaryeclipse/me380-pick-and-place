#include "driver/i2c.h"
#include "esp_log.h"

#include "pinouts.h"
#include "comms.h"

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