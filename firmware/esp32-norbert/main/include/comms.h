#ifndef COMMS_H
#define COMMS_H

#define PERIPHERAL_SETUP_TAG_NAME "peripheral-setup"

// Utility method to create a byte for each bit
#define MAKE_BYTE(x7, x6, x5, x4, x3, x2, x1, x0) ((x7 << 7) | (x6 << 6) | (x5 << 5) | (x4 << 4) | (x3 << 3) | (x2 << 2) | (x1 << 1) | x0)

// I2C
#define I2C_MASTER_NUM 0

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_MASTER_FREQ_HZ 100000

#define I2C_REG_BIT_SET 1
#define I2C_REG_BIT_CLEAR 0

// GPIO interrupt
#define ESP_INTR_FLAG_DEFAULT 0

void setup_peripheral_comms();

#endif // COMMS_H