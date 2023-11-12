#ifndef COMMS_H
#define COMMS_H

#define MAKE_BYTE(x7, x6, x5, x4, x3, x2, x1, x0) ((x7 << 7) | (x6 << 6) | (x5 << 5) | (x4 << 4) | (x3 << 3) | (x2 << 2) | (x1 << 1) | x0)

// I2C
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_REG_BIT_SET 1
#define I2C_REG_BIT_CLEAR 0

// UART
#define UART_NUM UART_NUM_0

typedef struct {
    bool toggle_control_mode;

    bool activate_sweep;

    bool activate_pushoff_1;
    bool activate_pushoff_2;
    bool activate_pushoff_3;

    bool stepper_dir;

    uint16_t arm_1_servo;
    uint16_t arm_2_servo;
    uint16_t arm_3_servo;

    uint16_t steps;
} cntrl_cmd_t;

extern QueueHandle_t cmd_queue;

void setup_peripheral_comms();

#endif // COMMS_H