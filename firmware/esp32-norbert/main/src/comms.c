#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "pinouts.h"
#include "comms.h"

#define PERIPHERAL_SETUP_TAG_NAME "peripheral-setup"

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_FREQ_HZ 100000

#define UART_COMMS_TAG_NAME "uart-comms"
#define UART_BUF_SIZE 1024
#define UART_TX_BUF_SIZE (UART_BUF_SIZE)
#define UART_RX_BUF_SIZE (UART_BUF_SIZE)

#define ESP_INTR_FLAG_DEFAULT 0

#define MSG_BYTE_LEN 14 // Length of controller command

// Used to receive a controller command over UART serial
static QueueHandle_t uart_0_queue;
static uint8_t dtmp[UART_RX_BUF_SIZE];

QueueHandle_t cmd_queue; // Implements the external definition in the header
static uint8_t recv_cmd_buf[MSG_BYTE_LEN];
static uint16_t recv_cmd_idx;

static inline void check_recv_buf_for_cmd()
{
    // Checks for the message header
    bool contains_correct_header =
        recv_cmd_buf[0] == 'S' &&
        recv_cmd_buf[1] == 'A' &&
        recv_cmd_buf[2] == 'M';
    bool control_byte_reserved_correct = (~recv_cmd_buf[3] & 0x3); // Checks the last 2 bits are 0
    bool arm_servo_control_correct =
        recv_cmd_buf[4] != '\r' && // Checks arm 1 to make sure bytes are not \r\n
        recv_cmd_buf[5] != '\n' &&
        recv_cmd_buf[6] != '\r' && // Checks arm 2
        recv_cmd_buf[7] != '\n' &&
        recv_cmd_buf[8] != '\r' && // Checks arm 3
        recv_cmd_buf[9] != '\n';
    bool stepper_control_correct =
        recv_cmd_buf[10] != '\r' &&
        recv_cmd_buf[11] != '\n';
    bool contains_correct_terminator =
        recv_cmd_buf[12] == '\r' && // Checks the received terminator are
        recv_cmd_buf[13] == '\n';

    // ESP_LOGI(UART_COMMS_TAG_NAME, "Checking for command: valid header: %s, control byte correct: %s, arm servo control correct: %s, stepper control correct: %s, valid terminator: %s",
    //     contains_correct_header ? "true" : "false", control_byte_reserved_correct ? "true" : "false",
    //     arm_servo_control_correct ? "true" : "false", stepper_control_correct ? "true" : "false",
    //     contains_correct_terminator ? "true" : "false");

    bool cmd_valid =
        contains_correct_header &&
        control_byte_reserved_correct &&
        arm_servo_control_correct &&
        stepper_control_correct &&
        contains_correct_terminator;

    if (cmd_valid)
    {
        // Deciphers the command
        cntrl_cmd_t cmd = {
            .toggle_control_mode = (recv_cmd_buf[3] & 0x80) >> 7,
            .activate_sweep = (recv_cmd_buf[3] & 0x40) >> 6,
            .activate_pushoff_1 = (recv_cmd_buf[3] & 0x20) >> 5,
            .activate_pushoff_2 = (recv_cmd_buf[3] & 0x10) >> 4,
            .activate_pushoff_3 = (recv_cmd_buf[3] & 0x8) >> 3,
            .stepper_dir = (recv_cmd_buf[3] & 0x4) >> 2,
            .arm_1_servo = (recv_cmd_buf[5] << 8 | recv_cmd_buf[4]),
            .arm_2_servo = (recv_cmd_buf[7] << 8 | recv_cmd_buf[6]),
            .arm_3_servo = (recv_cmd_buf[9] << 8 | recv_cmd_buf[8]),
            .steps = (recv_cmd_buf[11] << 8 | recv_cmd_buf[10])};

        // Displays the command values
        ESP_LOGI(UART_COMMS_TAG_NAME, "Received command: toggle control: %s, activate sweep: %s, pushoff 1: %s, pushoff 2: %s, pushoff 3: %s, stepper dir: %s, arm 1 servo: %u, arm 2 servo: %u, arm 3 servo: %u, steps: %u",
            cmd.toggle_control_mode ? "true" : "false", cmd.activate_sweep ? "true" : "false",
            cmd.activate_pushoff_1 ? "true" : "false", cmd.activate_pushoff_2 ? "true" : "false",
            cmd.activate_pushoff_3 ? "true" : "false", cmd.stepper_dir ? "true" : "false",
            cmd.arm_1_servo, cmd.arm_2_servo, cmd.arm_3_servo, cmd.steps);
        xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(50));
    } else {
        // ESP_LOGI(UART_COMMS_TAG_NAME, "Reached end of receive buffer without receiving command");
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;

    while (true)
    {
        if (xQueueReceive(uart_0_queue, (void *)&event, pdMS_TO_TICKS(50)))
        {
            bzero(dtmp, UART_RX_BUF_SIZE);

            // ESP_LOGI(UART_COMMS_TAG_NAME, "uart[%d] event %d:", UART_NUM, event.type);
            switch (event.type)
            {
            case UART_DATA:
            {
                // ESP_LOGI(UART_COMMS_TAG_NAME, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_NUM, dtmp, event.size, pdMS_TO_TICKS(50));
                // ESP_LOGI(UART_COMMS_TAG_NAME, "[DATA EVT]");
                
                // Echos the command back to the host over UART
                // uart_write_bytes(UART_NUM, (const char *)dtmp, event.size);

                for (int i = 0; i < event.size; i++)
                {
                    recv_cmd_buf[recv_cmd_idx++] = dtmp[i];
                    // ESP_LOGI(UART_COMMS_TAG_NAME, "Received character: %x", dtmp[i]);
                    if (recv_cmd_idx == MSG_BYTE_LEN)
                    {
                        // This detectes that the receive buffer has been filled and thus it should
                        // be checked to see if it contains a full message. If the message does not
                        // match the expected format then the data is to be thrown out
                        check_recv_buf_for_cmd();
                        bzero(recv_cmd_buf, MSG_BYTE_LEN);
                        recv_cmd_idx = 0;
                    }
                    else if (dtmp[i] == '\n')
                    {
                        // ESP_LOGI(UART_COMMS_TAG_NAME, "Found newline character");

                        // If the preceeding byte is \r then the terminator has been found outside
                        // of the buffer full check indicating that an incomplete command has been
                        // detected and therefore the message has not been received correctly
                        if (recv_cmd_idx > 0 && recv_cmd_buf[recv_cmd_idx - 1] == '\r')
                        {
                            // ESP_LOGI(UART_COMMS_TAG_NAME, "Found preceeding carriage return character");
                            bzero(recv_cmd_buf, MSG_BYTE_LEN);
                            recv_cmd_idx = 0;
                        }
                    }
                }
            }
            case UART_FIFO_OVF:
            {
                uart_flush_input(UART_NUM);
                xQueueReset(uart_0_queue);

                break;
            }
            case UART_BUFFER_FULL:
            {
                uart_flush_input(UART_NUM);
                xQueueReset(uart_0_queue);

                break;
            }
            // case UART_BREAK: {
            //     // No behaviour to perform here
            //     break;
            // }
            // case UART_PARITY_ERR: {
            //     // No behaviour to perform here
            //     break;
            // }
            // case UART_FRAME_ERR: {
            //     // No behaviour to perform here
            //     break;
            // }
            // case UART_PATTERN_DET: {
            //     uart_get_buffered_data_len(UART_NUM, &buffered_size);
            //     int pos = uart_pattern_pop_pos(UART_NUM);

            //     ESP_LOGI(UART_COMMS_TAG_NAME, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
            //     if (pos == -1){
            //         // The pattern position queue is full so the position is not recorded
            //         ESP_LOGW(UART_COMMS_TAG_NAME, "UART pattern position queue is full, increase buffer size");
            //         uart_flush_input(UART_NUM);
            //     } else{
            //         uart_read_bytes(UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
            //         uint8_t pat[PATTERN_CHR_NUM + 1];
            //         memset(pat, 0, sizeof(pat));

            //         uart_read_bytes(UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
            //         ESP_LOGI(UART_COMMS_TAG_NAME, "read data: %s", dtmp);
            //         ESP_LOGI(UART_COMMS_TAG_NAME, "read pat: %s", pat);
            //     }

            //     break;
            // }
            default:
            {
                ESP_LOGI(UART_COMMS_TAG_NAME, "UART event type: %d", event.type);
                break;
            }
            }
        }
    }

    // free(dtmp);
    // dtmp = NULL;
    vTaskDelete(NULL);
}

void setup_peripheral_comms()
{
    // Sets up I2C communication on port 0
    ESP_LOGI(PERIPHERAL_SETUP_TAG_NAME, "Setting up I2C communication...");
    i2c_config_t i2c_master_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0};
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_master_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_master_cfg.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    // Sets to use individual ISRs for different GPIO pins
    ESP_LOGI(PERIPHERAL_SETUP_TAG_NAME, "Setting up GPIO ISR service...");
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));

    // Sets up UART communication and command receive capabilities
    ESP_LOGI(UART_COMMS_TAG_NAME, "Setting up UART communication...");
    bzero(recv_cmd_buf, MSG_BYTE_LEN);
    recv_cmd_idx = 0;

    cmd_queue = xQueueCreate(20, sizeof(cntrl_cmd_t));

    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 20, &uart_0_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0));

    xTaskCreate(uart_event_task, "UART-Event-Test", 4096, NULL, 0, NULL);
}