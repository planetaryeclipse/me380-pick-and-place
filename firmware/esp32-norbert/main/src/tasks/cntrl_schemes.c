#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "tasks/cntrl_schemes.h"
#include "comms.h"
#include "actuators/servo.h"
#include "actuators/stepper.h"

#define MANUAL_CNTRL_TAG_NAME "manual-cntrl"
#define AUTO_CNTRL_TAG_NAME "auto-cntrl"

#define MANUAL_CNTRL_UPDATE_MS 50
#define CNTRL_CMD_RECV_TOUT_MS 5

// Channels on the servo breakout board

#define ARM1_SERVO1_CHNL 0
#define ARM1_SERVO2_CHNL 1

#define ARM2_SERVO1_CHNL 2
#define ARM2_SERVO2_CHNL 3

#define ARM3_SERVO1_CHNL 4
#define ARM3_SERVO2_CHNL 5

#define CONVEYOR_SWEEP_SERVO_CHNL 6
#define PUSHOFF_SERVO1_CHNL 7
#define PUSHOFF_SERVO2_CHNL 8
#define PUSHOFF_SERVO3_CHNL 9

// Minimum and maximum positions of various servos

#define SERVO_FREQ_HZ 57

#define PUSHOFF_SERVO_MIN_PULSE_MS 1.0
#define PUSHOFF_SERVO_MAX_PULSE_MS 1.8

#define ARM_SERVO_MIN_PULSE_MS (1.0+0.15)
#define ARM_SERVO_MAX_PULSE_MS (2.0)

#define ARM_SERVO_2_MAX_PULSE_MS (2.0)
#define ARM_SERVO_2_MIN_PULSE_MS (1.0 + 0.15)

static bool upd_cntrl_scheme_on_last_cmd;

static bool prev_activate_sweep;
static bool prev_activate_pushoff_1;
static bool prev_activate_pushoff_2;
static bool prev_activate_pushoff_3;

static uint16_t prev_arm1_pos;
static uint16_t prev_arm2_pos;
static uint16_t prev_arm3_pos;

void setup_cntrl()
{
    setup_servo_driver(SERVO_FREQ_HZ);
    upd_cntrl_scheme_on_last_cmd = false;

    prev_activate_sweep = false;
    prev_activate_pushoff_1 = false;
    prev_activate_pushoff_2 = false;
    prev_activate_pushoff_3 = false;

    prev_arm1_pos = 0;
    prev_arm2_pos = 0;
    prev_arm3_pos = 0;

    setup_stepper(FULL_STEP);
}

// static QueueHandle_t up_down_event;

// #define HEIGHT_MOVEMENT_TIME (2.0)
// #define HEIGHT_MOVEMENT_UPD_MS (50)

// void up_down_task(void *){
//     static uint16_t curr_height_val = 0;
//     bool up_down_movement = false;

//     uint16_t movement = 0;

//     while(1){
//         while(xQueueReceive(up_down_event, &up_down_movement, pdMS_TO_TICKS(10))){
//             int32_t curr_height = 0;
//             uint16_t last_height = 0;

//             while(true){
//                 int16_t height_change = (int)((1000.0 / HEIGHT_MOVEMENT_UPD_MS) * 65535.0) * (up_down_movement ? 1 : -1);
//                 curr_height += height_change;

//                 float arm1_servo_ms = ARM_SERVO_MAX_PULSE_MS - (ARM_SERVO_MAX_PULSE_MS - ARM_SERVO_MIN_PULSE_MS) * (curr_height / 65535.0);

//                 set_servo_channel_pulse_width(0, SERVO_FREQ_HZ, arm1_servo_ms);
//                 set_servo_channel_pulse_width(1, SERVO_FREQ_HZ, arm1_servo_ms);

//                 set_servo_channel_pulse_width(2, SERVO_FREQ_HZ, arm1_servo_ms);
//                 set_servo_channel_pulse_width(3, SERVO_FREQ_HZ, arm1_servo_ms);

//                 set_servo_channel_pulse_width(4, SERVO_FREQ_HZ, arm1_servo_ms);
//                 set_servo_channel_pulse_width(5, SERVO_FREQ_HZ, arm1_servo_ms);

//                 if (up_down_movement){
//                     // Moving up
//                     if (curr_height > 65535){
//                         curr_height = 65535;
//                         break;
//                     }
//                 } else {
//                     // Moving down
//                     if (curr_height < 0){
//                         curr_height = 0;
//                         break;
//                     }
//                 }
//             }
//         }

//     }
// }

#define SERVO1_BIAS 0.0
#define SERVO2_BIAS 0.0
#define SERVO3_BIAS 0.095

#define SERVO1_2_BIAS 0.12
#define SERVO2_2_BIAS 0.097
#define SERVO3_2_BIAS -0.015

#define SERVO

void manual_cntrl_task(void *)
{
    // up_down_event = xQueueCreate(10, sizeof(bool));


    cntrl_cmd_t cmd;

    set_servo_channel_pulse_width(0, SERVO_FREQ_HZ, (ARM_SERVO_MIN_PULSE_MS + ARM_SERVO_MAX_PULSE_MS)/2 + SERVO1_BIAS);
    set_servo_channel_pulse_width(4, SERVO_FREQ_HZ, (ARM_SERVO_MIN_PULSE_MS + ARM_SERVO_MAX_PULSE_MS)/2 + SERVO3_BIAS);
    set_servo_channel_pulse_width(2, SERVO_FREQ_HZ, (ARM_SERVO_MIN_PULSE_MS + ARM_SERVO_MAX_PULSE_MS)/2 + SERVO2_BIAS);

    set_servo_channel_pulse_width(1, SERVO_FREQ_HZ, (ARM_SERVO_2_MIN_PULSE_MS + ARM_SERVO_2_MAX_PULSE_MS) / 2 + SERVO1_2_BIAS);
    set_servo_channel_pulse_width(3, SERVO_FREQ_HZ, (ARM_SERVO_2_MIN_PULSE_MS + ARM_SERVO_2_MAX_PULSE_MS) / 2 + SERVO2_2_BIAS);
    set_servo_channel_pulse_width(5, SERVO_FREQ_HZ, (ARM_SERVO_2_MIN_PULSE_MS + ARM_SERVO_2_MAX_PULSE_MS) / 2 + SERVO3_2_BIAS);



    // Sets the non-lifting servos
    set_servo_channel_pulse_width(CONVEYOR_SWEEP_SERVO_CHNL, SERVO_FREQ_HZ, PUSHOFF_SERVO_MIN_PULSE_MS);
    set_servo_channel_pulse_width(PUSHOFF_SERVO1_CHNL, SERVO_FREQ_HZ, PUSHOFF_SERVO_MIN_PULSE_MS);
    set_servo_channel_pulse_width(PUSHOFF_SERVO2_CHNL, SERVO_FREQ_HZ, PUSHOFF_SERVO_MIN_PULSE_MS);
    set_servo_channel_pulse_width(PUSHOFF_SERVO3_CHNL, SERVO_FREQ_HZ, PUSHOFF_SERVO_MIN_PULSE_MS);

    // // Sets the lifting servos
    // set_servo_channel_pulse_width(ARM1_SERVO1_CHNL, SERVO_FREQ_HZ, ARM_SERVO_MIN_PULSE_MS);
    // set_servo_channel_pulse_width(ARM1_SERVO2_CHNL, SERVO_FREQ_HZ, ARM_SERVO_MIN_PULSE_MS);
    
    // set_servo_channel_pulse_width(ARM2_SERVO1_CHNL, SERVO_FREQ_HZ, ARM_SERVO_MIN_PULSE_MS);
    // set_servo_channel_pulse_width(ARM2_SERVO2_CHNL, SERVO_FREQ_HZ, ARM_SERVO_MIN_PULSE_MS);
    
    // set_servo_channel_pulse_width(ARM3_SERVO1_CHNL, SERVO_FREQ_HZ, ARM_SERVO_MIN_PULSE_MS);
    // set_servo_channel_pulse_width(ARM3_SERVO2_CHNL, SERVO_FREQ_HZ, ARM_SERVO_MIN_PULSE_MS);

    while (true)
    {
        while (xQueueReceive(cmd_queue, &cmd, pdMS_TO_TICKS(CNTRL_CMD_RECV_TOUT_MS)))
        {
            if (cmd.toggle_control_mode && !upd_cntrl_scheme_on_last_cmd)
            {
                // Received a new command with the indication to toggle the control mode
                ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Switching to automatic control!");
                // TODO: switch to auto control
                upd_cntrl_scheme_on_last_cmd = true;
            }
            else if (!cmd.toggle_control_mode && upd_cntrl_scheme_on_last_cmd)
            {
                // Received a new command with no indication to toggle the control mode
                // and therefore clears the update control scheme flag
                upd_cntrl_scheme_on_last_cmd = false;
            }

            // Given that each set of the servo positions require calls on the I2C bus, this will be minimized
            // by first checking that the set values have changed

            if (cmd.activate_sweep != prev_activate_sweep)
            {
                ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Setting sweep mechanism: %s", cmd.activate_sweep ? "enabled" : "disabled");
                set_servo_channel_pulse_width(CONVEYOR_SWEEP_SERVO_CHNL, SERVO_FREQ_HZ, cmd.activate_sweep ? PUSHOFF_SERVO_MAX_PULSE_MS : PUSHOFF_SERVO_MIN_PULSE_MS);
                prev_activate_sweep = cmd.activate_sweep;
            }
            if (cmd.activate_pushoff_1 != prev_activate_pushoff_1)
            {
                ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Setting compartment 1 pushoff: %s", cmd.activate_pushoff_1 ? "enabled" : "disabled");
                set_servo_channel_pulse_width(PUSHOFF_SERVO1_CHNL, SERVO_FREQ_HZ, cmd.activate_pushoff_1 ? PUSHOFF_SERVO_MAX_PULSE_MS : PUSHOFF_SERVO_MIN_PULSE_MS);
                prev_activate_pushoff_1 = cmd.activate_pushoff_1;
            }
            if (cmd.activate_pushoff_2 != prev_activate_pushoff_2)
            {
                ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Setting compartment 2 pushoff: %s", cmd.activate_pushoff_2 ? "enabled" : "disabled");
                set_servo_channel_pulse_width(PUSHOFF_SERVO2_CHNL, SERVO_FREQ_HZ, cmd.activate_pushoff_2 ? PUSHOFF_SERVO_MAX_PULSE_MS : PUSHOFF_SERVO_MIN_PULSE_MS);
                prev_activate_pushoff_2 = cmd.activate_pushoff_2;
            }
            if (cmd.activate_pushoff_3 != prev_activate_pushoff_3)
            {
                ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Setting compartment 3 pushoff: %s", cmd.activate_pushoff_3 ? "enabled" : "disabled");
                set_servo_channel_pulse_width(PUSHOFF_SERVO3_CHNL, SERVO_FREQ_HZ, cmd.activate_pushoff_3 ? PUSHOFF_SERVO_MAX_PULSE_MS : PUSHOFF_SERVO_MIN_PULSE_MS);
                prev_activate_pushoff_3 = cmd.activate_pushoff_3;
            }

            // Sets the arm servo positions based on the received value
            if (((cmd.arm_1_servo == 65535 || cmd.arm_1_servo == 0) && cmd.arm_1_servo != prev_arm1_pos) || cmd.arm_1_servo - prev_arm1_pos > 1000 || prev_arm1_pos - cmd.arm_1_servo > 1000){
                ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Setting arm 1 position: %i", cmd.arm_1_servo);
                float arm1_servo_ms = ARM_SERVO_MAX_PULSE_MS - (ARM_SERVO_MAX_PULSE_MS - ARM_SERVO_MIN_PULSE_MS) * (cmd.arm_1_servo / 65535.0);
                float arm1_servo2_ms = ARM_SERVO_2_MIN_PULSE_MS + (ARM_SERVO_2_MAX_PULSE_MS - ARM_SERVO_2_MIN_PULSE_MS) * (cmd.arm_1_servo / 65535.0);

                set_servo_channel_pulse_width(0, SERVO_FREQ_HZ, arm1_servo_ms + SERVO1_BIAS);
                set_servo_channel_pulse_width(1, SERVO_FREQ_HZ, arm1_servo2_ms + SERVO1_2_BIAS);
                // set_servo_channel_pulse_width(1, SERVO_FREQ_HZ, arm1_servo_ms);

                // set_servo_channel_pulse_width(3, SERVO_FREQ_HZ, arm1_servo_ms);

                
                // set_servo_channel_pulse_width(5, SERVO_FREQ_HZ, arm1_servo_ms);

                // if (cmd.arm_1_servo < prev_arm1_pos)
                //      vTaskDelay(pdMS_TO_TICKS(100)); // adds a delay if going down
                set_servo_channel_pulse_width(2, SERVO_FREQ_HZ, arm1_servo_ms + SERVO2_BIAS);
                set_servo_channel_pulse_width(3, SERVO_FREQ_HZ, arm1_servo2_ms + SERVO2_2_BIAS);

                set_servo_channel_pulse_width(4, SERVO_FREQ_HZ, arm1_servo_ms + SERVO3_BIAS);
                set_servo_channel_pulse_width(5, SERVO_FREQ_HZ, arm1_servo2_ms + SERVO3_2_BIAS);

                // Apply twice for double measure
                // set_servo_channel_pulse_width(0, SERVO_FREQ_HZ, arm1_servo_ms);
                // set_servo_channel_pulse_width(2, SERVO_FREQ_HZ, arm1_servo_ms);
                // set_servo_channel_pulse_width(3, SERVO_FREQ_HZ, arm1_servo_ms);

                prev_arm1_pos = cmd.arm_1_servo;
            }
            // if (cmd.arm_2_servo != prev_arm2_pos){
            //     ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Setting arm 2 position: %i", cmd.arm_2_servo);
            //     float arm2_servo_ms = ARM_SERVO_MAX_PULSE_MS - (ARM_SERVO_MAX_PULSE_MS - ARM_SERVO_MIN_PULSE_MS) * (cmd.arm_2_servo / 65535.0);
            //     set_servo_channel_pulse_width(ARM2_SERVO1_CHNL, SERVO_FREQ_HZ, arm2_servo_ms);
            //     set_servo_channel_pulse_width(ARM2_SERVO2_CHNL, SERVO_FREQ_HZ, arm2_servo_ms);

            //     prev_arm2_pos = cmd.arm_2_servo;
            // }
            // if (cmd.arm_3_servo != prev_arm3_pos){
            //     ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Setting arm 3 position: %i", cmd.arm_3_servo);
            //     float arm3_servo_ms = ARM_SERVO_MAX_PULSE_MS - (ARM_SERVO_MAX_PULSE_MS - ARM_SERVO_MIN_PULSE_MS) * (cmd.arm_3_servo / 65535.0);
            //     set_servo_channel_pulse_width(ARM3_SERVO1_CHNL, SERVO_FREQ_HZ, arm3_servo_ms);
            //     set_servo_channel_pulse_width(ARM3_SERVO2_CHNL, SERVO_FREQ_HZ, arm3_servo_ms);

            //     prev_arm3_pos = cmd.arm_3_servo;
            // }

            if (cmd.steps > 0){
                ESP_LOGI(MANUAL_CNTRL_TAG_NAME, "Taking steps in %s direction: %i", cmd.stepper_dir ? "+ve" : "-ve", cmd.steps);
               
                // taskENTER_CRITICAL(); // To ensure that the timing of pulses isn't messed up
                rotate_stepper(cmd.steps, cmd.stepper_dir);
                // taskEXIT_CRITICAL();
            }
            
        }

        vTaskDelay(pdMS_TO_TICKS(MANUAL_CNTRL_UPDATE_MS));
    }

    vTaskDelete(NULL);
}