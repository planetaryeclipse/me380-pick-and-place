#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "comms.h"
#include "monitor/power_mntr.h"
#include "tasks/test_tasks.h"
#include "sys_state.h"
#include "tasks/cntrl_schemes.h"

void app_main(void)
{
    // Needs 100 ms to handle board setup
    vTaskDelay(pdMS_TO_TICKS(100));

    set_sys_state(MANUAL_CTRL);
    setup_peripheral_comms();
    setup_cntrl();

    xTaskCreate(power_mntr_task, "PowerMonitor", 4096, NULL, 0, NULL);
    xTaskCreate(manual_cntrl_task, "ManualControl", 4096, NULL, 0, NULL);

    // Initializes test tasks
    
    // xTaskCreate(test_blink_task, "Blink", 4096, NULL, 0, NULL);
    // xTaskCreate(test_read_sensors_task, "ReadSensorsTest", 4096, NULL, 0, NULL);
    // xTaskCreate(test_stepper_task, "StepperTest", 4096, NULL, 0, NULL);
    // xTaskCreate(test_servo_task, "ServoTest", 4096, NULL, 0, NULL);
}