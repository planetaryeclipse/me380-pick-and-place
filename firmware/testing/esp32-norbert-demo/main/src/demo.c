#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_log.h"


#include "demo.h"

#include "drivers/stepper_tb6600.h"

static const char *log_tag = "demo";

#define ROT_ANGLE 90.0
#define ROTATION_TASK_DELAY_MS 100

void demo_rotation_task(void *args){
    demo_rotation_params_t *params = (demo_rotation_params_t*)args;
    TickType_t prev_wake_tick = xTaskGetTickCount();

    for(;;)
    {
        ESP_LOGI(log_tag, "Running step task: tick=%lu, angle=%f", prev_wake_tick, ROT_ANGLE);
        
        step_tb6600(params->stepper, steps_for_angle_tb6600(params->stepper->cfg, ROT_ANGLE), NULL);
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // xTaskDelayUntil(&prev_wake_tick, pdMS_TO_TICKS(ROTATION_TASK_DELAY_MS));
    }
}

void demo_lifting_task(void *args){
    // Unused
}