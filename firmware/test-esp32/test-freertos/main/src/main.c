#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "esp_intr_alloc.h"
#include "test_rtos_tasks.h"

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    
    xTaskCreate(blink_task, "blink", 4096, NULL, 0, NULL);
    xTaskCreate(servo_test_task, "servo_test", 4096, NULL, 0, NULL);
    xTaskCreate(encoder_read_task, "encoder", 4096, NULL, 0, NULL);

    //Create and start stats task
    // xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, STATS_TASK_PRIO, NULL, tskNO_AFFINITY);
}