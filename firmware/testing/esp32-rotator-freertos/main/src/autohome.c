#include "autohome.h"
#include "rotator_pinouts.h"

#include "stdbool.h"

#include "drivers/stepper_tb6600.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_log.h"

#include "esp_intr_alloc.h"

static const char *log_tag = "autohome";
static bool is_homed = false;

float rot_angle = 90;

uint64_t autohome_bit_mask = (1ULL << HOME_DETECT_GPIO);

static void gpio_isr_handler(void* arg){
    ESP_LOGI(log_tag, "Triggered falling edge of button!");
}

void autohome_task(void *args)
{
    // Sets up switch interrupt
    gpio_config_t autohome_sensing = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = autohome_bit_mask,
        .pull_down_en = false,
        .pull_up_en = true
    };
    gpio_config(&autohome_sensing);

    gpio_install_isr_service(0); // defined as ESP_INTR_FLAG_DEFAULT
    gpio_isr_handler_add(autohome_bit_mask, gpio_isr_handler, (void*)autohome_bit_mask);

    autohome_params_t *params = (autohome_params_t *)args;
    TickType_t prev_wake_tick = xTaskGetTickCount();

    for(;;)
    {
        ESP_LOGI(log_tag, "Running autohome task: tick %lu", prev_wake_tick);
        
        if(!is_homed){
            step_tb6600(params->stepper, steps_for_angle_tb6600(params->stepper->cfg, rot_angle), &is_homed);
        }

        xTaskDelayUntil(&prev_wake_tick, pdMS_TO_TICKS(params->task_exec_ms));
    }
}