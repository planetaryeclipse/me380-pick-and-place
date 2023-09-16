#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/pulse_cnt.h"

#include "test_rtos_tasks.h"

#define ENCODER_CH_A_PULLED_UP 1 // if 1 starts as high, goes low on clock pulse
#define ENCODER_CH_B_PULLED_UP 0 // if 1 starts as high, goes low on clock pulse

#define ENCODER_CCW_A_LEADS_B 1 // if ch A leads ch B then is moving CCW

#define ENCODER_CPS 1024 // cycle pulses per rotation

#define PCNT_HIGH_LIMIT 1024*4
#define PCNT_LOW_LIMIT -1024*4

#define ENCODER_CH_A_GPIO 34
#define ENCODER_CH_B_GPIO 35

void encoder_read_task(void *arg){
    // This basically places a limit on the number of pulse counters we can use in our design
    ESP_LOGI("encoder-test", "Maximum number of pulse counter (PCNT) units: %i", SOC_PCNT_UNITS_PER_GROUP);

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_new_unit(&unit_config, &pcnt_unit);

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_CH_A_GPIO,
        .level_gpio_num = ENCODER_CH_B_GPIO,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_CH_B_GPIO,
        .level_gpio_num = ENCODER_CH_A_GPIO,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);

#if ENCODER_CH_A_PULLED_UP
    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
#else
    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
#endif

#if ENCODER_CH_B_PULLED_UP
    pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
#else
    pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
#endif

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);

    int pulse_count = 0;
    float encoder_ang = 0;

    while(1){
        pcnt_unit_get_count(pcnt_unit, &pulse_count);
        encoder_ang = (float)pulse_count / 4 / ENCODER_CPS * 360.0;

        ESP_LOGI("encoder-test", "Encoder count: %i, angle: %f", pulse_count, encoder_ang);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}