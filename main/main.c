#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define IR_TX_PIN        GPIO_NUM_25
#define IR_RX_PIN        GPIO_NUM_22
#define IR_CARRIER_HZ    38000

#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL     LEDC_CHANNEL_0
#define LEDC_DUTY_RES    LEDC_TIMER_10_BIT
#define LEDC_DUTY        512

#define POLL_PERIOD_MS   100
#define DEBOUNCE_SAMPLES 5

static const char *TAG = "ir_beam";

static void ir_carrier_start(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = IR_CARRIER_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = IR_TX_PIN,
        .duty       = LEDC_DUTY,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static void ir_rx_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << IR_RX_PIN,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

void app_main(void)
{
    ESP_LOGI(TAG, "IR break beam starting");
    ESP_LOGI(TAG, "  TX (5mm IR LED): GPIO %d @ %d Hz", IR_TX_PIN, IR_CARRIER_HZ);
    ESP_LOGI(TAG, "  RX (KSM-803):    GPIO %d", IR_RX_PIN);

    ir_carrier_start();
    ir_rx_init();

    vTaskDelay(pdMS_TO_TICKS(200));

    bool beam_broken = false;
    int  high_run    = 0;
    int  low_run     = 0;
    int  crossings   = 0;

    ESP_LOGI(TAG, "Beam armed. Waiting for crossings...");

    while (1) {
        int level = gpio_get_level(IR_RX_PIN);

        if (level) {
            high_run++;
            low_run = 0;
        } else {
            low_run++;
            high_run = 0;
        }

        if (!beam_broken && high_run >= DEBOUNCE_SAMPLES) {
            beam_broken = true;
            crossings++;
            ESP_LOGW(TAG, ">>> BEAM BROKEN  (crossing #%d)", crossings);
        } else if (beam_broken && low_run >= DEBOUNCE_SAMPLES) {
            beam_broken = false;
            ESP_LOGI(TAG, "<<< Beam restored");
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
