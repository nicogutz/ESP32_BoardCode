/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"

#define STEP_MOTOR_GPIO_SLP       GPIO_NUM_36
#define STEP_MOTOR_GPIO_DIR      GPIO_NUM_38
#define STEP_MOTOR_GPIO_STEP     GPIO_NUM_37
#define STEP_MOTOR_GPIO_RST     GPIO_NUM_35
#define STEP_MOTOR_SLEEP_LEVEL  1 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static const char *TAG = "example";

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_SLP | 1ULL << STEP_MOTOR_GPIO_RST,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
            .gpio_num = STEP_MOTOR_GPIO_STEP,
            .mem_block_symbols = 64,
            .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
            .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_SLP, STEP_MOTOR_SLEEP_LEVEL);
    gpio_set_level(STEP_MOTOR_GPIO_RST, 1);

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
            .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
            .loop_count = 0,
    };

    const static uint32_t uniform_speed_hz = 10000;

    while (1) {
        // acceleration phase
//        tx_config.loop_count = 0;
//        ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

        // uniform phase
        tx_config.loop_count = 5000;
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder,
                                     &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));

//        // deceleration phase
//        tx_config.loop_count = 0;
//        ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
        // wait all transactions finished
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));

        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(STEP_MOTOR_GPIO_DIR, !STEP_MOTOR_SPIN_DIR_CLOCKWISE);

    }
}
