#include <inttypes.h>
#include <rom/gpio.h>
#include <soc/io_mux_reg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mirf.h"
#include "http.h"

enum CLOCK_COMMANDS {
    CMD_LOCAL_PLAYER_START = 0x0,
    CMD_REMOTE_PLAYER_START = 0x1,
    CMD_GAME_OVER
};

NRF24_t dev;
uint8_t buf[32], res[32];

bool initialConnection(void) {
    TickType_t nowTick = xTaskGetTickCount();
    sprintf((char *) buf, "CON %" PRIu32, nowTick);
    Nrf24_send(&dev, buf);
    memset(res, 0, sizeof(res));
    if (!Nrf24_isSend(&dev, 1000)) {
        return false;
    }
    for (int i = 0; i < 1000; i++) {
        if (Nrf24_dataReady(&dev)) {
            Nrf24_getData(&dev, res);
            if (!memcmp(buf, res, sizeof(res))) {
                ESP_LOGI(pcTaskGetName(0), "Got response:[%s]", buf);
                return true;
            }
        };
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return false;
}

void nrf_init() {
    Nrf24_init(&dev);
    uint8_t payload = 32;
    uint8_t channel = 114;
    Nrf24_config(&dev, channel, payload);

    // Set own address using 5 characters
    esp_err_t ret = Nrf24_setRADDR(&dev, (uint8_t *) "ABCDE");
    if (ret != ESP_OK) {
        ESP_LOGE(pcTaskGetName(0), "nrf24l01 not installed");
        while (1) {
            vTaskDelay(1);
        }
    }

    // Set the receiver address using 5 characters
    ret = Nrf24_setTADDR(&dev, (uint8_t *) "FGHIJ");
    if (ret != ESP_OK) {
        ESP_LOGE(pcTaskGetName(0), "nrf24l01 not installed");
        while (1) {
            vTaskDelay(1);
        }
    }

    Nrf24_SetSpeedDataRates(&dev, 0);
    Nrf24_setRetransmitDelay(&dev, 1);

    // Print settings
    Nrf24_printDetails(&dev);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (!initialConnection()) {
    };
}

void nrf_send(char *data) {

    vTaskDelay(10 / portTICK_PERIOD_MS);
    for (int i = 0; i < 19; ++i) {
        buf[i] = data[i];
    }
    printf("STM DATA = %s\n", buf);
    Nrf24_send(&dev, buf);
    while (!Nrf24_isSend(&dev, 1000)) {
        Nrf24_send(&dev, buf);
    }

}
