//
// Created by kiran on 2/26/24.
//

#ifndef ESP32_BOARDCODE_WIFI_H
#define ESP32_BOARDCODE_WIFI_H

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "linenoise/linenoise.h"
#include "nvs_flash.h"
#include <esp_http_server.h>
#include <esp_timer.h>
#include <stdio.h>
#include <stdlib.h>

#define WIFI_SSID      "EE3_CHESS"
#define WIFI_PASS      "checkmate"
#define WIFI_CHANNEL   11
#define TAG_WIFI "WIFI"

void initNvs();

void setupWifi();

#endif //ESP32_BOARDCODE_WIFI_H
