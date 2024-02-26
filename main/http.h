//
// Created by kiran on 2/26/24.
//

#ifndef ESP32_BOARDCODE_HTTP_H
#define ESP32_BOARDCODE_HTTP_H

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

#define TAG_HTTP "HTTP"

httpd_handle_t startWebserver();

void processPostContent(const char* content);


#endif //ESP32_BOARDCODE_HTTP_H
