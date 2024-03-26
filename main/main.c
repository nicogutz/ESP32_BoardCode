/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/soc/gpio_sig_map.h"
#include "math.h"
#include "rom/gpio.h"
#include "stepper_motor_encoder.h"
#include "nrf.h"

#define MUX_RST GPIO_NUM_8
#define MUX_CLK GPIO_NUM_18
#define GPIO_OUTPUT_MUX_SEL ((1ULL << MUX_RST) | (1ULL << MUX_CLK))

#define SENSOR_ARRAY GPIO_NUM_17
// #define GPIO_INPUT_PIN_SEL  (1ULL<<SENSOR_ARRAY)

//#define USE_WIFI
#define USE_BLUETOOTH

#ifdef USE_WIFI

#include "http.h"
#include "wifi.h"
#include "mirf.h"

#elif defined(USE_BLUETOOTH)

#include "bt_server.h"

#else
#error "Please define either USE_WIFI or USE_BLUETOOTH"
#endif

#define STEP_MOTOR_GPIO_STEP1 GPIO_NUM_37
#define STEP_MOTOR_GPIO_STEP2 GPIO_NUM_47

#define STEP_MOTOR_SLP1 GPIO_NUM_36
#define STEP_MOTOR_DIR1 GPIO_NUM_38
#define STEP_MOTOR_RST1 GPIO_NUM_35

#define STEP_MOTOR_SLP2 GPIO_NUM_21
#define STEP_MOTOR_DIR2 GPIO_NUM_48
#define STEP_MOTOR_RST2 GPIO_NUM_45

#define EM_TOGGLE GPIO_NUM_1

#define GPIO_OUTPUT_RMT_SEL \
    ((1ULL << STEP_MOTOR_GPIO_STEP1) | (1ULL << STEP_MOTOR_GPIO_STEP2))

#define GPIO_OUTPUT_PIN_SEL (((1ULL << STEP_MOTOR_DIR1) | (1ULL << STEP_MOTOR_SLP1) | (1ULL << STEP_MOTOR_RST1) | (1ULL << STEP_MOTOR_DIR2) | (1ULL << STEP_MOTOR_SLP2) | (1ULL << STEP_MOTOR_RST2) | (1ULL << EM_TOGGLE)) | (GPIO_OUTPUT_RMT_SEL))

#define EMERGENCY_OUTER GPIO_NUM_16
#define EMERGENCY_INNER GPIO_NUM_15

#define GPIO_INPUT_PIN_SEL \
    ((1ULL << EMERGENCY_OUTER) | (1ULL << EMERGENCY_INNER))

#define ORTHOGONAL_TILE_IN_STEPS 5860
#define DIAGONAL_TILE_IN_STEPS 15913

#define STEP_MOTOR_RESOLUTION_HZ 2000000  // 1MHz resolution
#define TAG_RMT "RMT"

const static uint32_t uniform_speed_hz = 20000;

#define MAX_COMMANDS 10
#define MOVE_HEADER_LENGTH 32
#define MAX_PARAMS 10
#define MAX_PARAM_LENGTH 10

typedef enum {
    NO = 0,
    SO = 1,
    WE = 2,
    EA = 3,

    NE = 4,
    NW = 5,
    SW = 6,
    SE = 7
} Direction;

static const int dirConfigs[8][4] = {{1, 1, 1, 1},   // NO
                                     {0, 0, 1, 1},   // SO
                                     {0, 1, 1, 1},   // WE
                                     {1, 0, 1, 1},   // EA
                                     {1, 0, 1, 0},   // NE
                                     {0, 1, 0, 1},   // NW
                                     {0, 0, 1, 0},   // SW
                                     {0, 0, 0, 1}};  // SE

const uint8_t positions[] = {
        49,
        51,
        55,
        53,
        57,
        59,
        61,
        63,
        64,
        62,
        60,
        58,
        56,
        54,
        52,
        50,
        33,
        35,
        37,
        39,
        41,
        43,
        45,
        47,
        48,
        46,
        44,
        42,
        40,
        38,
        36,
        34,
        17,
        19,
        21,
        23,
        25,
        27,
        29,
        31,
        32,
        30,
        28,
        26,
        24,
        22,
        20,
        18,
        1,
        3,
        5,
        7,
        9,
        11,
        13,
        15,
        16,
        14,
        12,
        10,
        8,
        6,
        4,
        2

};

rmt_channel_handle_t motor_chan = NULL;
rmt_encoder_handle_t uniform_motor_encoder = NULL;

Direction extractDirection(const char *moveCommand) {
    if (strncmp(moveCommand + 2, "NO", 2) == 0) {
        return NO;
    } else if (strncmp(moveCommand + 2, "SO", 2) == 0) {
        return SO;
    } else if (strncmp(moveCommand + 2, "WE", 2) == 0) {
        return WE;
    } else if (strncmp(moveCommand + 2, "EA", 2) == 0) {
        return EA;
    } else if (strncmp(moveCommand + 2, "NE", 2) == 0) {
        return NE;
    } else if (strncmp(moveCommand + 2, "NW", 2) == 0) {
        return NW;
    } else if (strncmp(moveCommand + 2, "SW", 2) == 0) {
        return SW;
    } else {
        return SE;
    }
}

int extractDistance(const char *moveCommand) {
    return atoi(moveCommand + 4);
}

void disableMotor1() {
    gpio_set_level(STEP_MOTOR_RST1, 0);
    gpio_set_level(STEP_MOTOR_SLP1, 1);
}

void disableMotor2() {
    gpio_set_level(STEP_MOTOR_RST2, 0);
    gpio_set_level(STEP_MOTOR_SLP2, 1);
}

void enableMotor1() {
    gpio_set_level(STEP_MOTOR_RST1, 1);
    gpio_set_level(STEP_MOTOR_SLP1, 1);
}

void enableMotor2() {
    gpio_set_level(STEP_MOTOR_RST2, 1);
    gpio_set_level(STEP_MOTOR_SLP2, 1);
}

bool isPressed(gpio_num_t input) {
    return !gpio_get_level(input);
}

void toggleMotor(bool switchOn, int motor) {
    if (motor == 1) {
        if (switchOn) {
            enableMotor1();
        } else {
            disableMotor1();
        }
    } else if (motor == 2) {
        if (switchOn) {
            enableMotor2();
        } else {
            disableMotor2();
        }
    }
}

void executeToggleMagnet(uint8_t switchOn) {
    if (switchOn == '1') {
        gpio_set_level(EM_TOGGLE, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        printf("Magnet state: %d", switchOn);
    } else if (switchOn == '0') {
        gpio_set_level(EM_TOGGLE, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        printf("Magnet state: %d", switchOn);
    } else {
        printf("wrong command in magnet toggle");
    };
}

bool canMoveto(Direction dir) {
    switch (dir) {
        case NO:
            return true;
        case SO:
            if (!isPressed(EMERGENCY_INNER)) {
                return true;
            } else
                return false;
        case WE:
            if (!isPressed(EMERGENCY_OUTER)) {
                return true;
            } else
                return false;
        case EA:
            return true;
        case NE:
            return true;
        case NW:
            if (!isPressed(EMERGENCY_OUTER)) {
                return true;
            } else
                return false;
        case SW:
            if (!isPressed(EMERGENCY_OUTER) && !isPressed(EMERGENCY_INNER)) {
                return true;
            } else
                return false;
        case SE:
            if (!isPressed(EMERGENCY_INNER)) {
                return true;
            } else
                return false;
    }
    return false;
}

int executeMove(Direction dir, double numHalfTiles) {
    double tileDistance;
    if (dir > 3) {
        tileDistance = ((DIAGONAL_TILE_IN_STEPS / 2) * numHalfTiles * 0.75);
        if (dir % 2 == 0) {
            gpio_matrix_out(STEP_MOTOR_GPIO_STEP2, SIG_GPIO_OUT_IDX, false, false);
        } else {
            gpio_matrix_out(STEP_MOTOR_GPIO_STEP1, SIG_GPIO_OUT_IDX, false, false);
        }
    } else {
        tileDistance = (ORTHOGONAL_TILE_IN_STEPS / 2) * numHalfTiles;
    }

    gpio_set_level(STEP_MOTOR_DIR1, dirConfigs[dir][0]);
    gpio_set_level(STEP_MOTOR_DIR2, dirConfigs[dir][1]);
    toggleMotor(dirConfigs[dir][2], 1);
    toggleMotor(dirConfigs[dir][3], 2);
    printf("dirConfigs %i : DIR1 = %i, DIR2 = %i, M1  = %i, M2  = %i \n", dir, dirConfigs[dir][0],
           dirConfigs[dir][1], dirConfigs[dir][2], dirConfigs[dir][3]);

    if (canMoveto(dir)) {
        rmt_transmit_config_t tx_config = {
            .loop_count = tileDistance,
        };

        // uniform phase
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder,
                                     &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));

        // wait all transactions finished
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
    }

    disableMotor1();
    disableMotor2();

    gpio_matrix_out(STEP_MOTOR_GPIO_STEP1, RMT_SIG_OUT0_IDX, false, false);
    gpio_matrix_out(STEP_MOTOR_GPIO_STEP2, RMT_SIG_OUT0_IDX, false, false);

    return tileDistance;
}

void executeHome() {
    if (!isPressed(EMERGENCY_INNER) || !isPressed(EMERGENCY_OUTER)) {
        printf("IN GET HOME\n");
        while (!isPressed(EMERGENCY_INNER)) {
            executeMove(SO, .25);
        }
        while (!isPressed(EMERGENCY_OUTER)) {
            executeMove(WE, .25);
        }
    }
    printf("HOME");
}

void setupRMT() {
    ESP_LOGI(TAG_RMT, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,  // select clock source
            .gpio_num = STEP_MOTOR_GPIO_STEP1,
            .mem_block_symbols = 64,
            .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
            .trans_queue_depth = 10,  // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    // SIG_GPIO_OUT_IDX
    gpio_matrix_out(STEP_MOTOR_GPIO_STEP2, RMT_SIG_OUT0_IDX, false, false);

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
            .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    ESP_LOGI(TAG_RMT, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));
}

uint64_t readSensors() {
    uint64_t board[2] = {0};
    // Equivalent to loop()
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(MUX_CLK, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(MUX_CLK, 1);

    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(MUX_RST, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(MUX_RST, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    // Wait 1s
    for (int j = 0; j < 2; ++j) {
        for (int i = 0; i < 64; i++) {
            if (!gpio_get_level(SENSOR_ARRAY)) {
                board[j] = board[j] | ((uint64_t)!gpio_get_level(SENSOR_ARRAY) << (positions[i] - 1));
                vTaskDelay(150 / portTICK_PERIOD_MS);
            }
            gpio_set_level(MUX_CLK, 0);
            // Delay 50 ms
            vTaskDelay(15 / portTICK_PERIOD_MS);
            // Set HIGH
            gpio_set_level(MUX_CLK, 1);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
    }
    printf("0x%" PRIx64 "\n", board[0]);
    if (board[1] == board[0]) {
        return board[0];
    } else {
        return readSensors();
    }
}

int executeTextCommand(char *command) {
    if (strncmp(command, "MV", 2) == 0) {
        // eg. "MVNE7"
        printf("Executing move\n");
        executeMove(extractDirection(command), extractDistance(command));
    } else if (strncmp(command, "HM", 2) == 0) {
        // eg. "HM"
        printf("Executing home\n");
        executeHome();
    } else if (strncmp(command, "MG", 2) == 0) {
        // eg. "MG1"
        printf("Executing toggleMagnet\n");
        executeToggleMagnet(command[2]);
    }else if(strncmp(command, "TM", 2) == 0){
        // eg. TM[R/L][32byte time]
//        nrf_send(command + 2);
    } else if (strncmp(command, "RD", 2) == 0) {
#if defined(USE_BLUETOOTH)
        notifyBoard(readSensors());
        printf("READ DETECTED\n");
#endif
    }
    return 0;
}

int executeTextScript(const char script[]) {
    const char commandDelimiter[] = ",";
    char *rest, *command;

    rest = strdup(script);

    if (rest == NULL) {
        perror("Memory allocation error");
        exit(EXIT_FAILURE);
        return 1;
    }

    printf("Executing script\n");
    char *rest_copy = rest;
    while ((command = strtok_r(rest, commandDelimiter, &rest)) != NULL) {
        executeTextCommand(command);
    }

    free(rest_copy);
    return 0;
}

void app_main(void) {
    disableMotor1();
    disableMotor2();

#ifdef USE_WIFI
    initNvs();
    setupWifi();
    startWebserver();
#elif defined(USE_BLUETOOTH)
    startBT();

#endif

    ESP_LOGI(TAG_RMT, "Initialize EN + DIR GPIO");
    gpio_config_t io_config = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = GPIO_OUTPUT_PIN_SEL};
    gpio_config(&io_config);

    io_config.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = 1;
    gpio_config(&io_config);

    io_config.pin_bit_mask = GPIO_OUTPUT_MUX_SEL;
    io_config.intr_type = GPIO_INTR_DISABLE;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    gpio_config(&io_config);

    io_config.pin_bit_mask = SENSOR_ARRAY;
    io_config.mode = GPIO_MODE_INPUT;

    gpio_config(&io_config);

    setupRMT();

    nrf_init();
}
