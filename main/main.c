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
#include <string.h>

#include "wifi.h"
#include "http.h"
#include "bt_server.h"


#define STEP_MOTOR_GPIO_STEP     GPIO_NUM_37

#define STEP_MOTOR_SLP1       GPIO_NUM_36
#define STEP_MOTOR_DIR1      GPIO_NUM_38
#define STEP_MOTOR_RST1     GPIO_NUM_35

#define STEP_MOTOR_SLP2       GPIO_NUM_21

#define STEP_MOTOR_DIR2      GPIO_NUM_48
#define STEP_MOTOR_RST2     GPIO_NUM_45

#define GPIO_OUTPUT_PIN_SEL  ( (1ULL<<STEP_MOTOR_DIR1)     \
| (1ULL<<STEP_MOTOR_SLP1)     \
| (1ULL<<STEP_MOTOR_RST1)     \
| (1ULL<<STEP_MOTOR_DIR2)     \
| (1ULL<<STEP_MOTOR_SLP2)     \
| (1ULL<<STEP_MOTOR_RST2))

#define EMERGENCY_OUTER    GPIO_NUM_5
#define EMERGENCY_INNER    GPIO_NUM_7

#define GPIO_INPUT_PIN_SEL \
((1ULL<<EMERGENCY_OUTER) \
| (1ULL<<EMERGENCY_INNER))

#define ORTHOGONAL_TILE_IN_STEPS 5626
#define DIAGONAL_TILE_IN_STEPS 7956

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution
#define TAG_RMT "RMT"

const static uint32_t uniform_speed_hz = 10000;

typedef enum {
    W = 0, E = 1, N = 2, S = 3, NE = 4, NW = 5, SW = 6, SE = 7
} Direction;

static const int dirConfigs[8][4] = {{1, 1, 1, 1}, // N
                                     {0, 0, 1, 1}, // S
                                     {0, 1, 1, 1}, // W
                                     {1, 0, 1, 1}, // E
                                     {1, 0, 1, 0}, // NE
                                     {0, 1, 0, 1}, // NW
                                     {0, 0, 1, 0}, // SW
                                     {0, 0, 0, 1}};// SE

rmt_channel_handle_t motor_chan = NULL;
rmt_encoder_handle_t uniform_motor_encoder = NULL;

Direction stringToEnum(const char *dirAsString) {
    if (strcmp(dirAsString, "N") == 0) {
        return N;
    } else if (strcmp(dirAsString, "S") == 0) {
        return S;
    } else if (strcmp(dirAsString, "W") == 0) {
        return W;
    } else if (strcmp(dirAsString, "E") == 0) {
        return E;
    } else if (strcmp(dirAsString, "NE") == 0) {
        return NE;
    } else if (strcmp(dirAsString, "NW") == 0) {
        return NW;
    } else if (strcmp(dirAsString, "SW") == 0) {
        return SW;
    } else {
        return SE;
    }
}

void disableMotor1() {
    gpio_set_level(STEP_MOTOR_RST1, 0);
    gpio_set_level(STEP_MOTOR_SLP1, 0);
}

void disableMotor2() {
    gpio_set_level(STEP_MOTOR_RST2, 0);
    gpio_set_level(STEP_MOTOR_SLP2, 0);
}

void enableMotor1() {
    gpio_set_level(STEP_MOTOR_RST1, 1);
    gpio_set_level(STEP_MOTOR_SLP1, 1);
}

void enableMotor2() {
    gpio_set_level(STEP_MOTOR_RST2, 1);
    gpio_set_level(STEP_MOTOR_SLP2, 1);
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

int move(Direction dir, int numHalfTiles) {
    int tileDistance;
    if (dir > 3) {
        tileDistance = (DIAGONAL_TILE_IN_STEPS / 2) * numHalfTiles;
    } else {
        tileDistance = (ORTHOGONAL_TILE_IN_STEPS / 2) * numHalfTiles;
    }

    gpio_set_level(STEP_MOTOR_DIR1, dirConfigs[dir][0]);
    gpio_set_level(STEP_MOTOR_DIR2, dirConfigs[dir][1]);
    toggleMotor(dirConfigs[dir][2], 1);
    toggleMotor(dirConfigs[dir][3], 2);
    printf("dirConfigs %i : DIR1 = %i, DIR2 = %i, M1  = %i, M2  = %i \n", dir, dirConfigs[dir][0],
           dirConfigs[dir][1], dirConfigs[dir][2], dirConfigs[dir][3]);

    ESP_LOGI(TAG_RMT, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
            .loop_count = tileDistance,
    };

    // uniform phase
    ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder,
                                 &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));

    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));

    return tileDistance;
}

void setupRMT() {
    ESP_LOGI(TAG_RMT, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
            .gpio_num = STEP_MOTOR_GPIO_STEP,
            .mem_block_symbols = 64,
            .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
            .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
            .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    ESP_LOGI(TAG_RMT, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));
}

int executeCommand(char *command) {

    char *action = NULL;
    char *direction = NULL;
    int distance = 0;

    char *test = "MOVE SE 3\n";

    /* get the first token */
    char *m = strtok(command, " ");

    /* walk through other tokens */
    int count = 0;
    while (m != NULL) {

        switch (count) {
            case 0:
                action = m;
            case 1:
                direction = m;
            case 2:
                distance = atoi(m);
            default:
                break;
        }
        m = strtok(NULL, " ");
        count++;
    }
    printf(" full command = %s\n", command);
    printf("action = %s\n", action);
    printf("direction = %s\n", direction);
    printf(" distance = %d\n", distance);

    if (strlen(command) < 3) {
        printf("UNKNOWN COMMAND\n");
        return 1;
    }
    if (strcmp(action, "MOVE") == 0) {
        printf("MOVE DETECTED\n");
        move(stringToEnum(direction), distance);
    }
    return 1;
}

void app_main(void) {

//    initNvs();
//    setupWifi();
//    startWebserver();
    startBT();

//    ESP_LOGI(TAG_RMT, "Initialize EN + DIR GPIO");
//    gpio_config_t en_dir_gpio_config = {
//            .mode = GPIO_MODE_OUTPUT,
//            .intr_type = GPIO_INTR_DISABLE,
//            .pin_bit_mask = GPIO_OUTPUT_PIN_SEL
//    };

//    en_dir_gpio_config.pin_bit_mask = GPIO_INPUT_PIN_SEL;
//    en_dir_gpio_config.mode = GPIO_MODE_INPUT;
//    en_dir_gpio_config.pull_up_en = 1;
//    gpio_config(&en_dir_gpio_config);

//    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));
//
//    setupRMT();
//
//    executeCommand("MOVE NE 2\n");
//    executeCommand("MOVE SE 2\n");
//    executeCommand("MOVE SW 2\n");
//    executeCommand("MOVE NW 2\n");
//
//    move(SE, 2);
//    move(SW, 2);
//    move(NW, 2);
//    move(NE, 2);
}
