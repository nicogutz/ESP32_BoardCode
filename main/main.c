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

#define MAX_COMMANDS 10
#define MAX_PARAMS 10
#define MAX_PARAM_LENGTH 10

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

bool isPressed(gpio_num_t input) {
    return !gpio_get_level(input);
}

bool isHome() {

    return (isPressed(EMERGENCY_INNER) && isPressed(EMERGENCY_OUTER));
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

void getHome() {
    if (!isPressed(EMERGENCY_INNER) || !isPressed(EMERGENCY_OUTER)) {
        printf("IN GETHOME\n");
        while (!isPressed(EMERGENCY_INNER)) {
            move(S, 1);
        }
        while (!isPressed(EMERGENCY_OUTER)) {
            move(W, 1);
        }
    }
    printf("HOME");
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

void parseCommands(const char command[], char Params[MAX_PARAMS][MAX_PARAM_LENGTH], int *numParams) {
    *numParams = 0;
    const char delimiter[] = " ";
    char *rest, *token;

    rest = strdup(command);

    if (rest == NULL) {
        perror("Memory allocation error");
        exit(EXIT_FAILURE);
    }

    char *rest_copy = rest;  // Store a copy of the pointer to free later

    // Parse the command into Params array
    while ((token = strtok_r(rest, delimiter, &rest)) != NULL && *numParams < MAX_PARAMS) {
        strcpy(Params[*numParams], token);
        (*numParams)++;
    }

    free(rest_copy);  // Free the original memory allocation
}

void parseScript(const char script[], char commands[MAX_COMMANDS][MAX_PARAMS][MAX_PARAM_LENGTH], int *numCommands,
                 int *numParams) {
    *numCommands = 0;
    const char commandDelimiter[] = "\n";

    char *rest, *command;

    rest = strdup(script);

    if (rest == NULL) {
        perror("Memory allocation error");
        exit(EXIT_FAILURE);
    }

    char *rest_copy = rest;  // Store a copy of the pointer to free later

    // Parse the script into commands array
    while ((command = strtok_r(rest, commandDelimiter, &rest)) != NULL && *numCommands < MAX_COMMANDS) {
        parseCommands(command, commands[*numCommands], &numParams[*numCommands]);
        (*numCommands)++;
    }

    free(rest_copy);  // Free the original memory allocation
}

int executeScript(char *script) {


    int numCommands = 0;
    int numParams[MAX_COMMANDS] = {0};
    char commands[MAX_COMMANDS][MAX_PARAMS][MAX_PARAM_LENGTH];

    parseScript(script, commands, &numCommands, numParams);


    // Output the parsed commands and parameters
    for (int i = 0; i < numCommands; ++i) {

        if (strcmp(commands[i][0], "MOVE") == 0) {
            printf("MOVE DETECTED\n");
            move(stringToEnum(commands[i][1]), atoi(commands[i][2]));

        } else if (strcmp(commands[i][0], "HOME") == 0) {
            printf("HOME DETECTED\n");
            getHome();

        } else if (strcmp(commands[i][0], "MAGNET") == 0) {
            printf("MAGNET DETECTED\n");
        }
    }

    // Output the parsed commands and parameters
    for (int i = 0; i < numCommands; ++i) {
        printf("{");
        for (int j = 0; j < numParams[i]; ++j) {
            printf("%s", commands[i][j]);
            if (j < numParams[i] - 1) {
                printf(", ");
            }
        }

    }
    printf("}\n");


    return 0;
}

void app_main(void) {

//    initNvs();
//    setupWifi();
//    startWebserver();
//    startBT();

    ESP_LOGI(TAG_RMT, "Initialize EN + DIR GPIO");
    gpio_config_t io_config = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = GPIO_OUTPUT_PIN_SEL
    };
    gpio_config(&io_config);

    io_config.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = 1;
    gpio_config(&io_config);

    setupRMT();

    char script1[] = "HOME\nMOVE N 4\nMOVE E 4\nMOVE W 3\nMOVE E 3";
    executeScript(script1);
}
