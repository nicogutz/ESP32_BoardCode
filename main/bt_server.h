//
// Created by kiran on 3/2/24.
//

#ifndef ESP32_BOARDCODE_BT_SERVER_H
#define ESP32_BOARDCODE_BT_SERVER_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    HRS_IDX_NB,
};

void startBT();

#endif //ESP32_BOARDCODE_BT_SERVER_H
