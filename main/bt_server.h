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
    IDX_CHAR_MOTOR,
    IDX_CHAR_VAL_MOTOR,

    IDX_CHAR_BOARD,
    IDX_CHAR_VAL_BOARD,
    IDX_CHAR_CFG_BOARD,

    CHESS_IDX_NB,
};

void startBT();
void notifyBoard();

#endif //ESP32_BOARDCODE_BT_SERVER_H
