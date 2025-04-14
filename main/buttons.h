#pragma once

#include "driver/gpio.h"

typedef enum
{
    BUTTON_T1_UP_ID = 0,
    BUTTON_T1_DOWN_ID,
    BUTTON_T2_UP_ID,
    BUTTON_T2_DOWN_ID,
    BUTTON_COUNT
} button_id_t;

void buttons_init(void);