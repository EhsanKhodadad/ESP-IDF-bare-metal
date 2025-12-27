/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_rom_sys.h"

static inline void delay_ms(uint32_t ms)
{
    esp_rom_delay_us((int)ms * 1000);
}

void app_main(void)
{
    printf("Hello world!\n");

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        delay_ms(1000);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
