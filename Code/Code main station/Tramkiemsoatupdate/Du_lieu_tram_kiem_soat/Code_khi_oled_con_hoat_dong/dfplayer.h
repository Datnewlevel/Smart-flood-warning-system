#ifndef DFPLAYER_H
#define DFPLAYER_H

#include <stdint.h>
#include "esp_err.h"

#define DFPLAYER_TXD 17
#define DFPLAYER_RXD 18
#define UART_NUM UART_NUM_1

esp_err_t dfplayer_init(void);
void dfplayer_set_volume(uint8_t volume);
void dfplayer_play(uint8_t track);
void dfplayer_deinit(void);

#endif