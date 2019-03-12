#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f0xx_hal.h"

#define BT_BUFFER_SIZE  250

extern uint8_t bt_recv_buffer[BT_BUFFER_SIZE];
extern short bt_recv_buffer_index;

void bt_send(uint8_t*data, uint16_t len);

#endif
