#ifndef STRUKT_DANYCH_H
#define STRUKT_DANYCH_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
	uint8_t startByte;
    uint16_t posX;
    uint16_t posY;
    int16_t rotation;
    uint8_t checksum;
} UART_Packet;

#endif
