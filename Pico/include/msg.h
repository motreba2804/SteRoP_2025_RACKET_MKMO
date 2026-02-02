#ifndef MSG_H
#define MSG_H
#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t startByte;   
    uint16_t posX;       
    uint16_t posY;       
    int16_t rotation;    
    uint8_t checksum;    
} UART_Packet;

uint16_t map_to_u16(float val) {
    if (val < -1.0f) val = -1.0f;
    if (val > 1.0f) val = 1.0f;
    return (uint16_t)(((val + 1.0f)/2.0f) * 65535.0f);
}



#endif
