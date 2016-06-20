#ifndef _CRC16_
#define _CRC16_

#if defined (PARTICLE)
#include "application.h"
#endif

uint16_t _crc16_update(uint16_t crc, uint8_t a) { 
    int i; 
    
    crc ^= a; 
    for (i = 0; i < 8; ++i) { 
        if (crc & 1)  crc = (crc >> 1) ^ 0xA001; 
        else          crc = (crc >> 1); 
    } 
    return crc; 
}
#endif
