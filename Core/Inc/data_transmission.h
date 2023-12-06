#ifndef __data_transmission_H__
#define __data_transmission_H__
#include "global.h"


void UDP_RECEIVE_DATA();
// UDP RECEIVE(PC->MCU)
void udpReceive(uint8_t socket, uint8_t* buffer, uint16_t* destRxSize, uint8_t* destIp, uint16_t* destPortRx);


// UDP RE DATA SEND(MCU->PC)
void RE_CO_DATA_UDP_SEND();

void CO_DATA_485_SEND();
void CO_DATA_485_SEND_TO();
void RE_DATA_485_SEND();
void RE_DATA_485_SEND_TO();

#endif
