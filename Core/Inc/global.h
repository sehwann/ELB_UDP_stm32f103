#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "socket.h"
#include "wizchip_conf.h"
#include "W5100S/w5100s.h"
#include "udp_configuration.h"
#include "data_transmission.h"

#define RE 0 		// UART TRANS RE
#define CO 1 		// UART TRANS CO
#define SEND_SET 1
#define SEND_RESET 0

#define RE_RECIVE_DATA_CNT 15	// RE DATA(17) - S0(2)
#define CO_RECIVE_DATA_CNT 6	// CO DATA(8) - S0(2)

#define UDP_CO_RECEIVE_DATA_SIZE 10


#define AUTOMATIC  0
#define MANUAL  1

#define READY 1
#define UNREADY 0

#endif
