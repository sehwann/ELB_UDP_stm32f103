#ifndef __udp_configuration_H__
#define __udp_configuration_H__
#include "global.h"

/* CHIP SETTING */
#define W5100S_CS_PIN			GPIO_PIN_7
#define W5100S_CS_PORT			GPIOD

#define RESET_W5100S_GPIO_Port	GPIOD
#define RESET_W5100S_Pin		GPIO_PIN_8

/* ETH */
#define ETH_MAX_BUF_SIZE		2048

#define SOCKET_LOOP 2

#define PORT_LOOP   5000

extern wiz_NetInfo gWIZNETINFO;
extern uint8_t destip[4];
extern uint16_t destport;
extern uint16_t destport_rx;


void csEnable(void);
void csDisable(void);

int _write(int fd, char *str, int len);
void wizchip_reset();
void wizchip_check(void);
void wizchip_initialize(void);
void print_network_information();
uint8_t spiReadByte(void);
void spiWriteByte(uint8_t writeByte);
uint8_t spiDmaReadByte(void);
void spiDmaWriteByte(uint8_t writeByte);
void spiReadBurst(uint8_t* pBuf, uint16_t len);
void spiWriteBurst(uint8_t* pBuf, uint16_t len);

void wizchip_start(void);

#endif
