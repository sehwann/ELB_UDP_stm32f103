#include "udp_configuration.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern UART_HandleTypeDef huart1;

uint8_t ethBuf[ETH_MAX_BUF_SIZE];


/* NET */
wiz_NetInfo gWIZNETINFO = {
		.mac = {0x00, 0x08, 0xdc, 0x6f, 0x00, 0x8a},
				.ip = {192, 168, 11, 109},
				.sn = {255, 255, 255, 0},
				.gw = {192, 168, 11, 1},
				.dns = {8, 8, 8, 8},
		.dhcp = NETINFO_STATIC
};

/* PCNET */
uint8_t  destip[4] = {192, 168, 11, 2};
uint16_t destport = 8080;
uint16_t destport_rx;



void wizchip_start(void)
{
	  reg_wizchip_cs_cbfunc(csEnable,csDisable);// CS function register
	  reg_wizchip_spi_cbfunc(spiReadByte, spiWriteByte);// SPI method callback registration

	  wizchip_initialize();

	  ctlnetwork(CN_SET_NETINFO, &gWIZNETINFO);
	  printf("--------STATIC Net Information------------\r\n");
	  print_network_information();

	  socket(SOCKET_LOOP, Sn_MR_UDP, PORT_LOOP, 0x00);
}


/* SPI CS CTRL */
void csEnable(void)
{
	HAL_GPIO_WritePin(W5100S_CS_PORT, W5100S_CS_PIN, GPIO_PIN_RESET);
}

void csDisable(void)
{
	HAL_GPIO_WritePin(W5100S_CS_PORT, W5100S_CS_PIN, GPIO_PIN_SET);
}

/* UART */
int _write(int fd, char *str, int len)
{
	for(int i=0; i<len; i++)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&str[i], 1, 0xFFFF);
	}
	return len;
}

/* CHIP INIT */
void wizchip_reset()
{
	HAL_GPIO_WritePin(RESET_W5100S_GPIO_Port, RESET_W5100S_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RESET_W5100S_GPIO_Port, RESET_W5100S_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}


void wizchip_check(void)
{
    /* Read version register */
    if (getVER() != 0x51) // W5100S
    {
        printf(" ACCESS ERR : VERSIONR != 0x51, read value = 0x%02x\n", getVER());
        while (1);
    }
}

void wizchip_initialize(void)
{
	uint8_t W5100S_AdrSet[2][4]= {{2,2,2,2},{2,2,2,2}};
    uint8_t tmp1, tmp2;
	intr_kind temp= IK_DEST_UNREACH;

	wizchip_reset();
	if (ctlwizchip(CW_INIT_WIZCHIP, (void*)W5100S_AdrSet) == -1)
		printf(">>>>W5100s memory initialization failed\r\n");

	if(ctlwizchip(CW_SET_INTRMASK,&temp) == -1)
		printf("W5100S interrupt\r\n");

	wizchip_check();
	while(1)
	{
		ctlwizchip(CW_GET_PHYLINK, &tmp1 );
		ctlwizchip(CW_GET_PHYLINK, &tmp2 );
		if(tmp1==PHY_LINK_ON && tmp2==PHY_LINK_ON) break;
	}
}

void print_network_information()
{
	wizchip_getnetinfo(&gWIZNETINFO);

	printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
	printf("IP address : %d.%d.%d.%d\n\r",gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
	printf("SM Mask	   : %d.%d.%d.%d\n\r",gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
	printf("Gate way   : %d.%d.%d.%d\n\r",gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
	printf("DNS Server : %d.%d.%d.%d\n\r",gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
}


/* SPI */
uint8_t spiReadByte(void)
{
	uint8_t readByte=0;
	uint8_t writeByte=0xFF;

	while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(&hspi2, &writeByte, &readByte, 1, 10);

	return readByte;
}

void spiWriteByte(uint8_t writeByte)
{
	uint8_t readByte=0;

	while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(&hspi2, &writeByte, &readByte, 1, 10);
}

/* SPI DMA */
uint8_t spiDmaReadByte(void)
{
	uint8_t readByte=0;

	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY &&
		  HAL_DMA_GetState(hspi2.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi2.hdmatx) != HAL_DMA_STATE_READY);

	HAL_SPI_Receive_DMA(&hspi2, &readByte, 1);

	while (HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_RESET);
	while (HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_RESET);

	return readByte;

}

void spiDmaWriteByte(uint8_t writeByte)
{

	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY &&
		  HAL_DMA_GetState(hspi2.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi2.hdmatx) != HAL_DMA_STATE_READY);

	HAL_SPI_Transmit_DMA(&hspi2, &writeByte, 1);

	while (HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_RESET);
	while (HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_RESET);

	return;
}

void spiReadBurst(uint8_t* pBuf, uint16_t len)
{
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY &&
		  HAL_DMA_GetState(hspi2.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi2.hdmatx) != HAL_DMA_STATE_READY);

	HAL_SPI_Receive_DMA(&hspi2, pBuf, len);

	while (HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_RESET);
	while (HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_RESET);

	return;
}

void spiWriteBurst(uint8_t* pBuf, uint16_t len)
{
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY &&
		  HAL_DMA_GetState(hspi2.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi2.hdmatx) != HAL_DMA_STATE_READY);

	HAL_SPI_Transmit_DMA(&hspi2, pBuf, len);

	while (HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmarx) == HAL_DMA_STATE_RESET);
	while (HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi2.hdmatx) == HAL_DMA_STATE_RESET);

	return;
}
