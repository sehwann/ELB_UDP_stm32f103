/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "W5100S/w5100s.h"

#include "queue_t.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* CHIP SETTING */

#define W5100S_CS_PORT			GPIOB
#define W5100S_CS_PIN			GPIO_PIN_12


#define RESET_W5100S_GPIO_Port	GPIOB
#define RESET_W5100S_Pin		GPIO_PIN_8

/* ETH */
#define ETH_MAX_BUF_SIZE		2048

#define SOCKET_LOOP 0

#define PORT_LOOP   5000


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* NET */
wiz_NetInfo gWIZNETINFO = {
		.mac = {0x00, 0x08, 0xdc, 0x6f, 0x00, 0x8a},
				.ip = {192, 168, 11, 109},
				.sn = {255, 255, 255, 0},
				.gw = {192, 168, 11, 1},
				.dns = {8, 8, 8, 8},
		.dhcp = NETINFO_STATIC
};
uint8_t  destip[4] = {192, 168, 11, 2};
uint16_t destport = 8080;
uint16_t destport_rx;
uint8_t buf[ETH_MAX_BUF_SIZE] = {0,};
uint16_t destrx_size;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

static void csEnable(void);
static void csDisable(void);

void print_network_information();


/* UART */
int _write(int fd, char *str, int len)
{
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, 1);
	for(int i=0; i<len; i++)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)&str[i], 1, 0xFFFF);
	}
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, 0);
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

/* SPI CS CTRL */
static void csEnable(void)
{
	HAL_GPIO_WritePin(W5100S_CS_PORT, W5100S_CS_PIN, GPIO_PIN_RESET);
}

static void csDisable(void)
{
	HAL_GPIO_WritePin(W5100S_CS_PORT, W5100S_CS_PIN, GPIO_PIN_SET);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define MAX_BUFFER_SIZE 20
char U2buffer[MAX_BUFFER_SIZE];
uint32_t U2buffer_length = 0;
Queue_t gQueue;

uint8_t u2_recv;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(&huart2, &u2_recv, 1);
        Node* node = node_new(&u2_recv, sizeof(u2_recv));
		queue_enqueue(&gQueue, node);
    }
}

typedef struct {
    uint32_t rx_stx[2], rx_id[3], rx_mode[2], \
             rx_v[4], rx_c[4], rx_leak_curr[3], rx_status[3],\
             rx_etx[4];
    uint32_t parsing_id, parsing_mode, parsing_v, \
             parsing_c, power, insulation;

    uint32_t elb_mode[10], elb_status[10], elb_vol[10], \
             elb_load_curr[10], elb_leak_curr[10], elb_power[10], \
             elb_insulation[10];

    uint32_t elb_door;
    uint32_t elb_mode_str[50], elb_status_str[50], \
             elb_vol_str[50], elb_load_curr_str[50], elb_leak_curr_str[50],\
             elb_power_str[50], elb_insulation_str[50];

    uint8_t elb_RE_json[2048];
} parseData;


void parse_send_data()
{
//	uint32_t rx_stx[2], rx_id[3], rx_mode[2], \
//		         rx_v[4], rx_c[4], rx_leak_curr[3], rx_status[3],\
//				 rx_etx[4];
//	uint32_t parsing_id, parsing_mode, parsing_v, \
//				 parsing_c, power, insulation;
//
//	uint32_t elb_mode[10], elb_status[10], elb_vol[10], \
//		         elb_load_curr[10], elb_leak_curr[10], elb_power[10], \
//				 elb_insulation[10];
//
//	uint32_t elb_door = 0;
//	uint32_t elb_mode_str[50], elb_status_str[50], \
//				elb_vol_str[50], elb_load_curr_str[50], elb_leak_curr_str[50],\
//				elb_power_str[50], elb_insulation_str[50];

	parseData data = {0};
#if 0
	setIMR(0b00000000);
	sendto(SOCKET_LOOP, (uint8_t *)U2buffer, strlen(U2buffer), destip, destport);
	char bsiz_str[10];
	uint32_t bsiz = strlen(U2buffer);
	sprintf(bsiz_str, "%u", bsiz);
	sendto(SOCKET_LOOP, (uint8_t *)bsiz_str, strlen(bsiz_str), destip, destport);	// 19
	sendto(SOCKET_LOOP, &U2buffer[0], sizeof(U2buffer[0]), destip, destport);		// (hex)0A
	sendto(SOCKET_LOOP, &U2buffer[1], sizeof(U2buffer[1]), destip, destport);		// S
	sendto(SOCKET_LOOP, &U2buffer[2], sizeof(U2buffer[2]), destip, destport);		// 0
	sendto(SOCKET_LOOP, &U2buffer[15], sizeof(U2buffer[15]), destip, destport);		// X
	sendto(SOCKET_LOOP, &U2buffer[16], sizeof(U2buffer[16]), destip, destport);		// X
	sendto(SOCKET_LOOP, &U2buffer[17], sizeof(U2buffer[17]), destip, destport);		// X
	sendto(SOCKET_LOOP, &U2buffer[18], sizeof(U2buffer[18]), destip, destport);		// E
	setIMR(0b00000001);
#endif
#if 1
	memmove(&U2buffer[0], &U2buffer[1], U2buffer_length - 2);
	U2buffer_length-=2;
	if (U2buffer_length == 17 && U2buffer[0] == 'S' && U2buffer[16] == 'X')
	{
		sscanf((char *)U2buffer, "%1s%2s%1s%3s%3s%2s%2s%3s",
				   (char *) data.rx_stx, (char *) data.rx_id, (char *) data.rx_mode, \
				   (char *) data.rx_v, (char *) data.rx_c, (char *) data.rx_leak_curr, \
				   (char *) data.rx_status, (char *) data.rx_etx);

		data.parsing_id = (uint32_t)atoi((char *) data.rx_id);
		//	 parsing_mode = (uint32_t)atoi((char *) rx_mode);
		data.parsing_v = (uint32_t)atoi((char *) data.rx_v);
		data.parsing_c = (uint32_t)atoi((char *) data.rx_c);

		data.power = (uint32_t)(data.parsing_v * data.parsing_c);
		if (data.parsing_c != 0)
		{
			data.insulation = (uint32_t)(data.parsing_v / (float)data.parsing_c);
		}
		else
		{
			// 예외처리추가해야 함.
			printf("error : c = 0 \n");
			data.insulation = 1;
		}
		if ( data.parsing_id > 0 &&  data.parsing_id <= 10) {
			data.elb_mode[ data.parsing_id - 1] = (uint32_t)atoi((char *) data.rx_mode);
			data.elb_status[ data.parsing_id - 1] = (uint32_t)atoi((char *) data.rx_status);
			data.elb_vol[ data.parsing_id - 1] =  data.parsing_v;
			data.elb_load_curr[ data.parsing_id - 1] =  data.parsing_c;
			data.elb_leak_curr[ data.parsing_id - 1] = (uint32_t)atoi((char *) data.rx_leak_curr);
			data.elb_power[ data.parsing_id - 1] =  data.power;
			data.elb_insulation[ data.parsing_id - 1] =  data.insulation;
		}
		sprintf((char *) data.elb_mode_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",  data.elb_mode[0],  data.elb_mode[1],  data.elb_mode[2],  data.elb_mode[3],  data.elb_mode[4],  data.elb_mode[5],  data.elb_mode[6],  data.elb_mode[7],  data.elb_mode[8],  data.elb_mode[9]);
		//door
		sprintf((char *) data.elb_status_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",  data.elb_status[0],  data.elb_status[1],  data.elb_status[2],  data.elb_status[3],  data.elb_status[4],  data.elb_status[5],  data.elb_status[6],  data.elb_status[7],  data.elb_status[8],  data.elb_status[9]);
		sprintf((char *) data.elb_vol_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",  data.elb_vol[0],  data.elb_vol[1],  data.elb_vol[2],  data.elb_vol[3],  data.elb_vol[4],  data.elb_vol[5],  data.elb_vol[6],  data.elb_vol[7],  data.elb_vol[8],  data.elb_vol[9]);
		sprintf((char *) data.elb_load_curr_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",  data.elb_load_curr[0],  data.elb_load_curr[1],  data.elb_load_curr[2],  data.elb_load_curr[3],  data.elb_load_curr[4],  data.elb_load_curr[5],  data.elb_load_curr[6],  data.elb_load_curr[7],  data.elb_load_curr[8],  data.elb_load_curr[9]);
		sprintf((char *) data.elb_leak_curr_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",  data.elb_leak_curr[0],  data.elb_leak_curr[1],  data.elb_leak_curr[2],  data.elb_leak_curr[3],  data.elb_leak_curr[4],  data.elb_leak_curr[5],  data.elb_leak_curr[6],  data.elb_leak_curr[7],  data.elb_leak_curr[8],  data.elb_leak_curr[9]);
		sprintf((char *) data.elb_power_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",  data.elb_power[0],  data.elb_power[1],  data.elb_power[2],  data.elb_power[3],  data.elb_power[4],  data.elb_power[5],  data.elb_power[6],  data.elb_power[7],  data.elb_power[8],  data.elb_power[9]);
		sprintf((char *) data.elb_insulation_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",  data.elb_insulation[0],  data.elb_insulation[1],  data.elb_insulation[2],  data.elb_insulation[3],  data.elb_insulation[4],  data.elb_insulation[5],  data.elb_insulation[6],  data.elb_insulation[7],  data.elb_insulation[8],  data.elb_insulation[9]);
		sprintf((char *) data.elb_RE_json, "{\"mod_id\":%ld,\"mod_door\":%ld,\"elb_mode\":%s,\"elb_status\":%s,\"elb_volt\":%s,\"elb_load_curr\":%s,\"elb_leak_curr\":%s,\"elb_power\":%s,\"elb_insulation\":%s}",  data.parsing_id,  data.elb_door, (char *) data.elb_mode_str, (char *) data.elb_status_str, (char *) data.elb_vol_str, (char *) data.elb_load_curr_str, (char *) data.elb_leak_curr_str,(char *) data.elb_power_str, (char *) data.elb_insulation_str);

		setIMR(0b00000000);
		sendto(SOCKET_LOOP, (uint8_t *)data.elb_RE_json, strlen(data.elb_RE_json), destip, destport);
		setIMR(0b00000001);
	}
	else {
		U2buffer_length = 0;
	}
#endif
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	uint8_t RE_DATA_SEND[10] = "S01Re01xxE";

	if(htim->Instance==TIM2)		// 100ms
	{

	}
	if(htim->Instance==TIM3)		// 1s
	{
		HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, 1);
		HAL_UART_Transmit(&huart2, (uint8_t *)RE_DATA_SEND, strlen((char *)RE_DATA_SEND), 10);
		HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, 0);
	}

	if(htim->Instance==TIM4)		// 1s
	{
		if (U2buffer_length > 0)
		{
			parse_send_data();
			U2buffer_length = 0;
		}
	}
}

#if 1
uint32_t udp_rx_data_id;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int32_t len;
	// Co DATA recvfrom server
	if(GPIO_Pin == W5100S_INT_Pin)
	{
		len = recvfrom(SOCKET_LOOP, buf, &destrx_size, destip, &destport_rx);
		if (len != 10) {
			memset(buf,0,sizeof(buf));
			setSn_IR(SOCKET_LOOP, 0xff);		// Clear flag
			return;
		}
		if ((buf[0] != 'S') || (buf[9] != 'E')){
			memset(buf,0,sizeof(buf));
			setSn_IR(SOCKET_LOOP, 0xff);		// Clear flag
			return;
		}
		udp_rx_data_id = (buf[2]%16);	// elb_mode check id

		// Co DATA Sendto 485
		HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, 1);
		HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 10);
		HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, 0);
	}
	setSn_IR(SOCKET_LOOP, 0xff);		// Clear flag

}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


#if 1
	reg_wizchip_cs_cbfunc(csEnable,csDisable);// CS function register
	reg_wizchip_spi_cbfunc(spiReadByte, spiWriteByte);// SPI method callback registration
	wizchip_initialize();
	ctlnetwork(CN_SET_NETINFO, &gWIZNETINFO);
	print_network_information();
	setMR2(0b01000000);			// MR2[6] IEN Bit enable INTn
	setIMR(0b00000001);			// IMR[0]S0_INT Bit enable SOCKET 0 Interrupt
	socket(SOCKET_LOOP, Sn_MR_UDP, PORT_LOOP, 0x00);

	HAL_UART_Receive_IT(&huart2, &u2_recv, 1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
#endif

	queue_new(&gQueue, MAX_BUFFER_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while (gQueue.qsize > 0 && U2buffer_length < MAX_BUFFER_SIZE)
	  {
			Node queued_node = queue_dequeue(&gQueue);
			int node_length = strlen(queued_node.str);

			if (U2buffer_length + node_length < MAX_BUFFER_SIZE) {
				memcpy(U2buffer + U2buffer_length, queued_node.str, node_length);
				U2buffer_length += node_length;
			}
			else {
				break;
			}
	 }






//	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|SPI2_CS_Pin|W5100S_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DE_Pin */
  GPIO_InitStruct.Pin = DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin SPI2_CS_Pin W5100S_RST_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|SPI2_CS_Pin|W5100S_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : W5100S_INT_Pin */
  GPIO_InitStruct.Pin = W5100S_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(W5100S_INT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
