/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "global.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


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

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


int32_t FLAG_RE_CO; 			// RE, CO
int32_t FLAG_UDP_RE_SEND;		// UDP RE SEND
int32_t FLAG_UDP_CO_SEND;		// UDP CO SEND
int32_t ELB_ID; 				// elb id (0~9)

int32_t FLAG_UDP_RX_CO_DATA;

int32_t FLAG_TIM_RE_DATA_1S_SEND;

// INT_UART
uint8_t rx_data;

uint8_t rx_buffer[20];						// [RE_RECIVE_DATA_CNT + 2]	안전하게 더 증가시킴이 필요
uint32_t rx_index = 0;
uint32_t data_count = 0;

// RE TIM RX DATA
uint8_t TIM_rx_buffer[20];
uint32_t TIM_rx_index = 0;


uint8_t co_rx_buffer[20];
uint32_t co_rx_index = 0;
uint32_t co_data_count = 0;
// CO TIM RX DATA
uint8_t TIM_co_rx_buffer[20];
uint32_t TIM_co_rx_index = 0;


typedef enum {
	RE_IDLE = 0,
	RE_HEADER,
	RE_RECEIVING_DATA
} STATE;
STATE RE_state;

typedef enum {
	CO_IDLE = 0,
	CO_HEADER,
	CO_RECEIVING_DATA
} CO_STATE;
CO_STATE CO_state;


// parsing data
uint32_t rx_stx[2], rx_id[3], rx_mode[2], rx_v[4], rx_c[4], rx_leak_curr[3], rx_status[3], rx_etx[4];

uint32_t elb_mode[10] = {0};
uint32_t elb_status[10] = {0};
uint32_t elb_vol[10] = {0};
uint32_t elb_load_curr[10] = {0};
uint32_t elb_leak_curr[10] = {0};
uint32_t elb_power[10] = {0};
uint32_t elb_insulation[10] = {0};


uint32_t elb_door = 1;
uint32_t elb_mode_str[50], elb_status_str[50], \
		elb_vol_str[50], elb_load_curr_str[50], elb_leak_curr_str[50],\
		elb_power_str[50], elb_insulation_str[50];
uint8_t elb_RE_json[2048];


uint32_t parsing_id;
uint32_t parsing_mode;
uint32_t parsing_v;
uint32_t parsing_c;
uint32_t power;
uint32_t insulation;


// UDP_RECEIVE
uint8_t buf[ETH_MAX_BUF_SIZE] = {0,};
uint16_t destrx_size;





// UDP CO RECEIVE
uint8_t udp_rx_co_buf[10];
uint32_t udp_rx_data_id;
uint8_t udp_rx_data_Co[2];
uint32_t udp_rx_co_buf_size = UDP_CO_RECEIVE_DATA_SIZE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

#define DATA_SIZE 17	// RE DATA SIZE // CO data size 추가해야 함
#define NUM_START 3
#define NUM_END 14
#define FLAG_START 15
#define FLAG_END 17

#define CO_DATA_SIZE 8	// CO DATA SIZE
#define CO_NUM_START 3
#define CO_NUM_END 14
#define CO_FLAG_START 15
#define CO_FLAG_END 17

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




void parse_and_update_data(uint32_t parsing_id)
{
    // 데이터 파싱
    sscanf((char *)TIM_rx_buffer, "%1s%2s%1s%3s%3s%2s%2s%3s",
           (char *)rx_stx, (char *)rx_id, (char *)rx_mode, (char *)rx_v, (char *)rx_c, (char *)rx_leak_curr, (char *)rx_status, (char *)rx_etx);

    parsing_id = (uint32_t)atoi((char *)rx_id);
    parsing_mode = (uint32_t)atoi((char *)rx_mode);
    parsing_v = (uint32_t)atoi((char *)rx_v);
    parsing_c = (uint32_t)atoi((char *)rx_c);


    // 계산
    power = (uint32_t)(parsing_v * parsing_c);
    if (parsing_c != 0)
    {
		insulation = (uint32_t)(parsing_v / parsing_c);
	}
    else
    {
		// 예외처리추가
		printf("error : c = 0 \n");
		insulation = 1;
	}

    // 각 ID의 버퍼 업데이트
    if (parsing_id > 0 && parsing_id <= 10) {
        elb_mode[parsing_id - 1] = (uint32_t)atoi((char *)rx_mode);
        elb_status[parsing_id - 1] = (uint32_t)atoi((char *)rx_status);
        elb_vol[parsing_id - 1] = parsing_v;
        elb_load_curr[parsing_id - 1] = parsing_c;
        elb_leak_curr[parsing_id - 1] = (uint32_t)atoi((char *)rx_leak_curr);
        elb_power[parsing_id - 1] = power;
        elb_insulation[parsing_id - 1] = insulation;
    }

    sprintf((char *)elb_mode_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]", elb_mode[0], elb_mode[1], elb_mode[2], elb_mode[3], elb_mode[4], elb_mode[5], elb_mode[6], elb_mode[7], elb_mode[8], elb_mode[9]);
	//door
	sprintf((char *)elb_status_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]", elb_status[0], elb_status[1], elb_status[2], elb_status[3], elb_status[4], elb_status[5], elb_status[6], elb_status[7], elb_status[8], elb_status[9]);
	sprintf((char *)elb_vol_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]", elb_vol[0], elb_vol[1], elb_vol[2], elb_vol[3], elb_vol[4], elb_vol[5], elb_vol[6], elb_vol[7], elb_vol[8], elb_vol[9]);
	sprintf((char *)elb_load_curr_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]", elb_load_curr[0], elb_load_curr[1], elb_load_curr[2], elb_load_curr[3], elb_load_curr[4], elb_load_curr[5], elb_load_curr[6], elb_load_curr[7], elb_load_curr[8], elb_load_curr[9]);
	sprintf((char *)elb_leak_curr_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]", elb_leak_curr[0], elb_leak_curr[1], elb_leak_curr[2], elb_leak_curr[3], elb_leak_curr[4], elb_leak_curr[5], elb_leak_curr[6], elb_leak_curr[7], elb_leak_curr[8], elb_leak_curr[9]);
	sprintf((char *)elb_power_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]", elb_power[0], elb_power[1], elb_power[2], elb_power[3], elb_power[4], elb_power[5], elb_power[6], elb_power[7], elb_power[8], elb_power[9]);
	sprintf((char *)elb_insulation_str, "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]", elb_insulation[0], elb_insulation[1], elb_insulation[2], elb_insulation[3], elb_insulation[4], elb_insulation[5], elb_insulation[6], elb_insulation[7], elb_insulation[8], elb_insulation[9]);

	sprintf((char *)elb_RE_json, "{\"mod_id\":%ld,\"mod_door\":%ld,\"elb_mode\":%s,\"elb_status\":%s,\"elb_volt\":%s,\"elb_load_curr\":%s,\"elb_leak_curr\":%s,\"elb_power\":%s,\"elb_insulation\":%s}", parsing_id, elb_door, (char *)elb_mode_str, (char *)elb_status_str, (char *)elb_vol_str, (char *)elb_load_curr_str, (char *)elb_leak_curr_str,(char *)elb_power_str, (char *)elb_insulation_str);


	// flag 설정
	FLAG_UDP_RE_SEND = SEND_SET;
	//printf("%s\n", (char *)elb_RE_json);
//	sendto(SOCKET_LOOP, (uint8_t *)elb_RE_json, strlen((const char *)elb_RE_json), destip, destport);

}


/* MCU-> SERVER */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	static uint32_t tim7_count = 0;
	int32_t DATA_COUNT;

	if(htim->Instance==TIM3)		// 100ms
	{
		RE_DATA_485_SEND_TO();
	}

	if(htim->Instance==TIM6)		// 100ms
	{
//		HAL_UART_Transmit(&huart1, Re_data, sizeof(Re_data)-1, 10);
		HAL_TIM_Base_Stop(&htim6);

		// FLAG_RE_CO = RE?���? ?��?��
		if (FLAG_RE_CO == RE)
		{
			// rx_index 17�?
			if(TIM_rx_index == DATA_SIZE)
			{
				TIM_rx_index = 0;

				// 3번째부터 14번째 데이터는  숫자, 3번째(id)는 0이 아니어야함
				if(TIM_rx_buffer[NUM_START-1] != 0x30)
				{
					for(DATA_COUNT = NUM_START-1; DATA_COUNT <= NUM_END-1; DATA_COUNT++)
					{

						if(TIM_rx_buffer[DATA_COUNT] < 0x30 || TIM_rx_buffer[DATA_COUNT] > 0x39)
						{
							// ?��?���? ?��?���? break;
							printf("error : Data(Number) exceeded.\n");
	//						memset(rx_buffer, 0, sizeof(rx_buffer));
	//						memset(TIM_rx_buffer, 0, sizeof(TIM_rx_buffer));
	//						rx_index = 0;
	//						data_count = 0;
	//						RE_state = RE_IDLE;
							break;
						}
					}

					// 15번째부터 17번째 데이터 'XXX'확인
					if(strncmp((char *)&TIM_rx_buffer[FLAG_START-1], "XXX", FLAG_END - FLAG_START + 1) == 0)
					{
						printf("%s\n", TIM_rx_buffer);	// ?��?��?���? 출력
						// parsing
						parse_and_update_data((uint32_t)atoi((char *)rx_id));

					}
				}
				return;
			}
			else
			{
				memset(rx_buffer, 0, sizeof(rx_buffer));
				memset(TIM_rx_buffer, 0, sizeof(TIM_rx_buffer));
				rx_index = 0;
				data_count = 0;
				RE_state = RE_IDLE;
			}
		}

		if (FLAG_RE_CO == CO)
		{
			//
			if(TIM_co_rx_index == CO_DATA_SIZE)
			{
				TIM_co_rx_index = 0;
				// 3번째 데이터가 0~9 사이의 숫자인지 확인(ID)
				if(TIM_co_rx_buffer[2] >= '0' && TIM_co_rx_buffer[2] <= '9')
				{
					// 6,7번째 데이터가 3,3 또는 4,4인지 확인
					if((TIM_co_rx_buffer[5] == '3' && TIM_co_rx_buffer[6] == '3') ||
					   (TIM_co_rx_buffer[5] == '4' && TIM_co_rx_buffer[6] == '4'))
					{
						// 4,5,8번째 데이터가 각각 'C', 'o', 'E'인지 확인
						if(TIM_co_rx_buffer[3] == 'C' && TIM_co_rx_buffer[4] == 'o' && TIM_co_rx_buffer[7] == 'E')
						{
							// 모든 조건이 맞으면, 받은 데이터를 그대로 출력
							FLAG_UDP_CO_SEND = SEND_SET;
//							printf("%s\n", TIM_co_rx_buffer);

						}
						else
						{
							printf("error : It's not correct data.(C,o,E)\n");
							FLAG_RE_CO = RE;
						}
					}
					else
					{
						printf("error : It's not correct data.(33,44)\n");
						FLAG_RE_CO = RE;
					}
				}
			}
			else
			{
				memset(co_rx_buffer, 0, sizeof(co_rx_buffer));
				memset(TIM_co_rx_buffer, 0, sizeof(TIM_co_rx_buffer));
				co_rx_index = 0;
				co_data_count = 0;
				CO_state = CO_IDLE;
				FLAG_RE_CO = RE;
			}
		}
	}

	else if(htim->Instance==TIM7) {		//1s
//		tim7_count++;
//		if(tim7_count % 10 == 0) {

			// 여기가 UDP 송신 (1~9의 데이터)
			FLAG_TIM_RE_DATA_1S_SEND = 1;


//			tim7_count = 0;
//		}

	}
}

/* ELB -> MCU */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);

        if (FLAG_RE_CO == RE)				// FLAG_RE_CO check
	    {
        	switch(RE_state)
			{
				case RE_IDLE: {
					if(rx_data == 'S') // 첫 번째 문자 'S'를 확인
					{
						rx_buffer[rx_index++] = rx_data;
						RE_state = RE_HEADER; // ?��?�� ?��?���? ?��?��
					}
				} break;

				case RE_HEADER: {
					if(rx_data == '0') // 두 번째 문자 '0'를 확인
					{
						rx_buffer[rx_index++] = rx_data;
						RE_state = RE_RECEIVING_DATA; // 데이터 수신 상태로 전환
						HAL_TIM_Base_Start(&htim6);
						data_count = 0; // 데이터 카운트 초기화
					}
					else
					{
						// '0'이 아니면 버퍼와 인덱스초기화하고 IDLE 상태로 전환
						memset(rx_buffer, 0, sizeof(rx_buffer));
						rx_index = 0;
						RE_state = RE_IDLE;
					}
				} break;

				case RE_RECEIVING_DATA: {
					rx_buffer[rx_index++] = rx_data; // 데이터를 버퍼에 저장
					data_count++;

					if(data_count == RE_RECIVE_DATA_CNT) // 정해진 데이터 개수를 받았는지 확인(15)
					{
						TIM_rx_index = rx_index;
						memcpy(TIM_rx_buffer, rx_buffer, strlen((char *)rx_buffer));
	//					printf("%s\n", TIM_rx_buffer); // 데이터 출력

						// 버퍼와 인덱스를 초기화하고 IDLE 상태로
						memset(rx_buffer, 0, sizeof(rx_buffer));
						rx_index = 0;
						data_count = 0;
						RE_state = RE_IDLE;
					}
				} break;
				default:
					break;
			}
	    }


        // UART_RECEIVE_CO_DATA
        if (FLAG_RE_CO == CO)
		{
        	switch(CO_state)
			{
				case CO_IDLE: {
					if(rx_data == 'S') // 첫 번째 문자 'S'를 확인
					{
						co_rx_buffer[co_rx_index++] = rx_data;
						CO_state = CO_HEADER; // ?��?�� ?��?���? ?��?��
					}
				} break;

				case CO_HEADER: {
					if(rx_data == '0') // 두 번째 문자 '0'를 확인
					{
						co_rx_buffer[co_rx_index++] = rx_data;
						CO_state = CO_RECEIVING_DATA; // 데이터 수신 상태로 전환
						HAL_TIM_Base_Start(&htim6);
						co_data_count = 0; // 데이터 카운트 초기화
					}
					else
					{
						// '0'이 아니면 버퍼와 인덱스초기화하고 IDLE 상태로 전환
						memset(co_rx_buffer, 0, sizeof(co_rx_buffer));
						co_rx_index = 0;
						CO_state = CO_IDLE;
					}
				} break;

				case CO_RECEIVING_DATA: {
					co_rx_buffer[co_rx_index++] = rx_data; // 데이터를 버퍼에 저장
					co_data_count++;

					if(co_data_count == CO_RECIVE_DATA_CNT) // 정해진 데이터 개수를 받았는지 확인(15)
					{
						TIM_co_rx_index = co_rx_index;
						memcpy(TIM_co_rx_buffer, co_rx_buffer, strlen((char *)co_rx_buffer));
	//					printf("%s\n", TIM_co_rx_buffer); // 데이터 출력

						// 버퍼와 인덱스를 초기화하고 IDLE 상태로
						memset(co_rx_buffer, 0, sizeof(co_rx_buffer));
						co_rx_index = 0;
						co_data_count = 0;
						CO_state = CO_IDLE;
					}
				} break;
				default:

					break;
			}

		}

    }
}


void interrupt_setup()
{
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Stop(&htim6);
	HAL_TIM_Base_Stop(&htim3);
}


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
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  wizchip_start();
  interrupt_setup();
  socket(SOCKET_LOOP, Sn_MR_UDP, PORT_LOOP, 0x00);



  // init 따로 함수 만들기
  RE_state = RE_IDLE;
  FLAG_RE_CO = RE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (FLAG_RE_CO == RE)	{			// FLAG_RE_CO check
		  RE_DATA_485_SEND();			// ex) S01Re01xxE
	  }

	  if (FLAG_RE_CO == CO)	{			// CO명령어를 받은 후, 그대로 전송
		  CO_DATA_485_SEND();
	  }
	 // UDP RE,CO DATA SEND(MCU->PC)
	 RE_CO_DATA_UDP_SEND();

	 // UDP RECEIVE(PC->MCU)
	 UDP_RECEIVE_DATA();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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
  htim3.Init.Prescaler = 7200-1;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7200-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD8 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
