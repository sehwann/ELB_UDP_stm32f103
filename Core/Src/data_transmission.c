#include "data_transmission.h"

extern int32_t FLAG_RE_CO;

extern int32_t FLAG_UDP_RX_CO_DATA;

extern int32_t FLAG_UDP_RE_SEND;
extern int32_t FLAG_UDP_CO_SEND;
extern uint8_t elb_RE_json[2048];

// RE DATA
uint8_t RE_DATA_SEND[10] = "S00Re01xxE";
extern int32_t ELB_ID; 				// elb id (0~9)
extern UART_HandleTypeDef huart1;

extern uint8_t buf[];

extern uint16_t destrx_size;


extern uint8_t udp_rx_co_buf[10];
extern uint32_t udp_rx_data_id;
extern uint8_t udp_rx_data_Co[2];
extern uint32_t udp_rx_co_buf_size;


extern uint8_t TIM_co_rx_buffer[20];

extern int32_t FLAG_TIM_RE_DATA_1S_SEND;

extern TIM_HandleTypeDef htim3;


//extern uint32_t parsing_mode;
extern uint32_t elb_mode[10];
//extern uint32_t parsing_id;

void UDP_RECEIVE_DATA()
{
	 udpReceive(SOCKET_LOOP, buf, &destrx_size, destip, &destport_rx);
}

// UDP RECEIVE(PC->MCU)
void udpReceive(uint8_t socket, uint8_t* buffer, uint16_t* destRxSize, uint8_t* destIp, uint16_t* destPortRx)
{
    if ((*destRxSize = getSn_RX_RSR(socket)) > 0)
    {
    	recvfrom(socket, buffer, (size_t)*destRxSize, destIp, destPortRx);
        memcpy((uint8_t*)udp_rx_co_buf, buffer, udp_rx_co_buf_size);

        if (udp_rx_co_buf_size == UDP_CO_RECEIVE_DATA_SIZE)
        {
        	if (udp_rx_co_buf[0] == 'S' &&
				udp_rx_co_buf[1] == '0' &&
				udp_rx_co_buf[3] == 'C' &&
				udp_rx_co_buf[4] == 'o' &&
				udp_rx_co_buf[7] == 'x' &&
				udp_rx_co_buf[8] == 'x' &&
				udp_rx_co_buf[9] == 'E')
			{
        		FLAG_UDP_RX_CO_DATA = READY;
                FLAG_RE_CO = CO;
				// 버퍼에 저장 >>  할 이유 x?, 전달만 하면 되지않을까?
				udp_rx_data_id = (udp_rx_co_buf[2]%16);	// 수동모드 확인id

			}
        	else	// 형식에 맞지 않는 데이터가 들어왔을 때
        	{
        		FLAG_RE_CO = RE;
        		FLAG_UDP_RX_CO_DATA = UNREADY;
				memset(udp_rx_co_buf, 0, UDP_CO_RECEIVE_DATA_SIZE);
				return;
			}
        }

        else {		// 데이터 사이즈 부터 다르면,
        	FLAG_RE_CO = RE;
        	return;	//구문 빠져나가기
        }
        memset(buf, 0, ETH_MAX_BUF_SIZE);
    }
}

// UDP RE DATA SEND(MCU->PC)
void RE_CO_DATA_UDP_SEND()
{
	if (FLAG_TIM_RE_DATA_1S_SEND == 1)		// 1s마다 송신
	{
		sendto(SOCKET_LOOP, (uint8_t *)elb_RE_json, strlen((const char *)elb_RE_json), destip, destport);
		FLAG_TIM_RE_DATA_1S_SEND = 0;
	}

	if (FLAG_UDP_RE_SEND == SEND_SET)
	{
		sendto(SOCKET_LOOP, (uint8_t *)elb_RE_json, strlen((const char *)elb_RE_json), destip, destport);
		FLAG_UDP_RE_SEND = SEND_RESET;	// FLAG_UDP_RE_SEND RESET
	}

	if (FLAG_UDP_CO_SEND == SEND_SET)
	{
		sendto(SOCKET_LOOP, (uint8_t *)TIM_co_rx_buffer, strlen((const char *)TIM_co_rx_buffer), destip, destport);
		FLAG_UDP_CO_SEND = SEND_RESET;
		FLAG_RE_CO = RE;
	}
}





void CO_DATA_485_SEND()
{
	if (elb_mode[udp_rx_data_id -1] == MANUAL)		// mode가 1(수동모드)일 때는 Co데이터 전송 x
	{
		printf("error : Manual mode.(1)\n");
		FLAG_RE_CO = RE;
	}
	else if (elb_mode[udp_rx_data_id-1] == AUTOMATIC)
	{
		printf("AUTOMATIC mode(0).\n");
		// 시간경과하면 RE상태로 가는 것 추가
			CO_DATA_485_SEND_TO();				// ex)S01Co11xxE
	}
}

void CO_DATA_485_SEND_TO()
{
	if (FLAG_UDP_RX_CO_DATA==READY)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)udp_rx_co_buf, UDP_CO_RECEIVE_DATA_SIZE, 10);
		FLAG_UDP_RX_CO_DATA = UNREADY;
	}
}

void RE_DATA_485_SEND()
{
	HAL_TIM_Base_Start(&htim3);		// 100ms, ELB_RE_DATA_485_SEND
}

void RE_DATA_485_SEND_TO()
{
	if (ELB_ID >= 10) {
	  ELB_ID = 0;
	}

	else if (ELB_ID < 10) {
	  RE_DATA_SEND[1] = (ELB_ID + 1) / 10 + '0';
	  RE_DATA_SEND[2] = (ELB_ID + 1) % 10 + '0';
	  HAL_UART_Transmit(&huart1, (uint8_t *)RE_DATA_SEND, strlen((char *)RE_DATA_SEND), 10);
	  ELB_ID++;
	}
	HAL_TIM_Base_Stop(&htim3);
}

/**/
