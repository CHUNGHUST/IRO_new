
#include "dataProcess.h"
#include "dataType.h"
#include "DEFINE.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

extern RX_buffer rxBuffer_t;
extern RX_TYPE rxTable_t, rxDebug_t;
extern MCP mcp1;
extern uint8_t flag_debug_done;

extern UART_HandleTypeDef huart1;

/* Rab = 10k, Rw = 73
 * Raw = Rab*(256-data)/256 + Rw
 * Rwb = Rab*data/256 + Rw
 */
void SPISW_writeData(uint8_t channel, uint8_t datain)
{
	/* init bus */
	CS_TIN_SET;
	CS_TOUT_SET;
	CLK_RESET;
	SDO_RESET;
	/* count 16 bit : 8bit command and 8bit data */
	uint8_t cnt;
	/* 2byte temp = byte com + byte data */
	uint16_t temp;
	temp = (WRITE<<8) | datain;

	if(channel == IN_CHANNEL)  CS_TIN_RESET;
	else 	if(channel == OUT_CHANNEL) CS_TOUT_RESET;
						
	/* SPI shift 16bit */
	for(cnt = 0; cnt < 16; cnt++)
	{
		CLK_RESET;
		(temp & 0x8000) ? SDO_SET : SDO_RESET;
		CLK_SET;
		temp <<= 1;
	}
	
	CLK_RESET;
	CS_TIN_SET;
	CS_TOUT_SET;
}

static void tachDau(RX_TYPE *RX, RX_buffer *rxBuf)
{
	uint8_t *ptrLVT  = RX->arrLVT;
	uint8_t cnt;
	uint8_t countVT = 0;
	
	uint8_t *ptrBufferRX = rxBuf->arrRX;
	int8_t countRX = rxBuf->countRX;

	ptrLVT[countVT] = 0;  /* vi tri dau '[' */
	for(cnt = 1; cnt < countRX; cnt++)
	{
		if(ptrBufferRX[cnt] == ',')
		{
			countVT++;
			ptrLVT[countVT] = cnt;
		}
	}
	countVT++;
	RX->countVT = countVT;
	ptrLVT[countVT] = countRX; /* vi tri dau ']' */
	ptrLVT[countVT+1] = 0xFF; /* thong bao ket thuc mang LVT */
}

/* destination : dich den */
static void tachChuoi(uint8_t *des, uint8_t *src, uint8_t begin, uint8_t length)
{
	memcpy(des, (const char*)(src+begin), length);
	des[length] = '\0';
}

/* tach tu mang rx buffer cac gia tri luu vao value */
void tachGiatri(RX_TYPE *RX, RX_buffer *rxBuf)
{
	
	tachDau(RX, rxBuf);
	
	uint8_t *ptrLVT = RX->arrLVT;
	int8_t countVT = RX->countVT;
	int16_t *ptrValue = RX->value;
	
	uint8_t *ptrBufferRX = rxBuf->arrRX;	
	uint8_t buf[15];

	for(uint8_t cnt = 0; cnt < countVT; cnt++)
	{
		tachChuoi(buf, ptrBufferRX, ptrLVT[cnt]+1, ptrLVT[cnt+1] - ptrLVT[cnt] - 1);
		ptrValue[cnt] = atoi((const char*)buf);
	}
}

void layGiaTri()
{
	rxBuffer_t.arrRX[0] = '\0';
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);				
	while(flag_debug_done == NO) // cho den khi nhan duoc chuoi ban tin moi [asdadsa]
	{
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		HAL_Delay(20);
	}
	flag_debug_done = NO;							
	tachGiatri(&rxDebug_t, &rxBuffer_t);
}
