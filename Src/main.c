
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CLK_SET        HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET)
#define CLK_RESET      HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET)
 
#define SDO_SET        HAL_GPIO_WritePin(SPI_SDO_GPIO_Port, SPI_SDO_Pin, GPIO_PIN_SET)
#define SDO_RESET		   HAL_GPIO_WritePin(SPI_SDO_GPIO_Port, SPI_SDO_Pin, GPIO_PIN_RESET)

#define CS_TIN_SET     HAL_GPIO_WritePin(SPI_CS_TIN_GPIO_Port, SPI_CS_TIN_Pin, GPIO_PIN_SET)
#define CS_TIN_RESET   HAL_GPIO_WritePin(SPI_CS_TIN_GPIO_Port, SPI_CS_TIN_Pin, GPIO_PIN_RESET)

#define CS_TOUT_SET    HAL_GPIO_WritePin(SPI_CS_TOUT_GPIO_Port, SPI_CS_TOUT_Pin, GPIO_PIN_SET)
#define CS_TOUT_RESET  HAL_GPIO_WritePin(SPI_CS_TOUT_GPIO_Port, SPI_CS_TOUT_Pin, GPIO_PIN_RESET)

#define DEBUG_MODE_    HAL_GPIO_ReadPin(DEBUG_BT_GPIO_Port, DEBUG_BT_Pin)

#define READ_TANG      HAL_GPIO_ReadPin(TANG_GPIO_Port, TANG_Pin)
#define READ_GIAM    	 HAL_GPIO_ReadPin(GIAM_GPIO_Port, GIAM_Pin)

/* chon kenh TDS */
#define OUT_CHANNEL 1
#define IN_CHANNEL  0

/* lenh ghi du lieu vao MCP41010 */
#define WRITE 0x11

#define ON  1
#define OFF 0

#define NO  0
#define OK  1

#define EN  1
#define DIS 0

#define DEBUG_MODE 1
#define TABLE_MODE 0

#define GET_TABLE_CNO 	HAL_UART_Transmit(&huart1, (uint8_t*)"[ADC_TABLE,1]", 13, 100);
#define GET_TABLE_CNI 	HAL_UART_Transmit(&huart1, (uint8_t*)"[ADC_TABLE,0]", 13, 100);

#define DEBUG_ON				HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG_EN,1]", 12, 100);
#define DEBUG_OFF				HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG_EN,0]", 12, 100);
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct
{
	uint8_t byteRX;     /* byte nhan ve sau 1 lan ngat */
	uint8_t arrRX[110]; /* mang du lieu nhan duoc sau khi ket thuc chuoi */
	int8_t countRX;    /* bien dem so luong byte nhan duoc */
} RX_buffer;

RX_buffer rxBuffer_t;

typedef struct
{
	uint8_t arrLVT[30]; /* mang luu vi tri cac dau '['   ','   ']' */
	uint8_t countVT;    /* bien dem so luong phan tu cuar arrLVT */	
	
	int16_t value[25];
} RX_TYPE;

RX_TYPE rxTable_t, rxDebug_t;

uint8_t flag_mode  = TABLE_MODE;
uint8_t flag_table_done = NO;
uint8_t flag_debug_mode = OFF;
uint8_t flag_debug_done = NO;
uint16_t MIN = 0;
uint8_t save;
uint16_t valueMCP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


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

void tachDau(RX_TYPE *RX, RX_buffer *rxBuf)
{
	uint8_t *ptrLVT  = RX->arrLVT;
	uint8_t cnt;
	uint8_t countVT = 0;
	
	uint8_t *ptrBufferRX = rxBuf->arrRX;
	int8_t countRX = rxBuf->countRX;
	//count  = RX->countVT;
	
//	if(ptrBufferRX[0] == '[')
//	{
		//RX->countVT = 0;
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
//	}
}

/* destination : dich den */
void tachChuoi(uint8_t *des, uint8_t *src, uint8_t begin, uint8_t length)
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
void xoaBuffer()
{
	for(uint8_t i = 0; i < 110; i++) rxBuffer_t.arrRX[i] = '\0';
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
	{
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
	}
}

/* ngat nhan chi de doc ban tin du lieu co dinh dang [abcd,efgh,z] => countRX = 12 */
/* nhan het 1 ban tin thi dung lai khong doc nua */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		if(rxBuffer_t.byteRX == '[') 
		{
			//xoaBuffer();
			rxBuffer_t.countRX = 0;
			rxBuffer_t.arrRX[rxBuffer_t.countRX] = rxBuffer_t.byteRX;
			HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
		}

		else if(rxBuffer_t.byteRX != ']')
		{
			rxBuffer_t.countRX++;
			if(rxBuffer_t.countRX >= 110) rxBuffer_t.countRX = 0;
			rxBuffer_t.arrRX[rxBuffer_t.countRX] = rxBuffer_t.byteRX;
			HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
		}

		else if(rxBuffer_t.byteRX == ']') 
		{
			rxBuffer_t.countRX++; 
			if(rxBuffer_t.countRX >= 110) rxBuffer_t.countRX = 0;
			rxBuffer_t.arrRX[rxBuffer_t.countRX] = rxBuffer_t.byteRX; 
			if(flag_mode == DEBUG_MODE && flag_debug_done == NO)				
			{ 
				if(rxBuffer_t.arrRX[0] == '[') flag_debug_done = OK; 
				else HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);  // neu nhan het 1 chuoi ma ky tu dau khong phai '[' thi nhan lai
			}
			else if(flag_table_done == NO) flag_table_done = OK; 	  // chi nhan 1 lan ADC table  
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	/* cho phep nhan 1 ky tu roi nhay vao ngat */
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
	GET_TABLE_CNI;
	
	flag_mode = TABLE_MODE;
	/* cho cho den khi gui ve bang gia tri table ADC 1 lan, led nhay tan so 5Hz, neu nhan xong led nhay voi tan so 1Hz */
	while(flag_table_done == NO)
  {
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		HAL_Delay(200);
	}
	flag_debug_done = NO;
	tachGiatri(&rxTable_t, &rxBuffer_t);			
	
	//flag_debug_mode = EN;
	
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
	flag_mode = DEBUG_MODE;
	DEBUG_ON;          // gui lenh nhan gia tri debug
	while(flag_debug_done == NO)
  {
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		HAL_Delay(100);
	}  // bo qua chuoi trang thai debug tra ve [DEBUG_EN,0]
	flag_debug_done = NO;					
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
	//HAL_TIM_Base_Start_IT(&htim2);
//	while(flag_debug_done == NO)   // doc ve gia tri 2 kenh IN, OUT lan dau [xxxx,yyyy]
//	flag_debug_done = NO;				
//	tachGiatri(&rxDebug_t, &rxBuffer_t);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		/* lay bang gia tri 1 lan */
			
		/* cai tung gia tri dien tro roi doc ve gia tri ADC, so sanh neu bam sat gia tri luu trong bang ADC_TABLE thi dung */
//		for(valueMCP = 0; valueMCP < 256; valueMCP++)
//		{
//			SPISW_writeData(IN_CHANNEL, valueMCP);
//			xoaBuffer();
//			HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);				
//			while(flag_debug_done == NO)
//			{
//				HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
//				HAL_Delay(1000);
//			}
//			flag_debug_done = NO;							
//			tachGiatri(&rxDebug_t, &rxBuffer_t);					
//			if(abs(rxDebug_t.value[0] - rxTable_t.value[7]) <= 2) break;
//		}
//		
//		while(1)
//		{
//		}
		xoaBuffer();
		HAL_UART_Receive(&huart1, (uint8_t*)rxBuffer_t.arrRX, 15, 100);
	  //HAL_Delay(1000);
		//HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI_SDO_Pin|SPI_CLK_Pin|SPI_CS_TIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led1_Pin|SPI_CS_TOUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_SDO_Pin SPI_CLK_Pin SPI_CS_TIN_Pin */
  GPIO_InitStruct.Pin = SPI_SDO_Pin|SPI_CLK_Pin|SPI_CS_TIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_BT_Pin */
  GPIO_InitStruct.Pin = DEBUG_BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DEBUG_BT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GIAM_Pin TANG_Pin */
  GPIO_InitStruct.Pin = GIAM_Pin|TANG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led1_Pin SPI_CS_TOUT_Pin */
  GPIO_InitStruct.Pin = led1_Pin|SPI_CS_TOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
