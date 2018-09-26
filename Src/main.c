
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
#include "dataType.h"
#include "DEFINE.h"
#include "dataProcess.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


RX_buffer rxBuffer_t;

RX_TYPE rxTable_t, rxDebug_t;

MCP mcp1;

uint8_t flag_mode  = TABLE_MODE;
uint8_t flag_table_done = NO;
uint8_t flag_debug_mode = OFF;
uint8_t flag_debug_done = NO;


uint8_t save; // luu vi tri gia tri dien tro
int16_t arrTest[10]; // luu cac gia tri de doi den gia tri UART on dinh
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
	
	// LAY BANG GIA TRI ADC_TABLE KENH IN BAT DAU O DAY
	GET_TABLE_CNI;
	flag_mode = TABLE_MODE;
	while(flag_table_done == NO)        /* cho cho den khi gui ve bang gia tri table ADC 1 lan, led nhay tan so 5Hz, neu nhan xong led nhay voi tan so 10Hz */
  {
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		HAL_Delay(200);
	}
	flag_debug_done = NO;
	tachGiatri(&rxTable_t, &rxBuffer_t);			
	// KET THUC LAY GIA TRI ADC_TABLE, DU LIEU DUOC LUU VAO MANG rxTable_t.value

	/*
	 *
	 */
	
	// BAT DAU LAY DU LIEU DEBUG O DAY, BO QUA DU LIEU XAC NHAN TRA VE lAN DAU
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
	flag_mode = DEBUG_MODE;
	DEBUG_ON;        									 /* gui lenh nhan gia tri debug */
	while(flag_table_done == NO)
  {
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		HAL_Delay(100);
	}  // bo qua chuoi trang thai debug tra ve [DEBUG_EN,0]
	flag_debug_done = NO;	
	// KET THUC NHAN DU LIEU XAC NHAN 
	
	/*
	 *
	 */			
	
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer_t.byteRX, 1);
	mcp1.valueMCP = 1;
	
	/* 2 bien tam luu vi tri va gia tri truoc khi gia tri do - gia tri trong bang < 0 */
	uint8_t temp1;
	int16_t temp2;
	
	/* lay ra 10 gia tri dau de doi cho gia tri on dinh */
	SPISW_writeData(IN_CHANNEL, mcp1.valueMCP);
	for(uint8_t x = 0; x < 5; x++)
	{
		layGiaTri();
		arrTest[x] = rxDebug_t.value[IN_CHANNEL];
	}				
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */	
		for(save = 2; save < rxTable_t.countVT; save++)																												/* cai tung gia tri dien tro roi doc ve gia tri ADC, so sanh neu bam sat gia tri luu trong bang ADC_TABLE thi dung */
		{
			if(rxTable_t.value[save] < rxDebug_t.value[IN_CHANNEL])
			{
				for(mcp1.valueMCP = mcp1.valueMCP; mcp1.valueMCP < 256; mcp1.valueMCP++)
				{
					SPISW_writeData(IN_CHANNEL, mcp1.valueMCP);
					layGiaTri();
					if((rxDebug_t.value[IN_CHANNEL] - rxTable_t.value[save]) < 0) 																							/* so sanh gia tri nhan ve va gia tri trong bang */
					{
						if((rxTable_t.value[save] - rxDebug_t.value[IN_CHANNEL]) < (temp1 - rxTable_t.value[save]))
						{
							mcp1.arrVMCP[save] = 	mcp1.valueMCP;
							mcp1.arrValueCom[save] = rxDebug_t.value[IN_CHANNEL];
						}
						
						else
						{
							mcp1.arrVMCP[save] = 	temp1;
							mcp1.arrValueCom[save] = temp2;
						}
						break;
					}
					
					else
					{
						temp1 = mcp1.valueMCP;
						temp2 = rxDebug_t.value[0];
					}
				}
			}
		}

		/* sau khi set xong gia tri thi vao vonf lap nhay led tan so 1Hz */
		while(1)
		{
				HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
				HAL_Delay(1000);
		}
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
