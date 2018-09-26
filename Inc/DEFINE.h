
#ifndef _DEFINE_H_
#define _DEFINE_H_

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

#endif
