
#ifndef _DATATYPE_H_
#define _DATATYPE_H_

#include "stm32f1xx_hal.h"

typedef struct
{
	uint8_t byteRX;     /* byte nhan ve sau 1 lan ngat */
	uint8_t arrRX[110]; /* mang du lieu nhan duoc sau khi ket thuc chuoi */
	int8_t countRX;    /* bien dem so luong byte nhan duoc */
} RX_buffer;

typedef struct
{
	uint8_t arrLVT[30]; /* mang luu vi tri cac dau '['   ','   ']' */
	uint8_t countVT;    /* bien dem so luong phan tu cuar arrLVT */	
	int16_t value[25];
} RX_TYPE;

typedef struct
{
	uint16_t valueMCP;
	uint8_t arrVMCP[25];
	int16_t arrValueCom[25];
} MCP;

//
#endif
