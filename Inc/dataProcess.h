
#ifndef _DATAPROCESS_H_
#define _DATAPROCESS_H_

#include "stm32f1xx_hal.h"
#include "dataType.h"

/*
 * @biref: viet gia tri dien tro vao cho kenh channel
 * @param: channel: kenh muon tac dong 
 * @param: datain: du lieu muon ghi vao
 * @retval: khong
 */
void SPISW_writeData(uint8_t channel, uint8_t datain);

/*
 * @biref: tach dau '['  ','  ']'
 * @param: *RX: con tro tro den cau truc nhan ( cua TABLE, cua DEBUG)
 * @param: *rxBuf : con tro tro den cau truc du lieu dem nhan ve qua UART
 * @retval: khong
 */
static void tachDau(RX_TYPE *RX, RX_buffer *rxBuf);

/*
 * @biref: tach chuoi tu chuoi nguon, luu vao chuoi dich
 * @param: *des: con tro tro den mang dich
 * @param: *src: con tro tro den mang dau vao
 * @param: begin: cat chuoi tu vi tri begin
 * @param: length: do dai muon cat
 * @retval: khong
 */
static void tachChuoi(uint8_t *des, uint8_t *src, uint8_t begin, uint8_t length);

/*
 * @biref: tach gia tri ra o giua cac cap dau '['  ','  ']'
 * @param: *RX: con tro tro den cau truc nhan ( cua TABLE, cua DEBUG)
 * @param: *rxBuf : con tro tro den cau truc du lieu dem nhan ve qua UART
 * @retval: khong
 */
void tachGiatri(RX_TYPE *RX, RX_buffer *rxBuf);

/*
 * @biref: lay gia tri kieu int16_t 
 * @retval: khong
 */
void layGiaTri(void);

#endif
