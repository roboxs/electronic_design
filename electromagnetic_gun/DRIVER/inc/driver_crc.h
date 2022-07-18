#ifndef _CRC_H
#define _CRC_H
#include <stdio.h>

typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;


#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);

uint16_t get_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);


#endif

