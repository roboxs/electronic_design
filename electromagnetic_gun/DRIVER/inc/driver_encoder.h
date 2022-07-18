#ifndef _DRIVER_ENCODER_H
#define _DRIVER_ENCODER_H

#include <stm32f4xx.h>

//将编码器的值转化成角度值
#define ENCODER_ECD_TO_DEG  (0.04395067757294591624954218044195f)
#define MOTOR_ECD_TO_DEG	(0.00122085215480405322915394945672f)
typedef struct
{
	uint16_t ecd;
	uint16_t last_ecd;

	int16_t  speed_rpm;
	int16_t  given_current;
	
	int32_t  round_cnt;
	int32_t  total_ecd;
	float    total_angle;
	float    current_angle;
	
	uint16_t offset_ecd;
	int32_t  ecd_current_value;
	u32 count;
}MotoMeasure_t;



void encoder_data_handler(MotoMeasure_t* encoder, CanRxMsg * rcan);
void get_moto_offset(MotoMeasure_t* ptr, CanRxMsg* rcan);

#endif
