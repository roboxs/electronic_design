#include <driver_encoder.h>

//电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

void encoder_data_handler(MotoMeasure_t* encoder, CanRxMsg * rcan)
{
	encoder->last_ecd = encoder->ecd;
	encoder->ecd      = (uint16_t)((rcan->Data[0]<<8) | rcan->Data[1]);//电调反馈的编码器值
  
	if (encoder->ecd - encoder->last_ecd > 4096)
	{
		encoder->round_cnt--;
	}
	else if (encoder->ecd - encoder->last_ecd < -4096)
	{
		encoder->round_cnt++;
	}
	
	encoder->current_angle	= encoder->ecd * ENCODER_ECD_TO_DEG;//电机角度的当前值
	encoder->total_ecd 		= encoder->round_cnt * 8192 + encoder->ecd;//电机总的编码器值
	encoder->total_angle 	= encoder->total_ecd * ENCODER_ECD_TO_DEG;//电机角度总值
	encoder->speed_rpm     = (int16_t)((rcan->Data[2]<<8) | rcan->Data[3]);
	encoder->given_current = (int16_t)((rcan->Data[4]<<8) | rcan->Data[5]);
}

void get_moto_offset(MotoMeasure_t* ptr, CanRxMsg* rcan)
{
    ptr->ecd        = (uint16_t)(rcan->Data[0] << 8 | rcan->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}
