#ifndef _DRIVER_IIC_H
#define _DRIVER_IIC_H

#include <stm32f4xx.h>
#include <sys.h>


//IO��������
#define SDA_IN()  {GPIOA->MODER&=~(3<<(1*2));GPIOA->MODER|=0<<1*2;}	//PA1����ģʽ
#define SDA_OUT() {GPIOA->MODER&=~(3<<(1*2));GPIOA->MODER|=1<<1*2;} //PA1���ģʽ
//IO��������	 
#define IIC_SCL    PAout(0) //SCL	 S
#define IIC_SDA    PAout(1) //SDA	 T
#define READ_SDA   PAin(1)  //����SDA 


void IIC_Init(void);			 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

#endif 
