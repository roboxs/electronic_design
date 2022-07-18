#include <bsp_system.h>





void system_init(void)
{
	delay_init(180);
	bsp_gpio_init();
	bsp_usart_init();
	bsp_dma_init();
	bsp_time_init();
	bsp_can_init();
	bsp_nvic_init();
	
//	//24v 输出 依次上电
//    for (uint8_t i = 0; i < 3 ; i++)
//    {
//        GPIO_SetBits(GPIOH, GPIO_Pin_3 << i);
//        delay_us(700);
//    }
	delay_us(1000);
	GPIO_SetBits(GPIOH, GPIO_Pin_4);
}

