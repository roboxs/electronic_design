#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stm32f4xx.h>
#include <sys.h>

void bsp_gpio_init(void);

#define POWER_SHOOT_CMD PHout(3) //¿É¿Ø¹è
#define POWER_BARRERY_CMD PHout(5) //³äµç
#endif


