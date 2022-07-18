#ifndef _TASK_LED_H
#define _TASK_LED_H

#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <delay.h>
#include <sys.h>

#define LED1 PGout(1)
#define LED2 PGout(2)
#define LED3 PGout(3)
#define LED4 PGout(4)
#define LED5 PGout(5)
#define LED6 PGout(6)
#define LED7 PGout(7)
#define LED8 PGout(8)


void led1_task(void *pvParameters);
void led2_task(void *pvParameters);

#endif

