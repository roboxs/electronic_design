#include <task_led.h>
#include <delay.h>
#include <stdio.h>



void led1_task(void *pvParameters)
{
	  while(1)
    {
        LED1=!LED1;
        vTaskDelay(1000);
    }
}

void led2_task(void *pvParameters)
{
    while(1)
    {
        LED2=!LED2;
       vTaskDelay(500);
    }
}
