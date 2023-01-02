#include<FreeRTOS.h>
#include<semphr.h>
#include<task.h>
#include <stdint.h>
#include "stm32f4xx.h"
int count=0;
SemaphoreHandle_t xbinarysemaphore;
const TickType_t xDelay500ms = pdMS_TO_TICKS( 500UL );


void toggle_led1(void *s);
void switch_1(void *s);
void toggle_led2(void *s);
void Interrupt_config();
void  GPIO_config(); 
static void vTask( void *pvParameters )
{
	while(1)
	{
		xSemaphoreTake(xbinarysemaphore, portMAX_DELAY );
   GPIOA->ODR^=(1<<1);
  //for(count=0;count<=10000; count++);
 // GPIOA->BSRR|=(1<<17);	
	}
}
int main(void)
{
 Interrupt_config();
 GPIO_config(); 
	
xTaskCreate(toggle_led2,"led blinking1",256,0,1,0);
xbinarysemaphore=xSemaphoreCreateBinary();
	if( xbinarysemaphore != NULL )
 {
	 xTaskCreate(vTask, "Handler",256 , NULL, 3, NULL );
 }

  vTaskStartScheduler();
	
while(1);
}
void toggle_led1(void *s)
{
	while(1)
	{
	GPIOA->ODR|= (1<<1);
	//vTaskDelay(1000);
	//GPIOA->ODR &= ~(1<<1);	
	//vTaskDelay(1000);
}
	}
void toggle_led2(void *s)
{
	while(1)
	{
	GPIOD->ODR|= (1<<12);
	vTaskDelay(1000);
	GPIOD->ODR &= ~(1<<12);
  vTaskDelay(1000);		
}
	}

	
void GPIO_config()
{
	GPIOA->MODER &=0xFFFFFFFC;
	GPIOA->PUPDR &=~(1<<1);
	GPIOA->PUPDR |=(1<<0);
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER&=~(3<<1);
	GPIOA->MODER|=(1<<2);
	RCC->AHB1ENR |= (1<<3);
	GPIOD->MODER|=(1<<24);	
}

void Interrupt_config()
{
RCC->APB2ENR|=(1<<14);
SYSCFG->EXTICR[0]&=~(0xF<<0);
EXTI->IMR|=(1<<0);
EXTI->FTSR|=(1<<0);
NVIC_EnableIRQ(EXTI0_IRQn);
NVIC_SetPriority(EXTI0_IRQn, 10);	
}


void EXTI0_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken;
	EXTI->PR|=(1<<0);
	xSemaphoreGiveFromISR(xbinarysemaphore, &xHigherPriorityTaskWoken );
	
}