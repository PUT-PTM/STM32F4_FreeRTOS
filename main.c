#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"

uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
uint64_t tickTime=0;        // Counts OS ticks (default = 1000Hz).

void vButtonTask (void * pvparameters);
void xButtonTask (void * pvparameters);
void yButtonTask (void * pvparameters);

int main(void)
{
	SystemInit();
		 /* GPIOD Periph clock enable */
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		 GPIO_InitTypeDef GPIO_InitStructure;

		 /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		 GPIO_Init(GPIOD, &GPIO_InitStructure);

		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
			  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			  GPIO_Init(GPIOA, &GPIO_InitStructure);

	xTaskCreate( vButtonTask, ( signed char * ) "Button Task", 100, NULL, 1, NULL );

	xTaskCreate( xButtonTask, ( signed char * ) "Button Task", 100, NULL, 1, NULL );
	
	xTaskCreate( yButtonTask, ( signed char * ) "Button Task", 100, NULL, 1, NULL );

	vTaskStartScheduler(); // This should never return.

	    while(1)
	    {
	    }
	    return 1;


}

void vButtonTask( void *pvparameters )
{

	uint8_t wasPressed = 0;

	for (;;){

		while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
		    wasPressed = 1;
		}

		if (wasPressed){
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    	    wasPressed = 0;
		}

		vTaskDelay(30 / portTICK_RATE_MS);
	}
}

void xButtonTask( void *pvparameters )
{
	unsigned int z=0;
	for (;;){
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		for (z=0;z<10000000;z++);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		for (z=0;z<10000000;z++);
	}
}

void yButtonTask( void *pvparameters )
{
	unsigned int z=0;
	for (;;){
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		for (z=0;z<1000000;z++);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		for (z=0;z<1000000;z++);
	}
}

// This FreeRTOS callback function gets called once per tick (default = 1000Hz).
// ----------------------------------------------------------------------------
void vApplicationTickHook( void ) {
    ++tickTime;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ----------------------------------------------------------------------------
void vApplicationIdleHook( void ) {
    ++u64IdleTicksCnt;
}

// A required FreeRTOS function.
// ----------------------------------------------------------------------------
void vApplicationMallocFailedHook( void ) {
    configASSERT( 0 );  // Latch on any failure / error.
}
