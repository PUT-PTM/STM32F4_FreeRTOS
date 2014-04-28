#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"

uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
uint64_t tickTime=0;        // Counts OS ticks (default = 1000Hz).
int current=0, s=0;

void GPIOInit(void);
void GPIOInit_PWM(void);
void TIM4_Init(void);
void TIM4_IRQHandler(void);

void Diody_PWM (void * pvparameters);
void Dioda (void * pvparameters);

int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();


	GPIO_SetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_13);

	xTaskCreate( Diody_PWM, ( signed char * ) "Diody PWM", 100, NULL, 1, NULL );
	
	xTaskCreate( Dioda, ( signed char * ) "Dioda", 100, NULL, 1, NULL );

	vTaskStartScheduler(); // This should never return.

	    while(1)
	    {
	    }
	    return 1;
}

/*void vButtonTask( void *pvparameters )
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
}*/

void Diody_PWM( void *pvparameters )
{
	GPIOInit_PWM();
	TIM4_Init();
}

void Dioda( void *pvparameters )
{
	GPIOInit();
	unsigned int z=0;
		while(1){
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			for (z=0;z<10000;z++);
			GPIO_ResetBits(GPIOD, GPIO_Pin_14);
			for (z=0;z<10000;z++);
		}
}

//********************************INICJALIZACJE*********************************//

void GPIOInit(void){

		/* GPIOD Periph clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure;

		/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
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
}

void GPIOInit_PWM(void){

		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(GPIOD,&GPIO_InitStructure);
}

void TIM4_Init(void){

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 40;
	TIM_TimeBaseStructure.TIM_Prescaler = 100;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

	//PWM
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM4,ENABLE);


	int delay;
	for(delay=0;delay<100000;delay++){
		asm("nop");
	}
}

void TIM4_IRQHandler(void){

		if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
		{
			uint8_t wasPressed = 0;

			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1)
			{
				TIM4->CCR1 = 40;
				TIM4->CCR2 = 40;
				wasPressed = 1;
			}

			if (wasPressed){

				TIM4->CCR1 = 5;
				TIM4->CCR2 = 10;
				wasPressed = 0;
			}

			vTaskDelay(30 / portTICK_RATE_MS);

			TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		}
}

/*void TIM4_IRQHandler(void){

		if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
		{
			while(current!=40)
			{
				TIM4->CCR1 = current;
				TIM4->CCR2 = current;
				for(s=0; s<=200000; s++);
				current++;
			}

			while(current!=0)
			{
					TIM4->CCR1 = current;
					TIM4->CCR2 = current;
					for(s=0; s<=200000; s++);
					current--;
			}
			TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		}
}*/

//******************************************************************************//

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
