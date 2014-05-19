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

int dioda_2_priority=0;

xTaskHandle xHandle_3;
xTaskHandle xHandle_4;
xTaskHandle xHandle_5;

void GPIOInit(void);
void GPIOInit_PWM(void);
void TIM4_Init(void);
void TIM4_IRQHandler(void);
void diody(int x);
void RNG_Config(void);

void Dioda_1 (void * pvparameters);
void Dioda_2 (void * pvparameters);
void Dioda_3 (void * pvparameters);
void Dioda_4 (void * pvparameters);
void Dioda_5 (void * pvparameters);

int main(void)
{

	//#define xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask )

	/*vATask,		// pvTaskCode - the function that implements the task.
	"ATask",	// pcName - just a text name for the task to assist debugging.
	100,		// usStackDepth	- the stack size DEFINED IN WORDS.
	NULL,		// pvParameters - passed into the task function as the function parameters.
	( 1UL | portPRIVILEGE_BIT ),// uxPriority - task priority, set the portPRIVILEGE_BIT if the task should run in a privileged state.
	cStackBuffer,// puxStackBuffer - the buffer to be used as the task stack.

	void vTaskDelete( xTaskHandle xTaskToDelete ) PRIVILEGED_FUNCTION;

	======================================================

	 // Block for 500ms.
 const portTickType xDelay = 500 / portTICK_RATE_MS;

	 for( ;; )
	 {
		 // Simply toggle the LED every 500ms, blocking between each toggle.
		 vToggleLED();
		 vTaskDelay( xDelay );
	 }
 }

 	 ======================================================

 	  // Perform an action every 10 ticks.
 void vTaskFunction( void * pvParameters )
 {
 portTickType xLastWakeTime;
 const portTickType xFrequency = 10;

	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount ();
	 for( ;; )
	 {
		 // Wait for the next cycle.
		 vTaskDelayUntil( &xLastWakeTime, xFrequency );

		 // Perform action here.
	 }
 }
	void vTaskDelayUntil( portTickType * const pxPreviousWakeTime, portTickType xTimeIncrement ) PRIVILEGED_FUNCTION;

	==========================================================

	 void vAFunction( void )
 {
 xTaskHandle xHandle;

	 // Create a task, storing the handle.
	 xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );

	 // ...

	 // Use the handle to obtain the priority of the created task.
	 // It was created with tskIDLE_PRIORITY, but may have changed
	 // it itself.
	 if( uxTaskPriorityGet( xHandle ) != tskIDLE_PRIORITY )
	 {
		 // The task has changed it's priority.
	 }

	 // ...

	 // Is our priority higher than the created task?
	 if( uxTaskPriorityGet( xHandle ) < uxTaskPriorityGet( NULL ) )
	 {
		 // Our priority (obtained using NULL handle) is higher.
	 }
 }

 	 ======================================================

 	  void vAFunction( void )
 {
 xTaskHandle xHandle;

	 // Create a task, storing the handle.
	 xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );

	 // ...

	 // Use the handle to raise the priority of the created task.
	 vTaskPrioritySet( xHandle, tskIDLE_PRIORITY + 1 );

	 // ...

	 // Use a NULL handle to raise our priority to the same value.
	 vTaskPrioritySet( NULL, tskIDLE_PRIORITY + 1 );
 }

void vTaskPrioritySet( xTaskHandle xTask, unsigned portBASE_TYPE uxNewPriority ) PRIVILEGED_FUNCTION;

	===========================================================

	 void vAFunction( void )
 {
 xTaskHandle xHandle;

	 // Create a task, storing the handle.
	 xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );

	 // ...

	 // Use the handle to suspend the created task.
	 vTaskSuspend( xHandle );

	 // ...

	 // The created task will not run during this period, unless
	 // another task calls vTaskResume( xHandle ).

	 //...


	 // Suspend ourselves.
	 vTaskSuspend( NULL );

	 // We cannot get here unless another task calls vTaskResume
	 // with our handle as the parameter.
 }

	void vTaskSuspend( xTaskHandle xTaskToSuspend ) PRIVILEGED_FUNCTION;

	==============================================================

	 void vAFunction( void )
 {
 xTaskHandle xHandle;

	 // Create a task, storing the handle.
	 xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );

	 // ...

	 // Use the handle to suspend the created task.
	 vTaskSuspend( xHandle );

	 // ...

	 // The created task will not run during this period, unless
	 // another task calls vTaskResume( xHandle ).

	 //...


	 // Resume the suspended task ourselves.
	 vTaskResume( xHandle );

	 // The created task will once again get microcontroller processing
	 // time in accordance with it priority within the system.
 }

	void vTaskResume( xTaskHandle xTaskToResume ) PRIVILEGED_FUNCTION;

	==========================================================================
	*/

	SystemInit();
	SystemCoreClockUpdate();
	GPIOInit();
	RNG_Config();
	
	xTaskCreate( Dioda_1, ( signed char * ) "Dioda_1", 100, NULL, 2, NULL );

	xTaskCreate( Dioda_2, ( signed char * ) "Dioda_2", 100, NULL, 1, NULL );

	xTaskCreate( Dioda_3, ( signed char * ) "Dioda_3", 100, NULL, 2, &xHandle_3 );

	xTaskCreate( Dioda_4, ( signed char * ) "Dioda_4", 100, NULL, 2, &xHandle_4 );

	xTaskCreate( Dioda_5, ( signed char * ) "Dioda_5", 100, NULL, 2, &xHandle_5 );

	vTaskStartScheduler(); // This should never return.

	for( ;; );
}

void Dioda_1( void *pvparameters )
{
	const portTickType xDelay = 30 / portTICK_RATE_MS;
	unsigned int z=0;
	uint8_t wasPressed = 0;

		for (;;){

			while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1)
			{
				vTaskPrioritySet( NULL, tskIDLE_PRIORITY + 3 );
				GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				for (z=0;z<2000000;z++);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				for (z=0;z<2000000;z++);
			    wasPressed = 1;
			    vTaskSuspend( xHandle_3 );
			    vTaskSuspend( xHandle_4 );
			    vTaskSuspend( xHandle_5 );
			    dioda_2_priority=1;
			}

			if (wasPressed){
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	    	    wasPressed = 0;

			}

			//co 30 ms opozniany, zeby sprawdzic, czy jest wcisniecie guzika
			vTaskDelay(xDelay);
		}
}

void Dioda_2( void *pvparameters )
{
	const portTickType xDelay = 10 / portTICK_RATE_MS;
	const portTickType xDelay_2 = 200 / portTICK_RATE_MS;
	unsigned int i=0;

	while(1){

		if(dioda_2_priority==1)
		{
		for(i=0; i<5;i++)
		{
				GPIO_SetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_15);
				vTaskDelay( xDelay_2 );
				GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_15);
				vTaskDelay( xDelay_2 );
				GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_14);
				vTaskDelay( xDelay_2 );
				GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_14);
				vTaskDelay( xDelay_2 );
		}

		dioda_2_priority=0;
		vTaskResume( xHandle_3 );
		vTaskResume( xHandle_4 );
		vTaskResume( xHandle_5 );
		vTaskDelay( xDelay );

		}
	}
}

void Dioda_3( void *pvparameters )
{

	const portTickType xDelay = 1000 / portTICK_RATE_MS;
	unsigned int i=0;

	portTickType xLastWakeTime;
	 const portTickType xFrequency = 5000;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();

	while(1){

		 vTaskDelayUntil( &xLastWakeTime, xFrequency );

		for(i=0; i<5;i++)
		{
				GPIO_SetBits(GPIOD, GPIO_Pin_14);
				vTaskDelay( xDelay );
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				vTaskDelay( xDelay );

		}

		// Block for 3000ms.
		/*GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		 vTaskDelay( xDelay );
		 GPIO_ResetBits(GPIOD, GPIO_Pin_14);*/
		//vTaskPrioritySet( NULL, tskIDLE_PRIORITY + 1 );
	}
}

void Dioda_4( void *pvparameters )
{

	const portTickType xDelay = 100 / portTICK_RATE_MS;
	unsigned int i=0;
	while(1){

		GPIO_SetBits(GPIOD, GPIO_Pin_15);

		for(i=0; i<5;i++)
		{
				GPIO_SetBits(GPIOD, GPIO_Pin_15);
				vTaskDelay( xDelay );
				GPIO_ResetBits(GPIOD, GPIO_Pin_15);
				vTaskDelay( xDelay );
		}
	}
}

void Dioda_5( void *pvparameters )
{
	const portTickType xDelay = 250 / portTICK_RATE_MS;
	unsigned int i=0, counter=0;

	uint32_t losowa = 0;

	portTickType xLastWakeTime;
	const portTickType xFrequency = 10000;

	xLastWakeTime = xTaskGetTickCount ();

	while(1){

		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		if(counter==6)
		{
			vTaskDelete( xHandle_5 );
		}

		while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET)
		{

		}

		/* Get a 32bit Random number */
		losowa = RNG_GetRandomNumber();

		if(losowa>=25000)
		{
			vTaskSuspend( xHandle_3 );
			vTaskSuspend( xHandle_4 );

			for(i=0;i<8;i++)
			{
				diody(i%7);
				vTaskDelay( xDelay );
			}

			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

			vTaskResume( xHandle_3 );
			vTaskResume( xHandle_4 );

			counter++;
		}
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
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
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

void diody(int x)
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

	switch(x)
	{
	case 0:
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		break;
	case 1:
		GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_15);
		break;
	case 2:
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		break;
	case 3:
		GPIO_SetBits(GPIOD, GPIO_Pin_14 | GPIO_Pin_15);
		break;
	case 4:
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		break;
	case 5:
		GPIO_SetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14);
		break;
	case 6:
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		break;
	case 7:
		GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
		break;
	case 8:
			break;
	}
}

void RNG_Config(void)
{
	/* Enable RNG clock source */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

	/* RNG Peripheral enable */
	RNG_Cmd(ENABLE);
}

/*void TIM4_IRQHandler(void){

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
}*/

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
