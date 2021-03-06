#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/stm32f4_discovery_lis302dl.h"

uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
uint64_t tickTime=0;        // Counts OS ticks (default = 1000Hz).

int dioda_2_priority=0;

// Uchwyty do zada�
xTaskHandle xHandle_1;
xTaskHandle xHandle_3;
xTaskHandle xHandle_4;
xTaskHandle xHandle_5;
xTaskHandle xHandle_6;

uint32_t LIS302DL_TIMEOUT_UserCallback()
{
	return -1;
}

int8_t acc_z;

void GPIOInit(void);
void diody(int x);
void RNG_Config(void);
void SPIInit(void);

void Przycisk_1 (void * pvparameters);
void Dioda_2 (void * pvparameters);
void Dioda_3 (void * pvparameters);
void Dioda_4 (void * pvparameters);
void Dioda_Random_5 (void * pvparameters);
void Akcelerometr_6 (void * pvparameters);

int main(void)
{
	// Inicjalizacje
	SystemInit();
	SystemCoreClockUpdate();
	GPIOInit();
	SPIInit();
	RNG_Config();
	
	// Tworzenie zada�
	// xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask )

	xTaskCreate( Przycisk_1, ( signed char * ) "Dioda_1", 100, NULL, 2, &xHandle_1 );

	xTaskCreate( Dioda_2, ( signed char * ) "Dioda_2", 100, NULL, 1, NULL );

	xTaskCreate( Dioda_3, ( signed char * ) "Dioda_3", 100, NULL, 3 | portPRIVILEGE_BIT, &xHandle_3 );

	xTaskCreate( Dioda_4, ( signed char * ) "Dioda_4", 100, NULL, 3 | portPRIVILEGE_BIT, &xHandle_4 );

	xTaskCreate( Dioda_Random_5, ( signed char * ) "Dioda_5", 100, NULL, 2, &xHandle_5 );

	xTaskCreate( Akcelerometr_6, ( signed char * ) "Dioda_6", 100, NULL, 1, &xHandle_6 );

	//Uruchomienie planisty
	vTaskStartScheduler();

	for( ;; );
}

//*******************************PROCEDURY*ZADA�********************************//

void Przycisk_1( void *pvparameters )
{
	//Wci�ni�cie przycisku powoduje mryganie diody, a nast�pnie uruchomienie zadania Dioda_2

	const portTickType xDelay = 30 / portTICK_RATE_MS;
	unsigned int z=0;
	uint8_t wasPressed = 0;

	while(1)
	{
			while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1)
			{
				//Zwi�kszenie priorytetu zadania Przycisk_1
				vTaskPrioritySet( xHandle_1, tskIDLE_PRIORITY + 2 );
				GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				for (z=0;z<2000000;z++);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				for (z=0;z<2000000;z++);
			    wasPressed = 1;

			    //Zawieszenie zada�
			    vTaskSuspend( xHandle_3 );
			    vTaskSuspend( xHandle_4 );
			    vTaskSuspend( xHandle_5 );

			    dioda_2_priority=1;
			}

			if (wasPressed)
			{
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	    	    wasPressed = 0;
			}

			//Op�nienie sprawdzaj�ce co 30[ms], czy przycisk zosta� wci�ni�ty
			vTaskDelay(xDelay);
		}
}

void Dioda_2( void *pvparameters )
{
	//Sekwencja di�d wy�wietlana po wci�ni�ciu przycisku

	const portTickType xDelay_3 = 1 / portTICK_RATE_MS;
	const portTickType xDelay_2 = 200 / portTICK_RATE_MS;
	const portTickType xDelay = 1000 / portTICK_RATE_MS;
	unsigned int i=0;

	while(1)
	{
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

			//Przywr�cenie zawieszonych zada�
			vTaskResume( xHandle_3 );
			vTaskResume( xHandle_4 );
			vTaskDelay( xDelay );
			vTaskResume( xHandle_5 );
		}

		vTaskDelay( xDelay_3 );
	}
}

void Dioda_3( void *pvparameters )
{
	//Zadanie mrygaj�ce diod� co sekund�

	const portTickType xDelay = 1000 / portTICK_RATE_MS;
	unsigned int i=0;

	portTickType xLastWakeTime;
	const portTickType xFrequency = 5000;
	xLastWakeTime = xTaskGetTickCount ();

	while(1)
	{
		//Op�nienie startu zadania o 5[s]
		 vTaskDelayUntil( &xLastWakeTime, xFrequency );

		for(i=0; i<5;i++)
		{
				GPIO_SetBits(GPIOD, GPIO_Pin_14);
				vTaskDelay( xDelay );
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				vTaskDelay( xDelay );
		}
	}
}

void Dioda_4( void *pvparameters )
{
	//Zadanie mrygaj�ce diodi� co 100[ms]

	const portTickType xDelay = 100 / portTICK_RATE_MS;
	unsigned int i=0;
	while(1)
	{
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

void Dioda_Random_5( void *pvparameters )
{
	//Zadanie losowo mrygaj�ce diodami okre�lon� sekwencj�

	const portTickType xDelay = 300 / portTICK_RATE_MS;
	unsigned int i=0, counter=0;

	uint32_t losowa = 0;

	portTickType xLastWakeTime;
	const portTickType xFrequency = 10000;

	xLastWakeTime = xTaskGetTickCount ();

	while(1)
	{
		//Op�nienie startu zadania o 10[s]
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		//Usuni�cie zadania, gdy licznik osi�gnie okre�lon� warto��
		if(counter==6)
		{
			vTaskDelete( xHandle_5 );
		}

		while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET)
		{

		}

		//Generowanie 32bitowej losowej liczby ca�kowitej
		losowa = RNG_GetRandomNumber();
		losowa/=100000;

		if(losowa>=25000)
		{
			//Zawieszenie zada�
			vTaskSuspend( xHandle_3 );
			vTaskSuspend( xHandle_4 );

			for(i=0;i<8;i++)
			{
				diody(i%7);
				vTaskDelay( xDelay );
			}

			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

			//Przywr�cenie zawieszonych zada�
			vTaskResume( xHandle_3 );
			vTaskResume( xHandle_4 );

			counter++;
		}
	}
}

void Akcelerometr_6( void *pvparameters )
{
	// Zadanie mrygaj�ce diodami sekwencj�, gdy o� z osi�gnie okre�lon� warto��

	const portTickType xDelay = 1000 / portTICK_RATE_MS;
	const portTickType xDelay2 = 1000 / portTICK_RATE_MS;
	unsigned int i=0;
	while(1)
	{
		LIS302DL_Read(&acc_z, LIS302DL_OUT_Z_ADDR, 1);

		if(acc_z<0)
		{
			// Zawieszenie zada�
			vTaskSuspend( xHandle_3 );
			vTaskSuspend( xHandle_4 );
			vTaskSuspend( xHandle_5 );

			for(i=0;i<3;i++)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
				vTaskDelay( xDelay );
				GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
				vTaskDelay( xDelay );
			}

			// Przywr�cenie zawieszonych zada�
			vTaskResume( xHandle_3 );
			vTaskResume( xHandle_4 );
			vTaskDelay( xDelay2 );
			vTaskResume( xHandle_5 );
		}
	}
}

//************************************KONIEC************************************//

//********************************INICJALIZACJE*********************************//

void GPIOInit(void)
{
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

void SPIInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	// konfiguracja SPI w trybie MASTER
		SPI_InitTypeDef SPI_InitStructure;
		SPI_I2S_DeInit(SPI2);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_Init(SPI2, &SPI_InitStructure);

		SPI_Cmd(SPI2, ENABLE);

		// struktura do konfiguracji akcelerometru
		 LIS302DL_InitTypeDef LIS302DL_InitStruct;

		 // uruchomienie ukladu
		 LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
		 // wybor czestotliwosci aktualizacji pomiarow
		 LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
		 // uruchomienie pomiaru dla wszystkich osi (x,y,z)
		 LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE |
		LIS302DL_Z_ENABLE;
		 // wybor zakresu pomiarow (+-2.3g)
		 LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
		 LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
		 // aktualizacja ustawien akcelerometru na podstawie wypelnionej struktury
		 LIS302DL_Init(&LIS302DL_InitStruct);
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

//************************************KONIEC************************************//

//****************************INICJALIZACJE*FreeRTOS****************************//

void vApplicationTickHook( void )
{
    ++tickTime;
}

void vApplicationIdleHook( void )
{
    ++u64IdleTicksCnt;
}

void vApplicationMallocFailedHook( void )
{
    configASSERT( 0 );
}

//************************************KONIEC************************************//
