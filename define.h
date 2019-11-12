
// include library
#include "stm32f4xx.h"
#include "PID.h"
#include <math.h>


// Right Motor Channels
#define ENCRA_PIN               				GPIO_Pin_0
#define ENCRA_GPIO_PORT         GPIOA
#define ENCRA_GPIO_CLK          	RCC_AHB1Periph_GPIOA
#define ENCRA_SOURCE            	GPIO_PinSource0
#define ENCRA_AF                				GPIO_AF_TIM2
#define ENCR_TOP_COUNT						2147400000
#define ENCR_BOT_COUNT						-2147400000

#define ENCRB_PIN               				GPIO_Pin_1
#define ENCRB_GPIO_PORT         GPIOA
#define ENCRB_GPIO_CLK          	RCC_AHB1Periph_GPIOA
#define ENCRB_SOURCE            	GPIO_PinSource1
#define ENCRB_AF                				GPIO_AF_TIM2

// determine the timers for ENCODER Read

#define ENCR_TIMER              			TIM2
#define ENCR_TIMER_CLK          	RCC_APB1Periph_TIM2
#define RIGHT_COUNT             			ENCR_TIMER->CNT
#define RIGHT_COUNT_MODE				TIM_EncoderMode_TI1
#define ENCR_TIMER_INT							TIM2_IRQn


// determine the timers to use for calculating
#define PWM_TIMER											TIM4
#define PWM_TIMER_CLK							RCC_APB1Periph_TIM4
#define PWM_TIMER_PRESCALE  0
#define PWM_TIMER_PERIOD				4199		
#define PWM_AF													GPIO_AF_TIM4
#define PWM_Duty_LED_A						PWM_TIMER->CCR1
#define PWM_Duty_LED_B						PWM_TIMER->CCR2
#define PWM_DutyA											PWM_TIMER->CCR3
#define PWM_DutyB											PWM_TIMER->CCR4
#define PWM_DutyA_init								0
#define PWM_DutyB_init								0

#define PWM_GPIO_LED_PORT		GPIOD
#define PWM_GPIO_LED_CLK				RCC_AHB1Periph_GPIOD
#define PWM_LED_CA									GPIO_Pin_12
#define PWM_LED_CB				 					GPIO_Pin_13
#define PWM_CA_LED_SOURCE  GPIO_PinSource12
#define PWM_CB_LED_SOURCE 	GPIO_PinSource13

#define PWM_GPIO_PORT						GPIOB
#define PWM_GPIO_CLK								RCC_AHB1Periph_GPIOB
#define PWM_CA    											GPIO_Pin_8
#define PWM_CB								 					GPIO_Pin_9
#define PWM_CA_SOURCE    				GPIO_PinSource8
#define PWM_CB_SOURCE 					GPIO_PinSource9

// determine Sampling params
#define SPLE_TIMER											TIM3
#define SPLE_TIMER_CLK							RCC_APB1Periph_TIM3
#define SPLE_TIME_PRESCALE		PWM_TIMER_PRESCALE
#define SPLE_PERIOD					 				PWM_TIMER_PERIOD
#define SPLE_TIMER_INT							TIM3_IRQn
#define SPLE_LED_PORT							GPIOD
#define SPLE_LED_CLK								RCC_AHB1Periph_GPIOD
#define SPLE_LED_PIN								  GPIO_Pin_14


// determine USART params
#define BUFF_SIZE_RX									52
#define BUFF_SIZE_TX									13

uint8_t txbuff[] = {'^','+','0','0','0','0','0','0','0','0','0','0','$'};				 
uint8_t rxbuff[2*BUFF_SIZE_RX];
//uint8_t reader[3*BUFF_SIZE_RX];

#define USART_FULL_BUFF_SIZE  3*BUFF_SIZE_RX
#define USART_														UART4
#define USART_CLK											RCC_APB1Periph_UART4
#define USART_AF												GPIO_AF_UART4
#define USART_BAUDRATE						9600
#define USART_WORDLENGTH			USART_WordLength_8b
#define USART_STOPBIT								USART_StopBits_1
#define USART_PARITY									USART_Parity_No

#define USART_PORT										GPIOC
#define USART_PORT_CLK						RCC_AHB1Periph_GPIOC
#define USART_TX_PIN									GPIO_Pin_10
#define USART_RX_PIN									GPIO_Pin_11
#define USART_TX_SOURCE					GPIO_PinSource10
#define USART_RX_SOURCE					GPIO_PinSource11
#define USART_PORT_AF							GPIO_Mode_AF

#define USART_DMA_CLK									RCC_AHB1Periph_DMA1
#define USART_DMA_RX_STREAM		DMA1_Stream2
#define USART_DMA_RX_CHANNEL	DMA_Channel_4
#define USART_DMA_RX_MODE				DMA_Mode_Circular
#define USART_DMA_RX_BUFFER		(uint32_t)rxbuff
#define USART_DMA_RX_IT								DMA1_Stream2_IRQn
#define USART_DMA_RX_IT_FLAG			DMA_IT_TCIF2


#define USART_DMA_TX_STREAM		DMA1_Stream4
#define USART_DMA_TX_CHANNEL		DMA_Channel_4
#define USART_DMA_TX_MODE				DMA_Mode_Circular
#define USART_DMA_TX_BUFFER			(uint32_t)txbuff
#define USART_DMA_TX_FLAG					DMA_FLAG_TCIF4

#define USART_TX_TIMER									TIM3

typedef struct
{  
	int32_t oldPos;
	int32_t curPos;
	int32_t vec;
} Position;
















