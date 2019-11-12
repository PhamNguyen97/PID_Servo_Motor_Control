#include "define.h"


PID PID1;
PID PID_SP;
PID* PID1pts = &PID1;
PID* PID2pts = &PID_SP;
	float Kp = 0;
	float Ki = 0;
	float Kd = 0;
	float Sp = 0;
static int32_t current_pos;
int32_t current_vel;
static Position Pos;
Position* Pos_i = &Pos; 
int flag=0;
int old_flag;
float vec;

int32_t temp;
int cnt = 0;
int setpoint = 0;
int vec_setpoint = 0;
int current_sample = 0;
static int max_sample_number = 1000; // 200Hz 

//int32_t oldRightEncoder;
//int32_t rightEncoder, rightCount, Position;
//int Position;




//void encodersRead (void);
void encodersReset (int32_t Te);
void encodersInit (void);
void PWM_timer_init(void);
void PWM_2Channel_init(void);
void PWM_2Channle_out_init(void);
void PWM_update(uint16_t DutyA, uint16_t DutyB);
void Sampling_Timer_init(void);
void usart_init(void);
void position_to_string(int Position);
int32_t get_position(TIM_TypeDef* TIMx);
void GetDataFromBuffer(void);
int32_t get_velocity(TIM_TypeDef* TIMx,Position* Pos, uint32_t sample_rate);


int main(void)
{
	SystemInit();
	encodersInit();
	PWM_timer_init();
	PWM_2Channel_init();
	PWM_2Channle_out_init();
	PID_Init(PID1pts, 0.00005,530);
	PID_Init(PID2pts, 0.5, 400);
	Sampling_Timer_init();
	usart_init();
	
	while(1)
	{
		
	}

}







 void encodersReset (int32_t Te)
{
  __disable_irq();
	current_pos = 0;
  //TIM_SetCounter (ENCR_TIMER, 0);
	RIGHT_COUNT = Te;
  __enable_irq();
}
void encodersInit (void)
{
	/*
 * Configure two timers as quadrature encoder counters. 
 * Details of which timers should be used are
 * in the project hardware header file.
 * Most timers can be used if channels 1 and 2 are available on pins.
 * The timers are mostly 16 bit. Timers can be set to 32 bit but they are
 * not very convenient for IO pins so the counters are simply set to to
 * 16 bit counting regardless.
 * A mouse needs 32 bits of positional data and, since it also needs the
 * current speed, distance is not maintained by the encoder code but will
 * be looked after by the motion control code.
 * The counters are set to X4 mode. The only alternative is X2 counting.
 */
 
 //available to the rest of the code
//speeds
	
  GPIO_InitTypeDef GPIO_InitStructure;
  // turn on the clocks for each of the ports needed

  RCC_AHB1PeriphClockCmd (ENCRA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCRB_GPIO_CLK, ENABLE);

  // now configure the pins themselves
  // they are all going to be inputs with pullups
  GPIO_StructInit (&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_InitStructure.GPIO_Pin = ENCRA_PIN;
  GPIO_Init (ENCRA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCRB_PIN;
  GPIO_Init (ENCRB_GPIO_PORT, &GPIO_InitStructure);

  // Connect the pins to their Alternate Functions

  GPIO_PinAFConfig (ENCRA_GPIO_PORT, ENCRA_SOURCE, ENCRA_AF);
  GPIO_PinAFConfig (ENCRB_GPIO_PORT, ENCRB_SOURCE, ENCRB_AF);

  // Timer peripheral clock enable
  //RCC_APB1PeriphClockCmd (ENCL_TIMER_CLK, ENABLE);
  RCC_APB1PeriphClockCmd (ENCR_TIMER_CLK, ENABLE);

  // set them up as encoder inputs
  // set both inputs to rising polarity to let it use both edges

  TIM_EncoderInterfaceConfig (ENCR_TIMER, RIGHT_COUNT_MODE, 
                              TIM_ICPolarity_Rising, 
                              TIM_ICPolarity_Rising);
  TIM_SetAutoreload (ENCR_TIMER, 0xffffffff);

  // turn on the timer/counters
  TIM_Cmd (ENCR_TIMER, ENABLE);
  encodersReset(0);
}

/// configure TIMER 4 for PWM channels
void PWM_timer_init(void)
{
	// initial Timer for PWM
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

  RCC_APB1PeriphClockCmd(PWM_TIMER_CLK, ENABLE);
	TIM_BaseStruct.TIM_Prescaler = PWM_TIMER_PRESCALE;
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = PWM_TIMER_PERIOD	; 
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
 
  TIM_TimeBaseInit(PWM_TIMER, &TIM_BaseStruct);
	
  TIM_Cmd(PWM_TIMER, ENABLE);
}

void PWM_2Channel_init(void)
{
		TIM_OCInitTypeDef TIM_OCStruct;
    /* PWM mode 2 = Clear on compare match */
    /* PWM mode 1 = Set on compare match */
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	
		TIM_OCStruct.TIM_Pulse = PWM_DutyA_init; // This channel for Led show PWM A
    TIM_OC1Init(PWM_TIMER, &TIM_OCStruct);
    TIM_OC1PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
	
		TIM_OCStruct.TIM_Pulse = PWM_DutyB_init; //This channel for Led show PWM B
    TIM_OC2Init(PWM_TIMER, &TIM_OCStruct);
    TIM_OC2PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
	
		TIM_OCStruct.TIM_Pulse = PWM_DutyA_init; // PWM A
    TIM_OC3Init(PWM_TIMER, &TIM_OCStruct);
    TIM_OC3PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
	
		TIM_OCStruct.TIM_Pulse = PWM_DutyB_init; //PWM B
    TIM_OC4Init(PWM_TIMER, &TIM_OCStruct);
    TIM_OC4PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
		
}


void PWM_2Channle_out_init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
    
    /* Clock for GPIOD */
		RCC_AHB1PeriphClockCmd(PWM_GPIO_LED_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(PWM_GPIO_CLK, ENABLE);
 
    /* Alternating functions for pins */
	  GPIO_PinAFConfig(PWM_GPIO_LED_PORT, PWM_CA_LED_SOURCE, PWM_AF);
    GPIO_PinAFConfig(PWM_GPIO_LED_PORT, PWM_CB_LED_SOURCE, PWM_AF);
    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CA_SOURCE, PWM_AF);
    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CB_SOURCE, PWM_AF);
	
    /*Set pims for PWM on leds*/
	  GPIO_InitStruct.GPIO_Pin = PWM_LED_CA | PWM_LED_CB ;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(PWM_GPIO_LED_PORT, &GPIO_InitStruct);
	
    /* Set pins for PWM channel*/
    GPIO_InitStruct.GPIO_Pin = PWM_CA | PWM_CB ;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStruct);
	
}


void PWM_update(uint16_t DutyA, uint16_t DutyB)
{
		PWM_DutyA = DutyA;
		PWM_DutyB = DutyB;
		PWM_Duty_LED_A = DutyA;
		PWM_Duty_LED_B = DutyB;
}
	

// Take sample and do calculation at 20KHz
void Sampling_Timer_init(void)
{
	
		// Enable timer for sampling
		RCC_APB1PeriphClockCmd(SPLE_TIMER_CLK, ENABLE);

		TIM_TimeBaseInitTypeDef TIM_BaseStruct;
		TIM_BaseStruct.TIM_Prescaler = SPLE_TIME_PRESCALE;
		TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_BaseStruct.TIM_Period = SPLE_PERIOD;
		TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_BaseStruct.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(SPLE_TIMER, &TIM_BaseStruct);
		TIM_Cmd(SPLE_TIMER, ENABLE);
	
		TIM_ITConfig(SPLE_TIMER, TIM_IT_Update, ENABLE);
	
	// Enable timer for interrupt

			NVIC_InitTypeDef nvicStructure;
			nvicStructure.NVIC_IRQChannel = SPLE_TIMER_INT;
			nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
			nvicStructure.NVIC_IRQChannelSubPriority = 1;
			nvicStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&nvicStructure);
			
		RCC_AHB1PeriphClockCmd(SPLE_LED_CLK, ENABLE);
		
		GPIO_InitTypeDef GPIO_InitStruct;	
	
    GPIO_InitStruct.GPIO_Pin = SPLE_LED_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPLE_LED_PORT, &GPIO_InitStruct);
 
    GPIO_WriteBit(SPLE_LED_PORT, SPLE_LED_PIN, Bit_RESET);
}
// Take sample, do PID control, start USART Tx
static int t = 0;
void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(SPLE_TIMER, TIM_IT_Update) != RESET)
    {
        current_pos = get_position(ENCR_TIMER);	
			  
			  if(flag == 0){
					t++;
					if(t == 10000){
					t = 0;
					current_vel = get_velocity(ENCR_TIMER, Pos_i ,2);	
					//vec = (Pos.vec/9000)*60;	
					PID_cal(vec_setpoint,current_vel,PID2pts);
					PWM_update(PID_SP.uB, PID_SP.uA);					
				}
			}
				if (flag == 1)
				{
					Pos.oldPos = Pos.curPos;
			  	Pos.curPos = ENCR_TIMER->CNT;
				  Pos.vec = (Pos.curPos - Pos.oldPos)*2;
					PID_cal(setpoint,current_pos,PID1pts);
					PWM_update(PID1.uB,PID1.uA);
				}
			 
			 if((old_flag == 0) & (flag == 1))
			 {
				 if (current_pos >= 0)
				 {
					 temp = current_pos%9000;
					 encodersReset(temp);
				 }
				 else if (current_pos < 0)
				 {
						current_pos = -current_pos;
						temp = current_pos%9000;
						temp = -temp;
						encodersReset(temp);
				 }
			 }
			 old_flag = flag;
						
				TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        GPIO_ToggleBits(SPLE_LED_PORT, SPLE_LED_PIN);
			// Enable DMA_USART_TX 
				
				if (++current_sample ==max_sample_number)
				{
						if(flag == 1)
							position_to_string(current_pos);
						else 
							position_to_string(current_vel);
						current_sample = 0;
						DMA_ClearFlag(USART_DMA_TX_STREAM, USART_DMA_TX_FLAG);
						USART_DMA_TX_STREAM->NDTR = BUFF_SIZE_TX;
						DMA_Cmd(USART_DMA_TX_STREAM, ENABLE);
				}
    }
}


/// USART4 configuration
void usart_init(void)
{
		// usart config
		USART_InitTypeDef USART_InitStructure;  
	
		RCC_APB1PeriphClockCmd(USART_CLK, ENABLE);
		USART_InitStructure.USART_BaudRate = USART_BAUDRATE;
		USART_InitStructure.USART_WordLength = USART_WORDLENGTH;
		USART_InitStructure.USART_StopBits = USART_STOPBIT;
		USART_InitStructure.USART_Parity = USART_PARITY;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART_, &USART_InitStructure);
	
		// GPIO PORT for USART

		RCC_AHB1PeriphClockCmd(USART_PORT_CLK, ENABLE);
		// Tx
		GPIO_InitTypeDef 	GPIO_InitStructure; 
		GPIO_InitStructure.GPIO_Pin   = USART_TX_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(USART_PORT, &GPIO_InitStructure);
	
		//Rx
		GPIO_InitStructure.GPIO_Pin   = USART_RX_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_Init(USART_PORT, &GPIO_InitStructure);
		
		/* Connect UART_PORT pins to AFsource */  
		GPIO_PinAFConfig(GPIOC, USART_TX_SOURCE, USART_AF);
		GPIO_PinAFConfig(GPIOC, USART_RX_SOURCE, USART_AF); 
		
		/* Enable USART */
		USART_Cmd(USART_, ENABLE);
		/* Enable UART DMA */
		USART_DMACmd(USART_, USART_DMAReq_Rx, ENABLE);
		USART_DMACmd(USART_, USART_DMAReq_Tx, ENABLE); 
	
	
		// DMA USART config
		RCC_AHB1PeriphClockCmd(USART_DMA_CLK, ENABLE);
		
		/* DMA1 Stream2 Channel4 for USART4 Rx configuration */	
		DMA_InitTypeDef   DMA_InitStructure;		
		DMA_InitStructure.DMA_Channel = USART_DMA_RX_CHANNEL;  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART_->DR;
		DMA_InitStructure.DMA_Memory0BaseAddr = USART_DMA_RX_BUFFER;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = USART_FULL_BUFF_SIZE;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = USART_DMA_RX_MODE;//DMA_Mode_Circular;//
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(USART_DMA_RX_STREAM, &DMA_InitStructure);
		DMA_Cmd(USART_DMA_RX_STREAM, ENABLE);
		
		/* Enable DMA Interrupt to the highest priority */
		NVIC_InitTypeDef  NVIC_InitStructure;	
		NVIC_InitStructure.NVIC_IRQChannel = USART_DMA_RX_IT;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

  /* Transfer complete interrupt mask */
		DMA_ITConfig(USART_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
		
		
		/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
		DMA_InitStructure.DMA_Channel = USART_DMA_TX_CHANNEL;  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART_->DR;
		DMA_InitStructure.DMA_Memory0BaseAddr = USART_DMA_TX_BUFFER;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_TX;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = USART_DMA_TX_MODE;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(USART_DMA_TX_STREAM, &DMA_InitStructure);
		DMA_Cmd(USART_DMA_TX_STREAM, ENABLE);
		
	
}

void DMA1_Stream2_IRQHandler(void)
{
	
  /* Clear the DMA1_Stream2 TCIF2 pending bit */
	GetDataFromBuffer();
  DMA_ClearITPendingBit(USART_DMA_RX_STREAM, USART_DMA_RX_IT_FLAG);
	DMA_Cmd(USART_DMA_RX_STREAM, ENABLE);
}

//^#+00.0000000#+00.0000000#+00.0000000#+0000000.00#0$ 

void GetDataFromBuffer(void)
 {

	for (int i = 0;i<=USART_FULL_BUFF_SIZE-BUFF_SIZE_RX;i++)
	{
			if (  (rxbuff[i]=='^') & (rxbuff[i+51]=='$'))
			{
				if ( (rxbuff[i+1]=='#') & (rxbuff[i+13]=='#' ) & rxbuff[i+25]=='#' & rxbuff[i+37]=='#' )
						Kp = (rxbuff[i+3]-'0')*10 + (rxbuff[i+4]-'0') + (rxbuff[i+6]-'0')*0.1 + (rxbuff[i+7]-'0')*0.01+ (rxbuff[i+8]-'0')*0.001 + (rxbuff[i+9]-'0')*0.0001 + (rxbuff[i+10]-'0')*0.00001 + (rxbuff[i+11]-'0')*0.000001+ (rxbuff[i+12]-'0')*0.0000001;
						Ki =  (rxbuff[i+15]-'0')*10 + (rxbuff[i+16]-'0') + (rxbuff[i+18]-'0')*0.1 + (rxbuff[i+19]-'0')*0.01+ (rxbuff[i+20]-'0')*0.001 + (rxbuff[i+21]-'0')*0.0001 + (rxbuff[i+22]-'0')*0.00001 + (rxbuff[i+23]-'0')*0.000001+ (rxbuff[i+24]-'0')*0.0000001 ;
						Kd = (rxbuff[i+27]-'0')*10 + (rxbuff[i+28]-'0') + (rxbuff[i+30]-'0')*0.1 + (rxbuff[i+31]-'0')*0.01+ (rxbuff[i+32]-'0')*0.001 + (rxbuff[i+33]-'0')*0.0001 + (rxbuff[i+34]-'0')*0.00001 + (rxbuff[i+35]-'0')*0.000001+ (rxbuff[i+36]-'0')*0.0000001 ;
						Sp = (rxbuff[i+39]-'0')*1000000 + (rxbuff[i+40]-'0')*100000 + (rxbuff[i+41]-'0')*10000+ (rxbuff[i+42]-'0')*1000 + (rxbuff[i+43]-'0')*100 + (rxbuff[i+44]-'0')*10 + (rxbuff[i+45]-'0')*1+ (rxbuff[i+47]-'0')*0.1 + (rxbuff[i+48]-'0')*0.01;
						flag= (rxbuff[i+50]-'0');
						
				if (rxbuff[i+2] =='-')							
							Kp = -Kp;
						if (rxbuff[i+14] =='-')
							Ki = -Ki;
						if (rxbuff[i+26]=='-')
							Kd = -Kd;
						if (rxbuff[i+38]=='-')
							Sp = -Sp;
						if (flag == 1)
						{
						PID_update(PID1pts,Kp,Ki,Kd,0.00005,530);
						setpoint = Sp/360*9000;
						}
						else if (flag == 0){
						PID_update(PID2pts,Kp,Ki,Kd,0.00005,400);	
						vec_setpoint = Sp*9000; 
						}
						break;
					}
			
				
			
	}
}
void position_to_string(int32_t Position)
{
	int i;
	int32_t temp = Position;
	int32_t div_ = 1000000000;
	if (temp>=0) 
			txbuff[1] = (char)'+';
	else 
		{
			txbuff[1] = (char)'-';
			temp = -temp;
			}
	for(i=2; i<BUFF_SIZE_TX-1; i++)
		{
			txbuff[i] = (char)( (uint8_t)(temp/div_)+'0');
			temp = temp%div_;
			div_ = div_/10;
		}
}


int32_t get_position(TIM_TypeDef* TIMx)
{
		return TIMx->CNT;
}
int32_t get_velocity(TIM_TypeDef* TIMx, Position* Pos, uint32_t sample_rate)
{
	Pos->oldPos = Pos->curPos;
	Pos->curPos = TIMx->CNT;
	Pos->vec = (Pos->curPos - Pos->oldPos)*sample_rate;
	return Pos->vec;
}
