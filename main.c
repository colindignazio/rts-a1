// REMEMBER TO REFERENCE THIS IN OUR README https://visualgdb.com/tutorials/arm/stm32/timers/

#include "stm32f4xx.h"
#include "main.h"

//times are all in ms
#define BLINK_TOGGLE 500
#define SHORT_PRESS 100
#define LONG_PRESS 1000
#define DOUBLE_CLICK_TIME 500

#define BREW_TIME_ESPRESSO 3000
#define BREW_TIME_LATTE 5000
#define BREW_TIME_MOCHA 7000
#define BREW_TIME_BLACK 10000

#define ALL_LEDS GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15

fir_8 filt;
static uint16_t sound_timer = 0;
static uint8_t playingSound = 0;

typedef enum {
	ESPRESSO = 0,
	LATTE = 1,
	MOCHA = 2,
	BLACK = 3,
	DEFAULT_COFFEE = 4
} Coffee;
void InitSound() {
	GPIO_InitTypeDef PinInitStruct;
	GPIO_StructInit(&PinInitStruct);
	I2S_InitTypeDef I2S_InitType;
	I2C_InitTypeDef I2C_InitType;

	//Reset pin as GPIO
	PinInitStruct.GPIO_Pin = CODEC_RESET_PIN;
	PinInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	PinInitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	PinInitStruct.GPIO_OType = GPIO_OType_PP;
	PinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_Init(GPIOD, &PinInitStruct);

	// I2C pins
	PinInitStruct.GPIO_Mode = GPIO_Mode_AF;
	PinInitStruct.GPIO_OType = GPIO_OType_OD;
	PinInitStruct.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	PinInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	PinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &PinInitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	//enable I2S and I2C clocks
	//RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI3, ENABLE);
	RCC_PLLI2SCmd(ENABLE);

	// I2S pins
	PinInitStruct.GPIO_OType = GPIO_OType_PP;
	PinInitStruct.GPIO_Pin = I2S3_SCLK_PIN | I2S3_SD_PIN | I2S3_MCLK_PIN;
	GPIO_Init(GPIOC, &PinInitStruct);

	PinInitStruct.GPIO_Pin = I2S3_WS_PIN;
	GPIO_Init(GPIOA, &PinInitStruct);

	//prepare output ports for alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	//keep Codec off for now
	GPIO_ResetBits(GPIOD, CODEC_RESET_PIN);

	// configure I2S port
	SPI_I2S_DeInit(CODEC_I2S);
	I2S_InitType.I2S_AudioFreq = I2S_AudioFreq_48k;
	I2S_InitType.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
	I2S_InitType.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitType.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitType.I2S_Standard = I2S_Standard_Phillips;
	I2S_InitType.I2S_CPOL = I2S_CPOL_Low;

	I2S_Init(CODEC_I2S, &I2S_InitType);
	//I2S_Cmd(CODEC_I2S, ENABLE);

	// configure I2C port
	I2C_DeInit(CODEC_I2C);
	I2C_InitType.I2C_ClockSpeed = 100000;
	I2C_InitType.I2C_Mode = I2C_Mode_I2C;
	I2C_InitType.I2C_OwnAddress1 = CORE_I2C_ADDRESS;
	I2C_InitType.I2C_Ack = I2C_Ack_Enable;
	I2C_InitType.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitType.I2C_DutyCycle = I2C_DutyCycle_2;

	I2C_Cmd(CODEC_I2C, ENABLE);
	I2C_Init(CODEC_I2C, &I2C_InitType);
	uint32_t delaycount;
	uint8_t CodecCommandBuffer[5];

	uint8_t regValue = 0xFF;

	GPIO_SetBits(GPIOD, CODEC_RESET_PIN);
	delaycount = 1000000;
	while (delaycount > 0)
	{
		delaycount--;
	}
	//keep codec OFF
	CodecCommandBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
	CodecCommandBuffer[1] = 0x01;
	send_codec_ctrl(CodecCommandBuffer, 2);

	//begin initialization sequence (p. 32)
	CodecCommandBuffer[0] = 0x00;
	CodecCommandBuffer[1] = 0x99;
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x47;
	CodecCommandBuffer[1] = 0x80;
	send_codec_ctrl(CodecCommandBuffer, 2);

	regValue = read_codec_register(0x32);

	CodecCommandBuffer[0] = 0x32;
	CodecCommandBuffer[1] = regValue | 0x80;
	send_codec_ctrl(CodecCommandBuffer, 2);

	regValue = read_codec_register(0x32);

	CodecCommandBuffer[0] = 0x32;
	CodecCommandBuffer[1] = regValue & (~0x80);
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x00;
	CodecCommandBuffer[1] = 0x00;
	send_codec_ctrl(CodecCommandBuffer, 2);
	//end of initialization sequence

	CodecCommandBuffer[0] = CODEC_MAP_PWR_CTRL2;
	CodecCommandBuffer[1] = 0xAF;
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
	CodecCommandBuffer[1] = 0x70;
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_CLK_CTRL;
	CodecCommandBuffer[1] = 0x81; //auto detect clock
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_IF_CTRL1;
	CodecCommandBuffer[1] = 0x07;
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x0A;
	CodecCommandBuffer[1] = 0x00;
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x27;
	CodecCommandBuffer[1] = 0x00;
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x1A | CODEC_MAPBYTE_INC;
	CodecCommandBuffer[1] = 0x0A;
	CodecCommandBuffer[2] = 0x0A;
	send_codec_ctrl(CodecCommandBuffer, 3);

	CodecCommandBuffer[0] = 0x1F;
	CodecCommandBuffer[1] = 0x0F;
	send_codec_ctrl(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_PWR_CTRL1;
	CodecCommandBuffer[1] = 0x9E;
	send_codec_ctrl(CodecCommandBuffer, 2);

}
uint8_t read_codec_register(uint8_t mapbyte)
{
	uint8_t receivedByte = 0;

	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
	{
		//just wait until no longer busy
	}

	I2C_GenerateSTART(CODEC_I2C, ENABLE);
	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB))
	{
		//wait for generation of start condition
	}

	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		//wait for end of address transmission
	}

	I2C_SendData(CODEC_I2C, mapbyte); //sets the transmitter address
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		//wait for transmission of byte
	}

	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
	{
		//just wait until no longer busy
	}

	I2C_AcknowledgeConfig(CODEC_I2C, DISABLE);

	I2C_GenerateSTART(CODEC_I2C, ENABLE);
	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB))
	{
		//wait for generation of start condition
	}

	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		//wait for end of address transmission
	}

	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		//wait until byte arrived
	}
	receivedByte = I2C_ReceiveData(CODEC_I2C);

	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

	return receivedByte;
}
void send_codec_ctrl(uint8_t controlBytes[], uint8_t numBytes)
{
	uint8_t bytesSent=0;

	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
	{
		//just wait until no longer busy
	}

	I2C_GenerateSTART(CODEC_I2C, ENABLE);
	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB))
	{
		//wait for generation of start condition
	}
	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		//wait for end of address transmission
	}
	while (bytesSent < numBytes)
	{
		I2C_SendData(CODEC_I2C, controlBytes[bytesSent]);
		bytesSent++;
		while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
		{
			//wait for transmission of byte
		}
	}
	while(!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF))
	{
	    //wait until it's finished sending before creating STOP
	}
	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

}
void InitializeLEDs() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef gpioStructure;
  gpioStructure.GPIO_Pin = ALL_LEDS;
  gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
  gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &gpioStructure);

  GPIO_WriteBit(GPIOD, ALL_LEDS, Bit_RESET);
}

void InitButton() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // Button
	
  GPIO_InitTypeDef gpioStructure;
  gpioStructure.GPIO_Mode = GPIO_Mode_IN;
  gpioStructure.GPIO_OType = GPIO_OType_PP;  
  gpioStructure.GPIO_Pin = GPIO_Pin_0;
  gpioStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &gpioStructure);
}

/* 
 * Set up the timer to interupt every 1ms.
 */
void InitializeTimer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_Prescaler = 83;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 999;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
  TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void EnableTimerInterrupt() {
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);
}

// customize the EXTI for button 
void InitEXTI()
{
	EXTI_InitTypeDef EXTI_InitStructure; // External interrupt
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  // Selects the GPIOA pin 0 used as external interrupt source
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_Init(&EXTI_InitStructure);
}

// NVCI : Nested Vectored Interrupt Controller for EXTI in IRQ
void EnableEXTIInterrupt()
{
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

uint16_t getPinForCoffeeType(Coffee type) {
	switch(type) {
		case ESPRESSO:
			return GPIO_Pin_12;
		case LATTE:
			return GPIO_Pin_13;
		case MOCHA:
			return GPIO_Pin_14;
		case BLACK:
			return GPIO_Pin_15;
		default:
			return 0;
	}
}

uint16_t getBrewTimeForCoffeeType(Coffee type) {
	switch(type) {
		case ESPRESSO:
			return BREW_TIME_ESPRESSO;
		case LATTE:
			return BREW_TIME_LATTE;
		case MOCHA:
			return BREW_TIME_MOCHA;
		case BLACK:
			return BREW_TIME_BLACK;
		default:
			return 0;
	}
}

static Coffee selected;

void changeSelection() {
	GPIO_ResetBits(GPIOD, ALL_LEDS);
	selected++;
	if(selected >= DEFAULT_COFFEE)
			selected = (Coffee) 0;
	GPIO_SetBits(GPIOD, getPinForCoffeeType(selected));
}

void alertBrewComplete() {
	sound_timer++;
	for(int i = 0; i < 36000; i++) {
	if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
    	{
				
    		SPI_I2S_SendData(CODEC_I2S, sample);

    		//only update on every second sample to insure that L & R ch. have the same sample value
    		
    			sawWave += NOTEFREQUENCY;
    			if (sawWave > 1.0)
    				sawWave -= 2.0;

    			filteredSaw = updateFilter(&filt, sawWave);
    			sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
    	}
		}
			if(sound_timer > 100) { 
				playingSound = 0;
				sound_timer = 0;
			}
}

void initializeSelection() {
	GPIO_ResetBits(GPIOD, ALL_LEDS);
	selected = (Coffee) -1;
	changeSelection();
}

static uint16_t brew_timer = 0;
static uint16_t button_timer = 0;
static uint8_t brewing = 0;
static uint8_t dblclick = 0;

static uint16_t double_click_timer = 0;

void brew() {
	brew_timer++;
	if(brew_timer % BLINK_TOGGLE == 0) {					
		GPIO_ToggleBits(GPIOD, getPinForCoffeeType(selected));
	}
			
	if(brew_timer > getBrewTimeForCoffeeType(selected)) {
		initFilter(&filt);
		I2S_Cmd(CODEC_I2S, ENABLE);
		playingSound = 1;
		brewing = 0;
		brew_timer = 0;
		GPIO_SetBits(GPIOD, getPinForCoffeeType(selected));
	}
}

uint8_t isShortClick() {
	return button_timer > SHORT_PRESS;
}

uint8_t isLongClick() {
	return button_timer > LONG_PRESS;
}

uint8_t isDoubleClick() {
	return dblclick && double_click_timer < DOUBLE_CLICK_TIME;
}

uint8_t button_pressed = 0;

void EXTI0_IRQHandler()
{
	uint8_t button_pin;
	
  // Checks whether the interrupt from EXTI0 or not
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {  
		button_pin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);	
		
		if(button_pin) {
			button_pressed = 1;
		}
		else {
			button_pressed = 0;
		}
			
    // Clears the EXTI line pending bit
    EXTI_ClearITPendingBit(EXTI_Line0);            
  }
}

void longClickHandler() {
	if(!brewing) {
		brewing = 1;
	}	
	button_timer = 0;
}

void doubleClickHandler() {
	brewing = 0;
	dblclick = 0;
	GPIO_SetBits(GPIOD, getPinForCoffeeType(selected));
}

void shortClickHandler() {
	if(!brewing) {
		changeSelection();
	} else {
		if(isDoubleClick()) {
			doubleClickHandler();
		} else {
			brew_timer = 0;
		}
		double_click_timer = 0;
		dblclick = 1;
	}
	button_timer = 0;			
}

/*
 * Handler that gets triggered every time the timer tickers (every 1ms in our case)
 * Only difference between this and an infinite loop is that we can control how often this gets run.
 */
void TIM2_IRQHandler() {	
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		if(dblclick)
			double_click_timer++;
		
		if(button_pressed) {
			button_timer++;
		} else if(isLongClick()) {
			longClickHandler();
		} else if(isShortClick()) {
			shortClickHandler();
		}
		
		if(brewing) {
			brew();
		}
		if(playingSound) {
			alertBrewComplete();
		}
	}
}
	float updateFilter(fir_8* filt, float val)
{
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}
void initFilter(fir_8* theFilter)
{
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}
int main() {
  InitializeLEDs();
	InitButton();
  InitializeTimer();
	InitEXTI();
	initializeSelection();
	
	EnableEXTIInterrupt();
	EnableTimerInterrupt();
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	InitSound();


  while(1) {
		
	}
}

