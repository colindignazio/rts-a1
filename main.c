// REMEMBER TO REFERENCE THIS IN OUR README https://visualgdb.com/tutorials/arm/stm32/timers/

#include "stm32f4xx.h"
#include "main.h"

//times are all in ms
#define BLINK_TOGGLE 500
#define SHORT_PRESS 100
#define LONG_PRESS 1000
#define DOUBLE_CLICK_TIME 500
#define ALERT_TIME 100

#define BREW_TIME_ESPRESSO 3000
#define BREW_TIME_LATTE 5000
#define BREW_TIME_MOCHA 7000
#define BREW_TIME_BLACK 10000

#define ALL_LEDS GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15

typedef enum {
	ESPRESSO = 0,
	LATTE = 1,
	MOCHA = 2,
	BLACK = 3,
	DEFAULT_COFFEE = 4
} Coffee;

static fir_8 filt;

static Coffee selected;

static uint8_t buttonPressed = 0;
static uint8_t brewing = 0;
static uint8_t checkForDoubleClick = 0;
static uint8_t playingAlert = 0;

static uint16_t brewTimer = 0;
static uint16_t buttonTimer = 0;
static uint16_t doubleClickTimer = 0;
static uint16_t alert_timer = 0;

void InitializeLEDs() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Pin = ALL_LEDS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_WriteBit(GPIOD, ALL_LEDS, Bit_RESET);
}

void InitButton() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // Button
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
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

void InitializeCodec() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	codec_init();
}

void PlayAlert() {
	codec_ctrl_init();
	initFilter(&filt);
	I2S_Cmd(CODEC_I2S, ENABLE);
	playingAlert = 1;
}

void StopAlert() {
	GPIO_ResetBits(GPIOD, CODEC_RESET_PIN);
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

void changeSelection() {
	GPIO_ResetBits(GPIOD, ALL_LEDS);
	selected++;
	if(selected >= DEFAULT_COFFEE)
			selected = (Coffee) 0;
	GPIO_SetBits(GPIOD, getPinForCoffeeType(selected));
}

void alertBrewComplete() {
	alert_timer++;
	for(int i = 0; i < 36000; i++) {
		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)) {
			SPI_I2S_SendData(CODEC_I2S, sample);

			//only update on every second sample to insure that L & R ch. have the same sample value
    	if (sampleCounter & 0x00000001) {	
				sawWave += NOTEFREQUENCY;
    
				if (sawWave > 1.0) {
					sawWave -= 2.0;
				}
		
				filteredSaw = updateFilter(&filt, sawWave);
				sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
    	}
    	sampleCounter++;
		}
	}

	if(alert_timer > ALERT_TIME) { 
		playingAlert = 0;
		alert_timer = 0;
		StopAlert();
	}
}

void InitializeSelection() {
	GPIO_ResetBits(GPIOD, ALL_LEDS);
	selected = (Coffee) -1;
	changeSelection();
}

void brew() {
	brewTimer++;
	if(brewTimer % BLINK_TOGGLE == 0) {					
		GPIO_ToggleBits(GPIOD, getPinForCoffeeType(selected));
	}
			
	if(brewTimer > getBrewTimeForCoffeeType(selected)) {
		PlayAlert();
		brewing = 0;
		brewTimer = 0;
		GPIO_SetBits(GPIOD, getPinForCoffeeType(selected));
	}
}

uint8_t isShortClick() {
	return buttonTimer > SHORT_PRESS;
}

uint8_t isLongClick() {
	return buttonTimer > LONG_PRESS;
}

uint8_t isDoubleClick() {
	return checkForDoubleClick && doubleClickTimer < DOUBLE_CLICK_TIME;
}

void EXTI0_IRQHandler() {
	uint8_t buttonPin;

  if(EXTI_GetITStatus(EXTI_Line0) == RESET)
		return;
	
	buttonPin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);	
		
	if(buttonPin) {
		buttonPressed = 1;
	}
	else {
		buttonPressed = 0;
	}
	
  EXTI_ClearITPendingBit(EXTI_Line0);            
}

void longClickHandler() {
	if(!brewing) {
		brewing = 1;
	}	
	buttonTimer = 0;
}

void doubleClickHandler() {
	brewing = 0;
	checkForDoubleClick = 0;
	GPIO_SetBits(GPIOD, getPinForCoffeeType(selected));
}

void shortClickHandler() {
	if(!brewing) {
		changeSelection();
	} else {
		if(isDoubleClick()) {
			doubleClickHandler();
		} else {
			brewTimer = 0;
		}
		doubleClickTimer = 0;
		checkForDoubleClick = 1;
	}
	buttonTimer = 0;			
}

void TIM2_IRQHandler() {	
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) == RESET)
		return;
	
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
	if(playingAlert) {
		alertBrewComplete();
		return;
	}
		
	if(checkForDoubleClick) {
		doubleClickTimer++;
	}
		
	if(buttonPressed) {
		buttonTimer++;
	} else if(isLongClick()) {
		longClickHandler();
	} else if(isShortClick()) {
		shortClickHandler();
	}
		
	if(brewing) {
		brew();
	}
}

float updateFilter(fir_8* filt, float val) {
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++) {
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter) {
	uint8_t i;

	theFilter->currIndex = 0;
	sampleCounter = 0;

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
	InitializeCodec();
	
	InitializeSelection();
	
	EnableEXTIInterrupt();
	EnableTimerInterrupt();

  while(1);
}

