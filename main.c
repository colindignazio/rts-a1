// REMEMBER TO REFERENCE THIS IN OUR README https://visualgdb.com/tutorials/arm/stm32/timers/

#include "stm32f4xx.h"

//times are all in ms
#define BLINK_TOGGLE 500
#define SHORT_PRESS 50
#define LONG_PRESS 1000

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

int getPinForCoffeeType(Coffee type) {
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

int getBrewTimeForCoffeeType(Coffee type) {
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
	
}

void initializeSelection() {
	GPIO_ResetBits(GPIOD, ALL_LEDS);
	selected = (Coffee) -1;
	changeSelection();
}

static uint16_t brew_timer = 0;
static uint16_t button_timer = 0;
static uint8_t brewing = 0;

/*
 * Handler that gets triggered every time the timer tickers (every 1ms in our case)
 * Only difference between this and an infinite loop is that we can control how often this gets run.
 */
void TIM2_IRQHandler() {
	uint8_t button_pin;
	
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		button_pin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);	
		
		if(!brewing) {
			if(button_pin) {
				button_timer++; //keep track of how long the button has been pressed for
												//I think theres a better way to do this using more interupts
			} else if(button_timer > LONG_PRESS) {
				brewing = 1;
				button_timer = 0;
			} else if(button_timer > SHORT_PRESS) {
				changeSelection();
				button_timer = 0;
			}
		}	else {
			brew_timer++;
			if(brew_timer % BLINK_TOGGLE == 0) {					
				GPIO_ToggleBits(GPIOD, getPinForCoffeeType(selected));
			}
			
			if(brew_timer > getBrewTimeForCoffeeType(selected)) {
				alertBrewComplete();
				brewing = 0;
				brew_timer = 0;
				GPIO_SetBits(GPIOD, getPinForCoffeeType(selected));
			}
		}
	}
}
	
int main() {
  InitializeLEDs();
	InitButton();
  InitializeTimer();
	
	initializeSelection();
	
	EnableTimerInterrupt();

  while(1);
}
