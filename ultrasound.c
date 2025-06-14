//
// Created by giangvu on 5/12/25.
//

#include "ultrasound.h"
#define INDEX_50ms 500

enum {
  TRIG = 0,
  ECHO,
  COMPUTE
} State;

volatile enum State state = TRIG;
volatile unsigned short index_time = 0;
volatile unsigned short nbr_un = 0;
volatile unsigned short nbrUnFinal = 0;
volatile unsigned short fin_reception = 0;
volatile float distance;

void UltraSoundMgt(void){
	//Will be call repetedly
	//voir dans l'enonce, on peut constater que il s'agit un FSM de 2 states : Generation and Reception
	switch (state) {
		case TRIG:
			//State Generation
			GPIOA->ODR |= GPIO_ODR_OD12;
			index_time += 1;
			nbr_un=0;
			if (index_time == 1){
				state = ECHO;
			}
			break;
		case ECHO:
			//State Reception
			if (index_time < INDEX_50ms){
				GPIOA->ODR &=~ GPIO_ODR_OD12;
				index_time++;
				if(GPIOA->IDR & GPIO_IDR_ID8) nbr_un++;
			} else { //index_time >= INDEX_50ms
				index_time = 0;
				fin_reception = 1;
				nbrUnFinal = nbr_un;
				state = TRIG;
			}
			break;
	}
}

void ComputeDistance(void){
	distance = ((float)nbrUnFinal * 10.0f) / 58.8235f; // in cm
	fin_reception = 0;
}

//PA5 need to be in input mode
void initGPIORadar(void){
	//=====Init GPIO=======
	/*
        PA1 : logical output - PWM (period echantillonage) - TIM2-CH2
	PA5: logical output - Period interruption (toogle from PA1 overflow)
 	PA12 : output numerical - Trig
	PA8 : entree numerique - Echo
 	*/

	RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN;

	//MODER Mode Output
	GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER12_Msk
			| GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER5_Msk);

	GPIOA->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODER1_Pos
			| GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER5_Pos
			| GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER12_Pos
	              	| GPIO_MODE_INPUT << GPIO_MODER_MODER8_Pos);

	//OTYPER Mode push-pull = 0 (push-pull)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_12
			| GPIO_OTYPER_OT_1 | GPIO_OTYPER_OT_5);

	//Reset OSPEEDR a 0 puis set a medium
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8_Msk
											| GPIO_OSPEEDR_OSPEED12_Msk
											| GPIO_OSPEEDR_OSPEED5_Msk
											| GPIO_OSPEEDR_OSPEED1_Msk);


	GPIOA->OSPEEDR |= (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED8_Pos)
									| (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED12_Pos)
									| (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED1_Pos)
									| (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED5_Pos);

	// Port no-pull
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk
									| GPIO_PUPDR_PUPD12_Msk
									| GPIO_PUPDR_PUPD1_Msk
									| GPIO_PUPDR_PUPD5_Msk);

	GPIOA->PUPDR |= (GPIO_NOPULL << GPIO_PUPDR_PUPD8_Pos
										| GPIO_NOPULL << GPIO_PUPDR_PUPD12_Pos
										| GPIO_NOPULL << GPIO_PUPDR_PUPD1_Pos
										| GPIO_NOPULL << GPIO_PUPDR_PUPD5_Pos);

	// function register for alternate function mode
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL1_Msk);
	GPIOA->AFR[0] |= (GPIO_AF1_TIM2 << GPIO_AFRL_AFSEL1_Pos);
}

void initTimerRadar(void){
	//Enable le clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//Configure the counter
	TIM2->SMCR &= ~TIM_SMCR_SMS;
	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);  //Count up and edge-aligned
	TIM2->CR1 |= TIM_CR1_ARPE;

	TIM2->PSC=179; //On veut 1MHz depuis 180Hz (formule CM4 pg 6)
	TIM2->ARR=99; //on veut 100u => 10kHz => ARR

	//Timer sur channel 2 (PA1 - AF01)
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M_Msk); //CC2 channel is mode output (CC2S = 0)

	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //TIM_CCMR1_OC2PE
							//| TIM_OCMODE_PWM1;  // = 6 = mode PWM1

	//Manage output preload enable
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;

	//Enable output compare
	
	TIM2->CCER &= ~(TIM_CCER_CC2E_Msk | TIM_CCER_CC2NP);
	TIM2->CCER |= TIM_CCER_CC2E;

	//Set duty cycle (50%)
	TIM2->CCR2 = 50-1;

	//======= INTERRUPTION ==========
	//Enable the generation of an interrupt when the timer overflows or reaches the update event
	TIM2->DIER |= (TIM_DIER_UIE | TIM_DIER_CC2IE);
	
	TIM2->SR &= ~(TIM_SR_UIF);	//Overflow flag
	TIM2->SR &=~TIM_SR_CC2IF;	//Capture/compare flag

	//Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	__enable_irq();
}

void TIM2_IRQHandler(void) {
	//interruption type debordement
	//UIF - update event occurs flags, if = 1, an event is occured
	if (TIM2->SR & TIM_SR_UIF) {
		GPIOA->ODR ^= GPIO_ODR_OD5;
		TIM2->SR &= ~(TIM_SR_UIF);  // Clear pending flag
		// Handle your interrupt here
		UltraSoundMgt();
	}
}


/*
int main(void){

	System_Clock_Init(); // Switch System Clock to maximum
  //TP2_US();

	//Configure PB8 comme output
	initGPIORadar();
	initTimerRadar();

	while(1) {
		//The interruption handler is called automatically since it is linked to the timer
		if (fin_reception == 1){
			ComputeDistance();
		}
	}
}

*/
