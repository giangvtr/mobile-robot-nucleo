//
// Created by giangvu on 5/12/25.
//

#include "ultrasound.h"

static unsigned short state = 0;

void UltraSoundMgt(void){
	//Will be call repetedly
	//voir dans l'enonce, on peut constater que il s'agit un FSM de 2 states : Generation and Reception
	switch (state) {
		case 0:
			//State Generation
			GPIOx_Set_Output_Pin(PA_12);
			index_time += 1;
			nbr_un=0;
			if (index_time == 1){
				state = 1;
			}
			break;
		case 1:
			//State Reception
			if (index_time < INDEX_50ms){
				GPIOx_Reset_Output_Pin(PA_12);
				index_time++;
				nbr_un += GPIOx_Read_Input_Pin(PA_8);
			} else { //index_time >= INDEX_50ms
				index_time = 0;
				fin_reception = 1;
				nbrUnFinal = nbr_un;
				state = 0;
			}
			break;
	}
}

void ComputeDistance(void){
	float T_dist = nbrUnFinal * (100.0f / 1000000.0f);
	distance = (unsigned short)(T_dist / 58.8235f);
}

//PA5 need to be in input mode
void initGPIORadar(void){
	//=====Init GPIO=======

	RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN;

	//MODER Mode Output
	GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk
										| GPIO_MODER_MODER12_Msk
										| GPIO_MODER_MODER1_Msk
										| GPIO_MODER_MODER5_Msk);

	GPIOA->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODER1_Pos
								| GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER5_Pos
								| GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER12_Pos
	              | GPIO_MODE_INPUT << GPIO_MODER_MODER8_Pos);

	//OTYPER Mode push-pull = 0 (push-pull)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_8
										| GPIO_OTYPER_OT_12
										| GPIO_OTYPER_OT_1
										| GPIO_OTYPER_OT_5);

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
	TIM2->SMCR &= TIM_SMCR_SMS;
	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);  //Count up and edge-aligned

	TIM2->PSC=179; //On veut 1MHz depuis 180Hz (formule CM4 pg 6)
	TIM2->ARR=99; //on veut 100u => 10kHz => ARR

	//Timer sur channel 2 (PA1 - AF01)
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S); //CC2 channel is mode output (CC2S = 0)

	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //TIM_CCMR1_OC2PE
							//| TIM_OCMODE_PWM1;  // = 6 = mode PWM1

	//Manage output preload (PAS SURE - disabled)
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2PE_Msk);

	//Enable output compare
	TIM2->CCER &= ~(TIM_CCER_CC2E_Msk);
	TIM2->CCER |= TIM_CCER_CC2E;

	//Set duty cycle (50%)
	TIM2->CCR2 = 50;

	//Enable the generation of an interrupt when the timer overflows or reaches the update event
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->SR &= ~(TIM_SR_UIF);

	//Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 |=TIM_CR1_ARPE;

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_ClearPendingIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
	//interruption type debordement
	//UIF - update event occurs flags, if = 1, an event is occured
	if (TIM2->SR & TIM_SR_UIF) {
		GPIOx_Togle_Output_Pin(PA_5);
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
			fin_reception = 0;
			ComputeDistance();
		}
	}
}

*/
