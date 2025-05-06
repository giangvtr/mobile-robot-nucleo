#include "ultrason.h"


void initGPIORadar(){
	//=====Init GPIO=======
	RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN;
	
	//MODER Mode Output
	GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk
										| GPIO_MODER_MODER12_Msk
										| GPIO_MODER_MODER1_Msk
										| GPIO_MODER_MODER5_Msk);
	
	GPIOA->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODER1_Pos
								| GPIO_MODE_AF_PP << GPIO_MODER_MODER5_Pos
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
	
	GPIOA->PUPDR |= ~(  GPIO_NOPULL << GPIO_PUPDR_PUPD8_Pos
										| GPIO_NOPULL << GPIO_PUPDR_PUPD12_Pos
										| GPIO_NOPULL << GPIO_PUPDR_PUPD1_Pos
										| GPIO_NOPULL << GPIO_PUPDR_PUPD5_Pos);	
	
	// function register
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk
									| GPIO_AFRH_AFSEL12_Msk);
									
	GPIOA->AFR[1] &= ~(GPIO_AFRL_AFSEL1_Msk
									| GPIO_AFRL_AFSEL5_Msk);								
					
	GPIOA->AFR[1] |= (GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL8_Pos
									| GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL12_Pos);
							
	GPIOA->AFR[1] |= (GPIO_AF1_TIM2 << GPIO_AFRL_AFSEL1_Pos
									| GPIO_AF1_TIM2 << GPIO_AFRL_AFSEL5_Pos);
}

void initTimerRadar(){
	//Enable le clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	//Configure the counter
	TIM2->SMCR &= TIM_SMCR_SMS;
	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	
	TIM2->PSC=179; //On veut 1MHz depuis 180Hz (formule CM4 pg 6)
	TIM2->ARR=99; //on veut 100u => 10kHz => ARR 
	
	//Timer sur channel 2 (PA1 - AF01)
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S); //CC2 channel is mode output
	
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //TIM_CCMR1_OC2PE
							//| TIM_OCMODE_PWM1;  // = 6 = mode PWM1
	
	//Manage output preload (disables)
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2PE_Msk);
	
	//Enable output compare
	TIM2->CCER &= ~(TIM_CCER_CC2E_Msk);
	TIM2->CCER |= TIM_CCER_CC2E; 
	
	//Set duty cycle (50%)
	TIM2->CCR2 = 50;
	
	//Enable interrupt
	TIM2->DIER |= TIM_DIER_UIE;
	
	NVIC_SetPriority(TIM_IRQn, 3);
	NVIC_ClearPending(TIM_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	
	//Enable counter
	TIM2->CNT = 0;
	
	//Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 |=TIM_CR1_ARPE;	
}


