/**
 * \file      main.c
 * \brief     Main function
 *
 */
 
#include "main.h"

unsigned char robot_direction = 0;
unsigned char robot_rightspeed = 0;
unsigned char robot_leftspeed = 0;

/*----------------------------------------------------------------------------
 MAIN function
 *----------------------------------------------------------------------------*/
	
/**
 * \brief     Main function entry
 * \details   
 * \return    \e none.
 */

									
			
void initPWM(){
	//=====Init GPIO=======
	
	//GPIO Direction en mode output
	//Enable le clock
	RCC->AHB1ENR = RCC_AHB1ENR_GPIOBEN;
	
	//MODER Mode Output
	GPIOB->MODER &= ~(GPIO_MODER_MODER8_Msk
										| GPIO_MODER_MODER12_Msk);
	GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER8_Pos
								| GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER12_Pos;
	
	//OTYPER Mode push-pull = 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_8 
										| GPIO_OTYPER_OT_12);
	
	//Reset OSPEEDR a 0
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8_Msk
											| GPIO_OSPEEDR_OSPEED12_Msk);
	
	// Set frequence medium
	GPIOB->OSPEEDR |= (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED8_Pos)
									| (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED12_Pos);
	
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk
									| GPIO_PUPDR_PUPD12_Msk);
	
	GPIOB->PUPDR |= ~(GPIO_NOPULL << GPIO_PUPDR_PUPD8_Pos
										|GPIO_NOPULL << GPIO_PUPDR_PUPD12_Pos );	
	
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk
									| GPIO_AFRH_AFSEL12_Msk);
					
	GPIOB->AFR[1] &= ~(GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL8_Pos
									| GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL12_Pos);
							
	
	
	//=====POUR LES PWM======
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->SMCR &= TIM_SMCR_SMS;
	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	
	TIM2->PSC=89; //On veut 2MHz depuis 180Hz (formule CM4 pg 6)
	TIM2->ARR=99;
	
	//GPIO PWM en Alternate mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER9_Msk
										| GPIO_MODER_MODER10_Msk);
	GPIOB->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODER9_Pos
								| GPIO_MODE_AF_PP << GPIO_MODER_MODER10_Pos;
	
	//OTYPER Mode push-pull = 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_9 
										| GPIO_OTYPER_OT_10);
	
	//Reset OSPEEDR a 0
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED9_Msk
											| GPIO_OSPEEDR_OSPEED10_Msk);
	
	// Set frequence medium
	GPIOB->OSPEEDR |= (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED9_Pos)
									| (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED10_Pos);
	
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD9_Msk
									| GPIO_PUPDR_PUPD10_Msk);
	
	GPIOB->PUPDR |= ~(GPIO_NOPULL << GPIO_PUPDR_PUPD9_Pos
										| GPIO_NOPULL << GPIO_PUPDR_PUPD10_Pos);
	
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL9_Msk
									| GPIO_AFRH_AFSEL10_Msk);
	

					
	GPIOB->AFR[1] |= GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL9_Pos
									| GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL10_Pos);
									
  //=========Output compare===========
	// ROUE DROITE
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S
									| TIM_CCMR1_OC2M_Msk);
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 ;//TIM_CCMR1_OC2PE
									| TIM_OCMODE_PWM1;
		
	TIM2->CCER &= ~TIM_CCER_CC2P;	
	TIM2->CCER |= TIM_CCER_CC2E; //ACTIF HIGH
	
	//ROUE GAUCHE
	TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S
									| TIM_CCMR2_OC3M_Msk);
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE
									| TIM_OCMODE_PWM1;
		
	TIM2->CCER &= ~TIM_CCER_CC3P;	
	TIM2->CCER |= TIM_CCER_CC3E; //ACTIF HIGH
	
	//Set duty cycle pour qu'il s'arrete
	TIM2->CCR3=0;
	TIM2->CCR2=0;
	
	//Demarrage timer
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 |=TIM_CR1_ARPE;				
}	

void setPWMParameters(unsigned char rightspeed, unsigned char leftspeed, unsigned char direction){
	robot_direction = direction;
	robot_leftspeed = leftspeed;
	robot_rightspeed = rightspeed;
	if(direction==1){
	//vers avant
		GPIOB->ODR |= (GPIO_ODR_OD8 | GPIO_ODR_OD12);
		TIM2->CCR2=rightspeed;
		TIM2->CCR3=leftspeed;
	}
	else if(direction == 0){
	//vers derrier
		GPIOB->BSRR |= (GPIO_BSRR_BR_8 | GPIO_BSRR_BR_12);
		TIM2->CCR2=100-rightspeed;
		TIM2->CCR3=100-leftspeed;
	}
}

void setSpeed(unsigned char speed){
	setPWMParameters(speed,speed,robot_direction);
}

void goForward(){
	initPWM();
	setPWMParameters(50, 50, 1);
	setSpeed(50);
}

	
int main(void){

	System_Clock_Init(); // Switch System Clock to maximum
  //TP2_US();
	
	//Configure PB8 comme output
	initPWM();
	
	while(1) {
		//Set PB8 to HIGH to open LED
		goForward;
		}		
}


