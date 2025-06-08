//
// Created by giangvu on 5/6/25.
//

#include "moteur.h"

#define DIR_FORWARD 0
#define DIR_BACKWARD 1

uint8_t robot_direction;	//direction 0 = forward, 1 = backward
uint8_t robot_speed = 0;
uint8_t pwm_init = 0;				//A boolean to know whether the moter is initialised

/// @brief Initializes GPIO pins (PB8 and PB12 for Direction)
void initGPIOMoteur(void){

    // Enable clock for GPIOB without touching other GPIOs
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Set PB8 and PB12 as general purpose output mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER12_Msk);
    GPIOB->MODER |= (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER8_Pos)
                 |  (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER12_Pos);

    // Set output type to push-pull (default)
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_12);

    // Set medium speed for both pins
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED12_Msk);
    GPIOB->OSPEEDR |= (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED8_Pos)
                   |  (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED12_Pos);

    // Disable pull-up/pull-down resistors
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk | GPIO_PUPDR_PUPD12_Msk);
}

/// @brief Initializes TIM2 to generate PWM signals for motor control (PB9 and PB10).
void initPWMMoteur(void) {
    // ===== GPIO SETUP FOR PWM OUTPUT CHANNELS (PB9 = CH2, PB10 = CH3) =====

    // Set PB9 and PB10 as alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);
    GPIOB->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODER9_Pos)
                 |  (GPIO_MODE_AF_PP << GPIO_MODER_MODER10_Pos);

    // Push-pull output
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10);

    // Set medium speed
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED9_Msk | GPIO_OSPEEDR_OSPEED10_Msk);
    GPIOB->OSPEEDR |= (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED9_Pos)
                   |  (GPIO_SPEED_FREQ_MEDIUM << GPIO_OSPEEDR_OSPEED10_Pos);

    // No pull-up/pull-down
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD9_Msk | GPIO_PUPDR_PUPD10_Msk);

    // Set alternate function TIM2 for PB9 and PB10
    //AFR[1] for pins 8-15
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL9_Msk | GPIO_AFRH_AFSEL10_Msk);
    //Connect PB9 and PB10 to canals TIM2_CH2 and TIM2_CH3 (implicitement - voir datasheet 1, pg.68/202)
    GPIOB->AFR[1] |= (GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL9_Pos)
                  |  (GPIO_AF1_TIM2 << GPIO_AFRH_AFSEL10_Pos);

    // ===== TIMER 2 CONFIGURATION FOR PWM OUTPUT =====

    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set TIM2 to upcounting mode, edge-aligned, no slave mode (OPTIONAL BUT SHOULD DO)
    // Disable Slave Mode Control Register (SMCR),
    //TIM2 is now a master timer, bc it generates PWM instead of listen to interruption
    TIM2->SMCR &= ~TIM_SMCR_SMS;

    // Prescaler: 180MHz / (89 + 1) = 2MHz timer clock
    TIM2->PSC = 90 - 1;

    // Auto-reload: 2MHz / (99 + 1) = 20kHz PWM frequency, timer overflow after 50 us
    TIM2->ARR = 100 - 1;

    //Set Direction = 0 (up-counting) and CMS = 00 (Edge-aligned mode)
    TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);

    //Enable Auto-reload preload enable (OPTIONAL BUT SHOULD DO EXPLICITELY)
    TIM2->CR1 |= TIM_CR1_ARPE;

    // ===== CONFIGURE TIM2 OUTPUT COMPARE FOR PWM MODE =====
    // Work with CCMR (Capture/Compare mode register; CCMR1 manage channel 1 and 2; CCMR2 channel 3-4 )
    // RIGHT WHEEL - TIM2_CH2 (PB9)
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M_Msk);   // CC2S = 00 - output compare mode, 
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE 			     // TIM2_CCR1 can only be written at update
	         | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;      // PWM mode 1 = 110 (first 2 bits are set to 1)
    
    TIM2->CCER &= ~TIM_CCER_CC2P;                            // CH2 Polarity : Active High = 0 (output will be high during actuve time - counter < CCR2)
    TIM2->CCER |= TIM_CCER_CC2E;                             // Enable output

    // LEFT WHEEL - TIM2_CH3 (PB10)
    TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M_Msk);
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // Set OC3M = 110 for PWM1
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE; 			// Enable preload
	
    TIM2->CCER &= ~TIM_CCER_CC3P;                       // Active High
    TIM2->CCER |= TIM_CCER_CC3E;                        // Enable output

    //Clear unecessary things for a known default state
    TIM2->CCER &=~(TIM_CCER_CC2NP|TIM_CCER_CC3NP);      //Explicitely clear for active high only
    TIM2->DIER &=~(TIM_DIER_CC2DE|TIM_DIER_CC2IE|TIM_DIER_CC3DE|TIM_DIER_CC3IE);   //No interrupt on capture/compare and DMA
	
    // Set initial duty cycle to 0 (stopped)
    TIM2->CCR2 = 0;     // Duty cycle for channel 2 - mapped to PA9
    TIM2->CCR3 = 0;	//Duty cycle for channel 3 - mapped to PA10

	
    // Start the timer
    TIM2->CNT = 0;                            //Reset counter
    TIM2->CR1 |= TIM_CR1_CEN;  // Enable counter
}

void initPWM(void){
    initGPIOMoteur();
    initPWMMoteur();
    pwm_init=1;
}

/// @brief Sets PWM duty cycles and direction based on speed and direction flags.
/// @param rightspeed Duty cycle for right wheel (0-255) - need to scale, if 0-99 then assign directly
/// @param leftspeed Duty cycle for left wheel (0-255)
/// @param direction 0 = forward, 1 = backward
/// If speed = 0, le rapport cyclique doit etre mis a 0
void setPWMParameters(unsigned char rightSpeed, unsigned char leftSpeed, unsigned char direction){
	if(direction == DIR_FORWARD){
		//If we are going forward, PB8 and PB12 (direction GPIO) should both be high
		//BSRR is a write-only register (so no |= because it will read BSRR to do the calculation)
		GPIOB->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_12;   // set bits
		TIM2->CCR2 = 100-rightSpeed;
		TIM2->CCR3 = 100-leftSpeed;
	}else if (direction == DIR_BACKWARD){
		GPIOB->BSRR = GPIO_BSRR_BR_8 | GPIO_BSRR_BR_12;   //reset bits
		TIM2->CCR2 = rightSpeed;
		TIM2->CCR3 = leftSpeed;
	}
}

/// @brief Sets the same speed for both wheels in the current direction.
/// @param speed Duty cycle 
/// Une trop busque modification de la vitesse n'est pas toujours souhaitable...
void setSpeed(unsigned char speed) {
    robot_speed = speed;  // update desired speed
}

///@breif Tell robot to move forward
void goForward(){
	//If the motrer has not been initialized
	if (pwm_init == 0){
    	    initPWM();
	}
	//If the robot has not been moving forward
	if(robot_direction != DIR_FORWARD){
		robot_direction = 0;
		setPWMParameters(robot_speed, robot_speed, robot_direction);
	}
}

void goBackward(){
	if (pwm_init == 0){
    	    initPWM();
	}
	//If the robot has not been moving backwards
	if(robot_direction != DIR_BACKWARD){
		robot_direction = 1;
		setPWMParameters(robot_speed, robot_speed, robot_direction);
	}
}

void stop(void ){
	if(robot_speed != 0){
		setSpeed(0);
		setPWMParameter(0,0,0);
	}
}

void turnLeft(void ){
	if (pwm_init == 0)	initPWM();
	setPWMParameters(robot_speed, robot_speed/4, ~robot_direction);
}

void turnRight(void){
	if (pwm_init == 0) 	initPWM();
	setPWMParameters(robot_speed/4, robot_speed, robot_direction);
}

void goStraight(void){
	setPWMParameters(robot_speed, robot_speed, robot_direction);
}

/*----------------------------------------------------------------------------
 MAIN function
 *----------------------------------------------------------------------------*/
	
/**
 * \brief     Main function entry
 * \details   
 * \return    \e none.
 */
/*
int main(void){

	System_Clock_Init(); // Switch System Clock to maximum		
	init_PWM();
	
	for(;;){
		setSpeed(80);
		goForward();
		delay(4000);
		stop();
		delay(4000);
		turnLeft();
		delay(8000);
		stop();
		delay(4000);
		goBackward();
		delay(4000);
		stop();
		delay(1000);
		turnRight();
		delay(1000);
		stop();
	}
}
*/

