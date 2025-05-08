//
// Created by giangvu on 5/6/25.
//

#include "moteur.h"

uint8_t robot_speed_right = 0;
uint8_t robot_speed_left = 0;

typedef enum {
	FORWARD,
	BACKWARD,
	TURN_LEFT,
	TURN_RIGHT
} robot_direction;
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

    // Set TIM2 to upcounting mode, edge-aligned, no slave mode
    //Disable Slave Mode Control Register (SMCR),
    //TIM2 is now a master timer, bc it generates PWM instead of listen to interruption
    TIM2->SMCR &= ~TIM_SMCR_SMS;

    //Set Direction = 0 (up-counting) and CMS = 00 (Edge-aligned mode)
    TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);

    // Prescaler: 180MHz / (89 + 1) = 2MHz timer clock
    TIM2->PSC = 89;

    // Auto-reload: 2MHz / (99 + 1) = 20kHz PWM frequency, timer overflow after 50 us
    TIM2->ARR = 99;

    // ===== CONFIGURE TIM2 OUTPUT COMPARE FOR PWM MODE =====
    // Work with CCMR (Capture/Compare mode register; CCMR1 manage channel 1 and 2; CCMR2 channel 3-4 )
    // RIGHT WHEEL - TIM2_CH2 (PB9)
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M_Msk);   // CC2S = 00 - output compare mode
    TIM2->CCMR1 |= TIM_CCMR2_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;      // PWM mode 1 = 110 (first 2 bits are set to 1)
    TIM2->CCER &= ~TIM_CCER_CC2P;                            // CH2 Polarity : Active High = 0 (output will be high during actuve time - counter < CCR2)
    TIM2->CCER |= TIM_CCER_CC2E;                             // Enable output

    // LEFT WHEEL - TIM2_CH3 (PB10)
    TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M_Msk);
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE | TIM_OCMODE_PWM1;   // Preload enable (=1) and PWM mode 1
    TIM2->CCER &= ~TIM_CCER_CC3P;                       // Active High
    TIM2->CCER |= TIM_CCER_CC3E;                        // Enable output

    // Set initial duty cycle to 0 (stopped)
    TIM2->CCR2 = 0;     // Ranging from 0-99% since ARR is set to 99
    TIM2->CCR3 = 0;

    // Start the timer
    TIM2->CNT = 0;                            //Reset counter
    TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;  // Enable counter + Auto-reload preload
}

void initPWM(void){
    initGPIOMoteur();
    initPWMMoteur();
}

/// @brief Sets PWM duty cycles and direction based on speed and direction flags.
/// @param rightspeed Duty cycle for right wheel (0-255) - need to scale, if 0-99 then assign directly
/// @param leftspeed Duty cycle for left wheel (0-255)
/// @param direction 0 = forward, 1 = backward
/// If speed = 0, le rapport cyclique doit etre mis a 0
void setPWMParameters(unsigned char rightspeed, unsigned char leftspeed, unsigned char direction) {
    robot_direction = direction;
    robot_speed_right = rightspeed;
    robot_speed_left = leftspeed;

    if (direction == 0) {
        // Move forward: set direction pins high
        GPIOB->ODR |= (GPIO_ODR_OD8 | GPIO_ODR_OD12);
        //Since the speed is from 0-255, we need to scale it to 0-100%
        if (rightspeed == 0) TIM2->CCR2 = 0;
        else TIM2->CCR2 = (rightspeed * TIM2->ARR)/255;
        if (leftspeed == 0) TIM2->CCR3 = 0;
        else TIM2->CCR3 = (leftspeed * TIM2->ARR)/255;
    }

    else if (direction == 1) {
        // Move backward: reset direction pins = 0
        GPIOB->BSRR |= ~(GPIO_BSRR_BR_8 | GPIO_BSRR_BR_12);
        if (rightspeed == 0) TIM2->CCR2 = 0;
        else TIM2->CCR2 = ((100 - rightspeed) * TIM2->ARR)/255;
        if (leftspeed == 0) TIM2->CCR3 = 0;
        else TIM2->CCR3 = ((100 - leftspeed) * TIM2->ARR)/255;
    }
}

/// @brief Sets the same speed for both wheels in the current direction.
/// @param speed Duty cycle (0-255)
/// Une trop busque modification de la vitesse n'est pas toujours souhaitable...
void setSpeed(unsigned char speed) {
    while (speed > robot_speed) {
        robot_speed += SPEED_STEP;
		//Si ca depasse
        if (robot_speed > speed) robot_speed = speed;
        setPWMParameters(robot_speed, robot_speed, 0);
    }
    while (speed < robot_speed) {
		robot_speed -= SPEED_STEP;
		if (robot_speed < speed) robot_speed = speed;
		setPWMParameters(robot_speed, robot_speed, 0);
    }
}

///@breif Tell robot to move forward
void goForward(){
	//If the motrer has not been initialized
	if (pwm_init == 0){
    	initPWM();
		pwm_init = 1;
	}
	//If the robot has not been moving forward
	if(robot_direction != 0){
		setPWMParameters(robot_speed_right, robot_speed_left, 0);
	}
}

void goBackwards(){
	if (pwm_init == 0){
    	initPWM();
		pwm_init = 1;
	}
	//If the robot has not been moving backwards
	if(robot_direction != 1){
		setPWMParameters(robot_speed_right, robot_speed_left, 1);
	}
}

void stop(void ){
	if(robot_speed != 0){
		setSpeed(0);
	}
}

void turnLeft(void ){
	if (pwm_init == 0){
    	initPWM();
		pwm_init = 1;
	}
	setPWMParameters(robot_speed, robot_speed/4, 0);
	robot_direction = TURN_LEFT;
}

void turnRight(void){
	if (pwm_init == 0){
    	initPWM();
		pwm_init = 1;
	}
	setPWMParameters(robot_speed/4, robot_speed, 0);
	robot_direction = TURN_RIGHT;
}

void goStraight(void){
	setPWMParameters(robot_speed, robot_speed, 0);
}


