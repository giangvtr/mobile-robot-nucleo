/**
 * \file      main.c
 * \brief     Main function
 *
 */
 
#include "main.h"

/*----------------------------------------------------------------------------
 MAIN function
 *----------------------------------------------------------------------------*/
	
/**
 * \brief     Main function entry
 * \details   
 * \return    \e none.
 */
#define REMOTE_AD_MSK 0xFFFF0000
#define REMOTE_ADDRESS 0xFFFFFFFF     //a changer eventuellement
#define CMD_MSK 0x0000FF00
#define NOT_CMD_MSK 0x000000FF
#define MAX_BITS 32

/* Variables could be modified by an interupt handler */
volatile int bit_buffer[MAX_BITS]; 	//will be modify often so we need to separate it with code_IR
volatile int bit_index = 0;
volatile uint32_t last_capture = 0;
volatile int fin_reception = 0;

/* global variables used in normal functions */
unsigned char pressed = 0; //boolean
unsigned char key; //derniere touche appuye
int code_IR;


unsigned char wasKeyPressed(void){
	if((code_IR & REMOTE_AD_MSK) == REMOTE_ADDRESS))
		&& (~code_IR & NOT_CMD_MSK) == (code_IR & CMD_MSK)){
		pressed = 1;
		return pressed;
	}
	pressed = 0;
	return 0; //La fonction rend initialement 0
}

unsigned char getKeyNumber(void){
	key = code_IR & CMD_MSK;
	return key;
}

  void initIRTimer(void){
    /* Configure in Input capture mode */
    /* Enable peripheral Clock for TIM5-CH2 (APB1 bus) et pin PA1 */
    /* Enable input pin for PA3 */
    RCC -> APB1ENR |= RCC_APB1ENR_TIM5EN ;
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;

    /* GPIO pin config . alternate function 1 to connect TIM5 - CH2 to PA1 */
    GPIOA -> MODER &= ~ (GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE3_Msk) ;
    GPIOA -> MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE1_Pos;

    /* Configurer les résistances de Pull-Up / Pull-Down */
    GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPDR1); // Clear bits
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0;      // Mode pull-up
    // 00: GPIO_NOPULL | 01: GPIO_PULLUP
    // 10: GPIO_PULLDOWN | 11: Reserved

    GPIOA ->AFR[0] &= ~ GPIO_AFRL_AFSEL1 ;                    // Reset AF2 (pin Low in AFR[0])
    GPIOA ->AFR[0] |= GPIO_AF2_TIM5 << GPIO_AFRL_AFSEL1_Pos ; //On aattend AFRL = 0xX2X ????


    /* Clock definition in Reset mode (100 = 0x8U) */
    //We want it to automatically calculate the delta then reset the counter
    TIM5->SMCR &= ~(TIM_SMCR_SMS_Msk) ;
    TIM5->SMCR |= (0x4U << TIM_SMCR_SMS_Pos);  //Slave mode reset mode

    //Select the source of trigger (l'entree sur laquelle tu captures les fronts (ici est le signal IR)
    TIM5->SMCR &= ~TIM_SMCR_TS_Msk;
    TIM5->SMCR |= TIM_CLOCKSOURCE_TI2; //Capture channel 2 = TI2 input

    /* Counter Mode definition */
    TIM5->CR1 &= ~( TIM_CR1_DIR // Up counting
                                    | TIM_CR1_CMS) ; // Edge - aligned
    /* Prescaler value definition (pas definition pour ARR)*/
    //On veut la periode entre 2 fronts descendants est 13,5 ms
    TIM5->PSC = 1012; //resolution = 5.625 us
    TIM5->ARR = 10000;

    /* Input capture mode - channel configuration */
    TIM5 -> CCMR1 &= ~TIM_CCMR1_CC2S ; // CC2 channel bit is cleared
    TIM5 -> CCMR1 |= TIM_CCMR1_CC2S_0 ; // and IC2 mapped on TI2.
    TIM5 -> CCMR1 &= ~TIM_CCMR1_IC2F ; // No filter for ch2
    TIM5 -> CCMR1 &= ~TIM_CCMR1_IC2PSC ; // No prescaler, capture each time an edge is detected

    /* Input capture enable on channel 2 */
    TIM5 -> CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP ) ; // Falling edge detect
    TIM5 -> CCER |= TIM_CCER_CC2P
                                    | TIM_CCER_CC2E ; // Capture Enable

    /* Enable interrupt flag when capture occurs on channel 2 */
    TIM5 -> DIER |= TIM_DIER_CC2IE ; // Enable interrupt when falling edge captured
    TIM5 -> SR = ~TIM_SR_CC2IF ; // Clear the Capture event flag for channel 2
    // Enable update interrupt (triggered by counter overflow or update event)
    TIM5->DIER |= TIM_DIER_UIE;
    TIM5->SR &= ~(TIM_SR_UIF);

    //Enable counter
    TIM5->CR1 |= TIM_CR1_CEN;

    /* Enable interrupt in Interrupt controller */
    NVIC_SetPriority(TIM5_IRQn ,3) ;
    NVIC_ClearPendingIRQ(TIM5_IRQn) ;
    NVIC_EnableIRQ(TIM5_IRQn) ;
}

void TIM5_IRQHandler(void){
    //interruption type debordement
    if (TIM5->SR & TIM_SR_UIF) {
        //Case an overflows clock
        //Means that the IR Reception is taking for too long
        TIM5->SR &= ~(TIM_SR_UIF);  // Clear pending flag
        // Vider le buffer pour recommencer
        bit_index = 0;
        fin_reception = 0;
        code_IR = 0;
        return;
    }
    else if (TIM5->SR & TIM_SR_CC2IF) {
        //Case an interrupt is detected (front descendant)
        TIM5->SR &= ~(TIM_SR_CC2IF);
        //Detect bit value and add in a buffer
        uint32_t pulse_duration = TIM5->CCR2;  // Valeur du timer au front descendant
        if(pulse_duration > 2200 && pulse_duration < 2600 && bit_index == 0){
        //This is the SOF signal, nothing is put into the code_IR buffer
        //Use 12000 since the resolution is in 5.625 us, we want a frame that covers 13.5 ms => 2400 fois
            bit_index = 0;

        } else if (bit_index < MAX_BITS) {
            //Bit 0: 200 fois 5.625 us
            if (pulse_duration > 150 && pulse_duration < 300) {
                    bit_buffer[bit_index++] = 0;
            //Bit 1: 400 fois
            } else if (pulse_duration > 300 && pulse_duration < 500) {
                    bit_buffer[bit_index++] = 1;
            }
        }
        if (bit_index >= MAX_BITS) {
            // Convertir le buffer en entier 32 bits
            code_IR = 0; //Clear the old IR_command
            for (int i = 0; i < 32; i++) {
                    code_IR <<= 1;
                    code_IR |= bit_buffer[i];
            }
            fin_reception = 1;
            bit_index = 0;
        }
    }
}

	
int main(void){

	System_Clock_Init(); // Switch System Clock to maximum
  //TP2_US();
	
	//Configure PB8 comme output
	initIRTimer();
	
	while(1) {
		//The interruption handler is called automatically since it is linked to the timer
		if (fin_reception && wasKeyPressed()){
			getKeyNumber();
			fin_reception = 0;
		}
	}
}


