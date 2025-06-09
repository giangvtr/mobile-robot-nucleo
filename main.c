/**
 * \file      main.c
 * \brief     Main function
 *
 */
 
#include "main.h"

#define MY_COMMAND 0xA07F
#define MAX_BITS 32

/* Variables could be modified by an interupt handler */
volatile int code =0;         //will be modify often so we need to separate it with code_IR
volatile int bit_index = -1;
volatile int fin_reception = 0;
volatile int code_IR;


/*
uint32_t reverse32(uint32_t n) {
    uint32_t rev = 0;
    for (int i = 0; i < 32; i++) {
        rev <<= 1;
        rev |= (n & 1);
        n >>= 1;
    }
    return rev;
}

*/

/*----------------------------------------------------------------------------
 MAIN function
 *----------------------------------------------------------------------------*/
        
/**
 * \brief     Main function entry
 * \details  
 * \return    \e none.
 */

 /* global variables used in normal functions */
unsigned char pressed = 0; //boolean
unsigned char key = 0xFF; //derniere touche appuye
uint8_t oldData;

typedef struct{
	uint16_t address;
	uint8_t command;
} trameDecode;

trameDecode maTrameDecode;

char decodeTrame(trameDecode* trame) {
    trame->address = (code_IR >> 16) & 0xFFFF;
    uint8_t data = (code_IR >> 8) & 0xFF;
    uint8_t data_rev = code_IR & 0xFF;

    if (data != (uint8_t)(~data_rev)) {
        return -1;  // Données invalides
    }

    trame->command = data;
    return 1; // Donnée valide
}

unsigned char wasKeyPressed(void) {
    if (decodeTrame(&maTrameDecode) != 1) {
        return 0; // Trame invalide
    }

    if (maTrameDecode.address != MY_COMMAND) {
        return 0; // Mauvaise adresse
    }

    return 1; // OK
}


unsigned char getKeyNumber(void) {
    key = maTrameDecode.command;
    return key;
}

void initIRTimer(void){
        /* Configure in Input capture mode */
        /* Enable peripheral Clock for TIM5-CH2 (APB1 bus) et pin PA1 */
        RCC -> APB1ENR |= RCC_APB1ENR_TIM5EN ;
        RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;

        /* GPIO pin config . alternate function 1 to connect TIM5 - CH2 to PA1 */
        GPIOA -> MODER &= ~(GPIO_MODER_MODE1_Msk) ;
        GPIOA -> MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE1_Pos;

        //Push-pull
	GPIOA->OTYPER &=~(GPIO_OTYPER_OT_1);
	
        //Low speed 2Mhz
	GPIOA->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED1);

        /* Configurer les résistances de Pull-Up / Pull-Down */
        GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPDR1); //Mode no pull
        // 00: GPIO_NOPULL | 01: GPIO_PULLUP
        // 10: GPIO_PULLDOWN | 11: Reserved

        GPIOA ->AFR[0] &= ~GPIO_AFRL_AFSEL1 ;                    // Reset AF2 (pin Low in AFR[0])
        GPIOA ->AFR[0] |= GPIO_AF2_TIM5 << GPIO_AFRL_AFSEL1_Pos ; //On aattend AFRL = 0xX2X ????
                                                                                        

        //*************************************************************** */
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
        TIM5 -> CCMR1 &= ~(TIM_CCMR1_IC2F ; // No filter for ch2
                        | TIM_CCMR1_IC2PSC ; // No prescaler, capture each time an edge is detected

        // Enable UDIS and URS
        //TIM5 -> CR1 |= TIM_CR1_URS;
        //TIM5 -> CR1 |= TIM_CR1_UDIS;
        
        /* Input capture enable on channel 2 */
        TIM5 -> CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP ) ; // Falling edge detect
        TIM5 -> CCER |= TIM_CCER_CC2P  
                        | TIM_CCER_CC2E ; // Capture Enable

        // Set slave mode to Reset, so CNT resets on TI2 trigger
        TIM5->SMCR &= ~(TIM_SMCR_SMS_Msk | TIM_SMCR_TS_Msk);
        TIM5->SMCR |= (0b100 << TIM_SMCR_SMS_Pos)    // SMS = 100 = Reset mode
                        | (0b101 << TIM_SMCR_TS_Pos);    // TS = 101 = TI2FP2

        /* Enable interrupt flag when capture occurs on channel 2 */
        TIM5 -> DIER |= TIM_DIER_CC2IE ; // Enable interrupt when falling edge captured
        TIM5 -> SR &= ~TIM_SR_CC2IF ; // Clear the Capture event flag for channel 2
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
        if (TIM5->SR & TIM_SR_CC2IF) {
                //Case an interrupt is detected (front descendant)
                TIM5->SR &= ~(TIM_SR_CC2IF);    //clear flag

                //Detect bit value and add in a buffer
                uint32_t pulse_duration = TIM5->CCR2;  // Valeur du timer au front descendant capte sur CH2-TIM5

                if(pulse_duration > 2200 && pulse_duration < 2600){
                //This is the SOF signal, nothing is put into the code_IR buffer
                //Use 12000 since the resolution is in 5.625 us, we want a frame that covers 13.5 ms => 2400 fois
                        bit_index = 0;
                        code = 0;
                }

                //Code repete
                if (pulse_duration > 1800 && pulse_duration < 2200){
                        bit_index=-1;
                }

                else if (bit_index >= 0 && bit_index < MAX_BITS) {
                        //Bit 0: 200 fois 5.625 us
                        if (pulse_duration > 150 && pulse_duration < 300) {
                                code |= 0x0 << (31 - bit_index);
                                bit_index++;
                        //Bit 1: 400 fois
                        } else if (pulse_duration > 300 && pulse_duration < 500) {
                                code |= 0x1 << (31 - bit_index);
                                bit_index++;
                        }
                }
                
                //Trame complete recue
                if (bit_index >= MAX_BITS) {
                        // Convertir le buffer en entier 32 bits
                        fin_reception = 1;
                        code_IR = code;
                        code = 0;
                        bit_index = 0;
                }
        }
        else if (TIM5->SR & TIM_SR_UIF) {
                //Case an overflows clock
                //Means that the IR Reception is taking for too long
                TIM5->SR &= ~(TIM_SR_UIF);  // Clear pending flag
                // Vider le buffer pour recommencer
                fin_reception = 0;
                code=0;
                bit_index=-1;
        //NVIC_ClearPendingIRQ(TIM5_IRQn) ;
        }
}
        
int main(void){

        System_Clock_Init(); // Switch System Clock to maximum
        
        initIRTimer();
        
        while(1) {
                if (fin_reception) {
        f               in_reception = 0;

                        if (wasKeyPressed()) {
                                unsigned char touche = getKeyNumber();
                                // do something with touche (e.g. display it or act upon it)
                        }
                }
        }                
}

