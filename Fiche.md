# RESUME CONFIGURATIONS GPIOs ET TIMER

## GPIO Configuration
### Mode input 
```c++

/* 1. Enable x GPIO port, remplace x par A, B, C, D, E*/
RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOxEN);

/* 2. Configure x.y to input mode */
GPIOx->MODER &= ~(GPIO_MODER_MODERy_Msk);    //Reset MODER bit (00 - input)

// 00: GPIO_MODE_INPUT | 01: GPIO_MODE_OUTPUT_PP
// 10: GPIO_MODE_AF_PP | 11: GPIO_MODE_ANALOG

/* 3. Configurer les résistances de Pull-Up / Pull-Down */
GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDRy_Msk); // Effacer d'abord
GPIOx->PUPDR |= GPIO_PUPDR_PUPDRy_1;      // 01 = Pull-down

// 00: GPIO_NOPULL | 01: GPIO_PULLUP
// 10: GPIO_PULLDOWN | 11: Reserved

/* 4. Read data in pins */
uint16_t port_state = GPIOA->IDR;                   //Read 16 pins of GPIOA
uint8_t pin_state = (GPIOA->IDR >> 5) & 0x01;       //Read only PA5
```

### Mode output push-pull
```c++
/* 1. Enable x GPIO port, remplace x par A, B, C, D, E*/
RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOxEN);

/* 2. Configure x.y to output mode */
GPIOx->MODER &= ~(GPIO_MODER_MODERy_Msk);    //Reset MODER bit
GPIOx->MODER |= GPIO_MODER_MODERy_0;         //Set MODER bit (01 - output)

// 00: GPIO_MODE_INPUT | 01: GPIO_MODE_OUTPUT_PP
// 10: GPIO_MODE_AF_PP | 11: GPIO_MODE_ANALOG

/* 3. OTYPER : configurer le type de sortie */
GPIOx->OTYPER &= ~(GPIO_OTYPER_OTy);
// 0 = Push-Pull (valeur par défaut), 1 = Open-Drain

/* 4. OSPEEDR : configurer le slew rate (optionnelle selon application) */
GPIOx->OSPEEDR |= GPIO_OSPEEDER_OSPEEDRy_1;   
// 00: Low speed | 01: Medium speed | 10: High speed | 11: Very high speed

/* 5. Configurer les résistances de Pull-Up / Pull-Down */
//A remplacer y = numero dans le port
GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDRy_Msk); // Clear bits
GPIOx->PUPDR |= GPIO_PUPDR_PUPDRy_1;      // Mode pull-up
// 00: GPIO_NOPULL | 01: GPIO_PULLUP
// 10: GPIO_PULLDOWN | 11: Reserved

/* Write to output */
//Write to a single pin (BSRR)
GPIOA->BSRR = (1 << 5);      // Set PA5 HIGH
GPIOA->BSRR = (1 << (5+16)); // Set PA5 LOW

//Write to a port (ODR)
GPIOA->ODR |= (1 << 5);  // Set PA5 HIGH
GPIOA->ODR &= ~(1 << 5); // Set PA5 LOW
```

### Mode output compare (PWM generation) with TIM2
```c++
/* Enable peripheral Clock for TIM2-CH2 (APB1 bus) et pin PA1 */
RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN ;
RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;

/* GPIO pin config . alternate function 1 to connect TIM2 - CH2 to PA1 */
GPIOA -> MODER &= ~GPIO_MODER_MODE1_Msk ;
GPIOA -> MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE1_Pos ;

GPIOA ->AFR [0] &= ~ GPIO_AFRL_AFSEL1 ;                    // Reset AF1 (pin Low in AFR[0])
GPIOA ->AFR [0] |= GPIO_AF1_TIM2 << GPIO_AFRL_AFSEL1_Pos ;

/* Clock definition - internal clock */
TIM2 -> SMCR &= ~TIM_SMCR_SMS ;


/* Counter Mode definition */
//— DIR : Direction (Up or down) ⇒ Counter CNT used as upcounter
//— CMS[1:0] : Edge or Center-aligned mode selection ⇒ Edge-aligned mode selection
//— ARPE : Auto-reload preload enable ⇒ TIM2_ARR register is not buffered
//— UDIS : Update disable ⇒ UEV events not disabled
//— OPM : One Pulse Mode ⇒ One Pulse Mode Disable.
//Pour notre cas, nous considérons une direction de comptage Up, le mode Edge-aligned, l’activation
//de l’Auto-reload et de l’Update et la désactivation du mode One Pulse
TIM2 ->CR1 &= ~( TIM_CR1_DIR // Up counting
                | TIM_CR1_CMS ) ; // Edge - aligned

/* Prescaler value definition */
// Kpsc = Tck_cnt (= 180 MHz) / Tck_psc (ce qu'on veut obtenir) 
TIM2 ->PSC = 180;

/* Auto - reload value definition */
//Ici on veut le counter increment par un chaque 1 KHz
//ARR = 1 MHz / 1 KHz
TIM2 ->ARR = 1000;

/* =========== OUTPUT COMPARE MODE =========== */
/* Output compare mode - channel configuration */
//CCMR1 manages channel 1 et 2
TIM2 -> CCMR1 &= ~ TIM_CCMR1_OC2M_Msk
                    | TIM_CCMR1_CC2S ; // TIM_CH2 configured as output
TIM2 -> CCMR1 |= TIM_OCMODE_PWM1 << TIM_CCMR1_OC2M_Pos // PWM Mode 1
                | TIM_CCMR1_OC2PE_Msk ; // Preload enable
                
/* Duty-cycle definition : Output compare value for channel 2 */
TIM2 -> CCR2 = 400;

// (OPTIONAL) If want to enable interruption upon front PWM
TIM2->SR &= ~(TIM_SR_UIF); 	//Enable update interrupt flags	

/* Enable output compare for channel 2 */
TIM2 -> CCER |= TIM_CCER_CC2E ;

/* Timer enable CEN - start to count */
TIM2 -> CR1 |= TIM_CR1_CEN ;
```

### Mode input capture
L’objectif de cette partie est de mesurer la période du signal y(t) présent sur la broche PA6 à l’aide
du Timer TIM3 en mode capture.

```c++
/* Enable peripheral Clock for TIM2-CH2 (APB1 bus) et pin PA1 */
RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN ;
RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;

/* GPIO pin config . alternate function 1 to connect TIM2 - CH2 to PA6 */
GPIOA -> MODER &= ~GPIO_MODER_MODE6_Msk ;
GPIOA -> MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE6_Pos ;

GPIOA ->AFR [0] &= ~ GPIO_AFRL_AFSEL6 ;                    // Reset AF1 (pin Low in AFR[0])
GPIOA ->AFR [0] |= GPIO_AF6_TIM2 << GPIO_AFRL_AFSEL6_Pos ;

/* Clock definition - internal clock */
TIM3 -> SMCR &= ~TIM_SMCR_SMS ;

/* Counter Mode definition */
TIM3 ->CR1 &= ~( TIM_CR1_DIR // Up counting
                | TIM_CR1_CMS ) ; // Edge - aligned
/* Prescaler value definition (pas definition pour ARR)*/
TIM3 ->PSC = 180;

/* Input capture mode - channel configuration */
TIM3 -> CCMR1 &= ~ TIM_CCMR1_CC1S ; // CC1 channel bit is cleared
TIM3 -> CCMR1 |= TIM_ICSELECTION_DIRECTTI ; // and IC1 mapped on TI1.
TIM3 -> CCMR1 &= ~TIM_CCMR1_IC1F ; // No filter
TIM3 -> CCMR1 &= ~ TIM_CCMR1_IC1PSC ; // No prescaler, capture each time an edge is detected

/* Input capture enable on channel 1 */
TIM3 -> CCER &= ~( TIM_CCER_CC1P | TIM_CCER_CC1NP ) ; // Rising edge detect
TIM3 -> CCER |= TIM_INPUTCHANNELPOLARITY_RISING << TIM_CCER_CC1P_Pos
                | TIM_CCER_CC1E ; // Capture Enable
                
/* Enable interrupt when capture occurs on channel 1 */
TIM3 -> DIER |= TIM_DIER_CC1IE ; // Enable interrupt when raising edge captured
TIM3 -> SR = ~TIM_SR_CC1IF ; // Clear the Capture event flag for channel 1

/* Enable interrupt in Interrupt controller */
NVIC_SetPriority(TIM3_IRQn ,3) ;
NVIC_ClearPendingIRQ(TIM3_IRQn) ;
NVIC_EnableIRQ(TIM3_IRQn) ;
```

### Interruption Handler (for input capture)
```c++
void TIM3_IRQHandler(void){
    if( (TIM3->SR & TIM_SR_CC1IF) != 0){
        do_sth();
        //Clear flags
        TIM3->SR &= ~TIM_SR_CC1IF;
						long t_captured = TIM3->CCR1;
    }
}
```
* To read captured value: `long t_captured = TIM3->CCR1;`

### Interruption Handler (for output compare)
```c++
void TIM2_IRQHandler(void) {
    //interruption type debordement
    //UIF - update event occurs flags, if = 1, an event is occured
    if (TIM2->SR & TIM_SR_UIF) {
        do_sth();
        TIM2->SR &= ~(TIM_SR_UIF);  // Clear pending flag
        // Handle your interrupt here
    }
}
```

### ADC Conversion
```c++
``` 


