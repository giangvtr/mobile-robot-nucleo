# STM32 GPIO, Timer, and ADC Configuration Template

This template provides **generalized and corrected code snippets** for configuring STM32 GPIOs, Timers, and ADCs.  
**Variables:**  
- Replace `x` with the GPIO port letter (A, B, C, D, E, etc.).
- Replace `y` with the pin number (0-15).
- `n` with timer number (2, 3, ...)
- `z` with timer channel (1, 2, ...)

Each section includes **high-level explanations** and usage notes.

---

## 1. GPIO Modes

### 1.1. Analog Mode (ADC/DAC)
**Purpose:** Use pin as analog input/output (e.g., for ADC/DAC).

```c++
// 1. Enable GPIOx clock
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOxEN;

// 2. Set pin y to analog mode (MODER = 11)
GPIOx->MODER &= ~(GPIO_MODER_MODEy_Msk);
GPIOx->MODER |= (0x3U << GPIO_MODER_MODEy_Pos); // 11: GPIO_MODE_ANALOG, can replace 0x3U

```

### 1.2. Input Mode (Manual Read)
**Purpose:** Read digital input value (polling, not interrupt-driven).

```c++

/* 1. Enable x GPIO port, remplace x par A, B, C, D, E*/ (see fig.16/202 poly.1 pour savoir quel timer)
RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOxEN);

/* 2. Configure x.y to input mode */
GPIOx->MODER &= ~(GPIO_MODER_MODERy_Msk);    //Reset MODER bit (00 - input)

/* 3. Configurer les résistances de Pull-Up / Pull-Down */
GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDRy_Msk); 
GPIOx->PUPDR |= GPIO_PUPDR_PUPDRy_1;      // 01 = Pull-down

// 00: GPIO_NOPULL | 01: GPIO_PULLUP
// 10: GPIO_PULLDOWN | 11: Reserved

/* 4. Read data in pins (Don't put in initialization part */
uint16_t port_state = GPIOx->IDR;                   //Read 16 pins of GPIOA
uint8_t pin_state = (GPIOx->IDR >> y) & 0x01;       //Read only one pin Pxy 
```

### 1.3. Mode output push-pull
**Purpose:** This mode is a regular digital output that we can manually toggle them to generate output. (To drive a LED, relay, etc)

```c++
/* 1. Enable x GPIO port, remplace x par A, B, C, D, E*/
RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOxEN);

/* 2. Configure x.y to output mode */
GPIOx->MODER &= ~(GPIO_MODER_MODERy_Msk);    
GPIOx->MODER |= GPIO_MODER_MODERy_0;         //Set MODER bit (01 - output)

/* 3. OTYPER : configurer le type de sortie */
GPIOx->OTYPER &= ~(GPIO_OTYPER_OTy);
// 0 = Push-Pull (valeur par défaut), 1 = Open-Drain

/* 4. OSPEEDR : configurer le slew rate*/
GPIOx->OSPEEDR |= GPIO_OSPEEDER_OSPEEDRy_1;   
// 00: Low speed | 01: Medium speed | 10: High speed | 11: Very high speed

/* 5. Configurer les résistances de Pull-Up / Pull-Down */
GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDRy_Msk); 
GPIOx->PUPDR |= GPIO_PUPDR_PUPDRy_1;      // Mode pull-up
// 00: GPIO_NOPULL | 01: GPIO_PULLUP
// 10: GPIO_PULLDOWN | 11: Reserved

/* Write to output */
//Write to a single pin: bit set/reset 
GPIOx->BSRR = (1 << y);      // Set Pxy HIGH
GPIOx->BSRR = (1 << (y+16)); // Set Pxy LOW

//Write to a port (ODR) (so need to use |= to avoid overwrite other pins)
GPIOA->ODR |= (1 << 5);  // Set Pxy HIGH
GPIOA->ODR &= ~(1 << 5); // Set Pxy LOW
```

---

## 2. Timer Modes

### 2.1. Output Compare (PWM Generation)
**Purpose:** Generate PWM signals using a timer and corresponding output pin (see last page poly 1).
```c++
/* Enable peripheral Clock for TIM2-CH2 (APB1 bus) et pin PA1 */
RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN ;
RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;

/* GPIO pin config . alternate function 1 to connect TIM2 - CH2 to PA1 */
GPIOA -> MODER &= ~GPIO_MODER_MODE1_Msk ;
GPIOA -> MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE1_Pos ;

GPIOA -> OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk ;
GPIOx->OSPEEDR |= GPIO_OSPEEDER_OSPEEDRy_1;   
// 00: Low speed | 01: Medium speed | 10: High speed | 11: Very high speed

GPIOA ->AFR [0] &= ~ GPIO_AFRL_AFSEL1 ;                    // Reset AF1 (pin Low in AFR[0])
GPIOA ->AFR [0] |= GPIO_AF1_TIM2 << GPIO_AFRL_AFSEL1_Pos ;

/* Clock definition - internal clock */
TIM2 -> SMCR &= ~TIM_SMCR_SMS;

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

### 2.2. Mode input capture (Signal Measurement)
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


//========== Only do this when TIM3 call an interruption ===========
/* Enable interrupt when capture occurs on channel 1 */
TIM3 -> DIER |= TIM_DIER_CC1IE ; // Enable interrupt when raising edge captured
TIM3 -> SR = ~TIM_SR_CC1IF ; // Clear the Capture event flag for channel 1

/* Enable interrupt in Interrupt controller */
NVIC_SetPriority(TIM3_IRQn ,3) ;
NVIC_ClearPendingIRQ(TIM3_IRQn) ;
NVIC_EnableIRQ(TIM3_IRQn) ;
```

---

## 3. Interrupt Handlers

### 3.1. Timer Input Capture Interrupt
**Purpose**: Measure a time of an external event. For example, used in Ultrasound captor to measure distance.

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

### 3.2. Timer Output Compare (PWM) Interrupt
**Purpose:** Call an interruption upon clock overflow for example

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

### 3.3.a. Interrupt for ADC conversion (Single conversion)
**Purpose:** Pour convertir un echantillon unique. La fin de conversion est attendue par test du bit d'etat.
```c++
short int conversion_ADC_CH8(){
    short int val_ADC;
			
			//Lancement de la conversion
			ADC->CR2 |= ADC_CR2_SWSTART;
			
			//Attente fin conversion (ADC_SR_end of conversion)
			while (!(ADC1->SR & ADC_SR_EOC));
			
			//Lecture valeur
			val_ADC = ADC1->DR;
			
			return val_ADC;
}
```
### 3.3.b Single Conversion (Interrupt)
**Purpose:** : Faire une routine d'interruption (SW declenche) pour attendre la fin de la conversion et copier la valeur obtenue dans une variable globale `res_ADC`.

```c++
//We start the conversion manually 
void start_conversion_ADC_CH8(){
    //Lancement de la conversion
			ADC->CR2 |= ADC_CR2_SWSTART;
}

void ADC_IRQHandler(){
    if (ADC1->DR & ADC_SR_EOC){
        res_ADC = ADC1->DR;
						 ADC1->SR &= ~ADC_SR_EOC; //Remettre le flag a 0
			}
}			
```

### 3.3.c. HW (timer) declenche la conversion et attendre la fin de la conversion et copier la valeur obtenue dans une variable globale `res_ADC`.
**Purpose:** Une conversion est lancee par le Timer
* Delete void Start_ADC_CH8

```c++
void ADC_IRQHandler(){
    if (ADC1->DR & ADC_SR_EOC){
        res_ADC = ADC1->DR;
						 ADC1->SR &= ~ADC_SR_EOC; //Remettre le flag a 0
			}
}			
```

### 3.4. External interruption (EXTI)
**Purpose:** An external action will call an interuption (for example: a pushed button)

```c++
int main(void) {
	// Enable GPIO Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	// GPIO Mode: Input(00), Output (01), AF(10), Analog (11)
	GPIOA->MODER &= ~3U << 6; //Mode AF pour PA6
	
	// GPIO Push-Pull: No pull-up, pull-down (00), // Pull-up (01), Pull-down (10), Reserved (11)
	GPIOA->PUPDR &= ~3U << 6;
	GPIOA->PUPDR |= 2U << 6; // Pull down

	NVIC_EnableIRQ(EXTI3_IRQn); // Enable Interrupt

	// Connect External Line to the GPIO
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3; 
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA; 

	// Rising edge trigger selection
	// 0 = trigger disabled, 1 = trigger enabled
	EXTI->RTSR1 |= EXTI_RTSR1_RT3; 

	// Interrupt Mask Register
	// 0 = masked, 1 = not masked (enabled)
	EXTI->IMR1 |= EXTI_IMR1_IM3;

	while(1);	
}
```

* Example of an ISR
```c++
void EXTI3_IRQHandler(void) { 
	if ((EXTI->PR1 & EXTI_PR1_PIF3) != 0) {
		// Toggle LED...
		// Cleared flag by writing 1
		EXTI->PR1 |= EXTI_PR1_PIF3;
	}
}

---

## 4. ADC Configuration
**Purpose:** Init ADC1_IN8 (TD9, cas 1)

```c++
    /* Enable clock */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	/* Disable ADC */
	ADC1->CR2 &=~ADC_CR2_ADON;
	
	/* Commun à tous les ADC */
	/* Set prescaler (2,4,6 or 8) */
	ADC->CCR &= ~ADC_CCR_ADCPRE;
	ADC->CCR |= ADC_CLOCK_SYNC_PCLK_DIV4 << ADC_CCR_ADCPRE_Pos;	// prescaler = 4
	
	/* SMPR1 for channel 10 to 18, SMPR2 for 0 to 9 */
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP8;
	ADC1->SMPR2 |= ADC_SAMPLETIME_112CYCLES << ADC_SMPR2_SMP8;	// 112 cycles à 250kHz = 0.448ms de temps d'acquisition
	
	/* Main config for ADC */
	ADC1->CR1 &= ~ADC_CR1_SCAN;	// Sigle channel mode
	ADC1->CR2 &= ~ADC_CR2_CONT;	// Single conversion mode
	ADC1->CR1 &= ~ADC_CR1_DISCEN; // Discontinuous mode disabled
	ADC1->CR2 &= ~ADC_CR2_EXTEN; // Trigger detection disable
	
	ADC1->SQR1 &= ~ADC_SQR1_SQ1_L;	// 1 conversion, not several at once (for averaging purposes)
	
	/* SQR1 = channel 13 to 16 */
	/* SQR2 = channel 7 to 12 */
	/* SQR3 = channel 1 to 6 */
	ADC1->SQR3 &= ~ADC_SQR3_SQ1;
	ADC1->SQR3 |= 8 << ADC_SQR3_SQ1_Pos;	// Channel 8 for conversion 1	
	
	ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC
```
---
## 5. DAC Configuration

---

