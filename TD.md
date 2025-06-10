# TD micro-controler

## GPIO
---
## Function `read_length_for_mean`
```c++
#define MASK_INPUT 0x0000F000
/**
* \fn unsigned char read_length_for_mean(void)
* \brief Read of the switch Port C[12..15] defining the number of points
* for the mean
* + Read of PC[12..15]
* « Alignment on the LSB bits
*  \return val_input: value of the inputs PC[12..15] \in [@, 15]
*/
unsigned char read_length_for_mean(void) {
  unsigned char val_input;
  val_input = (~GPIOC->IDR & MASK_INPUT)>>12; // Read of Port C[12..15] and alignment on the LSB
  if(val_input == @)
    val_input = 1; // To have min 1 when all switch low
  return val_input;
}
```

---
## Fonction `vumeter_mngt()`
Le but de cette partie est de réaliser l’équivalent d’un vumétre avec les 8 LEDs associées aux 8 bits en sortie La
fonction de gestion du vumétre doit allumer les N premiéres LEDs associées aux sorties. L’entrée
de cette fonction est la variable mean_distance qui est comprise dans l’intervalle [0 ; 425] (dis-
tance en cm). Ensuite, il est nécessaire de calculer l’entier N, compris entre 0 et 8 indiquant le
nombre de LEDs a allumer. Deux exemples sont fournis a la Figure 4 pour N égal 4 3 et N égal a 6.

```c++
#define cst_1_50 5368709 // round(2*28/58) en Q-5.28
#define PATTERN_LED 0x000FF000 // pattern for 8 LED off - 8 LED ON
#define MASK_LED 0x00000FF0 // Mask for the LEDs

* \fn void vumeter_mngt(unsigned char distance)
*  \brief Vumeter management
* computation of n, the number of LED to be switch on
* « Set of PB[3+n .. 4], Reset of PB[11.. 4+n]»
* \param distance : target distance to be displayed on the vumeter
*  \return
*/

void vumeter_mngt(unsigned char distance){
  unsigned char n_led_on;
  unsigned char led_on_off;
  n_led_on = (unsigned char) ((distance * cst_1_5@) >> 28);
  led_on_off = (PATTERN_LED >> (16 — n_led_on)) & MASK_LED;
  GPIOx->BSRR = led_on_off | (~led_on_off & MASK_LED)<< 16;
}
```

---
## TIMER
Mode output compare
Développer le code d’initialisation du timer TIM2 pour générer le signal Trig

```c++
void init_TIM2_Counter_radar_US(void) {
  /* Timer TIM2 Clock Enable (APB1 bus) */
  RCC->APB1ENR |= ( 0x1U << RCC_APB1ENR_TIM2EN_Pos);
  
  /* Select the Counter Mode */
  
  TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Reset before to set
  TIM2->CR1 |= ( TIM_COUNTERMODE_UP << TIM_CR1_DIR_Pos) // Direction UP
  | TIM_CR1_ARPE // ARR update buffered
  
  | (TIM_CounterModeEdgeAligned << TIM_CR1_CMS_Pos) ; // Dir UP + Mode Edge Aligned Mode
  
  /* Set the Prescaler value */ // CK_CNT = 1Mhz
  TIM2->PSC = 179;
  
  /* Set the Auto-reload value */ // Period of 5@ms
  TIM2->ARR = 49999; //

  /* Output compare mode */
  TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk // Reset Mode 1
                | TIM_CCMR1_CC1S_Msk) ; // CC1 as output
  
  TIM2->CCMR1 |= = TIM_OCMODE_PWM1 // PWM Mode 1
                | TIM_CCMR1_OC1PE; // 1: Preload register on TIMx_CCR1 enabled.
  
  /* Capture/compare reaister 1 */
  
  TIM2->CCR1 = 19; // High-level during 20 us // Output Compare Register for channel 1

  /* Capture/Compare Enable Register */  
  TIM2->CCER &= ~TIM_CCER_CC1NP_Msk; // OC1 Active High
  TIM2->CCER |=  TIM_CCER_CC1E; // Enable output for CH1: OC1
  
  /* Capture/Compare Enable Register */
  TIM2->DIER &= ~(TIM_DIER_CC1DE_Msk // No DMA request on ch1
                | TIM_DIER_CC1IE_Msk ); // No interrupt on chi



//=============== MODE INPUT CAPTURE ==============
//Développer le code d’initialisation du timer TIM2 pour capturer le signal Echo.
  /* Input timer settings for TIMx */
  TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S /// Reset default CC2S
                  | TIM_CCMR1_IC2F // No filter
                  | TIM_CCMR1_IC2PSC);/ // No prescaler, capture each time an edge is detected
  
  // Channel : CAPTURE TI2
  TIM2->CCMR1 |= TIM_ICSELECTION_DIRECTTI << (TIM_CCMR1_CC2S_Pos - TIM_CCMR1_CC1S_Pos);
  
  /* Capture/Compare Enable Register */
  TIM2->CCER &= ~(TIM_CCER_CC2P // Reset default
            | TIM_CCER_CC2NP) ;
  // Rising edge detection
  TIM2->CCER |= TIM_INPUTCHANNELPOLARITY_RISING << (TIM_CCER_CC2P_Pos - TIM_CCER_CC1P_Pos)
              | TIM_CCER_CC2E; \ // Capture Enable
  
  /* Input timer settings for TIMx */ 
  TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S // Reset default
  
  | TIM_CCMR2_IC3F // No filter
  
  | TIM_CCMR2_IC3PSC); // No prescaler, capture each time an edge is detected
  
  // Channel : CAPTURE TI3
  TIM2->CCMR2 |= TIM_ICSELECTION_DIRECTTI << (TIM_CCMR2_CC3S_Pos - TIM_CCMR2_CC1S_Pos);
  
  /* Capture/Compare Enable Register */
  TIM2->CCER &= ~(TIM_CCER_CC3P
  | TIM_CCER_CC3NP) ;
  // Falling edge detection
  
  TIM2->CCER |= TIM_INPUTCHANNELPOLARITY_FALLING << (TIM_CCER_CC3P_Pos - TIM_CCER_CC1P_Pos)
  | TIM_CCER_CC3E; // Capture Enable
  
  TIM2->DIER |= TIM_DIER_CC3IE; // Enable interrupt when falling edge captured
  
  TIM2->SR &= ~TIM_SR_CC3IF; // Clear the Capture event flag for channel 3
  
  // Gestion de 1'interruption
  
  NVIC_SetPriority(TIM2_IRQn,3);
  NVIC_ClearPendingIRQ(TIM2_IRQn) ;
  NVIC_EnableIRQ(TIM2_IRQn) ;
  }
```

---
## Interruption sub-routine


```c++
/* Global variables */

volatile uint16_t delta_time;
volatile bool new_delta_time;

void routine_IT_TIM2() {
  delta_time = TIM2->CCR3 - TIM2->CCR2;  
  new_delta_time == TRUE;
}

void TIM2_IRQHandler( void) {
  if((TIM2->SR & TIM_SR_CC3IF) != 0) // Capture occurs Channel3 = Falling edge
  {
    routine_IT_TIM2();
    TIM2->SR &= ~TIM_SR_CC3IF; // reset by reading TIM2->CCRx register
  }
}
```

---
## Main function
```c++
int main (void) {
  uint16_t mean_distance;
  uint8_t lenght_for_mean;
  stm32_Init (); // STM32 setup
  init_GPIO_Radar_US();
  init_TIM2_radar_US();
  
  __enable_irq();

  while(1)
  {
    if(new_delta_time == TRUE)
    {
      new_delta_time = FALSE;
      mean_distance = compute_mean_distance(delta_time, lenght_for_mean) ;
      vumeter_mngt(mean_distance) ;
        lenght_for_mean = read_length_for_mean();
    }
  }
  
} // end main
```
