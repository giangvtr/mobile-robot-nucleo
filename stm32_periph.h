/** @defgroup GPIO_pins_define GPIO pins define
  * @{
  */
#define GPIO_PIN_0                 ((uint16_t)0x0001U)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002U)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004U)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008U)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010U)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020U)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040U)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080U)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100U)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200U)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400U)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800U)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000U)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000U)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000U)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000U)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFFU)  /* All pins selected */

#define GPIO_PIN_MASK              ((uint32_t)0x0000FFFFU) /* PIN mask for assert test */
/**
  * @}
  */

#define  GPIO_MODE_INPUT                        ((uint32_t)0x00000000U)   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    ((uint32_t)0x00000001U)   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    ((uint32_t)0x00000011U)   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        ((uint32_t)0x00000002U)   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        ((uint32_t)0x00000012U)   /*!< Alternate Function Open Drain Mode    */

#define  GPIO_MODE_ANALOG                       ((uint32_t)0x00000003U)   /*!< Analog Mode  */
    
#define  GPIO_MODE_IT_RISING                    ((uint32_t)0x10110000U)   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   ((uint32_t)0x10210000U)   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            ((uint32_t)0x10310000U)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
 
#define  GPIO_MODE_EVT_RISING                   ((uint32_t)0x10120000U)   /*!< External Event Mode with Rising edge trigger detection               */
#define  GPIO_MODE_EVT_FALLING                  ((uint32_t)0x10220000U)   /*!< External Event Mode with Falling edge trigger detection              */
#define  GPIO_MODE_EVT_RISING_FALLING           ((uint32_t)0x10320000U)   /*!< External Event Mode with Rising/Falling edge trigger detection       */
/**
  * @}
  */

/** @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
#define  GPIO_SPEED_FREQ_LOW         ((uint32_t)0x00000000U)  /*!< IO works at 2 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_MEDIUM      ((uint32_t)0x00000001U)  /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_HIGH        ((uint32_t)0x00000002U)  /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
#define  GPIO_SPEED_FREQ_VERY_HIGH   ((uint32_t)0x00000003U)  /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */
/**
  * @}
  */

 /** @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @{
   */  
#define  GPIO_NOPULL        ((uint32_t)0x00000000U)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        ((uint32_t)0x00000001U)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      ((uint32_t)0x00000002U)   /*!< Pull-down activation                */



/** @defgroup TIM_Input_Channel_Polarity TIM Input Channel Polarity
  * @{
  */
#define  TIM_INPUTCHANNELPOLARITY_RISING      ((uint32_t)0x00000000U)            /*!< Polarity for TIx source */
#define  TIM_INPUTCHANNELPOLARITY_FALLING     (TIM_CCER_CC1P)                   /*!< Polarity for TIx source */
#define  TIM_INPUTCHANNELPOLARITY_BOTHEDGE    (TIM_CCER_CC1P | TIM_CCER_CC1NP)  /*!< Polarity for TIx source */
/**
  * @}
  */

/** @defgroup TIM_ETR_Polarity  TIM ETR Polarity
  * @{
  */
#define TIM_ETRPOLARITY_INVERTED              (TIM_SMCR_ETP)                    /*!< Polarity for ETR source */
#define TIM_ETRPOLARITY_NONINVERTED           ((uint32_t)0x00000000U)                /*!< Polarity for ETR source */
/**
  * @}
  */

/** @defgroup TIM_ETR_Prescaler  TIM ETR Prescaler
  * @{
  */
#define TIM_ETRPRESCALER_DIV1                 ((uint32_t)0x00000000U)                /*!< No prescaler is used */
#define TIM_ETRPRESCALER_DIV2                 (TIM_SMCR_ETPS_0)                 /*!< ETR input source is divided by 2 */
#define TIM_ETRPRESCALER_DIV4                 (TIM_SMCR_ETPS_1)                 /*!< ETR input source is divided by 4 */
#define TIM_ETRPRESCALER_DIV8                 (TIM_SMCR_ETPS)                   /*!< ETR input source is divided by 8 */
/**
  * @}
  */

/** @defgroup TIM_Counter_Mode  TIM Counter Mode
  * @{
  */
#define TIM_COUNTERMODE_UP                 ((uint32_t)0x00000000U)
#define TIM_COUNTERMODE_DOWN               TIM_CR1_DIR
#define TIM_COUNTERMODE_CENTERALIGNED1     TIM_CR1_CMS_0
#define TIM_COUNTERMODE_CENTERALIGNED2     TIM_CR1_CMS_1
#define TIM_COUNTERMODE_CENTERALIGNED3     TIM_CR1_CMS
/**
  * @}
  */

/** @defgroup TIM_ClockDivision TIM Clock Division
  * @{
  */
#define TIM_CLOCKDIVISION_DIV1                       ((uint32_t)0x00000000U)
#define TIM_CLOCKDIVISION_DIV2                       (TIM_CR1_CKD_0)
#define TIM_CLOCKDIVISION_DIV4                       (TIM_CR1_CKD_1)
/**
  * @}
  */

/** @defgroup TIM_Output_Compare_and_PWM_modes  TIM Output Compare and PWM modes
  * @{
  */
#define TIM_OCMODE_TIMING                   ((uint32_t)0x00000000U)
#define TIM_OCMODE_ACTIVE                   (TIM_CCMR1_OC1M_0)
#define TIM_OCMODE_INACTIVE                 (TIM_CCMR1_OC1M_1)
#define TIM_OCMODE_TOGGLE                   (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1)
#define TIM_OCMODE_PWM1                     (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
#define TIM_OCMODE_PWM2                     (TIM_CCMR1_OC1M)
#define TIM_OCMODE_FORCED_ACTIVE            (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)
#define TIM_OCMODE_FORCED_INACTIVE          (TIM_CCMR1_OC1M_2)

/**
  * @}
  */

/** @defgroup TIM_Output_Fast_State  TIM Output Fast State 
  * @{
  */
#define TIM_OCFAST_DISABLE                ((uint32_t)0x00000000U)
#define TIM_OCFAST_ENABLE                 (TIM_CCMR1_OC1FE)
/**
  * @}
  */

/** @defgroup TIM_Output_Compare_Polarity  TIM Output Compare Polarity
  * @{
  */
#define TIM_OCPOLARITY_HIGH                ((uint32_t)0x00000000U)
#define TIM_OCPOLARITY_LOW                 (TIM_CCER_CC1P)
/**
  * @}
  */

/** @defgroup TIM_Output_Compare_N_Polarity  TIM Output CompareN Polarity 
  * @{
  */
#define TIM_OCNPOLARITY_HIGH               ((uint32_t)0x00000000U)
#define TIM_OCNPOLARITY_LOW                (TIM_CCER_CC1NP)
/**
  * @}
  */

/** @defgroup TIM_Output_Compare_Idle_State  TIM Output Compare Idle State
  * @{
  */
#define TIM_OCIDLESTATE_SET                (TIM_CR2_OIS1)
#define TIM_OCIDLESTATE_RESET              ((uint32_t)0x00000000U)
/**
  * @}
  */ 

/** @defgroup TIM_Output_Compare_N_Idle_State  TIM Output Compare N Idle State
  * @{
  */
#define TIM_OCNIDLESTATE_SET               (TIM_CR2_OIS1N)
#define TIM_OCNIDLESTATE_RESET             ((uint32_t)0x00000000U)
/**
  * @}
  */ 

/** @defgroup TIM_Channel  TIM Channel
  * @{
  */
#define TIM_CHANNEL_1                      ((uint32_t)0x00000000U)
#define TIM_CHANNEL_2                      ((uint32_t)0x00000004U)
#define TIM_CHANNEL_3                      ((uint32_t)0x00000008U)
#define TIM_CHANNEL_4                      ((uint32_t)0x0000000CU)
#define TIM_CHANNEL_ALL                    ((uint32_t)0x00000018U)
                                 
/**
  * @}
  */

/** @defgroup TIM_Input_Capture_Polarity  TIM Input Capture Polarity 
  * @{
  */
#define  TIM_ICPOLARITY_RISING             TIM_INPUTCHANNELPOLARITY_RISING
#define  TIM_ICPOLARITY_FALLING            TIM_INPUTCHANNELPOLARITY_FALLING
#define  TIM_ICPOLARITY_BOTHEDGE           TIM_INPUTCHANNELPOLARITY_BOTHEDGE
/**
  * @}
  */

/** @defgroup TIM_Input_Capture_Selection  TIM Input Capture Selection
  * @{
  */
#define TIM_ICSELECTION_DIRECTTI           (TIM_CCMR1_CC1S_0)   /*!< TIM Input 1, 2, 3 or 4 is selected to be 
                                                                     connected to IC1, IC2, IC3 or IC4, respectively */
#define TIM_ICSELECTION_INDIRECTTI         (TIM_CCMR1_CC1S_1)   /*!< TIM Input 1, 2, 3 or 4 is selected to be
                                                                     connected to IC2, IC1, IC4 or IC3, respectively */
#define TIM_ICSELECTION_TRC                (TIM_CCMR1_CC1S)     /*!< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC */

/**
  * @}
  */

/** @defgroup TIM_Input_Capture_Prescaler  TIM Input Capture Prescaler
  * @{
  */
#define TIM_ICPSC_DIV1                     ((uint32_t)0x00000000U)       /*!< Capture performed each time an edge is detected on the capture input */
#define TIM_ICPSC_DIV2                     (TIM_CCMR1_IC1PSC_0)     /*!< Capture performed once every 2 events */
#define TIM_ICPSC_DIV4                     (TIM_CCMR1_IC1PSC_1)     /*!< Capture performed once every 4 events */
#define TIM_ICPSC_DIV8                     (TIM_CCMR1_IC1PSC)       /*!< Capture performed once every 8 events */
/**
  * @}
  */ 

/** @defgroup TIM_One_Pulse_Mode TIM One Pulse Mode
  * @{
  */
#define TIM_OPMODE_SINGLE                  (TIM_CR1_OPM)
#define TIM_OPMODE_REPETITIVE              ((uint32_t)0x00000000U)
/**
  * @}
  */

/** @defgroup TIM_Encoder_Mode TIM Encoder Mode
  * @{
  */
#define TIM_ENCODERMODE_TI1                (TIM_SMCR_SMS_0)
#define TIM_ENCODERMODE_TI2                (TIM_SMCR_SMS_1)
#define TIM_ENCODERMODE_TI12               (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0)
   
/**
  * @}
  */

/** @defgroup TIM_Interrupt_definition  TIM Interrupt definition
  * @{
  */ 
#define TIM_IT_UPDATE           (TIM_DIER_UIE)
#define TIM_IT_CC1              (TIM_DIER_CC1IE)
#define TIM_IT_CC2              (TIM_DIER_CC2IE)
#define TIM_IT_CC3              (TIM_DIER_CC3IE)
#define TIM_IT_CC4              (TIM_DIER_CC4IE)
#define TIM_IT_COM              (TIM_DIER_COMIE)
#define TIM_IT_TRIGGER          (TIM_DIER_TIE)
#define TIM_IT_BREAK            (TIM_DIER_BIE)
/**
  * @}
  */
  
/** @defgroup TIM_Commutation_Source  TIM Commutation Source 
  * @{
  */  
#define TIM_COMMUTATION_TRGI              (TIM_CR2_CCUS)
#define TIM_COMMUTATION_SOFTWARE          ((uint32_t)0x00000000U)
/**
  * @}
  */

/** @defgroup TIM_DMA_sources  TIM DMA sources
  * @{
  */
#define TIM_DMA_UPDATE                     (TIM_DIER_UDE)
#define TIM_DMA_CC1                        (TIM_DIER_CC1DE)
#define TIM_DMA_CC2                        (TIM_DIER_CC2DE)
#define TIM_DMA_CC3                        (TIM_DIER_CC3DE)
#define TIM_DMA_CC4                        (TIM_DIER_CC4DE)
#define TIM_DMA_COM                        (TIM_DIER_COMDE)
#define TIM_DMA_TRIGGER                    (TIM_DIER_TDE)
/**
  * @}
  */

/** @defgroup TIM_Event_Source  TIM Event Source 
  * @{
  */
#define TIM_EVENTSOURCE_UPDATE              TIM_EGR_UG  
#define TIM_EVENTSOURCE_CC1                 TIM_EGR_CC1G
#define TIM_EVENTSOURCE_CC2                 TIM_EGR_CC2G
#define TIM_EVENTSOURCE_CC3                 TIM_EGR_CC3G
#define TIM_EVENTSOURCE_CC4                 TIM_EGR_CC4G
#define TIM_EVENTSOURCE_COM                 TIM_EGR_COMG
#define TIM_EVENTSOURCE_TRIGGER             TIM_EGR_TG
#define TIM_EVENTSOURCE_BREAK               TIM_EGR_BG

/**
  * @}
  */

/** @defgroup TIM_Flag_definition  TIM Flag definition
  * @{
  */
#define TIM_FLAG_UPDATE                    (TIM_SR_UIF)
#define TIM_FLAG_CC1                       (TIM_SR_CC1IF)
#define TIM_FLAG_CC2                       (TIM_SR_CC2IF)
#define TIM_FLAG_CC3                       (TIM_SR_CC3IF)
#define TIM_FLAG_CC4                       (TIM_SR_CC4IF)
#define TIM_FLAG_COM                       (TIM_SR_COMIF)
#define TIM_FLAG_TRIGGER                   (TIM_SR_TIF)
#define TIM_FLAG_BREAK                     (TIM_SR_BIF)
#define TIM_FLAG_CC1OF                     (TIM_SR_CC1OF)
#define TIM_FLAG_CC2OF                     (TIM_SR_CC2OF)
#define TIM_FLAG_CC3OF                     (TIM_SR_CC3OF)
#define TIM_FLAG_CC4OF                     (TIM_SR_CC4OF)
/**
  * @}
  */

/** @defgroup TIM_Clock_Source  TIM Clock Source
  * @{
  */
#define	TIM_CLOCKSOURCE_ETRMODE2    (TIM_SMCR_ETPS_1) 
#define	TIM_CLOCKSOURCE_INTERNAL    (TIM_SMCR_ETPS_0) 
#define	TIM_CLOCKSOURCE_ITR0        ((uint32_t)0x00000000U)
#define	TIM_CLOCKSOURCE_ITR1        (TIM_SMCR_TS_0)
#define	TIM_CLOCKSOURCE_ITR2        (TIM_SMCR_TS_1)
#define	TIM_CLOCKSOURCE_ITR3        (TIM_SMCR_TS_0 | TIM_SMCR_TS_1)
#define	TIM_CLOCKSOURCE_TI1ED       (TIM_SMCR_TS_2)
#define	TIM_CLOCKSOURCE_TI1         (TIM_SMCR_TS_0 | TIM_SMCR_TS_2)
#define	TIM_CLOCKSOURCE_TI2         (TIM_SMCR_TS_1 | TIM_SMCR_TS_2)
#define	TIM_CLOCKSOURCE_ETRMODE1    (TIM_SMCR_TS)
/**
  * @}
  */

/** @defgroup TIM_Clock_Polarity  TIM Clock Polarity
  * @{
  */
#define TIM_CLOCKPOLARITY_INVERTED           TIM_ETRPOLARITY_INVERTED          /*!< Polarity for ETRx clock sources */ 
#define TIM_CLOCKPOLARITY_NONINVERTED        TIM_ETRPOLARITY_NONINVERTED       /*!< Polarity for ETRx clock sources */ 
#define TIM_CLOCKPOLARITY_RISING             TIM_INPUTCHANNELPOLARITY_RISING   /*!< Polarity for TIx clock sources */ 
#define TIM_CLOCKPOLARITY_FALLING            TIM_INPUTCHANNELPOLARITY_FALLING   /*!< Polarity for TIx clock sources */ 
#define TIM_CLOCKPOLARITY_BOTHEDGE           TIM_INPUTCHANNELPOLARITY_BOTHEDGE  /*!< Polarity for TIx clock sources */ 
/**
  * @}
  */

/** @defgroup TIM_Clock_Prescaler  TIM Clock Prescaler
  * @{
  */
#define TIM_CLOCKPRESCALER_DIV1              TIM_ETRPRESCALER_DIV1     /*!< No prescaler is used */
#define TIM_CLOCKPRESCALER_DIV2              TIM_ETRPRESCALER_DIV2     /*!< Prescaler for External ETR Clock: Capture performed once every 2 events. */
#define TIM_CLOCKPRESCALER_DIV4              TIM_ETRPRESCALER_DIV4     /*!< Prescaler for External ETR Clock: Capture performed once every 4 events. */
#define TIM_CLOCKPRESCALER_DIV8              TIM_ETRPRESCALER_DIV8     /*!< Prescaler for External ETR Clock: Capture performed once every 8 events. */
/**
  * @}
  */

/** @defgroup TIM_ClearInput_Source TIM Clear Input Source
  * @{
  */
#define TIM_CLEARINPUTSOURCE_ETR           ((uint32_t)0x00000001U) 
#define TIM_CLEARINPUTSOURCE_NONE          ((uint32_t)0x00000000U)
/**
  * @}
  */

/** @defgroup TIM_ClearInput_Polarity  TIM Clear Input Polarity
  * @{
  */
#define TIM_CLEARINPUTPOLARITY_INVERTED           TIM_ETRPOLARITY_INVERTED                    /*!< Polarity for ETRx pin */ 
#define TIM_CLEARINPUTPOLARITY_NONINVERTED        TIM_ETRPOLARITY_NONINVERTED                 /*!< Polarity for ETRx pin */ 
/**
  * @}
  */

/** @defgroup TIM_ClearInput_Prescaler TIM Clear Input Prescaler
  * @{
  */
#define TIM_CLEARINPUTPRESCALER_DIV1                    TIM_ETRPRESCALER_DIV1      /*!< No prescaler is used */
#define TIM_CLEARINPUTPRESCALER_DIV2                    TIM_ETRPRESCALER_DIV2      /*!< Prescaler for External ETR pin: Capture performed once every 2 events. */
#define TIM_CLEARINPUTPRESCALER_DIV4                    TIM_ETRPRESCALER_DIV4      /*!< Prescaler for External ETR pin: Capture performed once every 4 events. */
#define TIM_CLEARINPUTPRESCALER_DIV8                    TIM_ETRPRESCALER_DIV8        /*!< Prescaler for External ETR pin: Capture performed once every 8 events. */
/**
  * @}
  */

/** @defgroup TIM_OSSR_Off_State_Selection_for_Run_mode_state TIM OSSR OffState Selection for Run mode state
  * @{
  */  
#define TIM_OSSR_ENABLE         (TIM_BDTR_OSSR)
#define TIM_OSSR_DISABLE        ((uint32_t)0x00000000U)
/**
  * @}
  */
  
/** @defgroup TIM_OSSI_Off_State_Selection_for_Idle_mode_state TIM OSSI OffState Selection for Idle mode state
  * @{
  */
#define TIM_OSSI_ENABLE             (TIM_BDTR_OSSI)
#define TIM_OSSI_DISABLE            ((uint32_t)0x00000000U)
/**
  * @}
  */
  
/** @defgroup TIM_Lock_level  TIM Lock level
  * @{
  */
#define TIM_LOCKLEVEL_OFF          ((uint32_t)0x00000000U)
#define TIM_LOCKLEVEL_1            (TIM_BDTR_LOCK_0)
#define TIM_LOCKLEVEL_2            (TIM_BDTR_LOCK_1)
#define TIM_LOCKLEVEL_3            (TIM_BDTR_LOCK)
/**
  * @}
  */  
/** @defgroup TIM_Break_Input_enable_disable  TIM Break Input State
  * @{
  */
#define TIM_BREAK_ENABLE          (TIM_BDTR_BKE)
#define TIM_BREAK_DISABLE         ((uint32_t)0x00000000U)
/**
  * @}
  */
  
/** @defgroup TIM_Break_Polarity  TIM Break Polarity 
  * @{
  */
#define TIM_BREAKPOLARITY_LOW        ((uint32_t)0x00000000U)
#define TIM_BREAKPOLARITY_HIGH       (TIM_BDTR_BKP)
/**
  * @}
  */
  
/** @defgroup TIM_AOE_Bit_Set_Reset  TIM AOE Bit State
  * @{
  */
#define TIM_AUTOMATICOUTPUT_ENABLE           (TIM_BDTR_AOE)
#define	TIM_AUTOMATICOUTPUT_DISABLE          ((uint32_t)0x00000000U)
/**
  * @}
  */  
  
/** @defgroup TIM_Master_Mode_Selection TIM Master Mode Selection
  * @{
  */  
#define	TIM_TRGO_RESET            ((uint32_t)0x00000000U)
#define	TIM_TRGO_ENABLE           (TIM_CR2_MMS_0)
#define	TIM_TRGO_UPDATE           (TIM_CR2_MMS_1)
#define	TIM_TRGO_OC1              ((TIM_CR2_MMS_1 | TIM_CR2_MMS_0))
#define	TIM_TRGO_OC1REF           (TIM_CR2_MMS_2)
#define	TIM_TRGO_OC2REF           ((TIM_CR2_MMS_2 | TIM_CR2_MMS_0))
#define	TIM_TRGO_OC3REF           ((TIM_CR2_MMS_2 | TIM_CR2_MMS_1))
#define	TIM_TRGO_OC4REF           ((TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0))
/**
  * @}
  */ 
  
/** @defgroup TIM_Slave_Mode TIM Slave Mode
  * @{
  */
#define TIM_SLAVEMODE_DISABLE              ((uint32_t)0x00000000U)
#define TIM_SLAVEMODE_RESET                ((uint32_t)0x00000004U)
#define TIM_SLAVEMODE_GATED                ((uint32_t)0x00000005U)
#define TIM_SLAVEMODE_TRIGGER              ((uint32_t)0x00000006U)
#define TIM_SLAVEMODE_EXTERNAL1            ((uint32_t)0x00000007U)
/**
  * @}
  */

/** @defgroup TIM_Master_Slave_Mode  TIM Master Slave Mode
  * @{
  */
#define TIM_MASTERSLAVEMODE_ENABLE          ((uint32_t)0x00000080U)
#define TIM_MASTERSLAVEMODE_DISABLE         ((uint32_t)0x00000000U)
/**
  * @}
  */ 
  
/** @defgroup TIM_Trigger_Selection  TIM Trigger Selection
  * @{
  */
#define TIM_TS_ITR0                        ((uint32_t)0x00000000U)
#define TIM_TS_ITR1                        ((uint32_t)0x00000010U)
#define TIM_TS_ITR2                        ((uint32_t)0x00000020U)
#define TIM_TS_ITR3                        ((uint32_t)0x00000030U)
#define TIM_TS_TI1F_ED                     ((uint32_t)0x00000040U)
#define TIM_TS_TI1FP1                      ((uint32_t)0x00000050U)
#define TIM_TS_TI2FP2                      ((uint32_t)0x00000060U)
#define TIM_TS_ETRF                        ((uint32_t)0x00000070U)
#define TIM_TS_NONE                        ((uint32_t)0x0000FFFFU)
/**
  * @}
  */  

/** @defgroup TIM_Trigger_Polarity TIM Trigger Polarity
  * @{
  */
#define TIM_TRIGGERPOLARITY_INVERTED           TIM_ETRPOLARITY_INVERTED      /*!< Polarity for ETRx trigger sources */ 
#define TIM_TRIGGERPOLARITY_NONINVERTED        TIM_ETRPOLARITY_NONINVERTED   /*!< Polarity for ETRx trigger sources */ 
#define TIM_TRIGGERPOLARITY_RISING             TIM_INPUTCHANNELPOLARITY_RISING        /*!< Polarity for TIxFPx or TI1_ED trigger sources */ 
#define TIM_TRIGGERPOLARITY_FALLING            TIM_INPUTCHANNELPOLARITY_FALLING       /*!< Polarity for TIxFPx or TI1_ED trigger sources */ 
#define TIM_TRIGGERPOLARITY_BOTHEDGE           TIM_INPUTCHANNELPOLARITY_BOTHEDGE      /*!< Polarity for TIxFPx or TI1_ED trigger sources */ 
/**
  * @}
  */

/** @defgroup TIM_Trigger_Prescaler TIM Trigger Prescaler
  * @{
  */
#define TIM_TRIGGERPRESCALER_DIV1             TIM_ETRPRESCALER_DIV1     /*!< No prescaler is used */
#define TIM_TRIGGERPRESCALER_DIV2             TIM_ETRPRESCALER_DIV2     /*!< Prescaler for External ETR Trigger: Capture performed once every 2 events. */
#define TIM_TRIGGERPRESCALER_DIV4             TIM_ETRPRESCALER_DIV4     /*!< Prescaler for External ETR Trigger: Capture performed once every 4 events. */
#define TIM_TRIGGERPRESCALER_DIV8             TIM_ETRPRESCALER_DIV8     /*!< Prescaler for External ETR Trigger: Capture performed once every 8 events. */
/**
  * @}
  */


/** @defgroup TIM_TI1_Selection TIM TI1 Selection
  * @{
  */
#define TIM_TI1SELECTION_CH1                ((uint32_t)0x00000000U)
#define TIM_TI1SELECTION_XORCOMBINATION     (TIM_CR2_TI1S)
/**
  * @}
  */ 

/** @defgroup TIM_DMA_Base_address  TIM DMA Base address
  * @{
  */
#define TIM_DMABASE_CR1                    (0x00000000U)
#define TIM_DMABASE_CR2                    (0x00000001U)
#define TIM_DMABASE_SMCR                   (0x00000002U)
#define TIM_DMABASE_DIER                   (0x00000003U)
#define TIM_DMABASE_SR                     (0x00000004U)
#define TIM_DMABASE_EGR                    (0x00000005U)
#define TIM_DMABASE_CCMR1                  (0x00000006U)
#define TIM_DMABASE_CCMR2                  (0x00000007U)
#define TIM_DMABASE_CCER                   (0x00000008U)
#define TIM_DMABASE_CNT                    (0x00000009U)
#define TIM_DMABASE_PSC                    (0x0000000AU)
#define TIM_DMABASE_ARR                    (0x0000000BU)
#define TIM_DMABASE_RCR                    (0x0000000CU)
#define TIM_DMABASE_CCR1                   (0x0000000DU)
#define TIM_DMABASE_CCR2                   (0x0000000EU)
#define TIM_DMABASE_CCR3                   (0x0000000FU)
#define TIM_DMABASE_CCR4                   (0x00000010U)
#define TIM_DMABASE_BDTR                   (0x00000011U)
#define TIM_DMABASE_DCR                    (0x00000012U)
#define TIM_DMABASE_OR                     (0x00000013U)
/**
  * @}
  */ 

/** @defgroup TIM_DMA_Burst_Length  TIM DMA Burst Length 
  * @{
  */
#define TIM_DMABURSTLENGTH_1TRANSFER           (0x00000000U)
#define TIM_DMABURSTLENGTH_2TRANSFERS          (0x00000100U)
#define TIM_DMABURSTLENGTH_3TRANSFERS          (0x00000200U)
#define TIM_DMABURSTLENGTH_4TRANSFERS          (0x00000300U)
#define TIM_DMABURSTLENGTH_5TRANSFERS          (0x00000400U)
#define TIM_DMABURSTLENGTH_6TRANSFERS          (0x00000500U)
#define TIM_DMABURSTLENGTH_7TRANSFERS          (0x00000600U)
#define TIM_DMABURSTLENGTH_8TRANSFERS          (0x00000700U)
#define TIM_DMABURSTLENGTH_9TRANSFERS          (0x00000800U)
#define TIM_DMABURSTLENGTH_10TRANSFERS         (0x00000900U)
#define TIM_DMABURSTLENGTH_11TRANSFERS         (0x00000A00U)
#define TIM_DMABURSTLENGTH_12TRANSFERS         (0x00000B00U)
#define TIM_DMABURSTLENGTH_13TRANSFERS         (0x00000C00U)
#define TIM_DMABURSTLENGTH_14TRANSFERS         (0x00000D00U)
#define TIM_DMABURSTLENGTH_15TRANSFERS         (0x00000E00U)
#define TIM_DMABURSTLENGTH_16TRANSFERS         (0x00000F00U)
#define TIM_DMABURSTLENGTH_17TRANSFERS         (0x00001000U)
#define TIM_DMABURSTLENGTH_18TRANSFERS         (0x00001100U)
/**
  * @}
  */

/** @defgroup DMA_Handle_index  DMA Handle index
  * @{
  */
#define TIM_DMA_ID_UPDATE                ((uint16_t) 0x0000U)       /*!< Index of the DMA handle used for Update DMA requests */
#define TIM_DMA_ID_CC1                   ((uint16_t) 0x0001U)       /*!< Index of the DMA handle used for Capture/Compare 1 DMA requests */
#define TIM_DMA_ID_CC2                   ((uint16_t) 0x0002U)       /*!< Index of the DMA handle used for Capture/Compare 2 DMA requests */
#define TIM_DMA_ID_CC3                   ((uint16_t) 0x0003U)       /*!< Index of the DMA handle used for Capture/Compare 3 DMA requests */
#define TIM_DMA_ID_CC4                   ((uint16_t) 0x0004U)       /*!< Index of the DMA handle used for Capture/Compare 4 DMA requests */
#define TIM_DMA_ID_COMMUTATION           ((uint16_t) 0x0005U)       /*!< Index of the DMA handle used for Commutation DMA requests */
#define TIM_DMA_ID_TRIGGER               ((uint16_t) 0x0006U)       /*!< Index of the DMA handle used for Trigger DMA requests */
/**
  * @}
  */ 

/** @defgroup Channel_CC_State  Channel CC State
  * @{
  */
#define TIM_CCx_ENABLE                   ((uint32_t)0x00000001U)
#define TIM_CCx_DISABLE                  ((uint32_t)0x00000000U)
#define TIM_CCxN_ENABLE                  ((uint32_t)0x00000004U)
#define TIM_CCxN_DISABLE                 ((uint32_t)0x00000000U)
/**
  * @}
  */ 
	
	
	

/** @defgroup ADC_ClockPrescaler  ADC Clock Prescaler
  * @{
  */ 
#define ADC_CLOCK_SYNC_PCLK_DIV2    ((uint32_t)0x00000000U)
#define ADC_CLOCK_SYNC_PCLK_DIV4    ((uint32_t)ADC_CCR_ADCPRE_0)
#define ADC_CLOCK_SYNC_PCLK_DIV6    ((uint32_t)ADC_CCR_ADCPRE_1)
#define ADC_CLOCK_SYNC_PCLK_DIV8    ((uint32_t)ADC_CCR_ADCPRE)
/**
  * @}
  */ 

/** @defgroup ADC_delay_between_2_sampling_phases ADC Delay Between 2 Sampling Phases
  * @{
  */ 
#define ADC_TWOSAMPLINGDELAY_5CYCLES    ((uint32_t)0x00000000U)
#define ADC_TWOSAMPLINGDELAY_6CYCLES    ((uint32_t)ADC_CCR_DELAY_0)
#define ADC_TWOSAMPLINGDELAY_7CYCLES    ((uint32_t)ADC_CCR_DELAY_1)
#define ADC_TWOSAMPLINGDELAY_8CYCLES    ((uint32_t)(ADC_CCR_DELAY_1 | ADC_CCR_DELAY_0))
#define ADC_TWOSAMPLINGDELAY_9CYCLES    ((uint32_t)ADC_CCR_DELAY_2)
#define ADC_TWOSAMPLINGDELAY_10CYCLES   ((uint32_t)(ADC_CCR_DELAY_2 | ADC_CCR_DELAY_0))
#define ADC_TWOSAMPLINGDELAY_11CYCLES   ((uint32_t)(ADC_CCR_DELAY_2 | ADC_CCR_DELAY_1))
#define ADC_TWOSAMPLINGDELAY_12CYCLES   ((uint32_t)(ADC_CCR_DELAY_2 | ADC_CCR_DELAY_1 | ADC_CCR_DELAY_0))
#define ADC_TWOSAMPLINGDELAY_13CYCLES   ((uint32_t)ADC_CCR_DELAY_3)
#define ADC_TWOSAMPLINGDELAY_14CYCLES   ((uint32_t)(ADC_CCR_DELAY_3 | ADC_CCR_DELAY_0))
#define ADC_TWOSAMPLINGDELAY_15CYCLES   ((uint32_t)(ADC_CCR_DELAY_3 | ADC_CCR_DELAY_1))
#define ADC_TWOSAMPLINGDELAY_16CYCLES   ((uint32_t)(ADC_CCR_DELAY_3 | ADC_CCR_DELAY_1 | ADC_CCR_DELAY_0))
#define ADC_TWOSAMPLINGDELAY_17CYCLES   ((uint32_t)(ADC_CCR_DELAY_3 | ADC_CCR_DELAY_2))
#define ADC_TWOSAMPLINGDELAY_18CYCLES   ((uint32_t)(ADC_CCR_DELAY_3 | ADC_CCR_DELAY_2 | ADC_CCR_DELAY_0))
#define ADC_TWOSAMPLINGDELAY_19CYCLES   ((uint32_t)(ADC_CCR_DELAY_3 | ADC_CCR_DELAY_2 | ADC_CCR_DELAY_1))
#define ADC_TWOSAMPLINGDELAY_20CYCLES   ((uint32_t)ADC_CCR_DELAY)
/**
  * @}
  */ 

/** @defgroup ADC_Resolution ADC Resolution
  * @{
  */ 
#define ADC_RESOLUTION_12B  ((uint32_t)0x00000000U)
#define ADC_RESOLUTION_10B  ((uint32_t)ADC_CR1_RES_0)
#define ADC_RESOLUTION_8B   ((uint32_t)ADC_CR1_RES_1)
#define ADC_RESOLUTION_6B   ((uint32_t)ADC_CR1_RES)
/**
  * @}
  */ 

/** @defgroup ADC_External_trigger_edge_Regular ADC External Trigger Edge Regular
  * @{
  */ 
#define ADC_EXTERNALTRIGCONVEDGE_NONE           ((uint32_t)0x00000000U)
#define ADC_EXTERNALTRIGCONVEDGE_RISING         ((uint32_t)ADC_CR2_EXTEN_0)
#define ADC_EXTERNALTRIGCONVEDGE_FALLING        ((uint32_t)ADC_CR2_EXTEN_1)
#define ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING  ((uint32_t)ADC_CR2_EXTEN)
/**
  * @}
  */ 

/** @defgroup ADC_External_trigger_Source_Regular ADC External Trigger Source Regular
  * @{
  */
/* Note: Parameter ADC_SOFTWARE_START is a software parameter used for        */
/*       compatibility with other STM32 devices.                              */
#define ADC_EXTERNALTRIGCONV_T1_CC1    ((uint32_t)0x00000000U)
#define ADC_EXTERNALTRIGCONV_T1_CC2    ((uint32_t)ADC_CR2_EXTSEL_0)
#define ADC_EXTERNALTRIGCONV_T1_CC3    ((uint32_t)ADC_CR2_EXTSEL_1)
#define ADC_EXTERNALTRIGCONV_T2_CC2    ((uint32_t)(ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0))
#define ADC_EXTERNALTRIGCONV_T2_CC3    ((uint32_t)ADC_CR2_EXTSEL_2)
#define ADC_EXTERNALTRIGCONV_T2_CC4    ((uint32_t)(ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_0))
#define ADC_EXTERNALTRIGCONV_T2_TRGO   ((uint32_t)(ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1))
#define ADC_EXTERNALTRIGCONV_T3_CC1    ((uint32_t)(ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0))
#define ADC_EXTERNALTRIGCONV_T3_TRGO   ((uint32_t)ADC_CR2_EXTSEL_3)
#define ADC_EXTERNALTRIGCONV_T4_CC4    ((uint32_t)(ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_0))
#define ADC_EXTERNALTRIGCONV_T5_CC1    ((uint32_t)(ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_1))
#define ADC_EXTERNALTRIGCONV_T5_CC2    ((uint32_t)(ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0))
#define ADC_EXTERNALTRIGCONV_T5_CC3    ((uint32_t)(ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_2))
#define ADC_EXTERNALTRIGCONV_T8_CC1    ((uint32_t)(ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_0))
#define ADC_EXTERNALTRIGCONV_T8_TRGO   ((uint32_t)(ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1))
#define ADC_EXTERNALTRIGCONV_Ext_IT11  ((uint32_t)ADC_CR2_EXTSEL)
#define ADC_SOFTWARE_START             ((uint32_t)ADC_CR2_EXTSEL + 1U)
/**
  * @}
  */ 

/** @defgroup ADC_Data_align ADC Data Align
  * @{
  */ 
#define ADC_DATAALIGN_RIGHT      ((uint32_t)0x00000000U)
#define ADC_DATAALIGN_LEFT       ((uint32_t)ADC_CR2_ALIGN)
/**
  * @}
  */ 

/** @defgroup ADC_channels  ADC Common Channels
  * @{
  */ 
#define ADC_CHANNEL_0           ((uint32_t)0x00000000U)
#define ADC_CHANNEL_1           ((uint32_t)ADC_CR1_AWDCH_0)
#define ADC_CHANNEL_2           ((uint32_t)ADC_CR1_AWDCH_1)
#define ADC_CHANNEL_3           ((uint32_t)(ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_4           ((uint32_t)ADC_CR1_AWDCH_2)
#define ADC_CHANNEL_5           ((uint32_t)(ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_6           ((uint32_t)(ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1))
#define ADC_CHANNEL_7           ((uint32_t)(ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_8           ((uint32_t)ADC_CR1_AWDCH_3)
#define ADC_CHANNEL_9           ((uint32_t)(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_10          ((uint32_t)(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_1))
#define ADC_CHANNEL_11          ((uint32_t)(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_12          ((uint32_t)(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2))
#define ADC_CHANNEL_13          ((uint32_t)(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_14          ((uint32_t)(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1))
#define ADC_CHANNEL_15          ((uint32_t)(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_16          ((uint32_t)ADC_CR1_AWDCH_4)
#define ADC_CHANNEL_17          ((uint32_t)(ADC_CR1_AWDCH_4 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_18          ((uint32_t)(ADC_CR1_AWDCH_4 | ADC_CR1_AWDCH_1))

#define ADC_CHANNEL_VREFINT     ((uint32_t)ADC_CHANNEL_17)
#define ADC_CHANNEL_VBAT        ((uint32_t)ADC_CHANNEL_18)
/**
  * @}
  */ 

/** @defgroup ADC_sampling_times  ADC Sampling Times
  * @{
  */ 
#define ADC_SAMPLETIME_3CYCLES    ((uint32_t)0x00000000U)
#define ADC_SAMPLETIME_15CYCLES   ((uint32_t)ADC_SMPR1_SMP10_0)
#define ADC_SAMPLETIME_28CYCLES   ((uint32_t)ADC_SMPR1_SMP10_1)
#define ADC_SAMPLETIME_56CYCLES   ((uint32_t)(ADC_SMPR1_SMP10_1 | ADC_SMPR1_SMP10_0))
#define ADC_SAMPLETIME_84CYCLES   ((uint32_t)ADC_SMPR1_SMP10_2)
#define ADC_SAMPLETIME_112CYCLES  ((uint32_t)(ADC_SMPR1_SMP10_2 | ADC_SMPR1_SMP10_0))
#define ADC_SAMPLETIME_144CYCLES  ((uint32_t)(ADC_SMPR1_SMP10_2 | ADC_SMPR1_SMP10_1))
#define ADC_SAMPLETIME_480CYCLES  ((uint32_t)ADC_SMPR1_SMP10)
/**
  * @}
  */ 

  /** @defgroup ADC_EOCSelection ADC EOC Selection
  * @{
  */ 
#define ADC_EOC_SEQ_CONV              ((uint32_t)0x00000000U)
#define ADC_EOC_SINGLE_CONV           ((uint32_t)0x00000001U)
#define ADC_EOC_SINGLE_SEQ_CONV       ((uint32_t)0x00000002U)  /*!< reserved for future use */
/**
  * @}
  */ 

/** @defgroup ADC_Event_type ADC Event Type
  * @{
  */ 
#define ADC_AWD_EVENT             ((uint32_t)ADC_FLAG_AWD)
#define ADC_OVR_EVENT             ((uint32_t)ADC_FLAG_OVR)
/**
  * @}
  */

/** @defgroup ADC_analog_watchdog_selection ADC Analog Watchdog Selection
  * @{
  */ 
#define ADC_ANALOGWATCHDOG_SINGLE_REG         ((uint32_t)(ADC_CR1_AWDSGL | ADC_CR1_AWDEN))
#define ADC_ANALOGWATCHDOG_SINGLE_INJEC       ((uint32_t)(ADC_CR1_AWDSGL | ADC_CR1_JAWDEN))
#define ADC_ANALOGWATCHDOG_SINGLE_REGINJEC    ((uint32_t)(ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_JAWDEN))
#define ADC_ANALOGWATCHDOG_ALL_REG            ((uint32_t)ADC_CR1_AWDEN)
#define ADC_ANALOGWATCHDOG_ALL_INJEC          ((uint32_t)ADC_CR1_JAWDEN)
#define ADC_ANALOGWATCHDOG_ALL_REGINJEC       ((uint32_t)(ADC_CR1_AWDEN | ADC_CR1_JAWDEN))
#define ADC_ANALOGWATCHDOG_NONE               ((uint32_t)0x00000000U)
/**
  * @}
  */ 
    
/** @defgroup ADC_interrupts_definition ADC Interrupts Definition
  * @{
  */ 
#define ADC_IT_EOC      ((uint32_t)ADC_CR1_EOCIE)
#define ADC_IT_AWD      ((uint32_t)ADC_CR1_AWDIE)
#define ADC_IT_JEOC     ((uint32_t)ADC_CR1_JEOCIE)
#define ADC_IT_OVR      ((uint32_t)ADC_CR1_OVRIE)
/**
  * @}
  */ 
    
/** @defgroup ADC_flags_definition ADC Flags Definition
  * @{
  */ 
#define ADC_FLAG_AWD    ((uint32_t)ADC_SR_AWD)
#define ADC_FLAG_EOC    ((uint32_t)ADC_SR_EOC)
#define ADC_FLAG_JEOC   ((uint32_t)ADC_SR_JEOC)
#define ADC_FLAG_JSTRT  ((uint32_t)ADC_SR_JSTRT)
#define ADC_FLAG_STRT   ((uint32_t)ADC_SR_STRT)
#define ADC_FLAG_OVR    ((uint32_t)ADC_SR_OVR)
/**
  * @}
  */ 

/** @defgroup ADC_channels_type ADC Channels Type
  * @{
  */ 
#define ADC_ALL_CHANNELS      ((uint32_t)0x00000001U)
#define ADC_REGULAR_CHANNELS  ((uint32_t)0x00000002U) /*!< reserved for future use */
#define ADC_INJECTED_CHANNELS ((uint32_t)0x00000003U) /*!< reserved for future use */
/**
  * @}
  */
