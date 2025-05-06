#include <stdio.h>    // For printf (icon manage run time/compiler/IO/Stdout/ITM)
#include <stdbool.h>
#include <stm32f4xx.h>
#include "SysClock.h"
#include "PinNames.h"
#include "stm32f4xx_gpio_AF.h"
#include "gpio.h"
#include "systick.h"
#include "stm32_periph.h"


void initGPIORadar();
void initTimerRadar();