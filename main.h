/**
 * \file      main.h
 * \brief     Interfaces for main program
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>    // For printf (icon manage run time/compiler/IO/Stdout/ITM)
#include <stdbool.h>
#include <stm32f4xx.h>
#include "SysClock.h"
#include "PinNames.h"
#include "stm32f4xx_gpio_AF.h"
#include "gpio.h"
#include "systick.h"
#include "stm32_periph.h"
#include "moteur.h"

#define INDEX_50ms 500

volatile char fin_reception = 0;
volatile unsigned short index_time=0;
volatile unsigned short nbr_un = 0;
volatile unsigned short distance = 0;
volatile unsigned short nbrUnFinal = 0;

#endif /* MAIN_H */

