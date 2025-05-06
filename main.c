/**
 * \file      main.c
 * \brief     Main function
 *
 */
 
#include "main.h"
#include "ultrason.h"

unsigned char robot_direction = 0;
unsigned char robot_rightspeed = 0;
unsigned char robot_leftspeed = 0;

/*----------------------------------------------------------------------------
 MAIN function
 *----------------------------------------------------------------------------*/
	
/**
 * \brief     Main function entry
 * \details   
 * \return    \e none.
 */

	
int main(void){

	System_Clock_Init(); // Switch System Clock to maximum
  //TP2_US();
	
	//Configure PB8 comme output
	initGPIORadar();
	initTimerRadar();
	
	while(1) {
		//Set PB8 to HIGH to open LED
		}		
}


