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


	
int main(void){

	System_Clock_Init(); // Switch System Clock to maximum
  //TP2_US();
	
	//Configure PB8 comme output
	initGPIORadar();
	initTimerRadar();
	
	while(1) {
		//The interruption handler is called automatically since it is linked to the timer
		if (fin_reception == 1){
			fin_reception = 0;
			ComputeDistance();
		}		
	}
}


