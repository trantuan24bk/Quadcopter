/*
 * 
 *
 * 
 * 
 */

#ifndef QUADCOPTER_H_
#define QUADCOPTER_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"				// Added for timer use
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"				  // Use API from ROM
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"			// System tick timer
#include "driverlib/timer.h"


/*===============================================================================================
                    Global variables + Function prototype
================================================================================================*/

void setup();
void loop();


/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void init_Clock(void)
{
	uint32_t i;

	// Set the clocking to run directly from the crystal.
	//ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	// f = 400MHZ(PLL) /2 / 2.5 = 80 Mhz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	for(i = 0; i < 0xfffff; i++);

	// Set up and enable the SysTick timer.  It will be used as a reference
	// for delay loops in the interrupt handlers.  The SysTick timer period
	// will be set up for one second.
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet());
	ROM_SysTickEnable();

	for(i = 0; i < 0xfffff; i++);
}


#endif /* QUADCOPTER_H_ */
