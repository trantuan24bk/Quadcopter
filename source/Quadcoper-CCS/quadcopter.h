/*
 * quadcopter.h
 *
 *  Created on: Nov 26, 2014
 *      Author: BLACKHAT
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
#include "driverlib/rom.h"				// Use API from ROM
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"			// System tick timer
#include "driverlib/timer.h"
#include "driverlib/pwm.h"

#include "driverlib/uart.h"				// UART interface
#include "utils/uartstdio.h"



/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void init_Clock(void)
{
	// Set the clocking to run directly from the crystal.
	//ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	// f = 400MHZ(PLL) /2 / 4 = 50 Mhz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	// Set up and enable the SysTick timer.  It will be used as a reference
	// for delay loops in the interrupt handlers.  The SysTick timer period
	// will be set up for one second.
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet());
	ROM_SysTickEnable();
}


/*===============================================================================================
  Name         :
  Description  :  Configure the UART and its pins.  This must be called before UARTprintf().
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void init_UART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 9600, 16000000);
}

#endif /* QUADCOPTER_H_ */
