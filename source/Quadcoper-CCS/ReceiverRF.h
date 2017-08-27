/*
 * ReceiverRF.h
 *
 *  Created on: Dec 5, 2014
 *      Author: BLACKHAT
 */

#ifndef RECEIVERRF_H_
#define RECEIVERRF_H_

#include "quadcopter.h"

#define QUAD_PERIPH_PORT	SYSCTL_PERIPH_GPIOE
#define QUAD_GPIO_RF_BASE	GPIO_PORTE_BASE

#define QUAD_GPIO_RF_CH1	GPIO_PIN_0
#define QUAD_GPIO_RF_CH2	GPIO_PIN_1
#define QUAD_GPIO_RF_CH3	GPIO_PIN_2
#define QUAD_GPIO_RF_CH4	GPIO_PIN_3
#define QUAD_GPIO_RF_CH5	GPIO_PIN_4
#define QUAD_GPIO_RF_CH6	GPIO_PIN_5

/* Global variables */
volatile uint32_t rf_channel[6] = {0,0,0,0,0,0};
volatile uint32_t rf_channelMin[6] = {0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF};
volatile uint32_t rf_channelMax[6] = {0,0,0,0,0,0};
volatile uint32_t rf_PinEdge;
volatile uint32_t rf_CurrentChannel = QUAD_GPIO_RF_CH1;


/*===============================================================================================
  	  	  	  	  	  	  Funtion prototypes
================================================================================================*/

void init_ReceiverRF(void);
uint32_t ReceiverRF_Read(uint8_t channel);

void PortRF_ISR(void);
void TimerRF_ISR(void);

/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void init_ReceiverRF(void)
{
	/*1. Setup measure timer */

	// Enable timer peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// Configure the 32-bit periodic/up timers.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
	TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFFFFFE);

	// Setup the interrupts for the timer timeout.
	// Timer is running but not generate interrupt yet!
	TimerIntRegister(TIMER0_BASE, TIMER_A, TimerRF_ISR);
	TimerEnable(TIMER0_BASE, TIMER_A);
	IntEnable(INT_TIMER0A);

	/*2. Setup RF Pins */
	SysCtlPeripheralEnable(QUAD_PERIPH_PORT);

	// Disable all RF pins before configuration
	GPIOIntDisable(QUAD_GPIO_RF_BASE, QUAD_GPIO_RF_CH1 | QUAD_GPIO_RF_CH2 | QUAD_GPIO_RF_CH3 |
			 	   QUAD_GPIO_RF_CH4 | QUAD_GPIO_RF_CH5 | QUAD_GPIO_RF_CH6);

	// Config RF pins
	GPIOPinTypeGPIOInput(QUAD_GPIO_RF_BASE, QUAD_GPIO_RF_CH1 | QUAD_GPIO_RF_CH2 | QUAD_GPIO_RF_CH3 |
						 QUAD_GPIO_RF_CH4 | QUAD_GPIO_RF_CH5 | QUAD_GPIO_RF_CH6);

	GPIOPadConfigSet(QUAD_GPIO_RF_BASE, QUAD_GPIO_RF_CH1 | QUAD_GPIO_RF_CH2 | QUAD_GPIO_RF_CH3 |
			 	 	 QUAD_GPIO_RF_CH4 | QUAD_GPIO_RF_CH5 | QUAD_GPIO_RF_CH6,
					 GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	// Enable RF interrupt
	rf_PinEdge = GPIO_RISING_EDGE;
	rf_CurrentChannel = QUAD_GPIO_RF_CH1;

	GPIOIntTypeSet(QUAD_GPIO_RF_BASE, rf_CurrentChannel, GPIO_RISING_EDGE);
	GPIOIntRegister(QUAD_GPIO_RF_BASE, PortRF_ISR);
	GPIOIntEnable(QUAD_GPIO_RF_BASE, rf_CurrentChannel);
}


/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  channel: 1, 2, 3, 4, 5, 6
  Return value :  None.
================================================================================================*/
uint32_t ReceiverRF_Read(uint8_t channel)
{
	uint8_t i = (channel - 1);
	uint32_t percent;
	float tmp;

	tmp = (rf_channel[i] - rf_channelMin[i])/
		  (rf_channelMax[i] - rf_channelMin[i])*100.0f;

	percent = (uint32_t)tmp;

	return percent;
}


/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void PortRF_ISR(void)
{
	int i;
	uint32_t time;

	// Clear timer interrupt
	GPIOIntClear(QUAD_GPIO_RF_BASE, rf_CurrentChannel);

	if(rf_PinEdge == GPIO_RISING_EDGE)								/* Rising edge */
	{
		// Enable Timer interrupt
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

		// Reset Timer0 ful-width counter
		HWREG(TIMER0_BASE + TIMER_O_TAV) = 0x00;

		// Change edge to FALLING
		rf_PinEdge = GPIO_FALLING_EDGE;
		GPIOIntTypeSet(QUAD_GPIO_RF_BASE, rf_CurrentChannel, GPIO_FALLING_EDGE);
	}
	else 															/* Falling edge */
	{
		// Get timer value
		time = TimerValueGet(TIMER0_BASE, TIMER_A);

		// Disable current channel
		GPIOIntDisable(QUAD_GPIO_RF_BASE, rf_CurrentChannel);

		// Disable Timer interrupt
		TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

		switch(rf_CurrentChannel)
		{
			case QUAD_GPIO_RF_CH1: rf_channel[0] = time; rf_CurrentChannel = QUAD_GPIO_RF_CH2; break;
			case QUAD_GPIO_RF_CH2: rf_channel[1] = time; rf_CurrentChannel = QUAD_GPIO_RF_CH3; break;
			case QUAD_GPIO_RF_CH3: rf_channel[2] = time; rf_CurrentChannel = QUAD_GPIO_RF_CH4; break;
			case QUAD_GPIO_RF_CH4: rf_channel[3] = time; rf_CurrentChannel = QUAD_GPIO_RF_CH5; break;
			case QUAD_GPIO_RF_CH5: rf_channel[4] = time; rf_CurrentChannel = QUAD_GPIO_RF_CH6; break;
			case QUAD_GPIO_RF_CH6: rf_channel[5] = time; rf_CurrentChannel = QUAD_GPIO_RF_CH1; break;
			default: break;
		}

		// Save min/max value
		for(i = 0; i < 6; i++)
		{
			if(rf_channel[i] < rf_channelMin[i])
					rf_channelMin[i] = rf_channel[i];

			if(rf_channel[i] > rf_channelMax[i])
				rf_channelMax[i] = rf_channel[i];
		}

		rf_PinEdge = GPIO_RISING_EDGE;
		GPIOIntTypeSet(QUAD_GPIO_RF_BASE, rf_CurrentChannel, GPIO_RISING_EDGE);
		GPIOIntEnable(QUAD_GPIO_RF_BASE, rf_CurrentChannel);
	}
}


/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void TimerRF_ISR(void)
{
	// Disable timer from future interrupt
	TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

#endif /* RECEIVERRF_H_ */
