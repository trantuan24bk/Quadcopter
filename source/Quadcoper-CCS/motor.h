/*
 * pwm.h
 *
 *  Created on: Dec 5, 2014
 *      Author: BLACKHAT
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "quadcopter.h"

#define PWM_PERIOD		15625 	// = 50MHz/64 divider/ 50Hz

// fPWM = 50Hz
// Duty (max) = 2ms
// Duty (min) = 1ms
// PWM1		-	PB6	-	M0PWM0
// PWM2		- 	PB7	-	M0PWM1
// PWM3		- 	PC5	-	M0PWM7
// PWM4		- 	PC4	-	M0PWM6


/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void init_PWM(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);

    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);

    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

    // Configure PWM Options
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_PERIOD);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
}

/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
void motor_UpdateDuty(uint8_t channel, uint8_t duty_percent)
{
	uint32_t speed;
	static const uint32_t PERCENT_1MS = 0.05*PWM_PERIOD;
	static const uint32_t PERCENT_2MS = 0.10*PWM_PERIOD;

	// 1ms - 2ms
	// 05% - 10%
	if(duty_percent > 100)
		duty_percent = 100;

	speed = PERCENT_1MS +  duty_percent* 0.01* (PERCENT_2MS - PERCENT_1MS);

	switch(channel)
	{
		case 1:	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, speed); break;
		case 2:	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, speed); break;
		case 3:	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, speed); break;
		case 4:	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed); break;
		default:
			break;
	}
}

#endif /* MOTOR_H_ */
