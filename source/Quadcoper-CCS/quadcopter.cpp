#include "quadcopter.h"
#include "ReceiverRF.h"
#include "motor.h"


/*===============================================================================================
  Name         :
  Description  :
  Argument(s)  :  None.
  Return value :  None.
================================================================================================*/
int main(void)
{
	init_Clock();
	init_ReceiverRF();
	init_UART();
	init_PWM();

	motor_UpdateDuty(1, 10);
	motor_UpdateDuty(2, 40);
	motor_UpdateDuty(3, 50);
	motor_UpdateDuty(4, 90);

	/* Enable global interrupt */
	IntMasterEnable();

	while(1)
	{
		UARTprintf("CH1: %u,\t  CH2: %u,\t CH3: %u,\t CH4: %u,\t CH5: %u,\t CH6: %u \n",
//				ReceiverRF_Read(1), ReceiverRF_Read(2), ReceiverRF_Read(3),
//				ReceiverRF_Read(4), ReceiverRF_Read(5), ReceiverRF_Read(6));
				rf_channel[0], rf_channel[1], rf_channel[2], rf_channel[3],
				rf_channel[4], rf_channel[5]);

		SysCtlDelay(SysCtlClockGet()/10);
	}
}

