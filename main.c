
#include <proc/p32mm0256gpm064.h>

#include "mcc_generated_files/system.h"
#include "../include/stdio.h"
#include "icm.h"
#include "VL53.h"
#include "mcc_generated_files/i2c1.h"
#include "mcc_generated_files/sccp4_compare.h"
#include "mcc_generated_files/mccp1_compare.h"
#include "Soft_PWM.h"
#include "sys/attribs.h"
#include "filter.h"
#include "api/vl53l1_platform.h"

uint8_t Data;
int16_t a=0;
static uint8_t Timer_cont, contt=0;
bool flag_100=0, flag_50=0, flag_10=0, qinit_flag=1;

int main(void)
{
	int i=0;
	uint8_t ans=0;
	bool init;
	uint8_t data2=0;
	uint16_t m1,m2,m3,m4;

    SYSTEM_Initialize();
	 ICM_Init();
	 VL_init();
	 NRF_Init();
	 Motor_PWM(0, 0, 0, 0);// 0-60000

	 while (1)
    {
		 if(Timer_cont==10)
		 {
			 flag_10=1;
		 }
		 if(Timer_cont==2 || Timer_cont==4 || Timer_cont==6 || Timer_cont==8 || Timer_cont==10)
		 {
			 flag_50=1;
		 }

		 if(flag_100)	//Loop 100hz
		 {
//			 ICM_Raw();	//Estimacion de orientación
//			 Print_ag();
			 Rate_Ctrl();	//Control velocidad de giro
//			 Motor_PWM(m1, m2, m3, m4);// 0-60000
			 flag_100=0;
		 }
		 if(flag_50)	//Loop 50hz
		 {
//			 Atti_Ctrl();
			 flag_50=0;
		 }
		 if(flag_10)	//Loop 10hz
		 {
//			 printf("hola\n");
//			 ICM_Raw();	//Estimacion de orientación
			 Distance();
			 NRF_Recive();
			 Timer_cont=1;
			 flag_10=0;
		 }
	 }
    return 1;
}

void __ISR(_TIMER_2_VECTOR, IPL3AUTO) Timer2(void)
{
	Timer_cont=Timer_cont+1;
	flag_100=1;
	IFS0bits.T2IF=0;
}


