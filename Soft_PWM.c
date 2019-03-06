#include "Soft_PWM.h"
#include <math.h>
//plib.h
#include "stdio.h"
#include <stdlib.h>
#include "p32xxxx.h"

/*Para la función de delay*/

void Motor_PWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
	CCP4RB=m3;	// <-
	CCP1RB=m2;	// ->
	CCP6RB=60002-m4;	// <_
	CCP5RB=60002-m1;	// _>
}


