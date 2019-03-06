
#ifndef _Soft_PWM_H
#define _Soft_PWM_H
#include "stdint.h"

//include

//

//registros ICM
#define CPU_CLOCK_HZ             (24000000UL)       // CPU Clock Speed en Hz
#define CPU_CT_HZ            (CPU_CLOCK_HZ/2)       // CPU CoreTimer   en Hz
#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)       // US a CoreTimer Ticks
#define MS_TO_CT_TICKS  (CPU_CT_HZ/1000UL)          // MS a CoreTimer Ticks


#define RELOJ_PERIFERICO 24000000UL
#define DIVISOR_TIMER 1
#define FRECUENCIA_TIMER 400
#define PERIODO (RELOJ_PERIFERICO/(DIVISOR_TIMER*FRECUENCIA_TIMER) -1) //PR3 Periodo en Ticks




#ifdef __cplusplus
extern "C" {
#endif

void init_Soft_PWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
#endif

