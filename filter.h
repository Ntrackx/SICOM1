/* ************************************************************************** */
/** Descriptive File Name

  @Company
	 Company Name

  @File Name
	 filename.h

  @Summary
	 Brief description of the file.

  @Description
	 Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _FILTER_H    /* Guard against multiple inclusion */
#define _FILTER_H


#define M_PI_F 3.1415926


#ifdef __cplusplus
extern "C" {
#endif

void Filter_Init(void);

void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);
float LPF2pApply_1(float sample);

void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq);
float LPF2pApply_2(float sample);

void LPF2pSetCutoffFreq_3(float sample_freq, float cutoff_freq);
float LPF2pApply_3(float sample);

void LPF2pSetCutoffFreq_4(float sample_freq, float cutoff_freq);
float LPF2pApply_4(float sample);

void LPF2pSetCutoffFreq_5(float sample_freq, float cutoff_freq);
float LPF2pApply_5(float sample);

void LPF2pSetCutoffFreq_6(float sample_freq, float cutoff_freq);
float LPF2pApply_6(float sample);


#endif
////
//#include <xc.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include <stddef.h>
////
//
////registros ICM

//#define REPZ 0x52
//
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//void ICM_Init(void);

//static void ICM_QuatInit();//float ax, float ay, float az, float mx, float my, float mz);
//
//
//uint8_t NRF_Write_Buf(uint8_t naddr,uint8_t *nregw, uint8_t size);
//uint8_t NRF_Read_Buf(uint8_t naddr,uint8_t *nregw, uint8_t size);

//#endif
