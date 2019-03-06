
#ifndef _ICM_H
#define _ICM_H

//
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
//

//registros ICM

#define WHO_ICM 117

#define SELF_TEST_X_GYRO 0
#define SELF_TEST_Y_GYRO 1
#define SELF_TEST_Z_GYRO 2

#define SELF_TEST_X_ACCEL 13
#define SELF_TEST_Y_ACCEL 14
#define SELF_TEST_Z_ACCEL 15

#define X_OFFS_USR_MSB 19
#define X_OFFS_USR_LSB 20
#define Y_OFFS_USR_MSB 21
#define Y_OFFS_USR_LSB 22
#define Z_OFFS_USR_MSB 23
#define Z_OFFS_USR_LSB 24

#define SMPLRT_DIV 25
#define ICM_CONFIG 26
#define GYRO_CONFIG 27
#define ACCEL_CONFIG 28
#define ACCEL_CONFIG2 29
#define LP_MODE_CFG 30
#define WOM_X_TH 32
#define WOM_Y_TH 33
#define WOM_Z_TH 34
#define FIFO_EN 35

#define FSYNC_INT 54
#define INT_PIN_CFG 55
#define INT_ENABLE 56
#define DMP_INT_STATUS 57
#define INT_STATUS 58

#define ACCEL_XOUT_H 59
#define ACCEL_XOUT_L 60
#define ACCEL_YOUT_H 61
#define ACCEL_YOUT_L 62
#define ACCEL_ZOUT_H 63
#define ACCEL_ZOUT_L 64

#define TEMP_OUT_H 65
#define TEMP_OUT_L 66

#define GYRO_XOUT_H 67
#define GYRO_XOUT_L 68
#define GYRO_YOUT_H 69
#define GYRO_YOUT_L 70
#define GYRO_ZOUT_H 71
#define GYRO_ZOUT_L 72

#define SIGNAL_PATH_RESET 104
#define ACCEL_INTEL_CTRL 105
#define USER_CTRL 106
#define PWR_MGMT_1 107
#define PWR_MGMT_2 108

#define FIFO_COUNTH 114
#define FIFO_COUNTL 115

#define FIFO_R_W 116

#define XA_OFFSET_H 119
#define XA_OFFSET_L 120
#define YA_OFFSET_H 121
#define YA_OFFSET_L 122
#define ZA_OFFSET_H 123
#define ZA_OFFSET_L 124


//Registros NRF

#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX		0b11100001
#define FLUSH_RX		0b11100010
#define REUSE_TX_PL	0b11100011
#define R_RX_PL_WID	0b01100000
#define W_TX_PAYLOAD_NO 0b10110000
#define NOP 0b11111111



#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define RPD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F

#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D






//Registros Mag

#define Mag_ID 0x40
#define DATAX_lsb 0x42
#define DATAX_msb 0x43
#define DATAY_lsb  0x44
#define DATAY_msb  0x45
#define DATAZ_lsb  0x46
#define DATAZ_msb  0x47
#define RHALL_lsb  0x48
#define RHALL_msb  0x49
#define Interrupt_Status 0x4A
#define Power_Ctrl 0x4B
#define Op_Ctrl 0x4C
#define Int_Ctrl 0x4D
#define Int_Ctrl2 0x4E
#define LowThreshold 0x4F
#define HighThreshold 0x50
#define REPXY 0x51
#define REPZ 0x52

/* TRIM REGISTERS      */
/* Trim Extended Registers */
#define BMM150_DIG_X1               UINT8_C(0x5D)
#define BMM150_DIG_Y1               UINT8_C(0x5E)
#define BMM150_DIG_Z4_LSB           UINT8_C(0x62)
#define BMM150_DIG_Z4_MSB           UINT8_C(0x63)
#define BMM150_DIG_X2               UINT8_C(0x64)
#define BMM150_DIG_Y2               UINT8_C(0x65)
#define BMM150_DIG_Z2_LSB           UINT8_C(0x68)
#define BMM150_DIG_Z2_MSB           UINT8_C(0x69)
#define BMM150_DIG_Z1_LSB           UINT8_C(0x6A)
#define BMM150_DIG_Z1_MSB           UINT8_C(0x6B)
#define BMM150_DIG_XYZ1_LSB         UINT8_C(0x6C)
#define BMM150_DIG_XYZ1_MSB         UINT8_C(0x6D)
#define BMM150_DIG_Z3_LSB           UINT8_C(0x6E)
#define BMM150_DIG_Z3_MSB           UINT8_C(0x6F)
#define BMM150_DIG_XY2              UINT8_C(0x70)
#define BMM150_DIG_XY1 UINT8_C(0x71)


#ifdef __cplusplus
extern "C" {
#endif

void ICM_Init(void);
void ICM_quat (void);
void ICM_Raw(void);
void Print_ag (void);
static float invSqrt(float number);
void ICM_QuatInit();//float ax, float ay, float az, float mx, float my, float mz);

void Rate_Ctrl(void);
void Atti_Ctrl(void);

uint8_t NRF_Write_Buf(uint8_t naddr,uint8_t *nregw, uint8_t size);
uint8_t NRF_Read_Buf(uint8_t naddr,uint8_t *nregw, uint8_t size);
void NRF_adr(void);
void NRF_Recive(void);
void NRF_Init(void);
void NRF_RxMode(void);

void BMM_init2(void);
void BMM_read(void);

#endif

