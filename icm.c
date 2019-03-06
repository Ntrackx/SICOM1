
#include "icm.h"
#include "../include/stdio.h"
#include "p32xxxx.h"
#include "../include/math.h"
#include "filter.h"
#include "Soft_PWM.h"
#include "BMM_API/bmm150_defs.h"
#include "mcc_generated_files/spi3.h"

uint8_t  RX_ADDRESS[5]= {0x34,0xc3,0x10,0x10,0x00};
uint8_t  RX_DATA[32];

uint16_t RC_thro,RC_yaw,RC_roll,RC_pitch;

uint8_t ans=0, armState, imuCaliFlag;
int16_t ax=0,ay=0,az=0;
int16_t ax_mag=0,ay_mag=0,az_mag=0,Rhall=0;
int16_t gx=0,gy=0,gz=0,cont=0;
//float ax_r=0, ay_r=0, az_r=0;
//float gx_r=0, gy_r=0, gz_r=0;
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
float integralFBx=0, integralFBy=0, integralFBz=0;
float Rot_matrix[9] = {1,  0,  0, 0,  1,  0, 0,  0,  1};
float gyro_ac[3], acc_ac[3];
float euler[3] = {0, 0, 0};
float hx, hy, bx, bz;
float halfwx, halfwy, halfwz;
float qa, qb, qc, twoKi=0.05, twoKp=1;

volatile float ax_r, ay_r, az_r, gx_r, gy_r, gz_r; //ax2, gx2, ay2, gy2, az2, gz2;

//CONTROL

float ang_pitch, ang_roll, ang_yaw;
float RollR_Error, PitchR_Error, YawR_Error, RollR_SP, PitchR_SP, YawR_SP;
float RollA_Error, PitchA_Error, YawA_Error, RollA_SP, PitchA_SP, YawA_SP;

float RollA_ant, PitchA_ant, YawA_ant;
float RollA_deriv, RollA_int, PitchA_deriv, PitchA_int, YawA_deriv, YawA_int;

float RollR_ant, PitchR_ant;
float RollR_deriv, RollR_ant, RollR_int, PitchR_deriv, PitchR_int, Roll, Pitch;

float RollA_lim=300, PitchA_lim=300, YawA_lim=300;
float RollR_lim=300, PitchR_lim=300;

float P_A=3.5, D_A=0, I_A=0, P_AY=20, D_AY=0, I_AY=0;
float P_R=0.7, D_R=0.03, I_R=0.5, P_RY=20, D_RY=0, I_RY=0;

float Rate_roll, Rate_pitch, Rate_yaw;

float mx=0, my=0, mz=0;
static float sampleFreq=1000;
int i;
static uint8_t sta;
uint16_t ds;
struct bmm150_dev mag;


void BMM_init2()
{
	mag.chip_id=UINT8_C(0x32);
	mag.delay_ms=&BMM_Delay;
	mag.write = &BMM_Read_Reg;
	mag.read = &BMM_Write_Reg;

	bmm150_init(mag);
}
void BMM_read()
{
	bmm150_read_mag_data(mag);
	printf("x = %i  ,  y = %i  ,  z = %i  \n",mag.data.x, mag.data.y, mag.data.z);
	ax_mag= MAG_Read_Reg(DATAX_msb)<<8;
	ax_mag= (ax_mag | MAG_Read_Reg(DATAX_lsb))>>3;
	ay_mag= MAG_Read_Reg(DATAY_msb)<<8;
	ay_mag= (ay_mag | MAG_Read_Reg(DATAY_lsb))>>3;
	az_mag= MAG_Read_Reg(DATAZ_msb)<<8;
	az_mag= (az_mag | MAG_Read_Reg(DATAZ_lsb))>>1;
	Rhall= MAG_Read_Reg(RHALL_msb)<<8;
	Rhall= (az_mag | MAG_Read_Reg(RHALL_lsb))>>2;
}
//void BMM_compensateX()
//{
//	int16_t retval;
//	uint16_t process_comp_x0 = 0;
//	int32_t process_comp_x1;
//	uint16_t process_comp_x2;
//	int32_t process_comp_x3;
//	int32_t process_comp_x4;
//	int32_t process_comp_x5;
//	int32_t process_comp_x6;
//	int32_t process_comp_x7;
//	int32_t process_comp_x8;
//	int32_t process_comp_x9;
//	int32_t process_comp_x10;
//
//	/* Overflow condition check */
//	if (ax_mag != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) {
//		if (Rhall != 0) {
//			/* Availability of valid data*/
//			process_comp_x0 = Rhall;
//		} else if (trim_data.dig_xyz1 != 0) {
//			process_comp_x0 = trim_data.dig_xyz1;
//		} else {
//			process_comp_x0 = 0;
//		}
//		if (process_comp_x0 != 0) {
//			/* Processing compensation equations*/
//			process_comp_x1 = ((int32_t)trim_data.dig_xyz1) * 16384;
//			process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
//			retval = ((int16_t)process_comp_x2);
//			process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
//			process_comp_x4 = (((int32_t)trim_data.dig_xy2) * (process_comp_x3 / 128));
//			process_comp_x5 = (int32_t)(((int16_t)trim_data.dig_xy1) * 128);
//			process_comp_x6 = ((int32_t)retval) * process_comp_x5;
//			process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
//			process_comp_x8 = ((int32_t)(((int16_t)trim_data.dig_x2) + ((int16_t)0xA0)));
//			process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
//			process_comp_x10 = ((int32_t)ax_mag) * process_comp_x9;
//			retval = ((int16_t)(process_comp_x10 / 8192));
//			retval = (retval + (((int16_t)trim_data.dig_x1) * 8)) / 16;
//		} else {
//			retval = BMM150_OVERFLOW_OUTPUT;
//		}
//	} else {
//		/* Overflow condition */
//		retval = BMM150_OVERFLOW_OUTPUT;
//	}
//
//return retval;
//}

void ICM_Init(void)
{
	ax=ICM_Write_Reg(PWR_MGMT_1,0);	// ICM SLEEP_MODE OFF
	ax=ICM_Write_Reg(GYRO_CONFIG,0b00011000);// ICM 2000°/s
	ax=ICM_Write_Reg(ACCEL_CONFIG,0b00010000);// ICM 8g
	ax= MAG_Write_Reg(Power_Ctrl,0b00000001); // Magnetometer SLEEP_MODE OFF
	ax= MAG_Write_Reg(Op_Ctrl,0b00000000); // Magnetometer ACTIVE_MODE
}


void NRF_Init(void)
{
	uint8_t NRFMatched;

	RX_ADDRESS[4]=100;
	do
	{
		NRFMatched=0;
		NRF_RxMode();	//
		sta = NRF_Read_Reg(STATUS);
		printf("matching... %i\n",sta);
		printf("RX_ADDRESS[4]... %i\n",RX_ADDRESS[4]);
		if((sta & 0x0E )== 0x00){
				NRFMatched = 1;
			}else{
				RX_ADDRESS[4] ++;		//Search the next RX_ADDRESS
				if(RX_ADDRESS[4] == 0xff ){
					RX_ADDRESS[4] = 0x00;
				}
			}
	} while((sta & 0x0E )== 0x0E);
	printf("matched\n",ans);
	NRF_RxMode();
}

void NRF_RxMode(void)
{
	LATBbits.LATB7=0;
	ax=NRF_Exe(FLUSH_RX);
	NRF_adr();
	ax=NRF_Write_Reg(EN_AA,0x01);
	ax=NRF_Write_Reg(EN_RXADDR,0x01);
	ax=NRF_Write_Reg(RF_CH,40);
	ax=NRF_Write_Reg(RX_PW_P0,32);
	ax=NRF_Write_Reg(RF_SETUP,0x0f);
	ax=NRF_Write_Reg(CONFIG,0x0f);
	LATBbits.LATB7=1;
	LATBbits.LATB14=1;
	for(i=0;i<15000;i++){}
}

//Recepcion datos del control
void NRF_Recive(void)
{
	uint8_t sta=NRF_Read_Reg(STATUS);
	if(sta & (1<<6))
	{
		NRF_Read_Buf(R_RX_PAYLOAD,RX_DATA,32);//Lectura datos control
		printf("RXDATA = %i\n",RX_DATA[4]);
		////
				 switch(RX_DATA[4])
		 {
			 case 7:	//Recepcion de informacion
					RC_thro=RX_DATA[5] + (RX_DATA[6]<<8);		//Aceleración
					RC_yaw=RX_DATA[7]  +  (RX_DATA[8]<<8);		//Yaw
					RC_pitch=RX_DATA[9] + (RX_DATA[10]<<8);	//Pitch
					RC_roll=RX_DATA[11] + (RX_DATA[12]<<8);	//Roll
					if(RC_thro<=1540)
					{
						Motor_PWM(0, 0, 0, 0);
					}
					else
					{
						Motor_PWM((RC_thro-1500)*16, (RC_thro-1500)*16, (RC_thro-1500)*16, (RC_thro-1500)*16);
					}
			 break;
			  case 5://
					armState =1;	//Armar
					Motor_PWM(2000, 2000, 2000, 2000);
				break;
			 case 6:
					armState =2;	//Desarm
					Motor_PWM(2000, 2000, 2000, 2000);
			 break;
			 case 205:				//Calibrar
					imuCaliFlag = 1;
			 break;
			 case 0:
					Motor_PWM(0, 0, 0, 0);
					NRF_Init();
			 break;
		 }
    }
		////
		sta=0;
//		RollA_SP=RC_roll;;
//		RollA_SP=RC_roll;
//		RollA_SP=RC_roll;
//		RollA_SP=RC_roll;

		NRF_Write_Reg(0x27, sta);	//Limpia flag NRF
		sta = 0;
//		printf("RC_thro= %i  -  RC_yaw= %i  -  RC_pitch= %i  -  RC_roll= %i\n",RC_thro,RC_yaw,RC_pitch,RC_roll);
}



uint8_t NRF_Write_Buf(uint8_t naddr,uint8_t *nregw, uint8_t size)
{
    uint8_t reg_val,i;
    LATBbits.LATB7=0;
    reg_val = SPI3_Exchange8bit(naddr | 0x20);
	 for (i=0;i<size;i++)
	 {
		 SPI3_Exchange8bit(nregw[i]);
	 }
    LATBbits.LATB7=1;

    return 	reg_val;
}

void NRF_adr(void)
{
	uint8_t reg_val,i;
   LATBbits.LATB7=0;
   reg_val = SPI3_Exchange8bit(RX_ADDR_P0 | 0x20);
	for (i=0;i<5;i++)
	{
		SPI3_Exchange8bit(RX_ADDRESS[i]);
	}
   LATBbits.LATB7=1;
}

uint8_t NRF_Read_Buf(uint8_t naddr,uint8_t *nreg, uint8_t size)
{
    uint8_t reg_val,i;
    LATBbits.LATB7=0;
    reg_val = SPI3_Exchange8bit(naddr);
	 for (i=0;i<size;i++)
	 {
		 nreg[i]=SPI3_Exchange8bit(0);
	 }
    LATBbits.LATB7=1;

    return 	reg_val;
}



void ICM_Raw(void)
{
	uint16_t i=0;

	ans= ICM_Read_Reg(PWR_MGMT_1);

	ax=ICM_Read_Reg(ACCEL_XOUT_H)<<8;
	ax=ax|ICM_Read_Reg(ACCEL_XOUT_L);

	ay=ICM_Read_Reg(ACCEL_YOUT_H)<<8;
	ay=ay|ICM_Read_Reg(ACCEL_YOUT_L);

	az=ICM_Read_Reg(ACCEL_ZOUT_H)<<8;
	az=az|ICM_Read_Reg(ACCEL_ZOUT_L);


	gx=ICM_Read_Reg(GYRO_XOUT_H)<<8;
	gx=gx|ICM_Read_Reg(GYRO_XOUT_H);

	gy=ICM_Read_Reg(GYRO_YOUT_H)<<8;
	gy=gy|ICM_Read_Reg(GYRO_YOUT_H);

	gz=ICM_Read_Reg(GYRO_ZOUT_H)<<8;
	gz=gz|ICM_Read_Reg(GYRO_ZOUT_H);

//	printf("%i,%i,%i,%i,%i,%i,",ax,ay,az,gx,gy,gz);//,euler[0],euler[1],euler[2]);

	///PRUEBA MATLAB
	gz=-gz;

	ax_r=-(ax*0.0024); //8/32768)*9.8;
	ay_r=-(ay*0.0024); //8/32768)*9.8;
	az_r=(az*0.0024); //8/32768)*9.8;

	gx_r=(gx*0.001065); //20000/32768)*0.0174533;
	gy_r=(gy*0.001065); //20000/32768)*0.0174533;
	gz_r=(gz*0.001065); //20000/32768)*0.0174533;
	if(cont==0)
	{
		acc_ac[0]=0;
		acc_ac[1]=0;
		acc_ac[2]=0;
		gyro_ac[0]=0;
		gyro_ac[1]=0;
		gyro_ac[2]=0;
	}
//	for(i=0;i<4000;i++){}
//	printf("%f,%f,%f,99\n",ax_r,ay_r,az_r/*,gx_r,gy_r,gz_r*/);
	if (cont<100)
	{
		gyro_ac[0]=gyro_ac[0]+gx_r;
		gyro_ac[1]=gyro_ac[1]+gy_r;
		gyro_ac[2]=gyro_ac[2]+gz_r;

		acc_ac[0]=acc_ac[0]+ax_r;
		acc_ac[1]=acc_ac[1]+ay_r;
		acc_ac[2]=acc_ac[2]+az_r;

      cont=cont+1;
	}

	if (cont==100)
	{
		gyro_ac[0]=gyro_ac[0]/cont;
		gyro_ac[1]=gyro_ac[1]/cont;
		gyro_ac[2]=gyro_ac[2]/cont;

		acc_ac[0]=acc_ac[0]/cont;
		acc_ac[1]=acc_ac[1]/cont;
		acc_ac[2]=acc_ac[2]/cont;
		acc_ac[2]=acc_ac[2]-9.8;
	}
	double auxa;


	if(cont>=100)
	{
		auxa=ax_r;
		ax_r=ay_r-acc_ac[1];
		ay_r=auxa-acc_ac[0];
		az_r=az_r-acc_ac[2];
	}

	ax_r=LPF2pApply_1(ax_r);
	ay_r=LPF2pApply_2(ay_r);
	az_r=LPF2pApply_3(az_r);

	gx_r=LPF2pApply_4(gx_r);
	gy_r=LPF2pApply_5(gy_r);
	gz_r=LPF2pApply_6(gz_r);

//	printf("DATA= %f,%f,%f,%f,%f,%f,99\n",ax_r,ay_r,az_r,gx_r,gy_r,gz_r);
		if(cont>=100)
	{
		auxa=gx_r;
		gx_r=gy_r-gyro_ac[0];
		gy_r=auxa-gyro_ac[1];
		gz_r=gz_r-gyro_ac[2];

		Rate_roll=gx_r*180/3.1416;
		Rate_pitch=gy_r*180/3.1416;
		Rate_yaw=gz_r*180/3.1416;
	}
	if (cont==100)
	{
		ICM_QuatInit();
 	cont=151;
	}

	if(cont>=100)
	{
		ICM_quat();
	}


	//Transformar a angulo

	Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
   Rot_matrix[1] = 2 * (q1*q2 + q0*q3);	// 12
   Rot_matrix[2] = 2 * (q1*q3 - q0*q2);	// 13
   Rot_matrix[3] = 2 * (q1*q2 - q0*q3);	// 21
   Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
   Rot_matrix[5] = 2 * (q2*q3 + q0*q1);	// 23
   Rot_matrix[6] = 2 * (q1*q3 + q0*q2);	// 31
   Rot_matrix[7] = 2 * (q2*q3 - q0*q1);	// 32
   Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

	euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll
   euler[1] = -asinf(Rot_matrix[2]);									//! Pitch
   euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);

//	printf("%f,%f,%f,99\n",euler[0],euler[1],euler[2]);

	ang_roll=euler[0]*180/3.1416;
	ang_pitch=euler[1]*180/3.1416;
	ang_yaw=euler[2]*180/3.1416;

	//printf("%f,%f,%f,99\n",ang_roll,ang_pitch,ang_yaw);
}

void ICM_QuatInit()//float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay_r, -az_r);
    initialPitch = atan2(ax_r, -az_r);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5);
    sinRoll = sinf(initialRoll * 0.5);

    cosPitch = cosf(initialPitch * 0.5);
    sinPitch = sinf(initialPitch * 0.5);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

void ICM_quat (void)
{
		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	//if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
	//	MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
	//	return;
	//}
	float recipNorm;
	float halfex=0, halfey=0, halfez=0;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

	twoKi=0.05;
	twoKp=1;
	if(!((ax_r == 0.0) && (ay_r == 0.0) && (az_r == 0.0)))
	{
		float halfvx, halfvy, halfvz;
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax_r * ax_r + ay_r * ay_r + az_r * az_r);
		ax_r *= recipNorm;
		ay_r *= recipNorm;
		az_r *= recipNorm;

		// Normalise magnetometer measurement
		//recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		//mx *= recipNorm;
		//my *= recipNorm;
		//mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

//        //// Reference direction of Earth's magnetic field
//        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
//        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
//        bx = sqrt(hx * hx + hy * hy);
//        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5 + q3q3;
//      halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
//      halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
//      halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay_r * halfvz - az_r * halfvy);// + (my * halfwz - mz * halfwy);
		halfey = (az_r * halfvx - ax_r * halfvz);// + (mz * halfwx - mx * halfwz);
		halfez = (ax_r * halfvy - ay_r * halfvx);// + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0) {
			integralFBx += twoKi * halfex * (1.0 / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0 / sampleFreq);
			integralFBz += twoKi * halfez * (1.0 / sampleFreq);
			gx_r += integralFBx;	// apply integral feedback
			gy_r += integralFBy;
			gz_r += integralFBz;
		}
		else {
			integralFBx = 0.0;	// prevent integral windup
			integralFBy = 0.0;
			integralFBz = 0.0;
		}

		// Apply proportional feedback
		gx_r += twoKp * halfex;
		gy_r += twoKp * halfey;
		gz_r += twoKp * halfez;
	}
	// Integrate rate of change of quaternion
	gx_r *= (0.5 * (1.0 / sampleFreq));		// pre-multiply common factors
	gy_r *= (0.5 * (1.0 / sampleFreq));
	gz_r *= (0.5 * (1.0 / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx_r - qc * gy_r - q3 * gz_r);
	q1 += (qa * gx_r + qc * gz_r - q3 * gy_r);
	q2 += (qa * gy_r - qb * gz_r + q3 * gx_r);
	q3 += (qa * gz_r + qb * gy_r - qc * gx_r);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void Rate_Ctrl(void)
{
	// Roll Rate
	RollR_Error= RollR_SP - Rate_roll;
	RollR_deriv = (RollR_Error- RollR_ant)/0.01;
	if(RollR_int < RollR_lim || RollR_int > -RollR_lim)
	{
		RollR_int = RollR_int + RollR_Error*0.01;
	}
	Roll = RollR_Error*P_R + RollR_deriv*D_R + RollR_int*I_R ;
	RollR_ant = RollR_Error;

	// Pitch Rate
	PitchR_Error= PitchR_SP - Rate_pitch;
	PitchR_deriv = (PitchR_Error- PitchR_ant)/0.01;
	if(PitchR_int < PitchR_lim || PitchR_int > -PitchR_lim)
	{
		PitchR_int = PitchR_int + PitchR_Error*0.01;
	}
	Pitch = PitchR_Error*P_R + PitchR_deriv*D_R + PitchR_int*I_R ;
	PitchR_ant = PitchR_Error;
}
void Atti_Ctrl(void)
{
	// Roll Atti/////////////////////////////////////////
	RollA_Error = RollA_SP - ang_roll;
	RollA_deriv = (RollA_Error- RollA_ant)/0.01;
	if(RollA_int < RollA_lim || RollA_int > -RollA_lim)
	{
		RollA_int = RollA_int + RollA_Error*0.01;
	}
	RollR_SP = RollA_Error*P_A + RollA_deriv*D_A + RollA_int*I_A ;
	RollA_ant = RollA_Error;

	// Pitch Atti//////////////////////////////////////
	PitchA_Error= PitchA_SP - ang_pitch;
	PitchA_deriv = (PitchA_Error- PitchA_ant)/0.01;
	if(PitchA_int < PitchA_lim || PitchA_int > -PitchA_lim)
	{
		PitchA_int = PitchA_int + PitchA_Error*0.01;
	}
	PitchR_SP = PitchA_Error*P_A + PitchA_deriv*D_A + PitchA_int*I_A ;
	PitchA_ant = PitchA_Error;

	// Yaw Atti///////////////////////////////////////
	YawA_Error= YawA_SP - ang_yaw;
	YawA_deriv = (YawA_Error- YawA_ant)/0.01;
	if(YawA_int < YawA_lim || YawA_int > -YawA_lim)
	{
		YawA_int = YawA_int + YawA_Error*0.01;
	}
	YawR_SP = YawA_Error*P_AY + YawA_deriv*D_AY + YawA_int*I_AY ;
	YawA_ant = YawA_Error;
}


void Print_ag (void)
{
}
static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5;

    x = number * 0.5;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}


//uint8_t NRF24L01_Check(void)
//{
////   uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2};
////   uint8_t buf1[5];
////   uint8_t i=0;
////
////   /*??5 ?????? Escribir una dirección de 5 bytes..  */
////   ax=NRF_Write_Buf(TX_ADDR,buf,5);
////
////   /*??????? Leer la direccion escrita*/
////   ax=NRF_Read_Buf(TX_ADDR,buf1,5);
////
////    /*Comparacion*/
////   for (i=0;i<5;i++)
////   {
////      if (buf1[i]!=0xC2)
////      break;
////   }
////
////   if (i==5)   {printf("NRF24L01 found...\r\n");return 1 ;}        //MCU ?NRF ????
////   else        {printf("NRF24L01 check failed...\r\n");return 0 ;}        //MCU?NRF?????
//}