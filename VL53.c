
#include "VL53.h"
#include "../include/stdio.h"
#include "../include/stdint.h"
#include "p32xxxx.h"
#include "api/VL53L1X_api.h"
#include "api/VL53L1X_calibration.h"
#include "BMM_API/bmm150.h"
#include "BMM_API/bmm150_defs.h"

struct bmm150_dev dev;
int8_t rslt = BMM150_OK;


int i;
bool vl_initf;
uint8_t aux_8, booted,state,dataReady[6],Rangestat;
int8_t status, Status;
uint16_t aux_16;
uint32_t debug;
uint16_t fast_osc_frequency, osc_calibrate_val;
VL53L1_Dev_t Dv, Dv1,Dh,Dh1,Du,Dd;
bool vlinit=0;

void BMM_compensate()
{

//	/* Sensor interface over SPI with native chip select line */
//	dev.dev_id = 0;
//	dev.intf = BMM150_SPI_INTF;
//	dev.read = user_spi_read;
//	dev.write = user_spi_write;
//	dev.delay_ms = user_delay_ms;
//
//	rslt = bmm150_init(&dev);
}

void VL_init()
{
	IEC2bits.CCP6IE=0;
	IEC2bits.CCT6IE=0;
	IEC2bits.CCP4IE=0;
	IEC2bits.CCT4IE=0;
   IEC2bits.CCP5IE=0;
	IEC2bits.CCT5IE=0;

	//Sensor 1
	LATCbits.LATC15=1;  //y+
	for(i=0;i<24000;i++){};
	Dv.dummy=0x29;
	while (state==0)
	{
		Status = VL53L1X_BootState(Dv, &state);
	}
	Status = VL53L1X_SensorInit(Dv);
	Status =VL53L1X_SetI2CAddress(Dv,0x11<<1);

	Dv1.dummy=0x11;
	Status = VL53L1X_StartRanging(Dv1);

	//Sensor 2
	LATCbits.LATC2=1;  //y+
	for(i=0;i<24000;i++){};
	while (state==0)
	{
		Status = VL53L1X_BootState(Dv, &state);
	}
	Status = VL53L1X_SensorInit(Dv);
	Status =VL53L1X_SetI2CAddress(Dv,0x22<<1);

	Dh.dummy=0x22;
	Status = VL53L1X_StartRanging(Dh);

	//Sensor 3
	LATCbits.LATC4=1;  //y+
	for(i=0;i<24000;i++){};
	while (state==0)
	{
		Status = VL53L1X_BootState(Dv, &state);
	}
	Status = VL53L1X_SensorInit(Dv);
	Status =VL53L1X_SetI2CAddress(Dv,0x33<<1);

	Dh1.dummy=0x33;
	Status = VL53L1X_StartRanging(Dh1);

	//Sensor 4

	LATBbits.LATB3=1;  //Z+
	for(i=0;i<24000;i++){};
	while (state==0)
	{
		Status = VL53L1X_BootState(Dv, &state);
	}
	Status = VL53L1X_SensorInit(Dv);
	Status =VL53L1X_SetI2CAddress(Dv,0x44<<1);

	Du.dummy=0x44;
	Status = VL53L1X_StartRanging(Du);

	//Sensor 5
	LATBbits.LATB4=1;
	for(i=0;i<24000;i++){};
	while (state==0)
	{
		Status = VL53L1X_BootState(Dv, &state);
	}
	Status = VL53L1X_SensorInit(Dv);
	/* enable the ranging*/
	Status = VL53L1X_StartRanging(Dv);

	IEC2bits.CCP6IE=1;
	IEC2bits.CCT6IE=1;
	IEC2bits.CCP4IE=1;
	IEC2bits.CCT4IE=1;
	IEC2bits.CCP5IE=1;
	IEC2bits.CCT5IE=1;
}
bool Distance()
{
	//uint8_t isDataReady[6];
	uint16_t distance[6];
	//Sensor 1
	while(dataReady[0]==0)
	{
		Status = VL53L1X_CheckForDataReady(Dv, &dataReady[0]);
	}
	status=VL53L1X_GetRangeStatus(Dv,&Rangestat);
	for(i=0;i<2400;i++){};
	status=VL53L1X_GetDistance(Dv,&distance[0]);
	status=VL53L1X_ClearInterrupt(Dv);
//	printf("----Distance back= %i\n", distance[0]);

	//Sensor 2
	while(dataReady[1]==0)
	{
		Status = VL53L1X_CheckForDataReady(Dv1, &dataReady[1]);
	}
	status=VL53L1X_GetRangeStatus(Dv1,&Rangestat);
	for(i=0;i<2400;i++){};
	status=VL53L1X_GetDistance(Dv1,&distance[1]);
	status=VL53L1X_ClearInterrupt(Dv1);
//	printf("----Distance front= %i\n", distance[1]);

	//Sensor 3
	while(dataReady[2]==0)
	{
		Status = VL53L1X_CheckForDataReady(Dh, &dataReady[2]);
	}
	status=VL53L1X_GetRangeStatus(Dh,&Rangestat);
	for(i=0;i<2400;i++){};
	status=VL53L1X_GetDistance(Dh,&distance[2]);
	status=VL53L1X_ClearInterrupt(Dh);
//	printf("----Distance right= %i\n", distance[2]);

	//Sensor 4
	while(dataReady[3]==0)
	{
		Status = VL53L1X_CheckForDataReady(Dh1, &dataReady[3]);
	}
	status=VL53L1X_GetRangeStatus(Dh1,&Rangestat);
	for(i=0;i<2400;i++){};
	status=VL53L1X_GetDistance(Dh1,&distance[3]);
	status=VL53L1X_ClearInterrupt(Dh1);
//	printf("----Distance left= %i\n", distance[3]);

	//Sensor 5
	while(dataReady[4]==0)
	{
		Status = VL53L1X_CheckForDataReady(Du, &dataReady[4]);
	}
	status=VL53L1X_GetRangeStatus(Du,&Rangestat);
	for(i=0;i<2400;i++){};
	status=VL53L1X_GetDistance(Du,&distance[4]);
	status=VL53L1X_ClearInterrupt(Du);
	printf("----Distance up= %i  (%i)\n", distance[4],Rangestat);
	return vlinit;
}

void setAddress(uint8_t new_addr)
{
  VL_write_8(I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F , VL_ADDRESS);
  address = new_addr;
}


bool vl_init()
{
	VL_write_8(SOFT_RESET, 0x00 , VL_ADDRESS);
	printf("%i\n",VL_Read_8(SOFT_RESET,VL_ADDRESS));
	for(i=0;i<2400;i++){}
	VL_write_8(SOFT_RESET, 0x01 , VL_ADDRESS);
	printf("%i\n",VL_Read_8(SOFT_RESET,VL_ADDRESS));
	for(i=0;i<24000;i++){}


	while ((VL_Read_8(FIRMWARE__SYSTEM_STATUS,0x29) & 0x01) == 0)
	{
		printf("soft_reseting.../n");
	}
	printf("..1\n");
	aux_8=VL_Read_8(PAD_I2C_HV__EXTSUP_CONFIG, VL_ADDRESS);
	printf("..aux_8=%i\n",aux_8);
	VL_write_8(PAD_I2C_HV__EXTSUP_CONFIG,aux_8 | 0x01, VL_ADDRESS);
	printf("..2   ...............01===%x\n",VL_Read_8(PAD_I2C_HV__EXTSUP_CONFIG, VL_ADDRESS));


	fast_osc_frequency= VL_Read_16(OSC_MEASURED__FAST_OSC__FREQUENCY, VL_ADDRESS);
	printf("..3 %i\n",fast_osc_frequency);
	osc_calibrate_val = VL_Read_16(RESULT__OSC_CALIBRATE_VAL, VL_ADDRESS);
	printf("..4 %i\n",osc_calibrate_val);
	VL_write_16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate, VL_ADDRESS);
	printf("..5\n");
	VL_write_8(GPIO__TIO_HV_STATUS, 0x02 , VL_ADDRESS);
	printf("..6  02 ==== %x\n",VL_Read_8(GPIO__TIO_HV_STATUS, VL_ADDRESS));
	VL_write_8(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8 , VL_ADDRESS);
	printf("..7\n");
	VL_write_8(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16 , VL_ADDRESS);
	printf("..8\n");
	VL_write_8(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01 , VL_ADDRESS);
	printf("..9\n");
	VL_write_8(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xff , VL_ADDRESS);
	printf("..10\n");
	VL_write_8(ALGO__RANGE_MIN_CLIP, 0 , VL_ADDRESS);
	printf("..11\n");
	VL_write_8(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2 , VL_ADDRESS);
	printf("..12\n");
	//general config
	VL_write_16(SYSTEM__THRESH_RATE_HIGH, 0x0000 , VL_ADDRESS);
	printf("..13\n");

	VL_write_16(SYSTEM__THRESH_RATE_LOW, 0x0000 , VL_ADDRESS);
	printf("..14\n");

	VL_write_8(DSS_CONFIG__APERTURE_ATTENUATION, 0x38 , VL_ADDRESS);
	printf("..15\n");

	//timing config
	VL_write_16(RANGE_CONFIG__SIGMA_THRESH, 360 , VL_ADDRESS);
	printf("..16\n");

	VL_write_16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192 , VL_ADDRESS);
	printf("..17\n");

	//dynamic config
	VL_write_8(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01 , VL_ADDRESS);
	printf("..18\n");

	VL_write_8(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01 , VL_ADDRESS);
	printf("..19\n");

	VL_write_8(SD_CONFIG__QUANTIFIER, 2 , VL_ADDRESS);
	printf("..20\n");

	VL_write_8(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00 , VL_ADDRESS);
	printf("..21\n");
	VL_write_8(SYSTEM__SEED_CONFIG, 1 , VL_ADDRESS);
	printf("..22\n");

	// from VL53L1_config_low_power_auto_mode
	VL_write_8(SYSTEM__SEQUENCE_CONFIG, 0x8b , VL_ADDRESS);
	printf("..23\n");
	VL_write_16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, (200<<8) , VL_ADDRESS);
	printf("..24\n");
	VL_write_8(DSS_CONFIG__ROI_MODE_CONTROL, 2 , VL_ADDRESS);
	printf("..25\n");
	printf("set distm\n");
	/////////////////
	setDistanceMode(3, VL_ADDRESS);//long
	printf("seted distmode\n");
	setMeasurementTimingBudget(50000, VL_ADDRESS);
	printf("seted Time budget\n");
	//////////////////
	aux_16= VL_Read_16(MM_CONFIG__OUTER_OFFSET_MM,VL_ADDRESS)*4;
	VL_write_16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, aux_16 , VL_ADDRESS);
	printf("initated\n");

	return true;

}

bool setDistanceMode(uint8_t mode, uint8_t ADDR)
{
	uint32_t budget_us = getMeasurementTimingBudget(ADDR);
	printf("..getbudget= %i\n",budget_us);

	switch (mode)
	{
		case 1://Short
			// timing config
			VL_write_8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07 , ADDR);
			VL_write_8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05 , ADDR);
			VL_write_8(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38 , ADDR);

			// dynamic config
			VL_write_8(SD_CONFIG__WOI_SD0, 0x07 , ADDR);
			VL_write_8(SD_CONFIG__WOI_SD1, 0x05 , ADDR);
			VL_write_8(SD_CONFIG__INITIAL_PHASE_SD0, 6 , ADDR);
			VL_write_8(SD_CONFIG__INITIAL_PHASE_SD1, 6 , ADDR);
		break;

		case 2://Medium
			// timing config
			VL_write_8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B , ADDR);
			VL_write_8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09 , ADDR);
			VL_write_8(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78 , ADDR);

			// dynamic config
			VL_write_8(SD_CONFIG__WOI_SD0, 0x0B , ADDR);
			VL_write_8(SD_CONFIG__WOI_SD1, 0x09 , ADDR);
			VL_write_8(SD_CONFIG__INITIAL_PHASE_SD0, 10 , ADDR);
			VL_write_8(SD_CONFIG__INITIAL_PHASE_SD1, 10 , ADDR);
		break;

		case 3://Long
			// timing config
			VL_write_8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F , ADDR);
			printf("..3.1\n");
			VL_write_8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D , ADDR);
			printf("..3.2\n");
			VL_write_8(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8 , ADDR);
			printf("..3.3\n");
			// dynamic config
			VL_write_8(SD_CONFIG__WOI_SD0, 0x0F , ADDR);
			printf("..3.4\n");
			VL_write_8(SD_CONFIG__WOI_SD1, 0x0D , ADDR);
			printf("..3.5\n");
			VL_write_8(SD_CONFIG__INITIAL_PHASE_SD0, 14 , ADDR);
			printf("..3.6\n");
			VL_write_8(SD_CONFIG__INITIAL_PHASE_SD1, 14 , ADDR);
		break;

		 default:
      // unrecognized mode - do nothing
		return 0;
	}
	printf("..30\n");
	setMeasurementTimingBudget(budget_us, ADDR);
	distance_mode = mode;

	return 1;
}

bool setMeasurementTimingBudget(uint32_t budget_us, uint8_t ADDR)
{
	if (budget_us <= TimingGuard) { return false; }
	uint32_t range_config_timeout_us = budget_us -= TimingGuard;
	if (range_config_timeout_us > 1100000) { return false; }

	range_config_timeout_us /= 2;

	uint32_t macro_period_us;

	aux_8=VL_Read_8(RANGE_CONFIG__VCSEL_PERIOD_A, ADDR);
	macro_period_us = calcMacroPeriod(aux_8);

	uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
	if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
	VL_write_8(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks , ADDR);


	VL_write_16(MM_CONFIG__TIMEOUT_MACROP_A,encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)), ADDR);
	VL_write_16(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)) , ADDR);

	macro_period_us = calcMacroPeriod(VL_Read_8(RANGE_CONFIG__VCSEL_PERIOD_B, ADDR));

	VL_write_16(MM_CONFIG__TIMEOUT_MACROP_B,encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)), ADDR);
	VL_write_16(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)) , ADDR);

	return true;
}

uint32_t calcMacroPeriod(uint8_t vcsel_period)
{

	uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;
	uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

	uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
	macro_period_us >>= 6;
	macro_period_us *= vcsel_period_pclks;
	macro_period_us >>= 6;

	return macro_period_us;
}

uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint16_t encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

uint32_t decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

uint32_t getMeasurementTimingBudget(uint8_t ADDR)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
	aux_8=VL_Read_8(RANGE_CONFIG__VCSEL_PERIOD_A, ADDR);
	printf("..26 %i\n",aux_8);

  uint32_t macro_period_us = calcMacroPeriod(aux_8);
  printf("..27\n");
  // "Get Range Timing A timeout"
  aux_8=VL_Read_8(RANGE_CONFIG__TIMEOUT_MACROP_A, ADDR);
  printf("..28\n");
  uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(aux_8), macro_period_us);
  printf("..29\n");
  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
}

void startContinuous(uint32_t period_ms, uint8_t VL_ADDR)
{
  // from VL53L1_set_inter_measurement_period_ms()

  VL_write_32(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val, VL_ADDR);

  VL_write_8(SYSTEM__INTERRUPT_CLEAR, 0x01, VL_ADDR); // sys_interrupt_clear_range
  VL_write_8(SYSTEM__MODE_START, 0x40, VL_ADDR);  // mode_range__timed
}


void stopContinuous(uint8_t VL_ADDR)
{
  VL_write_8(SYSTEM__MODE_START, 0x80, VL_ADDR); // mode_range__abort

  // VL53L1_low_power_auto_data_stop_range() begin

  calibrated = false;

  // "restore vhv configs"
  if (saved_vhv_init != 0)
  {
    VL_write_8(VHV_CONFIG__INIT, saved_vhv_init, VL_ADDR);
  }
  if (saved_vhv_timeout != 0)
  {
     VL_write_8(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout, VL_ADDR);
  }

  // "remove phasecal override"
  VL_write_8(PHASECAL_CONFIG__OVERRIDE, 0x00, VL_ADDR);

  // VL53L1_low_power_auto_data_stop_range() end
}


uint16_t read(uint8_t ADDR)
{
	printf("..31\n");
  readResults(ADDR);

  if (!calibrated)
  {
	  printf("..cal\n");
    setupManualCalibration(ADDR);
    calibrated = true;
	 printf("..cal end\n");
  }

  updateDSS(ADDR);
  printf("..33\n");
  getRangingData(ADDR);
  printf("..34\n");
  VL_write_8(SYSTEM__INTERRUPT_CLEAR, 0x01, ADDR);

  return ranging_data.range_mm;
}

//uint16_t spadCount;
//uint32_t totalRatePerSpad;
//uint32_t requiredSpads;

void updateDSS(uint8_t ADDR)
{
  uint16_t spadCount = results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =(uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 + results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      VL_write_16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads, ADDR);
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    }
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
   VL_write_16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000, ADDR);
}

  void readResults(uint8_t ADDR)
{
	  I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
	  static I2C1_TRANSACTION_REQUEST_BLOCK trb[2];
	  uint8_t result[17];
	  uint8_t writeBuffer[2];

	 writeBuffer[0] = (RESULT__RANGE_STATUS >> 8);                        // high address
    writeBuffer[1] = (uint8_t)(RESULT__RANGE_STATUS);


    I2C1_MasterWriteTRBBuild(&trb[0], writeBuffer, 2, ADDR);
    I2C1_MasterReadTRBBuild(&trb[1], result, 17, ADDR);
    I2C1_MasterTRBInsert(2, trb, &status);

	 while(status == I2C1_MESSAGE_PENDING){}      // blocking

	 results.range_status = result[0];
	 //1
	 results.stream_count = result[2];
	 results.dss_actual_effective_spads_sd0  = (uint16_t)result[3] << 8; // high byte
	 results.dss_actual_effective_spads_sd0 |= result[4]; // low byte
	 // 5 6
	 results.ambient_count_rate_mcps_sd0   = (uint16_t)result[7] << 8; // high byte
	 results.ambient_count_rate_mcps_sd0  |= result[8]; // low byte
	 // 9 10
	 // 11 12
	 results.final_crosstalk_corrected_range_mm_sd0    = (uint16_t)result[13] << 8; // high byte
	 results.final_crosstalk_corrected_range_mm_sd0   |= result[14]; // low byte
	 results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0    = (uint16_t)result[15] << 8; // high byte
	 results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0   |= result[16]; // low byte
	 printf("..32\n");
//	 I2C1_MasterWriteTRBBuild(&trbr[0], writeBuffer, 2, ADDR);
//    I2C1_MasterReadTRBBuild(&trbr[1], &result[0], 17, ADDR);
//    I2C1_MasterTRBInsert(2, trbr, &status);
//
//	 results.range_status = result[0];
//	 //1
//	 results.stream_count = result[2];
//	 results.dss_actual_effective_spads_sd0  = (uint16_t)result[3] << 8; // high byte
//	 results.dss_actual_effective_spads_sd0 |= result[4]; // low byte
//	 // 5 6
//	 results.ambient_count_rate_mcps_sd0   = (uint16_t)result[7] << 8; // high byte
//	 results.ambient_count_rate_mcps_sd0  |= result[8]; // low byte
//	 // 9 10
//	 // 11 12
//	 results.final_crosstalk_corrected_range_mm_sd0    = (uint16_t)result[13] << 8; // high byte
//	 results.final_crosstalk_corrected_range_mm_sd0   |= result[14]; // low byte
//	 results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0    = (uint16_t)result[15] << 8; // high byte
//	 results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0   |= result[16]; // low byte
//	 while(status == I2C1_MESSAGE_PENDING){}      // blocking

  }


  void getRangingData(uint8_t ADDR)
{
  // VL53L1_copy_sys_and_core_results_to_range_results() begin

  uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // Basically, this appears to scale the result by 2011/2048, or about 98%
  // (with the 1024 added for proper rounding).
  ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

  // VL53L1_copy_sys_and_core_results_to_range_results() end

  // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
  // mostly based on ConvertStatusLite()
  switch(results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 1: // VCSELCONTINUITYTESTFAILURE
    case 3: // NOVHVVALUEFOUND
      // from SetSimpleData()
      ranging_data.range_status = HardwareFail;
      break;

    case 13: // USERROICLIP
     // from SetSimpleData()
      ranging_data.range_status = MinRangeFail;
      break;

    case 18: // GPHSTREAMCOUNT0READY
      ranging_data.range_status = SynchronizationInt;
      break;

    case 5: // RANGEPHASECHECK
      ranging_data.range_status =  OutOfBoundsFail;
      break;

    case 4: // MSRCNOTARGET
      ranging_data.range_status = SignalFail;
      break;

    case 6: // SIGMATHRESHOLDCHECK
      ranging_data.range_status = SignalFail;
      break;

    case 7: // PHASECONSISTENCY
      ranging_data.range_status = WrapTargetFail;
      break;

    case 12: // RANGEIGNORETHRESHOLD
      ranging_data.range_status = XtalkSignalFail;
      break;

    case 8: // MINCLIP
      ranging_data.range_status = RangeValidMinRangeClipped;
      break;

    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (results.stream_count == 0)
      {
        ranging_data.range_status = RangeValidNoWrapCheckFail;
      }
      else
      {
        ranging_data.range_status = RangeValid;
      }
      break;

    default:
      ranging_data.range_status = None;
  }

  // from SetSimpleData()
  ranging_data.peak_signal_count_rate_MCPS =
    countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  ranging_data.ambient_count_rate_MCPS =
    countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}


  void setupManualCalibration(uint8_t ADDR)
{
  // "save original vhv configs"
  saved_vhv_init = VL_Read_8(VHV_CONFIG__INIT, ADDR);
  saved_vhv_timeout = VL_Read_8(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, ADDR);
  printf("..34\n");
  // "disable VHV init"
  VL_write_8(VHV_CONFIG__INIT, saved_vhv_init & 0x7F, ADDR);
  printf("..35\n");
  // "set loop bound to tuning param"
  VL_write_8(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (saved_vhv_timeout & 0x03) + (3 << 2), ADDR);   // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
  printf("..36\n");
  // "override phasecal"
  VL_write_8(PHASECAL_CONFIG__OVERRIDE, 0x01, ADDR);
  printf("..37\n");
  aux_8=VL_Read_8(PHASECAL_RESULT__VCSEL_START, ADDR);
  printf("..37.5 aux_8=%i\n",aux_8);
  VL_write_8(CAL_CONFIG__VCSEL_START, aux_8 , ADDR);
  printf("..38\n");
}


//void ICM_Init(void)
//{
//}
