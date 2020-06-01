/*
 * elecParamCalc.h
 *
 *  Created on: 30 May 2020
 *      Author: Mehmet Emre
 */

#ifndef ELECPARAMCALC_H_
#define ELECPARAMCALC_H_

#include "stdio.h"
#include <stdbool.h>
#include "math.h"
#include "lcd.h"


#define VCC  5.01						 // Acs712 supply voltage is from 4.5 to 5.5V. Normally 5V.
#define AMP_MODE 0
#define VOLT_MODE 1
#define SAMPLE_NUM  200				    // Sample element number.




/*  Voltage and current variable. */
struct elec_params{

	float fl_voltage;
	float fl_current;
	float fl_pf;
	float fl_cosfi;


};


enum calc_error{

	CALC_NO_ERROR=0,
	CALC_WRONG_ADC=-1
};


extern struct elec_params g_elec_param_s;
extern float fl_rms_Voltage;
extern float fl_sampleDataArr[80];
extern uint8_t u8_arrCount;

float elecParamCalc_Calibration(float* array);
uint8_t elecParamCalc_Rms(uint8_t mode,float* fl_adcval,struct elec_params* elec_param_s);
uint8_t elecParamCalc_Voltage(float* fl_adcval);
uint8_t elecParamCalc_Current(float* fl_adcval);

#endif /* ELECPARAMCALC_H_ */
