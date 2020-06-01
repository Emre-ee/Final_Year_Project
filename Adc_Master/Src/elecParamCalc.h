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
#define VCC  5.01						// Acs712 supply voltage is from 4.5 to 5.5V. Normally 5V.
#define AMP_MODE 0
#define VOLT_MODE 1
#define SAMPLE_NUM  200				   // Sample element number.

/* In the array I created below, the element in index 0 shows the error multiplier.
 * If the adjacent elements, for example, 81.82.83.84.85 values come in, multiply by the error factor in index 0 and
 * calculate the correct value.
 *
 *  */
float Error_Array[66][8]={
							{49.12,81,82,83,84,85},
							{50.28,86,87,88,89,90,91},
							{51.97,92,93},
							{53.19,94,95,96,97},
							{53.81,98,99},
							{57.29,100,101,102},
							{57.81,103,104},
							{59.75,105,106,107,108},
							{60.43,109},
							{61.82,110,111,112},
							{61.95,113},
							{63.16,114,115,116,117},
							{64.30,118,119,120},
							{65.40,121,122,123},
							{66.18,124,125,126},
							{66.85,127,128},
							{68.04,129,130},
							{68.51,131},
							{70.24,132,133,134},
							{70.52,135,136},
							{71.61,137,138},
							{72.20,139,140,141,142},
							{72.98,143,144,145},
							{73.9,146,147},
							{75.68,148},
							{76.51,149,150},
							{76.75,151,152},
							{77.84,153},
							{78.52,154,155},
							{79.41,156,157},
							{79.53,158,159},
							{80.42,160},
							{81.37,161,162,163},
							{81.31,164,165},
							{81.85,166,167},
							{82.95,168,169},
							{83.65,170,171},
							{84.11,172,173},
							{85.30,174,175,176},
							{86.12,177,178},
							{86.66,179,180,181,182},
							{86.98,183},
							{88.72,184},
							{89.19,185,186},
							{89.84,187},
							{90.43,188,189},
							{90.53,190},
							{91.46,191,192},
							{92.785,193,194,195},
							{95.09,196,197},
							{95.67,198},
							{96.47,199},
							{99.50,200},		//2.0
							{100.02,201,202,203},
							{100.93,204},
							{104.29,205},    // +4.0
							{106.03,206},
							{106.27,207},
							{106.52,208},
							{108.84,209,210},
							{111.49,211},
							{113.49,212},
							{114.49,213},
							{115.49,214},
							{116.49,215},
							{117.49,216},
};

/* Rms voltage and current variable */
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


float Calibration_Function(float* array);
uint8_t calc_Rms(uint8_t mode,float* fl_adcval,struct elec_params* elec_param_s);
uint8_t calc_Voltage(float* fl_adcval);
uint8_t calc_Current(float* fl_adcval);

#endif /* ELECPARAMCALC_H_ */
