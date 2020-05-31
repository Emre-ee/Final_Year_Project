/*
 * elecParamCalc.h
 *
 *  Created on: 30 May 2020
 *      Author: Mehmet Emre
 */

#ifndef ELECPARAMCALC_H_
#define ELECPARAMCALC_H_


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

#endif /* ELECPARAMCALC_H_ */
