/*
 * elecParamCalc.c
 *
 *  Created on: 30 May 2020
 *      Author: Mehmet Emre
 */

#include "elecParamCalc.h"

/*Global variables*/
struct elec_params g_elec_param_s;


float fl_rms_Voltage=0,fl_temp,fl_avg,fl_sum=0,fl_acVolt=0;
float fl_avgArr[30],fl_sampleDataArr[80];
uint8_t u8_avgCount=0, u8_arrCount=0,u8_medCount=0;
uint8_t u8_sampleDataNum=10, u8_medVal=0,u8_lineCount=0,u8_columnCount=0,u8_i=0;
bool Check_OK=0;



int model = 2;   							// enter the model number (see below)

const float sensitivity[] ={
          0.185,// for ACS712ELCTR-05B-T --	0
          0.100,// for ACS712ELCTR-20A-T --	1
          0.066// for ACS712ELCTR-30A-T	 --	2

         };


const float QOV =   0.5 * VCC;				// set quiescent Output voltage of 0.5V


uint8_t calc_Rms(uint8_t mode,float* fl_adcval,struct elec_params* elec_param_s){

	/* Adc adc_value_data counter */
	uint8_t u8_i;

	/* Voltage Variables */
	float fl_sum=0;
	float fl_sample=0;


	for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++){

	/* We take the square of the voltage.*/
	fl_sample=fl_adcval[u8_i]*fl_adcval[u8_i];

	/* We sum the voltages we obtain.*/
	fl_sum=fl_sum+fl_sample;
	}

	if(mode==VOLT_MODE)
	/* We take the square root of voltages and we find the Rms  value.*/
	elec_param_s->fl_voltage=(sqrt(fl_sum/(float)SAMPLE_NUM));

	if(mode==AMP_MODE)
	{
		/* We take the square root of voltages and we find the Rms  value.*/

		    elec_param_s->fl_current=(sqrt(fl_sum/(float)SAMPLE_NUM));

		   	if((elec_param_s->fl_current>0.26)&&(elec_param_s->fl_current<30))
			elec_param_s->fl_current=(sqrt(fl_sum/(float)SAMPLE_NUM))-0.28;

		   	else
		   	elec_param_s->fl_current=0;


	}


    /*function is returned successfully*/
   return CALC_NO_ERROR;;
}



uint8_t calc_Current(float* fl_adcval)
{

	/*  Current Rms Operations */

		uint8_t u8_i;
		float fl_current[SAMPLE_NUM];
		for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++)
		{

			fl_current[u8_i] =  ((fl_adcval[u8_i]*1.67)-QOV);
			fl_current[u8_i] = fl_current[u8_i] / sensitivity[2];

		}
			calc_Rms(AMP_MODE,fl_current,&g_elec_param_s);

			/*function is returned successfully*/
			return CALC_NO_ERROR;;

}



 uint8_t calc_Voltage(float* fl_adcval)
{
	/* Adc adc_value_data counter */
	uint8_t u8_i;
	float fl_volt[SAMPLE_NUM];

	for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++)
	{

		fl_volt[u8_i]=(((fl_adcval[u8_i]-2.0215))/0.0028258);

	}
	calc_Rms(VOLT_MODE,fl_volt,&g_elec_param_s);

	/*function is returned successfully*/
	return CALC_NO_ERROR;;
}




float Calibration_Function(float* array)
{
	u8_columnCount=0,u8_lineCount=0;
	u8_medCount=0;
	fl_sum=0;
	Check_OK=0;
		/* We take average of the  sample array in while loop. */
		for(int i=0;i<u8_sampleDataNum;i++)
		{
			fl_sum=fl_sum+array[i];

		}
		fl_avg=fl_sum/(float)u8_sampleDataNum;



		/* We write the average values into the array to calculate the median. */
		if(u8_avgCount<10)
		{

			fl_avgArr[u8_avgCount]=fl_avg;
			u8_avgCount++;
		}

		else
		{
			u8_avgCount=0;

			for(u8_i=0;u8_i<(10-1);u8_i++)
			{
				for(int j=0;j<(10-1);j++)
				{
					if(fl_avgArr[j]>fl_avgArr[j+1])
					{
						fl_temp=fl_avgArr[j];
						fl_avgArr[j]=fl_avgArr[j+1];
						fl_avgArr[j+1]=fl_temp;
					}
				}
			}

			u8_medVal=fl_avgArr[5];
		}

		/* We have to ready. We will calculate Ac_Voltage. */
		if(u8_avgCount==0)
		{



			/*I used if for values of 81 and below and values from 216 to 230.*/
			if((u8_medVal<81))
			fl_acVolt=(u8_medVal*(float)43.04)/(float)100;

			if((u8_medVal>216)&&(u8_medVal<230))
				fl_acVolt=(u8_medVal*(float)113.49)/(float)100;

			while((u8_lineCount<66)&&(Check_OK==0))
			{
				u8_columnCount=0;

				/* I calculate element number of line in array .*/
				while (Error_Array[u8_lineCount][u8_columnCount+1] != 0)
				{
					u8_columnCount++;
				}

				/* I am trying to find the error factor by comparing the median value with the numbers in the array.*/
				for(u8_medCount=0;u8_medCount<u8_columnCount;)
				{
				    /* After finding the range of numbers in the array with the median value, I multiply by the error factor in index 0. */
					if(u8_medVal==Error_Array[u8_lineCount][u8_medCount+1])
					{
						fl_acVolt=(u8_medVal*Error_Array[u8_lineCount][0])/(float)100;
						Check_OK=1;
						break;
					}


					else
						u8_medCount++;

				}

				/* As soon as I find the error multiplier, I get out of the while loop. */
				if(Check_OK==1)
				{
					u8_lineCount=0;
					break;
				}
				/* If I can't find the value, I increase j, the line counter. */
				else
					u8_lineCount++;

			}


		}

		return fl_acVolt;
}

