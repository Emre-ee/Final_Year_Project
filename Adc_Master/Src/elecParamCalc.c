/*
 * elecParamCalc.c
 *
 *  Created on: 30 May 2020
 *      Author: Mehmet Emre
 */

#include "elecParamCalc.h"


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
							{99.50,200},
							{100.02,201,202,203},
							{100.93,204},
							{104.29,205},
							{106.03,206},
							{106.27,207},
							{106.52,208},
							{108.84,209,210},
							{111.49,211},
							{113.49,212},
							{114.49,213},
							{115.49,214},
							{116.49,215},
							{115.49,216},
};



/*Global variables*/
struct elec_params g_elec_param_s;

/* Current sensor parameters. */
int model = 2;   							// enter the model number (see below)

const float sensitivity[] ={
          0.185,				 // for ACS712ELCTR-05B-T --	0
          0.100,				 // for ACS712ELCTR-20A-T --	1
          0.066					// for ACS712ELCTR-30A-T	 --	2

         };


const float QOV =   0.5 * VCC;				// set quiescent Output voltage of 0.5V


/* Calibration Variables */
float fl_rms_Voltage=0,fl_temp,fl_avg,fl_sum=0,fl_acVolt=0;
float fl_avgArr[30],fl_sampleDataArr[80];
uint8_t u8_avgCount=0, u8_arrCount=0,u8_medCount=0;
uint8_t u8_sampleDataNum=10, u8_medVal=0,u8_lineCount=0,u8_columnCount=0,u8_i=0;
bool Check_OK=0;



uint8_t elecParamCalc_Rms(uint8_t mode,float* fl_adcval,struct elec_params* elec_param_s){

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
   return CALC_NO_ERROR;
}



uint8_t elecParamCalc_Current(float* fl_adcval)
{

	/*  Current Rms Operations */

		uint8_t u8_i;
		float fl_current[SAMPLE_NUM];
		for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++)
		{
			/* Voltage divider rate= 16.7 k /10 k= 1.67.
			 * QOV ofset value 2.51 V.
			 * Sensitivity 0.066 mV/A
			 * */
			fl_current[u8_i] =  ((fl_adcval[u8_i]*1.67)-QOV);
			fl_current[u8_i] = fl_current[u8_i] / sensitivity[2];

		}
		elecParamCalc_Rms(AMP_MODE,fl_current,&g_elec_param_s);

			/*function is returned successfully*/
			return CALC_NO_ERROR;

}



 uint8_t elecParamCalc_Voltage(float* fl_adcval)
{
	/* Adc adc_value_data counter */
	uint8_t u8_i;
	float fl_volt[SAMPLE_NUM];

	for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++)
	{
		/* We calculate the voltage value with the mathematical solution. */
		fl_volt[u8_i]=(((fl_adcval[u8_i]-2.0215))/0.0028258);

	}
	elecParamCalc_Rms(VOLT_MODE,fl_volt,&g_elec_param_s);

	/*function is returned successfully*/
	return CALC_NO_ERROR;
}




float elecParamCalc_Calibration(float* array)
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

