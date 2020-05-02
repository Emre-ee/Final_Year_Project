#include<stdio.h>
#include<math.h>
#include <stdint.h>


#define SAMPLE_NUM  20

struct elec_params{
	
	float fl_voltage;
	float fl_current;
	float fl_pf;
	float fl_cosfi;
	

};


/*Global variables*/
struct elec_params g_elec_param_s;

/*Voltage calculation function*/
int8_t calc_voltage(float* fl_adcval,struct elec_params* elec_param_s);

int main()
{
FILE *file;

//File Operations

float time[201],AcVolt[201], output[201];

int i=0;
if((file=fopen("Output_AcVoltmeter.txt","r"))!=NULL)	
{	
	while(!feof(file))
	{
		fscanf(file,"%f %f %f",&time[i],&AcVolt[i],&output[i]);
	
		i++;
	}
}
fclose(file);


// Output simulation parameter used. 
float volt=0,sum=0,Ac_Voltage=0;
int a,b;
printf("Please start time enter value:\n");   scanf("%d",&a);



/*  Time from start to finish is listed.  */

if(a>0)
a=a-1;

b=a+20;

while(a<b)
{

	
	printf("%.5f %.5f %.5f\n",time[a],AcVolt[a],output[a]);
	
	a++;
 }

	calc_voltage(output,&g_elec_param_s);
	printf("\nRms= %.2fV\n",g_elec_param_s.fl_voltage);
	


}






int8_t calc_voltage(float* fl_adcval,struct elec_params* elec_param_s){
	
	uint8_t u8_i;
	float fl_sumvoltage=0;
	float fl_sample=0;
	
	for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++){
	
	
	/*Devrenin matemiksel çözümü ile gerilim büyütüyoruz.*/
	fl_sample=((fl_adcval[u8_i]-2.02245)/0.0023604);						
	
	/* Büyüttüðümüz gerilimin karesini alýyoruz.*/
	fl_sample=fl_sample*fl_sample;				
	
	/* Elde elttiðimiz gerilimleri topluyoruz.*/						
	fl_sumvoltage=fl_sumvoltage+fl_sample;												
		
	}
	
	/*Topladýðýmýz gerilimlerin karekökünü alýp Rms deðerini hesaplýyoruz.*/
	elec_param_s->fl_voltage=sqrt(fl_sumvoltage/(float)SAMPLE_NUM);								


    /*function is returned successfully*/
   return 0;	
}

