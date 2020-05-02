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
FILE *dosya;

//Dosya Islemleri

float zaman[201],AcVolt[201], output[201];

int i=0;
if((dosya=fopen("Output_AcVoltmeter.txt","r"))!=NULL)	
{	
	while(!feof(dosya))
	{
		fscanf(dosya,"%f %f %f",&zaman[i],&AcVolt[i],&output[i]);
	//	printf("%.9f %.9f %.9f\n",zaman[i],AcVolt[i],output[i]);
		i++;
	}
}
fclose(dosya);


//Degerleri Rms hesabý icin kullanma
float volt=0,sum=0,Ac_Voltage=0;
int a,b,j=0,k=0;
basa:printf("Lutfen istediginiz baslagic zamanini girin:\n");   scanf("%d",&a);
sum=0;


if(a>0)
a=a-1;

b=a+20;

while(a<b)
{

	
	printf("%.5f %.5f %.5f\n",zaman[a],AcVolt[a],output[a]);
	
	a++;
 }

	calc_voltage(output,&g_elec_param_s);
	printf("\nRms= %.2fV\n",g_elec_param_s.fl_voltage);
	
	goto basa;


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

