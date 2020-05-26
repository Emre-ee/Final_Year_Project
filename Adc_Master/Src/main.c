/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "lcd.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_NUM  120
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





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
					{97.50,200},
					{98.02,201,202,203},
					{98.93,204},
					{100.29,205},
					{102.03,206},
					{102.27,207},
					{102.52,208},
					{104.84,209,210},
					{107.49,211},
					{109.49,212},
					{110.49,213},
					{111.49,214},
					{112.49,215},
					{113.49,216},
};




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

float Error_Function(float* array);







int8_t calc_voltage(float* fl_adcval,struct elec_params* elec_param_s){

	uint8_t u8_i;

	float fl_sumvoltage=0;
	float fl_sample=0;


	for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++){


	/*We increase the voltage with the mathematical solution of the circuit.*/
	fl_sample=(((fl_adcval[u8_i]-2.0215))/0.0028258);

	/* We take the square of the voltage.*/
	fl_sample=fl_sample*fl_sample;

	/* We sum the voltages we obtain.*/
	fl_sumvoltage=fl_sumvoltage+fl_sample;

	}

	/* We take the square root of voltages and we find the Rms  value.*/
	elec_param_s->fl_voltage=(sqrt(fl_sumvoltage/(float)SAMPLE_NUM));




    /*function is returned successfully*/
   return 0;
}


/*  Lcd data array. */
char str[20];
char buf[20];

/* Adc array data counter. */
int i=0;

/* Adc data array and adc value. */
uint16_t ADC_DATA1;
float channel_voltage[200];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{



	/* ADC Start. */
	HAL_ADC_Start(&hadc1);
	
	/* ADC Get Value.  */
	HAL_ADC_PollForConversion(&hadc1,1);
	ADC_DATA1=HAL_ADC_GetValue(&hadc1);

	/*  ADC is stopped. */
	HAL_ADC_Stop(&hadc1);
	
	/* Adc value convert to voltage. */
	channel_voltage[i]=(ADC_DATA1*(float)3)/4095;




	/* Adc data is get array in SAMPLE_NUM. */
		if(i<SAMPLE_NUM )
		i++;

		else
		{
			i=0;
			HAL_TIM_Base_Stop_IT(&htim6);
			calc_voltage(channel_voltage,&g_elec_param_s);		// Go to calc_voltage function and calculate rms.
		}




}



float Rms_Voltage=0,temp,Average,Sum=0,Ac_Voltage_Value=0;
float Average_Array[30],Sample_Data_Array[80];
int Average_Counter=0,Array_Counter=0,count=0;
int Sample_Data_Number=10,Median_Value=0,Line_Counter=0,Column_Counter;

float Error_Function(float* array)
{
	Column_Counter=0,Line_Counter=0;
	count=0;
	Sum=0;
	bool Check_OK=0;

		/* We take average of the  sample array in while loop. */
		for(int i=0;i<Sample_Data_Number;i++)
		{
			Sum=Sum+array[i];

		}
		Average=Sum/(float)Sample_Data_Number;



		/* We write the average values into the array to calculate the median. */
		if(Average_Counter<10)
		{

		 Average_Array[Average_Counter]=Average;
		 Average_Counter++;
		}

		else
		{
			Average_Counter=0;

			for(i=0;i<(10-1);i++)
			{
				for(int j=0;j<(10-1);j++)
				{
					if(Average_Array[j]>Average_Array[j+1])
					{
						temp=Average_Array[j];
						Average_Array[j]=Average_Array[j+1];
						Average_Array[j+1]=temp;
					}
				}
			}

			Median_Value=Average_Array[5];
		}

		/* We have to ready. We will calculate Ac_Voltage. */
		if(Average_Counter==0)
		{



			/*I used if for values of 81 and below and values from 216 to 230.*/
			if((Median_Value<81))
			Ac_Voltage_Value=(Median_Value*(float)43.04)/(float)100;

			if((Median_Value>216)&&(Median_Value<230))
			Ac_Voltage_Value=(Median_Value*(float)111.49)/(float)100;

			while((Line_Counter<66)&&(Check_OK==0))
			{
				Column_Counter=0;

				/* I calculate element number of line in array .*/
				while (Error_Array[Line_Counter][Column_Counter+1] != 0)
				{
					Column_Counter++;
				}

				/* I am trying to find the error factor by comparing the median value with the numbers in the array.*/
				for(count=0;count<Column_Counter;)
				{
				    /* After finding the range of numbers in the array with the median value, I multiply by the error factor in index 0. */
					if(Median_Value==Error_Array[Line_Counter][count+1])
					{
						Ac_Voltage_Value=(Median_Value*Error_Array[Line_Counter][0])/(float)100;
						Check_OK=1;
						break;
					}


					else
					count++;

				}

				/* As soon as I find the error multiplier, I get out of the while loop. */
				if(Check_OK==1)
				{
					Line_Counter=0;
					break;
				}
				/* If I can't find the value, I increase j, the line counter. */
				else
					Line_Counter++;

			}


		}

		return Ac_Voltage_Value;
}




int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  /*LCD Port Init */
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOEEN);
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN);

	GPIOE->MODER =GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0;
	GPIOB->MODER =GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0|GPIO_MODER_MODER2_0|GPIO_MODER_MODER3_0;
	
	/* Lcd Init */
	LCD_Init();
	LCD_Clear();

	/* Timer6 Start */
	HAL_TIM_Base_Start_IT(&htim6);


  /* USER CODE END 2 */

  while (1)
  {
  	  	  	  //Calibration if-else.
	 			if(Array_Counter<10)
	 		 	{

	 		 		Sample_Data_Array[Array_Counter]=g_elec_param_s.fl_voltage;
	 		 		Array_Counter++;
	 		 		HAL_TIM_Base_Start_IT(&htim6);
	 		 	}
	 		 	else
	 		 	{
	 		 		Rms_Voltage=Error_Function(Sample_Data_Array);
					sprintf(buf,"AC=%.3f",Rms_Voltage );
					LCD_OutString(buf,2);
					LCD_OutString("Rms Voltage",1);
					Array_Counter=0;
				}





}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
