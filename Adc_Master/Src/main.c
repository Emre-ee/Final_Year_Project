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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_NUM  40
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

/* Adc array data counter. */
int i=0;




int8_t calc_voltage(float* fl_adcval,struct elec_params* elec_param_s){

	uint8_t u8_i;

	float fl_sumvoltage=0;
	float fl_sample=0;


	for(u8_i=0;u8_i<SAMPLE_NUM;u8_i++){


	/*Devrenin matemiksel cozumu ile gerilim buyutuyoruz.*/
	 //220 ohm direnc icin
	//fl_sample=((fl_adcval[u8_i]-2.02245)/0.0023604);

	//440 ohm direnc icin
	fl_sample=((fl_adcval[u8_i]-2.0215)/0.0028258);

	/* Buyuttugumuz gerilimin karesini aliyoruz.*/
	fl_sample=fl_sample*fl_sample;

	/* Elde ettigimiz gerilimleri topluyoruz.*/
	fl_sumvoltage=fl_sumvoltage+fl_sample;

	}

	/* Topladigimiz gerilimlerin karekonunu alip Rms dgerini hesapliyoruz.*/
	elec_param_s->fl_voltage=sqrt(fl_sumvoltage/(float)SAMPLE_NUM);




    /*function is returned successfully*/
   return 0;
}


/*  Lcd data array. */
char str[20];
char buf[20];
uint16_t ADC_DATA1;
uint32_t ADC_DATA2;
float channel_voltage[200];
float data=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* Timer Interrupt test led. */
	HAL_GPIO_TogglePin(GPIOD,LED_BLUE_Pin);

	/* ADC Start. */
	HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start(&hadc2);
	
	/* ADC Get Value.  */
	HAL_ADC_PollForConversion(&hadc1,1);
	ADC_DATA1=HAL_ADC_GetValue(&hadc1);



	//HAL_ADC_PollForConversion(&hadc2,1);
	//ADC_DATA2=HAL_ADC_GetValue(&hadc2);

	/*  ADC is stopped. */
	HAL_ADC_Stop(&hadc1);
	//HAL_ADC_Stop(&hadc2);
	
	/* Adc value convert to voltage. */
	channel_voltage[i]=(ADC_DATA1*(float)2.94)/4095;




	/* Adc data is get array in 20ms. */
		if(i<SAMPLE_NUM )
		i++;
		else
		{
			HAL_TIM_Base_Stop_IT(&htim6);
			i=0;
			calc_voltage(channel_voltage,&g_elec_param_s);
		}




}

int a=0,eleman=15,indis;
float toplam=0,dizi[40];
float sonuc;
float hata[]={55.23,55.23,55.23,55.23,55.23,55.10,55.10,55.10,55.10,55.10,55.10,55.10,60.85,60.85,60.85,63.3,63.3,63.3,63.3,63.3,63.3
,64.15,64.15,64.15,64.15,64.15,64.15,64.15,62.99,65.9,67.45,67.45,68.15,68.15,68.15,68.15,68.43,69.28,69.28,69.16,69.16,
70.16,70.64,70.64,72.16,72.16,72.16,72.89,72.89,73.59,73.59,73.59,73.33,73.33,73.33,75.073,75.073,75.073,76.49,76.49,76.86,76.86,
77.22,77.22,77.22,77.22,76.69,79.33,79.26,79.26,79.56,79.71,79.71,81.12,81.12,81.12,81.12,81.12,80.41,84.06,84.06,84.06,84.06,
84.06,84.06,82.24,82.24,82.24,82.11,82.32,84.395,84.395,85.035,85.035,85.45,85.45,87.38,87.15,87.75,88.70,89.88,89.88,89.83,90.42,
90.70,91.18,91.18,92.35,92.35,93.05,93.05,93.30,93.69,93.92,93.87,93.87,94.02,95.14,95.26,95.94,96.63,97.35,97.63,98.08,98.96,
99.22,99.22,100.26,100.26,101.52,101.9,102.77,102.76,104.23,105.45,106.67};
float Error_Function(float* array)
{
	toplam=0;
	int i=0,j=0,k=0;

	/* ayni olan elemanlari diziden cikartiyoruz. */
	   	for (i = 0; i < eleman; i++)
		   {

	      for (j = i + 1; j < eleman;)
		  {
	         if (dizi[j] == dizi[i])
			 {
	            for (k = j; k < eleman; k++)
				{
	            	(dizi[k]) = dizi[k + 1];
	            }
	            eleman--;
	         }

			 else
	        j++;
	      }
	   }

		/* Burada dizinin ortalamasini aliyoruz. */
			for( i=0;i<eleman;i++)
		{
			toplam=toplam+dizi[i];

		}
			sonuc=toplam/(float)eleman;

		if((sonuc<67))
		sonuc=(sonuc*(float)52.99)/(float)100;

		else if((sonuc>=203)&&((sonuc<250)) )
		sonuc=(sonuc*(float)107.04)/(float)100;
		else
		{
			indis=sonuc-67;
			sonuc=(hata[indis]*sonuc)/100;

		}

return sonuc;
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




	 		  	calc_voltage(channel_voltage,&g_elec_param_s);
	 		 	if(a<40)
	 		 	{
	 		 		dizi[a]=g_elec_param_s.fl_voltage;
	 		 		a++;
	 		 		HAL_TIM_Base_Start_IT(&htim6);
	 		 	}
	 		 	else
	 		 	{
	 		 		data=Error_Function(dizi);
	 		 		sprintf(buf,"AC=%.6f",data );
	 		 		LCD_OutString(buf,1);
	 		 		a=0;
	 		 		HAL_Delay(20);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
