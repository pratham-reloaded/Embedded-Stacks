/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "mdds30.h"
#include "DC_MOTOR_cfg.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


float LPF_beta = 0.025;
int sample_time = 20;

// -- ignore on release
int linear_x_rec = 0;
int angular_z_rec = 0;

float linear_x = 0;
float angular_z = 0;



float linear_x_now = 0;
float angular_z_now = 0;

// Left Motor

PID_TypeDef left_SPID;
int left_Kp = 0;
int left_Ki = 0;
int left_Kd = 0;

double left_SpeedCurrent, left_PIDOut, left_SpeedSetpoint;

int16_t left_pulses = 0;
int8_t left_ppms = 0;

float left_sppms = 0;

int GetSpeedLeft(){ return left_sppms; }

// Right Motor

PID_TypeDef right_SPID;
int right_Kp = 0;
int right_Ki = 0;
int right_Kd = 0;

double right_SpeedCurrent, right_PIDOut, right_SpeedSetpoint;

int16_t right_pulses = 0;
int8_t right_ppms = 0;

float right_sppms = 0;


int GetSpeedRight(){ return right_sppms; }


// Encoder Callback

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	left_pulses = -(int16_t)__HAL_TIM_GET_COUNTER(&htim2)/4;
	right_pulses = -(int16_t)__HAL_TIM_GET_COUNTER(&htim3)/4;


}

// Wheel Velocity
void setWheelVelocity(float lnr_x, float ang_z, float r, double *left_vel, double *right_vel){

	// r is in m



	*left_vel = (lnr_x - (r*ang_z))*1.1542*sample_time/100;
	*right_vel = (lnr_x + (r*ang_z))*1.1542*sample_time/100;

}






 uint8_t sen_buff[8];
 uint8_t rec_buff[100];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Left Motor

  DC_MOTOR_Init(DC_MOTOR_CfgParam[0]);

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

  PID(&left_SPID, &left_SpeedCurrent, &left_PIDOut, &left_SpeedSetpoint, left_Kp, left_Ki, left_Kd, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&left_SPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&left_SPID, 500);
  PID_SetOutputLimits(&left_SPID, -65534, 65534);

  // Right Motor

  DC_MOTOR_Init(DC_MOTOR_CfgParam[1]);

  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

  PID(&right_SPID, &right_SpeedCurrent, &right_PIDOut, &right_SpeedSetpoint, right_Kp, right_Ki, right_Kd, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&right_SPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&right_SPID, sample_time);
  PID_SetOutputLimits(&right_SPID, -65534, 65534);

  //USB Init

  vcp_init();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Converting from into to float

	  linear_x = linear_x_rec/100;
	  angular_z = angular_z_rec/100;

	  // --ignore in final

	  if ( (left_Kp != PID_GetKp(&left_SPID)) | (left_Ki != PID_GetKi(&left_SPID)) | (left_Kd != PID_GetKd(&left_SPID))){
	  	 	PID_SetTunings(&left_SPID, left_Kp, left_Ki, left_Kd);
	  	 }
	  if ( (right_Kp != PID_GetKp(&right_SPID)) | (right_Ki != PID_GetKi(&right_SPID)) | (right_Kd != PID_GetKd(&right_SPID))){
	  	 	PID_SetTunings(&right_SPID, right_Kp, right_Ki, right_Kd);
	  	 }

	  // Low Pass Filter

	  left_sppms = left_sppms - (LPF_beta *(left_sppms-left_ppms));
	  right_sppms = right_sppms - (LPF_beta *(right_sppms-right_ppms));

	  // Get Wheel Velocities

	  setWheelVelocity(linear_x, angular_z, 0.355, &left_SpeedSetpoint, &right_SpeedSetpoint);

	  // PID

	  left_SpeedCurrent = GetSpeedLeft();
	  right_SpeedCurrent = GetSpeedRight();

	  PID_Compute(&left_SPID);
	  PID_Compute(&right_SPID);

	  // Current speed

	  linear_x_now = (right_sppms + left_sppms)*(0.1651/2)*(2*3.14/1196);
	  angular_z_now = (right_sppms - left_sppms)*(0.1651/0.355)*(2*3.14/1196);

	  // Motors

	  DC_MOTOR_Start( DC_MOTOR_CfgParam[0],
			  	  	  left_PIDOut < 0 ? DIR_ACW : DIR_CW,
					  left_PIDOut < 0 ? -left_PIDOut : left_PIDOut);

	  DC_MOTOR_Start( DC_MOTOR_CfgParam[1],
			  	  	  right_PIDOut < 0 ? DIR_ACW : DIR_CW,
			  	  	  right_PIDOut < 0 ? -right_PIDOut : right_PIDOut);

if(right_ppms==0 && left_ppms ==0){
			char packet[5];
		   sprintf(packet,"%d,%d\n",left_ppms,right_ppms);

		    sen_buff[0] = (uint8_t)0;
		    sen_buff[1] = (uint8_t)0;
		    sen_buff[2] = (uint8_t)0;
			sen_buff[3] = (uint8_t) packet[0];
			sen_buff[4] = (uint8_t) packet[1];
			sen_buff[5] = (uint8_t) packet[2];
			sen_buff[6] = (uint8_t) packet[3];
			sen_buff[7] = (uint8_t) packet[4];
			sen_buff[8] = (uint8_t) packet[5];
}
else if((right_ppms>=0 && left_ppms>0) || (right_ppms>0 && left_ppms>=0) ){
	  char packet[6];
	  sprintf(packet,"%d,%d\n",left_ppms,right_ppms);

	    sen_buff[0] = (uint8_t)0;
	    sen_buff[1] = (uint8_t)0;
	    sen_buff[2] = (uint8_t)packet[0];
		sen_buff[3] = (uint8_t) packet[1];
		sen_buff[4] = (uint8_t) packet[2];
		sen_buff[5] = (uint8_t) packet[3];
		sen_buff[6] = (uint8_t) packet[4];
		sen_buff[7] = (uint8_t) packet[5];
		sen_buff[8] = (uint8_t) packet[6];


}

else if(right_ppms<0 && left_ppms<0){
	  char packet[8];
	  sprintf(packet,"%d,%d\n",left_ppms,right_ppms);


		sen_buff[0] = (uint8_t) packet[0];
		sen_buff[1] = (uint8_t) packet[1];
		sen_buff[2] = (uint8_t) packet[2];
		sen_buff[3] = (uint8_t) packet[3];
		sen_buff[4] = (uint8_t) packet[4];
		sen_buff[5] = (uint8_t) packet[5];
		sen_buff[6] = (uint8_t) packet[6];
		sen_buff[7] = (uint8_t) packet[7];
		sen_buff[8] = (uint8_t) packet[8];

}

else {
	 char packet[7];
	 sprintf(packet,"%d,%d\n",left_ppms,right_ppms);


			sen_buff[0] = (uint8_t) 0;
			sen_buff[1] = (uint8_t) packet[0];
			sen_buff[2] = (uint8_t) packet[1];
			sen_buff[3] = (uint8_t) packet[2];
			sen_buff[4] = (uint8_t) packet[3];
			sen_buff[5] = (uint8_t) packet[4];
			sen_buff[6] = (uint8_t) packet[5];
			sen_buff[7] = (uint8_t) packet[6];
			sen_buff[8] = (uint8_t) packet[7];


}



      char rec_buff[100];
	  sscanf(rec_buff,"%d,%d\n",&linear_x_rec,&angular_z_rec);




	  vcp_recv((uint8_t *)&rec_buff,sizeof(rec_buff));

	  vcp_send(
			  (uint8_t *)&sen_buff,
			  sizeof(sen_buff)
			  );


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
