/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint64_t _micro = 0;
const float dt = 0.001;
float32_t F_data[9] = {1,dt,0.5*dt*dt,0,1,dt,0,0,1};
arm_matrix_instance_f32 F;
float32_t X_data[3] = {0};
arm_matrix_instance_f32 X;
float32_t X_l_data[3] = {0,0,0};
arm_matrix_instance_f32 X_l;
float32_t P_data[9] = {1,1,1,1,1,1,1,1,1};
arm_matrix_instance_f32 P;
float32_t P_l_data[9] = {1,1,1,1,1,1,1,1,1};
arm_matrix_instance_f32 P_l;
float32_t FP_data[9] = {0};
arm_matrix_instance_f32 FP;
float32_t Ft_data[9] = {1,0,0,dt,1,0,0.5*dt*dt,dt,1};
arm_matrix_instance_f32 Ft;
float32_t FPFt_data[9] = {0};
arm_matrix_instance_f32 FPFt;
const double var = 1000;
float32_t Q_data[9] = {dt*dt*dt*dt*var/4,dt*dt*dt*var/2,dt*dt*var/2,
		dt*dt*dt*var/2,dt*dt*var,dt*var,
		dt*dt*var/2,dt*var,var};
arm_matrix_instance_f32 Q;
float32_t R_data[4] = {0.00001,0,0,1000000};
arm_matrix_instance_f32 R;
float32_t H_data[6] = {1,0,0,0,1,0};
arm_matrix_instance_f32 H;
float32_t Ht_data[6] = {0};
arm_matrix_instance_f32 Ht;
float32_t HP_data[6] = {0};
arm_matrix_instance_f32 HP;
float32_t PHt_data[6] = {0};
arm_matrix_instance_f32 PHt;
float32_t HPHt_data[4] = {0};
arm_matrix_instance_f32 HPHt;
float32_t HPHt_R_data[4] = {0};
arm_matrix_instance_f32 HPHt_R;
float32_t HPHt_R_inv_data[4] = {0};
arm_matrix_instance_f32 HPHt_R_inv;
float32_t K_data[6] = {0};
arm_matrix_instance_f32 K;
float32_t KH_data[9] = {0};
arm_matrix_instance_f32 KH;
float32_t z_data[2] = {0};
arm_matrix_instance_f32 z;
float32_t y_data[2] = {0};
arm_matrix_instance_f32 y;
float32_t z_y_data[2] = {0};
arm_matrix_instance_f32 z_y;
float32_t Kz_y_data[3] = {0};
arm_matrix_instance_f32 Kz_y;
float32_t I_data[9] = {1,0,0,0,1,0,0,0,1};
arm_matrix_instance_f32 I;
float32_t I_KH_data[9] = {0};
arm_matrix_instance_f32 I_KH;
int q[2] = {0};
int step = 0;
int pos[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint64_t Micros();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //init matrix
  arm_mat_init_f32(&F, 3, 3, F_data);
  arm_mat_init_f32(&FP, 3, 3, FP_data);
  arm_mat_init_f32(&Ft, 3, 3, Ft_data);
  arm_mat_init_f32(&FPFt, 3, 3, FPFt_data);
  arm_mat_init_f32(&X, 3, 1, X_data);
  arm_mat_init_f32(&X_l, 3, 1, X_l_data);
  arm_mat_init_f32(&P, 3, 3, P_data);
  arm_mat_init_f32(&P_l, 3, 3, P_l_data);
  arm_mat_init_f32(&Q, 3, 3, Q_data);
  arm_mat_init_f32(&z, 2, 1, z_data);
  arm_mat_init_f32(&y, 2, 1, y_data);
  arm_mat_init_f32(&z_y, 2, 1, z_y_data);
  arm_mat_init_f32(&Kz_y, 3, 1, Kz_y_data);
  arm_mat_init_f32(&R, 2, 2, R_data);
  arm_mat_init_f32(&H, 2, 3, H_data);
  arm_mat_init_f32(&Ht, 3, 2, Ht_data);
  arm_mat_init_f32(&HP, 2, 3, HP_data);
  arm_mat_init_f32(&KH, 3, 3, KH_data);
  arm_mat_init_f32(&PHt, 3, 2, PHt_data);
  arm_mat_init_f32(&HPHt, 2, 2, HPHt_data);
  arm_mat_init_f32(&HPHt_R, 2, 2, HPHt_R_data);
  arm_mat_init_f32(&HPHt_R_inv, 2, 2, HPHt_R_inv_data);
  arm_mat_init_f32(&K, 3, 2, K_data);
  arm_mat_init_f32(&I, 3, 3, I_data);
  arm_mat_init_f32(&I_KH, 3, 3, I_KH_data);
  arm_mat_trans_f32(&H, &Ht);
  //
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT (&htim11);
  	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  	q[0]=TIM2->CNT;
  	q[1]=q[0];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static int timeStamp = 0;
	  		if (Micros() - timeStamp > 1000) {
	  			timeStamp = Micros();
	  			q[0] = TIM2->CNT;
	  			if(q[0]-q[1]<-1000){
	  				step+=3071;
	  			}
	  			else if(q[0]-q[1]>=1000){
	  				step-=3071;
	  			}
	  			pos[0] = q[0] + step;
	  			z_data[0]=pos[0];
	  			z_data[1]=(pos[0]-pos[1])/2;
	  			//predict
	  			arm_mat_mult_f32(&F, &X_l, &X);
	  			arm_mat_mult_f32(&F, &P_l, &FP);
	  			arm_mat_mult_f32(&FP, &Ft, &FPFt);
	  			arm_mat_add_f32(&FPFt, &Q, &P);
	  			//correct
	  			arm_mat_mult_f32(&P, &Ht, &PHt);
	  			arm_mat_mult_f32(&H, &P, &HP);
	  			arm_mat_mult_f32(&HP, &Ht, &HPHt);
	  			arm_mat_add_f32(&HPHt, &R, &HPHt_R);
	  			arm_mat_inverse_f32(&HPHt_R, &HPHt_R_inv);
	  			arm_mat_mult_f32(&PHt, &HPHt_R_inv, &K);
	  			arm_mat_mult_f32(&H, &X, &y);
	  			arm_mat_sub_f32(&z, &y, &z_y);
	  			arm_mat_mult_f32(&K, &z_y, &Kz_y);
	  			arm_mat_add_f32(&X, &Kz_y, &X_l);
	  			arm_mat_sub_f32(&I, &KH, &I_KH);
	  			arm_mat_mult_f32(&I_KH, &P, &P_l);
	  			q[1] = q[0];
	  			pos[1] = pos[0];
	  		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Period = 7199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		_micro += 65535;
	}
}

uint64_t Micros(){
	return _micro + TIM11->CNT;
}
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

