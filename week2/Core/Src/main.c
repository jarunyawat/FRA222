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
#include<string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t keypad = 0;
uint8_t numCode = 0;
char inputCode[] = "0";
char code[32];
char correctCode[] = "63340500005";
uint8_t save = 0;
uint8_t lock = 0;
uint8_t ok = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */

	void MatrixKeypadRead();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		static uint8_t buttonState = 0;
		MatrixKeypadRead();
		static enum {
			lock, unlock
		} STATE = unlock;
		if (keypad > 0) {
			if (buttonState == 0) {
				if (keypad == (1 << 0)) {
					inputCode[0] = '7';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 1)) {
					inputCode[0] = '8';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 2)) {
					inputCode[0] = '9';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 4)) {
					inputCode[0] = '4';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 5)) {
					inputCode[0] = '5';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 6)) {
					inputCode[0] = '6';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 8)) {
					inputCode[0] = '1';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 9)) {
					inputCode[0] = '2';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 10)) {
					inputCode[0] = '3';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 12)) {
					inputCode[0] = '0';
					strcat(code, inputCode);
					numCode++;
				} else if (keypad == (1 << 3)) {
					for (int i = 0; i < numCode + 1; i++) {
						code[i] = '\0';
					}
					numCode = 0;
				} else if (keypad == (1 << 7)) {
					if (numCode > 0) {
						numCode--;
					}
					code[numCode] = '\0';
				} else if (keypad == (1 << 15)) {
					ok = 1;
				}
				buttonState = 1;
			}
		} else {
			buttonState = 0;
		}
		switch (STATE) {
		case unlock:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			if (ok) {
				if (save && numCode >= 4) {
					if (strcmp(correctCode, code) == 0) {
						STATE = lock;
					} else {
						for (int i = 0; i < 5; i++) {
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
							HAL_Delay(100);
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
									GPIO_PIN_RESET);
							HAL_Delay(100);
						}
					}

				} else if (save == 0) {
					if (numCode >= 4) {
						save = 1;
						strcpy(correctCode, code);
						STATE = lock;
					} else {
						for (int i = 0; i < 5; i++) {
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
							HAL_Delay(100);
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
									GPIO_PIN_RESET);
							HAL_Delay(100);
						}
					}
				} else {
					for (int i = 0; i < 5; i++) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
						HAL_Delay(100);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
						HAL_Delay(100);
					}
				}
				for (int i = 0; i < numCode + 1; i++) {
					code[i] = '\0';
				}
				numCode = 0;
			}
			break;
		case lock:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			if (ok) {
				if (numCode >= 4) {
					if (strcmp(correctCode, code) == 0) {
						STATE = unlock;
					} else {
						for (int i = 0; i < 5; i++) {
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
							HAL_Delay(100);
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
									GPIO_PIN_RESET);
							HAL_Delay(100);
						}
					}
				} else {
					for (int i = 0; i < 5; i++) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
						HAL_Delay(100);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
						HAL_Delay(100);
					}
				}
				for (int i = 0; i < numCode + 1; i++) {
					code[i] = '\0';
				}
				numCode = 0;
			}
			break;
		}
		ok = 0;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | L4_Pin | L1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : L4_Pin L1_Pin */
	GPIO_InitStruct.Pin = L4_Pin | L1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : L2_Pin */
	GPIO_InitStruct.Pin = L2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : R1_Pin */
	GPIO_InitStruct.Pin = R1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : R2_Pin R4_Pin R3_Pin */
	GPIO_InitStruct.Pin = R2_Pin | R4_Pin | R3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : L3_Pin */
	GPIO_InitStruct.Pin = L3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
GPIO_TypeDef *Rport[] =
		{ R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port, R4_GPIO_Port };
uint16_t Rpin[] = { R1_Pin, R2_Pin, R3_Pin, R4_Pin };
GPIO_TypeDef *Lport[] =
		{ L1_GPIO_Port, L2_GPIO_Port, L3_GPIO_Port, L4_GPIO_Port };
uint16_t Lpin[] = { L1_Pin, L2_Pin, L3_Pin, L4_Pin };
void MatrixKeypadRead() {
	static uint32_t timeStamp = 0;
	static uint8_t currentL = 0;
	if (HAL_GetTick() - timeStamp >= 10) {
		timeStamp = HAL_GetTick();
		for (int i = 0; i < 4; i++) {
			if (HAL_GPIO_ReadPin(Rport[i], Rpin[i]) == GPIO_PIN_RESET) {
				keypad |= 1 << (i + (currentL * 4));
			} else {
				keypad &= ~(1 << (i + (currentL * 4)));
			}
		}
		uint8_t nextL = (currentL + 1) % 4;
		HAL_GPIO_WritePin(Lport[currentL], Lpin[currentL], GPIO_PIN_SET);
		HAL_GPIO_WritePin(Lport[nextL], Lpin[nextL], GPIO_PIN_RESET);
		currentL = nextL;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

