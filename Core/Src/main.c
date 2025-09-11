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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

/* USER CODE BEGIN PV */
uint8_t rxData[2];
uint8_t txData[1];
uint32_t rxCount;
uint32_t adcDmaValue[2];
uint32_t adcTempValue;
uint32_t adcVRefValue;

uint8_t i2cState;

uint32_t previousTick;
uint8_t timerOverloadedCount;
uint32_t timerCapturedCounter;

uint16_t dimmerValue[10];

uint8_t error;
uint8_t temperatureThreshold = 40;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t isOutputPin(uint8_t pin) {
	return (pin >= 1 && pin <= 10) || pin == 26 || pin == 27;
}

uint8_t isAnalogPin(uint8_t pin) {
	return pin == 28 || pin == 29;
}

uint8_t isInputPin(uint8_t pin) {
	return pin >= 11 && pin <= 25;
}

#define DIMMER_PINS_RESET (uint32_t)((PIN1_Pin | PIN2_Pin | PIN3_Pin | PIN4_Pin | PIN5_Pin | PIN6_Pin | PIN7_Pin | PIN8_Pin | PIN9_Pin | PIN10_Pin) << 16u)
#define DIMMER_RESET_TRESHHOLD 9800 // milliseconds
#define DEBUG 0

#define V25 1.57 // from datasheet fo CH32. for STM32 it is 1.43
#define VSENSE 3.3/4096 // VSENSE value 0.0008056640625
#define Avg_Slope 0.0043 // 4.3mV from datasheet

int getTemp(uint32_t variable) {
	return (int)(((V25 - (VSENSE * variable)) / Avg_Slope) + 25); // formula from datasheet
}

int getVoltage(uint32_t variable) {
	return (int)(VSENSE * variable); // formula from datasheet
}

uint8_t readAnalogValue(uint8_t pin) {
	if (pin == 28) {
		return (uint8_t)getTemp(adcTempValue);
	}
	if (pin == 29) {
		return (uint8_t)getVoltage(adcVRefValue);
	}
}

uint8_t readInputValue(uint8_t pin) {
	if (pin == 11) {
		return HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN11_Pin);
	} else if (pin == 12) {
		return HAL_GPIO_ReadPin(PIN12_GPIO_Port, PIN12_Pin);
	} else if (pin == 13) {
		return HAL_GPIO_ReadPin(PIN13_GPIO_Port, PIN13_Pin);
	} else if (pin == 14) {
		return HAL_GPIO_ReadPin(PIN14_GPIO_Port, PIN14_Pin);
	} else if (pin == 15) {
		return HAL_GPIO_ReadPin(PIN15_GPIO_Port, PIN15_Pin);
	} else if (pin == 16) {
		return HAL_GPIO_ReadPin(PIN16_GPIO_Port, PIN16_Pin);
	} else if (pin == 17) {
		return HAL_GPIO_ReadPin(PIN17_GPIO_Port, PIN17_Pin);
	} else if (pin == 18) {
		return HAL_GPIO_ReadPin(PIN18_GPIO_Port, PIN18_Pin);
	} else if (pin == 19) {
		return HAL_GPIO_ReadPin(PIN19_GPIO_Port, PIN19_Pin);
	} else if (pin == 20) {
		return HAL_GPIO_ReadPin(PIN20_GPIO_Port, PIN20_Pin);
	} else if (pin == 21) {
		return HAL_GPIO_ReadPin(PIN21_GPIO_Port, PIN21_Pin);
	} else if (pin == 22) {
		return HAL_GPIO_ReadPin(PIN22_GPIO_Port, PIN22_Pin);
	} else if (pin == 23) {
		return HAL_GPIO_ReadPin(PIN23_GPIO_Port, PIN23_Pin);
	} else if (pin == 24) {
		return HAL_GPIO_ReadPin(PIN24_GPIO_Port, PIN24_Pin);
	} else if (pin == 25) {
		return HAL_GPIO_ReadPin(PIN25_GPIO_Port, PIN25_Pin);
	}
}

void setOuptutValue(uint8_t pin, uint8_t value) {
	if (pin == 1) {
		dimmerValue[0] = (100 - value) * 100;
	} else if (pin == 2) {
		dimmerValue[1] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN2_GPIO_Port, PIN2_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 3) {
		dimmerValue[2] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN3_GPIO_Port, PIN3_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 4) {
		dimmerValue[3] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN4_GPIO_Port, PIN4_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 5) {
		dimmerValue[4] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN5_GPIO_Port, PIN5_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 6) {
		dimmerValue[5] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN6_GPIO_Port, PIN6_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 7) {
		dimmerValue[6] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN7_GPIO_Port, PIN7_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 8) {
		dimmerValue[7] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN8_GPIO_Port, PIN8_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 9) {
		dimmerValue[8] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN9_GPIO_Port, PIN9_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 10) {
		dimmerValue[9] = (100 - value) * 100;
		//HAL_GPIO_WritePin(PIN10_GPIO_Port, PIN10_Pin, value == 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (pin == 26) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, value == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	} else if (pin == 27 && value != 0) {
		temperatureThreshold = value;
	}
	error = 0;

}

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_I2C_EnableListen_IT(&hi2c1);
  HAL_ADC_Start_DMA(&hadc1, adcDmaValue, 2);
  HAL_GPIO_WritePin(GPIOC, 13, GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  printf("Stm 32 Port Expander by Eugen Scobich\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		#if DEBUG
			uint32_t currentTick = HAL_GetTick();
		#endif

		uint32_t timerCurrentCounter = TIM2->CNT;
		uint32_t delta = (timerCurrentCounter + 10000 * timerOverloadedCount) - timerCapturedCounter;
		if (delta > DIMMER_RESET_TRESHHOLD || error) {
			GPIOA->BSRR = DIMMER_PINS_RESET;
		} else {
			if (dimmerValue[0] < delta) {
				HAL_GPIO_WritePin(PIN1_GPIO_Port, PIN1_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[1] < delta) {
				HAL_GPIO_WritePin(PIN2_GPIO_Port, PIN2_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[2] < delta) {
				HAL_GPIO_WritePin(PIN3_GPIO_Port, PIN3_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[3] < delta) {
				HAL_GPIO_WritePin(PIN4_GPIO_Port, PIN4_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[4] < delta) {
				HAL_GPIO_WritePin(PIN5_GPIO_Port, PIN5_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[5] < delta) {
				HAL_GPIO_WritePin(PIN6_GPIO_Port, PIN6_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[6] < delta) {
				HAL_GPIO_WritePin(PIN7_GPIO_Port, PIN7_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[7] < delta) {
				HAL_GPIO_WritePin(PIN8_GPIO_Port, PIN8_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[8] < delta) {
				HAL_GPIO_WritePin(PIN9_GPIO_Port, PIN9_Pin, GPIO_PIN_SET);
			}
			if (dimmerValue[9] < delta) {
				HAL_GPIO_WritePin(PIN10_GPIO_Port, PIN10_Pin, GPIO_PIN_SET);
			}
		}

		if (i2cState == 1) {
			#if DEBUG
				printf("Received data: value:[%d], pin:[%d]\r\n", rxData[1], rxData[0]);
			#endif
			setOuptutValue(rxData[0], rxData[1]);
			i2cState = 0;
		}

		if (i2cState == 3) {
			i2cState = 0;
			if (isAnalogPin(rxData[0])) {
				txData[0] = readAnalogValue(rxData[0]);
				if (txData[0] > temperatureThreshold) {
					error = 1;
				}
				#if DEBUG
					printf("Transfer analog value:[%d] for pin:[%d]\r\n", txData[0], rxData[0]);
				#endif
			} else if (isInputPin(rxData[0])) {
				txData[0] = readInputValue(rxData[0]);
				#if DEBUG
					printf("Transfer input value:[%d] for pin:[%d]\r\n", txData[0], rxData[0]);
				#endif
			} else {
				printf("Error: Received request for pin:[%d] that is not input or output\r\n", rxData[0]);
			}

			if (HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c1, txData, 1, I2C_LAST_FRAME) != HAL_OK) {
				printf("Error: Could not send data for pin:[%d]\r\n", rxData[0]);
			}
		}
		#if DEBUG
			if (previousTick + 1000 < currentTick) {
				printf("ADC1:[%d], TEMP:[%d], VREF:[%d], V:[%d]\r\n", adcTempValue, getTemp(adcTempValue), adcVRefValue, getVoltage(adcVRefValue));
				previousTick = currentTick;
			}
		#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}

extern void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_EnableListen_IT(hi2c);
}

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (hi2c->Instance == I2C1) {
		if (TransferDirection == I2C_DIRECTION_TRANSMIT) { // if the master wants to transmit the data
			if (HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, rxData+rxCount, 1, I2C_FIRST_FRAME) != HAL_OK) {
				printf("Error: Could not read i2c values\r\n");
			}
		} else {
			if (i2cState == 2) {
				i2cState = 3;
			}
		}
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	rxCount++;
	if (rxCount == 1) {
		if(isOutputPin(rxData[0])) { // pins that need additional value
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxData+rxCount, 1, I2C_LAST_FRAME);
		} else {
			i2cState = 2;
			rxCount = 0;
		}
	} else if (rxCount == 2) {
		rxCount = 0;
		i2cState = 1;
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_DeInit(&hi2c1); // When master is reset we need to reinit i2c
	MX_I2C1_Init();
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcTempValue = adcDmaValue[0];
	adcVRefValue = adcDmaValue[1];
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	timerCapturedCounter = TIM2->CCR2;
    timerOverloadedCount = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	timerOverloadedCount++;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	printf("FATAL ERROR!!!");
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
