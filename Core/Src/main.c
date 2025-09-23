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

#define READ_ANALOG_INPUT_VALUES_CMD 0 // this cmd will be combined with pin number. Pins 0-15 will be analog in
#define READ_DIGITAL_INPUT_VALUES_CMD 16
#define WRITE_DIGITAL_OUTPUT_VALUES_CMD 17
#define WRITE_ANALOG_OUTPUT_VALUES_CMD 18

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t cmdData[1];
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
		if (delta > DIMMER_RESET_TRESHHOLD || getTemp(adcTempValue) > temperatureThreshold) {
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

	if (TransferDirection == I2C_DIRECTION_TRANSMIT) { // if the master wants to transmit the data
		if (HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, cmdData, 1, I2C_NEXT_FRAME) != HAL_OK) {
			printf("Error: Could not read i2c command\r\n");
			return;
		}
	} else {
		uint8_t cmd = cmdData[0];
		if (cmd > 0 && cmd < 16) {
			// Received command to return analog value
			// Read 1 more byte
		    uint8_t pin;
			if (HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, &pin, 1, I2C_LAST_FRAME) != HAL_OK) {
				printf("Error: Could not read i2c data\r\n");
				return;
			}
			if (pin == 0) {
				uint8_t value = getVoltage(adcVRefValue);
				HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, &value, 1, I2C_LAST_FRAME);
			} else if (pin == 0) {
				uint8_t value = getTemp(adcTempValue);
				HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, &value, 1, I2C_LAST_FRAME);
			}
		} else if (cmd == READ_DIGITAL_INPUT_VALUES_CMD) {
			uint8_t values[2] = {0x00};

			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN11_Pin) << 0;
			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN12_Pin) << 1;
			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN13_Pin) << 2;
			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN14_Pin) << 3;
			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN15_Pin) << 4;
			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN16_Pin) << 5;
			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN17_Pin) << 6;
			values[0] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN18_Pin) << 7;

			values[1] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN19_Pin) << 0;
			values[1] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN20_Pin) << 1;
			values[1] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN21_Pin) << 2;
			values[1] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN22_Pin) << 3;
			values[1] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN23_Pin) << 4;
			values[1] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN24_Pin) << 5;
			values[1] |= HAL_GPIO_ReadPin(PIN11_GPIO_Port, PIN25_Pin) << 6;

			HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, values, 2, I2C_LAST_FRAME);

		} else if (cmd == WRITE_DIGITAL_OUTPUT_VALUES_CMD) {
			uint8_t values[2];
			if (HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, values, 2, I2C_LAST_FRAME) != HAL_OK) {
				printf("Error: Could not read i2c values\r\n");
				return;
			}

			if ((values[0] & (1 << 0)) > 0) {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			}
		} else if (cmd == WRITE_ANALOG_OUTPUT_VALUES_CMD) {

			// Received command to set analog value
			// Read 2 more bytes
			uint8_t data[2];
			if (HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, data, 2, I2C_LAST_FRAME) != HAL_OK) {
				printf("Error: Could not read i2c data\r\n");
				return;
			}

			if (data[0] == 0) {
				dimmerValue[0] = data[1]/255 * 100;
			} else if (data[0] == 1) {
				dimmerValue[1] = data[1]/255 * 100;
			} else if (data[0] == 2) {
				dimmerValue[2] = data[1]/255 * 100;
			} else if (data[0] == 3) {
				dimmerValue[3] = data[1]/255 * 100;
			} else if (data[0] == 4) {
				dimmerValue[4] = data[1]/255 * 100;
			} else if (data[0] == 5) {
				dimmerValue[5] = data[1]/255 * 100;
			} else if (data[0] == 6) {
				dimmerValue[6] = data[1]/255 * 100;
			} else if (data[0] == 7) {
				dimmerValue[7] = data[1]/255 * 100;
			} else if (data[0] == 8) {
				dimmerValue[8] = data[1]/255 * 100;
			} else if (data[0] == 9) {
				dimmerValue[9] = data[1]/255 * 100;
			} else if (data[0] == 10) {
				dimmerValue[10] = data[1]/255 * 100;
			} else {
				temperatureThreshold = data[1];
			}

		} else {
			printf("Error: Unknown cmd: %d\r\n", cmd);
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
