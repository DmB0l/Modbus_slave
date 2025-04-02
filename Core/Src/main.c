/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "modbus.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_SILENT_PERIOD_MS 4  // 3.5 символа при 9600 бод ≈ 4 мс
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t start_time = 0;

uint8_t rx_buffer[256];                // Буфер для приема запросов
uint8_t rx_real_size = 0;
uint8_t func_code = 0;

uint8_t response_buffer[MODBUS_MAX_ADU_SIZE];
uint16_t resp_length;

ModbusDevice *device = NULL;

volatile uint8_t send_flag = 0;        // Флаг для отправки ответа
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Send_Message(void);
void PrintHexArray8(const uint8_t *array, int length);
void PrintHexArray16(const uint16_t *array, int length);
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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	start_time = HAL_GetTick();
	device = modbus_init_device(100, 100, 100, 100); // Инициализация устройства Modbus

	HAL_UART_Receive_IT(&huart1, rx_buffer, 8);      // Начинаем прием запросов
	HAL_GPIO_WritePin(GPIOA, DREnabled_Pin, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//		if (send_flag) {
//			Send_Message(); // Отправляем ответ, если флаг установлен
//			send_flag = 0;
//		}

		HAL_GPIO_TogglePin(GPIOC, LED_Pin); // Мигаем светодиодом для индикации работы
		HAL_Delay(100);

		if (HAL_GetTick() - start_time > 5000) {
			HAL_UART_AbortReceive(&huart1);  // Аварийная остановка приема (если он был запущен)
			HAL_UART_Receive_IT(&huart1, rx_buffer, 8);      // Начинаем прием запросов
			start_time = HAL_GetTick();
		}
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

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DREnabled_GPIO_Port, DREnabled_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DREnabled_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DREnabled_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		start_time = HAL_GetTick();
		if (func_code == 0) {
			func_code = rx_buffer[1];
			if (func_code == 0x10 || func_code == 0x0F) {
				uint8_t ost_size = rx_buffer[6] - 1 + 2; // Остаток данных для функций 0x10 и 0x0F
				rx_real_size = 8 + ost_size;
				if (rx_real_size > sizeof(rx_buffer)) {
					func_code = 0; // Ошибка: буфер слишком мал
					rx_real_size = 0;
					HAL_UART_Receive_IT(&huart1, rx_buffer, 8);
					return;
				}
				HAL_UART_Receive_IT(&huart1, &rx_buffer[8], ost_size);
			} else {
				func_code = 0;
				rx_real_size = 0;
				modbus_process_response_from_master(device, rx_buffer, 8, response_buffer, &resp_length);
				printf("Receiving request from master: ");
				PrintHexArray8(rx_buffer, 8);
				Send_Message();
				HAL_UART_Receive_IT(&huart1, rx_buffer, 8); // Готовимся к следующему запросу
			}
		} else {
			func_code = 0;
			modbus_process_response_from_master(device, rx_buffer, rx_real_size, response_buffer, &resp_length);
			printf("Receiving request from master: ");
			PrintHexArray8(rx_buffer, rx_real_size);
			Send_Message();
			rx_real_size = 0;
			HAL_UART_Receive_IT(&huart1, rx_buffer, 8); // Готовимся к следующему запросу
		}
	}
}

void Send_Message(void) {
//    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
	printf("Sending answer to master: ");
	PrintHexArray8(response_buffer, resp_length);
	HAL_GPIO_WritePin(GPIOA, DREnabled_Pin, GPIO_PIN_SET); // Переключаем в режим передачи
	HAL_UART_Transmit(&huart1, response_buffer, resp_length, 100); // Асинхронная отправка
	HAL_GPIO_WritePin(GPIOA, DREnabled_Pin, GPIO_PIN_RESET); // Переключаем в режим передачи
//    }
}

void PrintHexArray8(const uint8_t *array, int length) {
    for (int i = 0; i < length; i++) {
        printf("%02X ", array[i]); // %02X для HEX в верхнем регистре
    }
    printf("\n");
}

void PrintHexArray16(const uint16_t *array, int length) {
	for (uint16_t i = 0; i < length; i++) {
	    printf("%04X ", array[i]); // Вывод uint16_t в HEX
	}
	printf("\n");
}

// Перенаправление printf в UART2
int _write(int file, uint8_t *ptr, int len) {  // Лучше использовать uint8_t*
    HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
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
}
#endif /* USE_FULL_ASSERT */
