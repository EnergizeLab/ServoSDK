/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

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

#define TransmitBufferSize 256
#define ReceiveBufferSize 256

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t TransmitBuffer[TransmitBufferSize];
uint8_t ReceiveBuffer[ReceiveBufferSize];
uint8_t ReceiveLength = 0;

uint8_t TempBuffer[20];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t GetCheck(uint8_t *buffer, uint8_t length)
{
	uint8_t sum = 0;
	for(uint8_t i=0; i<length; i++)
	{
		sum += buffer[i];
	}
	sum = ~sum;
	return sum;
}

void GetBytes(uint64_t value, uint8_t *buffer, uint8_t length)
{
	switch(length)
	{
		case 1:
			buffer[0] = value&0xff;
			break;
		case 2:
			buffer[0] = value&0xff;
			buffer[1] = (value&0xff00)>>8;
			break;
		case 4:
			buffer[0] = value&0xff;
			buffer[1] = (value&0xff00)>>8;
			buffer[2] = (value&0xff0000)>>16;
			buffer[3] = (value&0xff000000)>>24;
			break;
		case 8:
			buffer[0] = value&0xff;
			buffer[1] = (value&0xff00)>>8;
			buffer[2] = (value&0xff0000)>>16;
			buffer[3] = (value&0xff000000)>>24;
			buffer[4] = (value&0xff00000000)>>32;
			buffer[5] = (value&0xff0000000000)>>40;
			buffer[6] = (value&0xff000000000000)>>48;
			buffer[7] = (value&0xff00000000000000)>>56;
			break;
	}
}

//Send data to the servo
//id: servo ID
//instruction: instruction, 0x01:Ping, 0x02:read, 0x03:write
//address: instructionp=1, address = null
//				 instructionp=2, address = read address
//				 instructionp=3, address = write address
//*buffer: transmitting data pointer
//length: *buffer length
void ServoTransmit(uint8_t id, uint8_t instruction, uint8_t address, uint8_t *buffer, uint8_t length)
{
	HAL_HalfDuplex_EnableTransmitter(&huart1);
	TransmitBuffer[0] = 0xff;
	TransmitBuffer[1] = 0xff;
	TransmitBuffer[2] = id;
	
	TransmitBuffer[4] = instruction;
	switch(instruction)
	{
		//ping
		case 0x01:
			TransmitBuffer[3] = 2;
			TransmitBuffer[5] = GetCheck(TransmitBuffer+2,3);
			HAL_UART_Transmit(&huart1,TransmitBuffer,6,200);
			break;
		//read
		case 0x02:
			TransmitBuffer[3] = length + 3;
			TransmitBuffer[5] = address;
			TransmitBuffer[6] = buffer[0];
			TransmitBuffer[7] = GetCheck(TransmitBuffer+2,5);
			HAL_UART_Transmit(&huart1,TransmitBuffer,8,500);
			break;
		//write
		case 0x03:
			if(length > 0)
			{
				TransmitBuffer[3] = length + 3;
				TransmitBuffer[5] = address;
				for(uint8_t i=0; i<length;i++){
					TransmitBuffer[6+i] = buffer[i];
				}
				TransmitBuffer[length+6] = GetCheck(TransmitBuffer+2,length+4);
				HAL_UART_Transmit(&huart1,TransmitBuffer,length+7,500);
			}
			else
				return;
			break;
		default:
			return;
	}
	
	HAL_HalfDuplex_EnableReceiver(&huart1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		ReceiveLength++;
		//length
		if(ReceiveLength < 5)
		{
			switch(ReceiveLength)
			{
				case 1:
				case 2:
					if(ReceiveBuffer[ReceiveLength-1]!=0xff)
					{
						ReceiveLength = 0;
					}
					break;
			}
		}
		else if(ReceiveLength == ReceiveBuffer[3]+4)
		{
			if(ReceiveBuffer[ReceiveLength-1] == GetCheck(ReceiveBuffer+2,ReceiveBuffer[3]+1))
			{
				HAL_UART_Transmit(&huart2,ReceiveBuffer,ReceiveLength,200);
			}
			ReceiveBuffer[0] = 0;
			ReceiveBuffer[1] = 0;
			ReceiveBuffer[2] = 0;
			ReceiveLength = 0;
		}
		else if(ReceiveLength>ReceiveBuffer[3]+4)
		{
			ReceiveBuffer[0] = 0;
			ReceiveBuffer[1] = 0;
			ReceiveBuffer[2] = 0;
			ReceiveLength = 0;
		}
		HAL_UART_Receive_IT(&huart1,ReceiveBuffer + ReceiveLength,1);
	}
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
		
	HAL_HalfDuplex_EnableReceiver(&huart1);
	HAL_UART_Receive_IT(&huart1,ReceiveBuffer + ReceiveLength,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
		//ping id 1
		ServoTransmit(1,1,0,0,0);
		HAL_Delay(400);
		
		//read
		GetBytes(2,TempBuffer,1);
		ServoTransmit(1,2,70,TempBuffer,1);
		HAL_Delay(400);
		
		//write 1000 position
		GetBytes(1000,TempBuffer,2);
		ServoTransmit(1,3,53,TempBuffer,2);
		HAL_Delay(400);
		
		//write 2000 position
		GetBytes(2000,TempBuffer,2);
		ServoTransmit(1,3,53,TempBuffer,2);
		HAL_Delay(400);
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
