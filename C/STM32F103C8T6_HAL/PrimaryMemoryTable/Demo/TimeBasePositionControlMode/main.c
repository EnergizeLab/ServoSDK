/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "PrimaryServo.h"
#include <stdio.h>

uint8_t order_buffer[20];								//Store Generated Instructions
uint8_t order_len;										//Instruction Length
uint8_t receive[20];									//Store the received status packet
uint8_t receive_len;									//packet Length
uint8_t ret;											//Status Flag
uint8_t write_buffer[20] = {0};                         //Write data to the memory table

void initTransmitMode(UART_HandleTypeDef *huart);
void initReceiveMode(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
    return ch;
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
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    HAL_UART_Receive_IT(&huart1, receive, 1);

    struct primary_servo_sync_parameter servo;

    servo.id_counts = 2;            //Sync write two servos
    servo.id[0] = 1;                //Set the ID of the first servo to 1
    servo.id[1] = 2;                //Set the ID of the second servo to 2

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //Change the torque switch of the servo ID1, ID2 to OFF respectively.
        servo.torque_switch[0] = 0;
        servo.torque_switch[1] = 0;
        primary_servo_sync_write_torque_switch(servo, order_buffer, &order_len);

        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
        PRINTF("sync write torque witch complete\r\n");
        HAL_Delay(1000);

        //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
        servo.control_mode[0] = 0;
        servo.control_mode[1] = 0;
        primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);

        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
        PRINTF("sync write control mode complete\r\n");
        HAL_Delay(1000);

        //Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
        primary_servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_len);

        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

        receive_len = 0;
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

        HAL_Delay(10);

        ret = primary_servo_set_time_base_target_position_and_moving_time_analysis(receive);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write time base target position and moving time complete\r\n");
        HAL_Delay(1000);

        //Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
        write_buffer[0] = 0;
        write_buffer[1] = 3000 & 0xff;
        write_buffer[2] = (3000 >> 8) & 0xff;
        write_buffer[3] = 1000 & 0xff;
        write_buffer[4] = (1000 >> 8) & 0xff;

        primary_servo_write(1, 0x3B, 5, write_buffer, order_buffer, &order_len);

        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

        receive_len = 0;
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

        PRINTF("write time base target acc, position and moving time status packet: ");
        for (uint8_t i = 0; i < receive_len; i++)
        {
            PRINTF("0x%02x ", receive[i]);
        }
        PRINTF("\r\n");

        HAL_Delay(1000);


        //In time base position control mode, let servo ID1 move to the 150° position at a velocity of 500ms,
        //and let servo ID2 move to the 0° position at a constant velocity of 1s.
        servo.position[0] = 1500;
        servo.position[1] = 0;
        servo.time[0] = 500;
        servo.time[1] = 1000;

        primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);

        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);
        PRINTF("sync write time base target position and moving time complete\r\n");
        HAL_Delay(1000);

        //In time base position control mode, let servo ID1 move to the 0° position at a velocity of 1s,
        //and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
        servo.position[0] = 0;
        servo.position[1] = 3000;
        servo.time[0] = 1000;
        servo.time[1] = 500;

        primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);

        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);
        PRINTF("sync write time base target position and moving time complete\r\n");
        HAL_Delay(1000);
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
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function Should not be modified, when the callback is needed,
             the HAL_UART_TxCpltCallback could be implemented in the user file
    */
    receive_len++;
    HAL_UART_Receive_IT(&huart1, receive + receive_len, 1);
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
