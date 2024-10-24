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

#define READ_TEST 0                 //Read Servo Data Test
#define WRITE_TEST 0                //Write Servo Data Test
#define SYNC_WRITE_TEST 0           //Sync Write Test
#define PING_TEST 0                 //PING Instruction Test
#define FACTORY_RESET_TEST 0        //Factory Reset Test
#define PARAMETER_RESET_TEST 0      //Parameter Reset Test
#define REBOOT_TEST 0               //Reboot Test
#define CALIBRATION_TEST 0          //Calibration Test
#define MODIFY_ID 0                 //Change Known Servo ID Test
#define MODIFY_UNKNOWN_ID 0         //Change Unknown Servo ID Test

uint8_t order_buffer[20];								//Store Generated Instructions
uint8_t order_len;											//Instruction Length
uint8_t receive[20];										//Store the received status packet
uint8_t receive_len;										//packet Length
uint32_t analysis_data;									//Data parsed from the status packet
uint8_t ret;														//Status Flag
uint16_t position = 0;                  //present position
uint16_t current = 0;                   //present current
uint8_t write_buffer[20] = {0};         //Write data to the memory table

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
#if FACTORY_RESET_TEST
		//Reset the servo to the factory default values.
    primary_servo_factory_reset(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		receive_len = 0;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		ret = primary_servo_factory_reset_analysis(receive);
		if (ret == PRIMARY_SUCCESS)
		{
			PRINTF("factory reset complete\r\n");
		}
		HAL_Delay(1000);
#endif			
		
#if PARAMETER_RESET_TEST
		//Reset the parameter settings of the servo.
    primary_servo_parameter_reset(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		receive_len = 0;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		ret = primary_servo_parameter_reset_analysis(receive);
		if (ret == PRIMARY_SUCCESS)
		{
				PRINTF("parameter reset complete\r\n");
		}
		HAL_Delay(1000);
#endif			

#if CALIBRATION_TEST
		//Calibrate the midpoint of the servo.
    primary_servo_calibration(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		ret = primary_servo_calibration_analysis(receive);
		if (ret == PRIMARY_SUCCESS)
		{
				PRINTF("calibration complete\r\n");
		}
		HAL_Delay(1000);
#endif				
		
#if REBOOT_TEST
		//Reboot the servo.
    primary_servo_reboot(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		HAL_Delay(1000);
#endif	

#if MODIFY_ID
		//Change the servo ID of servo ID1 to 2.
    primary_servo_modify_known_id(1, 2, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		HAL_Delay(1000);
#endif

#if MODIFY_ID
		//Change the servo ID of the servo with an unknown ID to 1.
    primary_servo_modify_unknown_id(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		HAL_Delay(1000);
#endif
		
#if PING_TEST
		//Query the model number of servo ID1.
    primary_servo_ping(1, order_buffer,&order_len);

		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_ping_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("model number: %d\r\n", analysis_data);
		HAL_Delay(1000);
#endif		
		
#if READ_TEST
		//Read the present position of servo ID1.
    primary_servo_read_present_position(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_position_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			  PRINTF("present position: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//Read the present current of servo ID1.
    primary_servo_read_present_current(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_current_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("present current: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif
		
#if READ_TEST
    //Read the present position and present current of servo ID1.
    primary_servo_read_present_position_and_present_current(1, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_position_and_present_current_analysis(receive, &position, &current);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("present position: %d, present current: %d\r\n", position, current);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//Read the present velocity of servo ID1.
    primary_servo_read_present_velocity(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		
    ret = primary_servo_read_present_velocity_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("present velocity: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the present profile position of servo ID1.
    primary_servo_read_present_profile_position(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_profile_position_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("present profile position: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the present profile velocity of servo ID1.
    primary_servo_read_present_profile_velocity(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_profile_velocity_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("present profile velocity: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the present PWM of servo ID1.
    primary_servo_read_present_pwm(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_pwm_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("present pwm: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the present temperature of servo ID1.
    primary_servo_read_present_temperature(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_temperature_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("present temperature: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the present voltage of servo ID1.
    primary_servo_read_present_voltage(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_present_voltage_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("present voltage: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the time base target moving time of servo ID1.
    primary_servo_read_time_base_target_moving_time(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_time_base_target_moving_time_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("present time base target moving time: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the time base target position of servo ID1.
    primary_servo_read_time_base_target_position(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_time_base_target_position_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("time base target position: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//Read the time base target ACC of servo ID1.
    primary_servo_read_time_base_target_acc(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_time_base_target_acc_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("time base target acc: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //Read the time base target position and moving time of servo ID1.
    primary_servo_read(1, 0x3C, 4, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		
    PRINTF("read time base target position and moving time status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
    //Read the time base target ACC, position and moving time of servo ID1.
    primary_servo_read(1, 0x3B, 5, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		
    PRINTF("read time base target acc, position and moving time status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//Read the velocity base target DEC of servo ID1.
    primary_servo_read_velocity_base_target_dec(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		
    ret = primary_servo_read_velocity_base_target_dec_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("velocity base target dec: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the velocity base target ACC of servo ID1.
    primary_servo_read_velocity_base_target_acc(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_velocity_base_target_acc_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("velocity base target acc: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the velocity base target velocity of servo ID1.
    primary_servo_read_velocity_base_target_velocity(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_velocity_base_target_velocity_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("velocity base target velocity: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the velocity base target position of servo ID1.
    primary_servo_read_velocity_base_target_position(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_velocity_base_target_position_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
      PRINTF("velocity base target position: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //Read the velocity base target position and velocity of servo ID1.
    primary_servo_read(1, 0x35, 4, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    PRINTF("read velocity base target position and velocity status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif

#if READ_TEST
    //Read the velocity base target position, velocity, ACC, and DEC of servo ID1.
    primary_servo_read(1, 0x35, 6, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    PRINTF("read velocity base target position, velocity, acc and dec status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//Read the target current of servo ID1.
    primary_servo_read_target_current(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_target_current_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("target current: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the target PWM of servo ID1.
    primary_servo_read_target_pwm(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_target_pwm_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("target pwm: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the torque switch of servo ID1.
    primary_servo_read_torque_switch(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_torque_switch_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("torque switch: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the LED switch of servo ID1.
    primary_servo_read_led_switch(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_led_switch_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("led switch: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//Read the Flash switch of servo ID1.
    primary_servo_read_flash_switch(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_flash_switch_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("flash switch: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the calibration of servo ID1.
    primary_servo_read_calibration(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_calibration_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("calibration: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the control mode of servo ID1.
    primary_servo_read_control_mode(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_control_mode_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("control mode: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the shutdown condition of servo ID1.
    primary_servo_read_shutdown_condition(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		receive_len = 0;
		
		HAL_Delay(10);

    ret = primary_servo_read_shutdown_condition_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("shutdown condition: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the LED condition of servo ID1.
    primary_servo_read_led_condition(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_led_condition_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("led condition: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the position control D gain of servo ID1.
    primary_servo_read_position_control_d_gain(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_position_control_d_gain_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("position control d gain: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the position control I gain of servo ID1.
    primary_servo_read_position_control_i_gain(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_position_control_i_gain_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("position control i gain: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the position control P gain of servo ID1.
    primary_servo_read_position_control_p_gain(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_position_control_p_gain_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("position control p gain: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //Read the position control PID gain of servo ID1.
    primary_servo_read(1, 0x1B, 6, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    PRINTF("read position control pid gain status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//Read the PWM punch of servo ID1.
    primary_servo_read_pwm_punch(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_pwm_punch_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("pwm punch: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the ccw deadband of servo ID1.
    primary_servo_read_ccw_deadband(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_ccw_deadband_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("ccw deadband: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the cw deadband of servo ID1.
    primary_servo_read_cw_deadband(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_cw_deadband_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("cw deadband: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the current shutdown time of servo ID1.
    primary_servo_read_current_shutdown_time(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_current_shutdown_time_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("current shutdown time: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the max current limit of servo ID1.
    primary_servo_read_max_current_limit(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_max_current_limit_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("max current limit: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the max PWM limit of servo ID1.
    primary_servo_read_max_pwm_limit(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_max_pwm_limit_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("max pwm limit: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the max voltage limit of servo ID1.
    primary_servo_read_max_voltage_limit(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_max_voltage_limit_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("max voltage limit: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the min voltage limit of servo ID1.
    primary_servo_read_min_voltage_limit(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_min_voltage_limit_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("min voltage limit: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif
		
#if READ_TEST
    //Read the voltage limit of servo ID1.
    primary_servo_read(1, 0x10, 2, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

		PRINTF("read voltage limit status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif

#if READ_TEST
		//Read the max temperature limit of servo ID1.
    primary_servo_read_max_temperature_limit(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_max_temperature_limit_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("max temperature limit: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//Read the max angle limit of servo ID1.
    primary_servo_read_max_angle_limit(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);


    ret = primary_servo_read_max_angle_limit_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("max angle limit: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the min angle limit of servo ID1.
    primary_servo_read_min_angle_limit(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_min_angle_limit_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("min angle limit: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //Read the angle limit of servo ID1.
    primary_servo_read(1, 0x0B, 4, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    PRINTF("read angle limit status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//Read the return level of servo ID1.
    primary_servo_read_return_level(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_return_level_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("return level: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the return delay time of servo ID1.
    primary_servo_read_return_delay_time(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_return_delay_time_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("return delay time: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the baud rate of servo ID1.
    primary_servo_read_baud_rate(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_baud_rate_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("baud rate: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the model information of servo ID1.
    primary_servo_read_model_information(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_model_information_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("model information: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//Read the firmware version of servo ID1.
    primary_servo_read_firmware_version(1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_read_firmware_version_analysis(receive, &analysis_data);
    if(ret == PRIMARY_SUCCESS)
			PRINTF("firmware version: %d\r\n",analysis_data);
		HAL_Delay(1000);
#endif
		
#if SYNC_WRITE_TEST
    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    primary_servo_sync_write_torque_switch(servo, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("sync write torque switch complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("sync write control mode complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target velocity of the servo ID1, ID2 to 360��/s2 and 720��/s2, respectively.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;
		
		primary_servo_sync_write_velocity_base_target_velocity(servo, order_buffer,&order_len);
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("sync write velocity base target velocity complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target ACC of servo ID1, ID2 to 500��/s2 and 50��/s2, respectively.
    servo.acc_velocity[0] = 10;          
    servo.acc_velocity[1] = 1;    
		
		primary_servo_sync_write_velocity_base_target_acc(servo, order_buffer,&order_len);
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("sync write velocity base target acc complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target DEC of servo ID1, ID2 to 50��/s2 and 500��/s2, respectively.
    servo.dec_velocity[0] = 1;           
    servo.dec_velocity[1] = 10;    
		
		primary_servo_sync_write_velocity_base_target_dec(servo, order_buffer,&order_len);
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("sync write velocity base target dec complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target velocity of the servo ID1, ID2 to 150�� midpoint and 0�� position, respectively.
    servo.position[0] = 1500;
    servo.position[1] = 0;
		
		primary_servo_sync_write_velocity_base_target_position(servo, order_buffer,&order_len);
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		PRINTF("sync write velocity base target position complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target velocity of servo ID1 ,ID2 to 1800 and 3600, and the position to 3000 and 3000, respectively
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;
		
		primary_servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer,&order_len);
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		PRINTF("sync write velocity base target position and velocity complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//SChange the velocity base target velocity of servo ID1 ,ID2 to 3600 and 3600, position to 0,0, acceleration to 500��/s2, 500��/s2, deceleration to 500��/s2, 500��/s2, respectively
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 10;
    servo.dec_velocity[0] = 10;
    servo.dec_velocity[1] = 10;
	
		primary_servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer,&order_len);
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		PRINTF("sync write velocity base target acc, dec, velocity and position complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    primary_servo_sync_write_torque_switch(servo, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("sync write torque witch complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("sync write control mode complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the time base target ACC of servo ID1 to 1 and 5 respectively.
    servo.acc_velocity_grade[0] = 1;
    servo.acc_velocity_grade[1] = 5;
		
    primary_servo_sync_write_time_base_target_acc(servo, order_buffer,&order_len);
   
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);
		PRINTF("sync write time base target acc complete\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the time base target position and moving time of servo ID1 to 150�� midpoint and 1s, 0�� and 500ms respectively.
    servo.position[0] = 1500;
    servo.position[1] = 0;
    servo.time[0] = 1000;
    servo.time[1] = 500;
		
    primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);
   
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);
		PRINTF("sync write time base target position and moving time complete\r\n");
		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the return level of servo ID1 to respond to all instruction.
    primary_servo_set_return_level(1, 2, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_return_level_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write return level complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the return delay time of servo ID1 to 500us.
		primary_servo_set_return_delay_time(1, 250, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_return_delay_time_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write return delay time complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the baud rate of servo ID1 to 1000000.
    primary_servo_set_baud_rate(1, 7, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_baud_rate_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write baud rate complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the min angle limit of servo ID1 to 0��.
    primary_servo_set_min_angle_limit(1, 0, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_min_angle_limit_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write min angle limit complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the max angle limit of servo ID1 to 300��.
    primary_servo_set_max_angle_limit(1, 3000, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_max_angle_limit_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write max angle limit complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the angle limit of servo ID1 to 0��~300��.
    write_buffer[0] = 0 & 0xff;;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3000 & 0xff;
    write_buffer[3] = (3000 >> 8) & 0xff;

    primary_servo_write(1, 0x0B, 4, write_buffer, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		
		PRINTF("write angle limit status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the max temperature limit of servo ID1 to 65��.
    primary_servo_set_max_temperature_limit(1, 65, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_max_temperature_limit_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write max temperature limit complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the max voltage limit of servo ID1 to 8.4V.
    primary_servo_set_max_voltage_limit(1,84, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_max_voltage_limit_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write max voltage limit complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the min voltage limit of servo ID1 to 3.5V.
    primary_servo_set_min_voltage_limit(1, 35, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_min_voltage_limit_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write min voltage limit complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the voltage limit of servo ID1 to 3.5~8.4V.
    write_buffer[0] = 84 & 0xff;
    write_buffer[1] = 35 & 0xff;

    primary_servo_write(1, 0x10, 2, write_buffer, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		
		PRINTF("write voltage limit status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the max PWM limit of servo ID1 to 90%.
    primary_servo_set_max_pwm_limit(1, 900, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_max_pwm_limit_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write max pwm limit complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the max current limit of servo ID1 to 900mA.
    primary_servo_set_max_current_limit(1, 900, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_max_current_limit_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write max current limit complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the current shutdown time of servo ID1 to 500ms.
    primary_servo_set_current_shutdown_time(1, 500, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_current_shutdown_time_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write current shutdown time complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the CW deadband of servo ID1 to 0.2��.
    primary_servo_set_cw_deadband(1, 2, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_cw_deadband_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write cw deadband complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the CCW deadband of servo ID1 to 0.2��.
    primary_servo_set_ccw_deadband(1, 2, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_ccw_deadband_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write ccw deadband complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the CW and CCW deadband of servo ID1 to 0.2��.
    write_buffer[0] = 2 & 0xff;
    write_buffer[1] = 2 & 0xff;

    primary_servo_write(1, 0x18, 2, write_buffer, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);
		
		PRINTF("write cw deadband and ccw deadband status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the PWM punch of servo ID1 to 1%.
    primary_servo_set_pwm_punch(1, 10, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_pwm_punch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write pwm punch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the position control P gain of servo ID1 to 5995.
    primary_servo_set_position_control_p_gain(1, 5995, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_position_control_p_gain_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write position control p gain complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the position control D gain of servo ID1 to 5.
    primary_servo_set_position_control_i_gain(1, 5, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_position_control_i_gain_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write position control i gain complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the position control D gain of servo ID1 to 145.
    primary_servo_set_position_control_d_gain(1, 145, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_position_control_d_gain_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write position control d gain complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the position control PID gain of servo ID1 to 5995, 5, and 145 respectively.
    write_buffer[0] = 5995 & 0xff;
    write_buffer[1] = (5995 >> 8) & 0xff;
    write_buffer[2] = 5 & 0xff;
    write_buffer[3] = (5 >> 8) & 0xff;
    write_buffer[4] = 145 & 0xff;
    write_buffer[5] = (145 >> 8) & 0xff;

    primary_servo_write(1, 0x1B, 6, write_buffer, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

		HAL_Delay(10);

		PRINTF("write position control pid gain status packet: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%02x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the LED condition of servo ID1 to turn on stall error, overheating error, and angle error.
    primary_servo_set_led_condition(1, 38, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_led_condition_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write led condition complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the shutdown condition of servo ID1 to turn on stall error, overheating error, voltage error, and angle error.
    primary_servo_set_shutdown_conditions(1, 39, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_shutdown_conditions_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write shutdown conditions complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the Flash switch of servo ID1 to ON.
    primary_servo_set_flash_switch(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_flash_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write flash switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the Flash switch of servo ID1 to OFF.
    primary_servo_set_flash_switch(1, 0, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_flash_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write flash switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the LED switch of servo ID1 to ON.
    primary_servo_set_led_switch(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_led_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write led switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the LED switch of servo ID1 to OFF.
    primary_servo_set_led_switch(1, 0, order_buffer, &order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_led_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write led switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the PWM control mode.
    primary_servo_set_control_mode(1, 3, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_control_mode_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write control mode complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the target PWM of servo ID1 to -50%.
    primary_servo_set_target_pwm(1, -500, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_target_pwm_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write target pwm complete\r\n");

		HAL_Delay(3000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the current control mode.
    primary_servo_set_control_mode(1, 2, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_control_mode_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write control mode complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the target current of servo ID1 to -400mA.
    primary_servo_set_target_current(1, -400, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_target_current_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write target current complete\r\n");

		HAL_Delay(3000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the velocity base position control mode.
    primary_servo_set_control_mode(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_control_mode_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write control mode complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the velocity base target velocity of servo ID1 to 360��/s.
    primary_servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_velocity_base_target_velocity_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write velocity base target velocity complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the velocity base target ACC of servo ID1 to 500��/s2.
    primary_servo_set_velocity_base_target_acc(1, 10, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_velocity_base_target_acc_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write velocity base target acc complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the velocity base target DEC of servo ID1 to 50��/s2.
    primary_servo_set_velocity_base_target_dec(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_velocity_base_target_dec_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write velocity base target dec complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the velocity base target position of servo ID1 to 150��.
    primary_servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_len);
   
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_velocity_base_target_position_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write velocity base target position complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the time base position control mode.
    primary_servo_set_control_mode(1, 0, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_control_mode_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write control mode complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the time base target ACC of servo ID1 to 5.
    primary_servo_set_time_base_target_acc(1, 5, order_buffer,&order_len);

    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		receive_len = 0;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(10);

    ret = primary_servo_set_time_base_target_acc_analysis(receive);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write time base target acc complete\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //Change the time base target position and moving time of servo ID1 to 300��, 500ms respectively.
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
#endif
		
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
