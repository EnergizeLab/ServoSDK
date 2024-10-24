#include "reg52.h"
#include "PrimaryServo.c"

#define READ_TEST 0                						//Read Servo Data Test
#define WRITE_TEST 0                					//Write Servo Data Test
#define SYNC_WRITE_TEST 0           					//Sync Write Test
#define PING_TEST 0                 					//PING Instruction Test
#define FACTORY_RESET_TEST 0        					//Factory Reset Test
#define PARAMETER_RESET_TEST 0      					//Parameter Reset Test
#define REBOOT_TEST 0               					//Reboot Test
#define CALIBRATION_TEST 0          					//Calibration Test
#define MODIFY_ID_TEST 0            					//Change Known Servo ID Test
#define MODIFY_UNKNOWN_ID_TEST 0    					//Change Unknown Servo ID Test

uint16_t ms_count;

sbit dir=P3^2;	//It is used to control the uart transmission direction

xdata uint8_t receive_data[20] = {0};					//Store the received status packet
xdata uint8_t receive_len = 0;         				//received Length
xdata uint8_t write_buffer[20] = {0}; 				//Write data to the memory table

void timer0_init()
{
    TMOD |= 0x01;
    TH0 = 0xFC;
    TL0 = 0x67;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

void delay_ms(uint16_t ms)
{
    ms_count =  2 * ms; 
    while (ms_count);
}

void timer0_isr() interrupt 1 using 1
{
    TH0 = 0xFC;
    TL0 = 0x67;
    if (ms_count)
        ms_count--;
}

void uart_init()
{
		TMOD|=0X20;             //8-bit automatic reload timer
		SCON=0X50;	            //8-bit UART with variable baud rate
		PCON=0X80;              //Baud rate doubling
		TH1=0xff;	            	//The baud rate is set to 115200
		TL1=0xff;
    ES=1;		            		//Turn off receive interrupt
    EA=1;	                	//CPU interrupts
    TR1=1;		            	//Timer1 starts counting
}

void uart_send(uint8_t order_data)
{
    SBUF = order_data;      
    while(!TI);    			
    TI = 0;      		
}


void uart_send_buffer(uint8_t *buffer, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++) {
        uart_send(buffer[i]);
    }
}


void main()
{
	xdata uint8_t order_buffer[40];												                 	//Store Generated Instructions
	xdata uint8_t order_buffer_len = 0;										                  //Instruction Length
	xdata uint32_t analysis_data = 0;											                  //Data parsed from the status packet
	xdata uint16_t position = 0;                                            //present position
  xdata uint16_t current = 0;   
	
	struct primary_servo_sync_parameter servo;
	
	timer0_init();
	uart_init();
	
	while(1)
	{
#if FACTORY_RESET_TEST
    //Reset the servo to the factory default values.
		dir = 1;
		primary_servo_factory_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		
		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_factory_reset_analysis(receive_data);
		delay_ms(1000);
#endif			
		
#if PARAMETER_RESET_TEST
    //Reset the parameter settings of the servo.
		dir = 1;
		primary_servo_parameter_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_parameter_reset_analysis(receive_data);
		delay_ms(1000);
#endif			

#if CALIBRATION_TEST
    //Calibrate the midpoint of the servo.
		dir = 1;
		primary_servo_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_calibration_analysis(receive_data);
		delay_ms(1000);
#endif				
		
#if REBOOT_TEST
    //Reboot the servo.
		dir = 1;
		primary_servo_reboot(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		delay_ms(1000);
#endif		

#if PING_TEST
    //Query the model number of servo ID1.
		dir = 1;
		primary_servo_ping(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_ping_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif	

#if MODIFY_ID_TEST
    //Change the servo ID of servo ID1 to 2.
		dir = 1;
		primary_servo_modify_known_id(1, 2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
#endif

#if MODIFY_UNKNOWN_ID_TEST
    //Change the servo ID of the servo with an unknown ID to 1.
		dir = 1;
		primary_servo_modify_unknown_id(2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
#endif

#if READ_TEST
    //Read the present current of servo ID1.
		dir = 1;
		primary_servo_read_present_current(1, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the present position of servo ID1.
		dir = 1;
		primary_servo_read_present_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_present_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
    //Read the present position and present current of servo ID1.
		dir = 1;
    primary_servo_read_present_position_and_present_current(1, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_present_position_and_present_current_analysis(receive_data, &position, &current);
		delay_ms(1000);

    //Read the present velocity of servo ID1.
		dir = 1;
		primary_servo_read_present_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_present_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
    //Read the present profile position of servo ID1.
		dir = 1;
		primary_servo_read_present_profile_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the present profile velocity of servo ID1.
		dir = 1;
		primary_servo_read_present_profile_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		
		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the present PWM of servo ID1.
		dir = 1;
		primary_servo_read_present_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_present_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the present temperature of servo ID1.
		dir = 1;
		primary_servo_read_present_temperature(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_present_temperature_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the present voltage of servo ID1.
		dir = 1;
		primary_servo_read_present_voltage(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_present_voltage_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the time base target moving time of servo ID1.
		dir = 1;
		primary_servo_read_time_base_target_moving_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the time base target position of servo ID1.
		dir = 1;
		primary_servo_read_time_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the time base target ACC of servo ID1.
		dir = 1;
		primary_servo_read_time_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		//Read the time base target position and moving time of servo ID1.
	  dir = 1;
    primary_servo_read(1, 0x3C, 4, order_buffer, &order_buffer_len);
    uart_send_buffer(order_buffer, order_buffer_len);
		
		dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Read the time base target ACC, position and moving time of servo ID1.
    dir = 1;
    primary_servo_read(1, 0x3B, 5, order_buffer, &order_buffer_len);
    uart_send_buffer(order_buffer, order_buffer_len);
		
		dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Read the velocity base target DEC of servo ID1.
		dir = 1;
		primary_servo_read_velocity_base_target_dec(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the velocity base target ACC of servo ID1.
		dir = 1;
		primary_servo_read_velocity_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the velocity base target velocity of servo ID1.
		dir = 1;
		primary_servo_read_velocity_base_target_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the velocity base target position of servo ID1.
		dir = 1;
		primary_servo_read_velocity_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
	 //Read the velocity base target position and velocity of servo ID1.
		dir = 1;
    primary_servo_read(1, 0x35, 4, order_buffer, &order_buffer_len);
    uart_send_buffer(order_buffer, order_buffer_len);
		
	  dir = 0;
		receive_len = 0;
		delay_ms(1000);
		
    //Read the velocity base target position, velocity, ACC, and DEC of servo ID1.
		dir = 1;
    primary_servo_read(1, 0x35, 6, order_buffer, &order_buffer_len);
    uart_send_buffer(order_buffer, order_buffer_len);
		
	  dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Read the target current of servo ID1.
		dir = 1;
		primary_servo_read_target_current(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_target_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the target PWM of servo ID1.
		dir = 1;
		primary_servo_read_target_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_target_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the torque switch of servo ID1.
		dir = 1;
		primary_servo_read_torque_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_torque_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the LED switch of servo ID1.
		dir = 1;
		primary_servo_read_led_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_led_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the Flash switch of servo ID1.
		dir = 1;
		primary_servo_read_flash_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_flash_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the calibration of servo ID1.
		dir = 1;
		primary_servo_read_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_calibration_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the control mode of servo ID1.
		dir = 1;
		primary_servo_read_control_mode(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_control_mode_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the shutdown condition of servo ID1.
		dir = 1;
		primary_servo_read_shutdown_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the LED condition of servo ID1.
		dir = 1;
		primary_servo_read_led_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_led_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the position control D gain of servo ID1.
		dir = 1;
		primary_servo_read_position_control_d_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the position control I gain of servo ID1.
		dir = 1;
		primary_servo_read_position_control_i_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the position control P gain of servo ID1.
		dir = 1;
		primary_servo_read_position_control_p_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		//Read the position control PID gain of servo ID1.
		dir = 1;
    primary_servo_read(1, 0x1B, 6, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Read the PWM punch of servo ID1.
		dir = 1;
		primary_servo_read_pwm_punch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the ccw deadband range of servo ID1.
		dir = 1;
		primary_servo_read_ccw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the cw deadband range of servo ID1.
		dir = 1;
		primary_servo_read_cw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the current shutdown time of servo ID1.
		dir = 1;
		primary_servo_read_current_shutdown_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the max current limit of servo ID1.
		dir = 1;
		primary_servo_read_max_current_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the max PWM limit of servo ID1.
		dir = 1;
		primary_servo_read_max_pwm_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the max voltage limit of servo ID1.
		dir = 1;
		primary_servo_read_max_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the min voltage limit of servo ID1.
		dir = 1;
		primary_servo_read_min_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		//Read the voltage limit of servo ID1.
		dir = 1;
    primary_servo_read(1, 0x10, 2, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(1000);
		
    //Read the max temperature limit of servo ID1.
		dir = 1;
		primary_servo_read_max_temperature_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the max angle limit of servo ID1.
		dir = 1;
		primary_servo_read_max_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the min angle limit of servo ID1.
		dir = 1;
		primary_servo_read_min_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		//Read the angle limit of servo ID1.
		dir = 1;
    primary_servo_read(1, 0x0B, 4, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Read the return level of servo ID1.
		dir = 1;
		primary_servo_read_return_level(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_return_level_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the return delay time of servo ID1.
		dir = 1;
		primary_servo_read_return_delay_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the baud rate of servo ID1.
		dir = 1;
		primary_servo_read_baud_rate(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_baud_rate_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the model information of servo ID1.
		dir = 1;
		primary_servo_read_model_information(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_model_information_analysis(receive_data, &analysis_data);
		delay_ms(1000);

    //Read the firmware version of servo ID1.
		dir = 1;
		primary_servo_read_firmware_version(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		
		primary_servo_read_firmware_version_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if WRITE_TEST
    //Change the return delay time of servo ID1 to 500us.
		dir = 1;
		primary_servo_set_return_delay_time(1, 250,order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_return_delay_time_analysis(receive_data);	 
		delay_ms(1000);

    //Change the return level of servo ID1 to respond to all instruction.
		dir = 1;
		primary_servo_set_return_level(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_return_level_analysis(receive_data);	   
		delay_ms(1000);

    //Change the baud rate of servo ID1 to 115200.
		dir = 1;
		primary_servo_set_baud_rate(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_baud_rate_analysis(receive_data);		 
		delay_ms(1000);

    //Change the min angle limit of servo ID1 to 0�X.
		dir = 1;
		primary_servo_set_min_angle_limit(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_min_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

    //Change the max angle limit of servo ID1 to 300�X.
		dir = 1;
		primary_servo_set_max_angle_limit(1, 3000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_max_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

	  //Change the angle limit of servo ID1 to 0�X~300�X.
    write_buffer[0] = 0 & 0xff;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3000 & 0xff;
    write_buffer[3] = (3000 >> 8) & 0xff;
		
		dir = 1;
    primary_servo_write(1, 0x0B, 4, write_buffer, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Change the max temperature limit of servo ID1 to 65�J.
		dir = 1;
		primary_servo_set_max_temperature_limit(1, 65, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_max_temperature_limit_analysis(receive_data);	  
		delay_ms(1000);

    //Change the max voltage limit of servo ID1 to 8.4V.
		dir = 1;
		primary_servo_set_max_voltage_limit(1,84, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_max_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

    //Change the min voltage limit of servo ID1 to 3.5V.
		dir = 1;
		primary_servo_set_min_voltage_limit(1, 35, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_min_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);
		
		//Change the voltage limit of servo ID1 to 3.5~8.4V.
    write_buffer[0] = 84 & 0xff;
    write_buffer[1] = 35 & 0xff;
		dir = 1;
    primary_servo_write(1, 0x10, 2, write_buffer, order_buffer, &order_buffer_len);
		
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Change the max PWM limit of servo ID1 to 90%.
		dir = 1;
		primary_servo_set_max_pwm_limit(1, 900, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_max_pwm_limit_analysis(receive_data);	  
		delay_ms(1000);

    //Change the max current limit of servo ID1 to 900mA.
		dir = 1;
		primary_servo_set_max_current_limit(1, 400, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_max_current_limit_analysis(receive_data);	  
		delay_ms(1000);

    //Change the current shutdown time of servo ID1 to 500ms.
		dir = 1;
		primary_servo_set_current_shutdown_time(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_current_shutdown_time_analysis(receive_data);	
		delay_ms(1000);

    //Change the CW deadband of servo ID1 to 0.2�X.
		dir = 1;
		primary_servo_set_cw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_cw_deadband_analysis(receive_data);		
		delay_ms(1000);

    //Change the CCW deadband of servo ID1 to 0.2�X.
		dir = 1;
		primary_servo_set_ccw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_ccw_deadband_analysis(receive_data); 
		delay_ms(1000);
		
		//Change the CW and CCW deadband of servo ID1 to 0.2�X.
    write_buffer[0] = 2 & 0xff;
    write_buffer[1] = 2 & 0xff;
		
		dir = 1;
    primary_servo_write(1, 0x18, 2, write_buffer, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;	 
		receive_len = 0;
		delay_ms(1000);

    //Change the PWM punch of servo ID1 to 1%.
		dir = 1;
		primary_servo_set_pwm_punch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_pwm_punch_analysis(receive_data);		 
		delay_ms(1000);

    //Change the position control P gain of servo ID1 to 5995.
		dir = 1;
		primary_servo_set_position_control_p_gain(1, 6000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_position_control_p_gain_analysis(receive_data); 
		delay_ms(1000);

    //Change the position control I gain of servo ID1 to 5.
		dir = 1;
		primary_servo_set_position_control_i_gain(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_position_control_i_gain_analysis(receive_data);  
		delay_ms(1000);

    //Change the position control D gain of servo ID1 to 145.
		dir = 1;
		primary_servo_set_position_control_d_gain(1, 151, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_position_control_d_gain_analysis(receive_data); 
		delay_ms(1000);
		
		//Change the position control PID gain of servo ID1 to 5995, 5, and 145 respectively.
    write_buffer[0] = 5995 & 0xff;
    write_buffer[1] = (5995 >> 8) & 0xff;
    write_buffer[2] = 5 & 0xff;
    write_buffer[3] = (5 >> 8) & 0xff;
    write_buffer[4] = 145 & 0xff;
    write_buffer[5] = (145 >> 8) & 0xff;

		dir = 1;
    primary_servo_write(1, 0x1B, 6, write_buffer, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(1000);

    //Change the LED condition of servo ID1 to turn on stall error, overheating error, and angle error.
		dir = 1;
		primary_servo_set_led_condition(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_led_condition_analysis(receive_data);		   
		delay_ms(1000);

    //Change the shutdown condition of servo ID1 to turn on stall error, overheating error, voltage error, and angle error.
		dir = 1;
		primary_servo_set_shutdown_conditions(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_shutdown_conditions_analysis(receive_data);	
		delay_ms(1000);

    //Change the LED switch of servo ID1 to ON.
		dir = 1;
		primary_servo_set_led_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_led_switch_analysis(receive_data);	 
		delay_ms(1000);

    //Change the torque switch of servo ID1 to OFF.
		dir = 1;
		primary_servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

    //Change the control mode of servo ID1 to the PWM control mode.
		dir = 1;
		primary_servo_set_control_mode(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

    //Change the torque switch of servo ID1 to ON.
		dir = 1;
		primary_servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

    //Change the target PWM of servo ID1 to -50%.
		dir = 1;
		primary_servo_set_target_pwm(1, -500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_target_pwm_analysis(receive_data);  
		delay_ms(3000);

    //Change the torque switch of servo ID1 to OFF.
		dir = 1;
		primary_servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

    //Change the control mode of servo ID1 to the current control mode.
		dir = 1;
		primary_servo_set_control_mode(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

    //Change the torque switch of servo ID1 to ON.
		dir = 1;
		primary_servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

    //Change the target current of servo ID1 to -400mA.
		dir = 1;
		primary_servo_set_target_current(1, -400, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);

		primary_servo_set_target_current_analysis(receive_data);   
		delay_ms(3000);

    //Change the torque switch of servo ID1 to OFF.
		dir = 1;
		primary_servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

    //Change the control mode of servo ID1 to the velocity base position control mode.
		dir = 1;
		primary_servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

    //Change the torque switch of servo ID1 to ON.
		dir = 1;
		primary_servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

    //Change the velocity base target velocity of servo ID1 to 360�X/s.
		dir = 1;
		primary_servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_velocity_base_target_velocity_analysis(receive_data);	
		delay_ms(1000);

    //Change the velocity base target ACC of servo ID1 to 500�X/s2.
		dir = 1;
		primary_servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_velocity_base_target_acc_analysis(receive_data);	
		delay_ms(1000);

    //Change the velocity base target DEC of servo ID1 to 50�X/s2.
		dir = 1;
		primary_servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_velocity_base_target_dec_analysis(receive_data);	
		delay_ms(1000);

    //Change the velocity base target position of servo ID1 to 150�X.
		dir = 1;
		primary_servo_set_velocity_base_target_position(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);

    //Change the torque switch of servo ID1 to OFF.
		dir = 1;
		primary_servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

    //Change the control mode of servo ID1 to the time base position control mode.
		dir = 1;
		primary_servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

    //Change the torque switch of servo ID1 to ON.
		dir = 1;
		primary_servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_torque_switch_analysis(receive_data);		 
		delay_ms(1000);

    //Change the time base target ACC of servo ID1 to 5.
		dir = 1;
		primary_servo_set_time_base_target_acc(1, 5, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_time_base_target_acc_analysis(receive_data);	 
		delay_ms(1000);

    //Change the time base target position and moving time of servo ID1 to 300�X, 500ms respectively.
		dir = 1;
		primary_servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		dir = 0;
		receive_len = 0;
		delay_ms(10);
		primary_servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
#endif

#if SYNC_WRITE_TEST
		servo.id_counts = 2;         //Sync write two servos
		servo.id[0] = 1;                //Set the ID of the first servo to 1
		servo.id[1] = 2;                //Set the ID of the second servo to 2
		
		//Change the torque switch of the servo ID1, ID2 to OFF respectively.
		servo.torque_switch[0] = 0;
		servo.torque_switch[1] = 0;
		dir = 1;
		primary_servo_sync_write_torque_switch(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
		dir = 1;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		//Change the velocity base target ACC of servo ID1, ID2 to 500�X/s2 and 50�X/s2, respectively.  
    //Set the acceleration of servo ID1 and ID2 to 10 and 1, respectively, corresponding to the previous ID settings.
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 1;
		dir = 1;
		primary_servo_sync_write_velocity_base_target_acc(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		//Change the velocity base target DEC of servo ID1, ID2 to 50�X/s2 and 500�X/s2, respectively.
    //Set the deceleration of servo ID1 and ID2 to 1 and 10, respectively, corresponding to the previous ID settings.
    servo.dec_velocity[0] = 1;
    servo.dec_velocity[1] = 10;
		dir = 1;
    primary_servo_sync_write_velocity_base_target_dec(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
	 //Change the velocity base target velocity of the servo ID1, ID2 to 360�X/s2 and 720�X/s2, respectively.
    //Set the velocity of servo ID1 and ID2 to 3600 and 7200, respectively, corresponding to the previous ID settings.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;
		dir = 1;
    primary_servo_sync_write_velocity_base_target_velocity(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		//Change the velocity base target velocity of the servo ID1, ID2 to 150�X midpoint and 0�X position, respectively.
    //Set the position of servo ID1 and ID2 to 1500 and 0, respectively, corresponding to the previous ID settings.
    servo.position[0] = 1500;
    servo.position[1] = 0;
		dir = 1;
    primary_servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		//Change the velocity base target velocity of servo ID1 ,ID2 to 1800 and 3600, and the position to 3000 and 3000, respectively
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;
		dir = 1;
    primary_servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		//Set the velocity of servo ID1 and ID2 to 3600 and 3600, set their positions to 0 and 0, set their accelerations to 100 and 100, and their decelerations to 100 and 100, respectively.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 100;
    servo.acc_velocity[1] = 100;
    servo.dec_velocity[0] = 100;
    servo.dec_velocity[1] = 100;
		dir = 1;
    primary_servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		//Change the torque switch of the servo ID1, ID2 to OFF respectively.
		servo.torque_switch[0] = 0;
		servo.torque_switch[1] = 0;
		dir = 1;
		primary_servo_sync_write_torque_switch(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		//Change the control mode of servo ID1, ID2 to time base position control mode respectively.
		servo.control_mode[0] = 0;
		servo.control_mode[1] = 0;
		dir = 1;
		primary_servo_sync_write_control_mode(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		//Change the time base target ACC of servo ID1 to 1 and 5 respectively
		servo.acc_velocity_grade[0] = 1;
		servo.acc_velocity_grade[1] = 5;
		dir = 1;
		primary_servo_sync_write_time_base_target_acc(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		//Change the time base target position and moving time of servo ID1 to 150�X midpoint and 1s, 0�X and 500ms respectively.
		servo.position[0] = 1500;
		servo.position[1] = 0;
		servo.time[0] = 1000;
		servo.time[1] = 500;
		dir = 1;
		primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
#endif
	}		
}

void uart() interrupt 4
{
	if(RI)                                       
	{
		RI = 0;                                
		receive_data[receive_len++] = SBUF;    
	}				
}
