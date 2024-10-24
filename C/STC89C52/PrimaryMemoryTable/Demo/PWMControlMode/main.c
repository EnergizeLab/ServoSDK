#include "reg52.h"
#include "PrimaryServo.c"

uint16_t ms_count;

sbit dir=P3^2;	//It is used to control the uart transmission direction

xdata uint8_t receive_data[20] = {0};       //Store the received status packet
xdata uint8_t receive_len = 0;              //Length of received packet

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
    ms_count =  2 * ms;   //Since double speed 6T is enabled, the delay time here needs to be multiplied by 2.
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
	TMOD|=0X20;	   //8-bit automatic reload timer
	SCON=0X50;	   //8-bit UART with variable baud rate
	PCON=0X80;	   //Baud rate doubling
	TH1=0xff;	   //The baud rate is set to 115200
	TL1=0xff;
	ES=1;		   //Turn off receive interrupt
	EA=1;		   //CPU interrupts
	TR1=1;		   //Timer1 starts counting
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
	xdata uint8_t order_buffer[20];							//Store Generated Instructions
	xdata uint8_t order_buffer_len = 0;						//Instruction Length
	xdata uint16_t analysis_data = 0;						//Data parsed from the status packet

	timer0_init();
    uart_init();

	while(1)
	{
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

