#include "reg52.h"
#include "PrimaryServo.c"

uint16_t ms_count;

sbit dir=P3^2;	//It is used to control the uart transmission direction

xdata uint8_t receive_data[20] = {0};       //Store the received status packet
xdata uint8_t receive_len = 0;              //Length of received packet
xdata uint8_t write_buffer[10] = {0};     //Write data to the memory table

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
    ms_count =  2 * ms;         //Since double speed 6T is enabled, the delay time here needs to be multiplied by 2.
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

struct primary_servo_sync_parameter servo;

void main()
{
    xdata uint8_t order_buffer[20];							//Store Generated Instructions
    xdata uint8_t order_buffer_len = 0;						//Instruction Length
    xdata uint16_t analysis_data = 0;						//Data parsed from the status packet

    timer0_init();
    uart_init();

    while(1)
    {
        servo.id_counts = 2;            //Sync write two servos
        servo.id[0] = 1;                //Set the ID of the first servo to 1
        servo.id[1] = 2;                //Set the ID of the second servo to 2

        //Change the torque switch of the servo ID1, ID2 to OFF respectively.
        servo.torque_switch[0] = 0;
        servo.torque_switch[1] = 0;
        primary_servo_sync_write_torque_switch(servo, order_buffer, &order_buffer_len);

        dir = 1;
        uart_send_buffer(order_buffer, order_buffer_len);
        delay_ms(1000);

        //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
        servo.control_mode[0] = 0;
        servo.control_mode[1] = 0;
        primary_servo_sync_write_control_mode(servo, order_buffer, &order_buffer_len);

        dir = 1;
        uart_send_buffer(order_buffer, order_buffer_len);
        delay_ms(1000);

        //Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
        primary_servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);

        dir = 1;
        uart_send_buffer(order_buffer, order_buffer_len);

        dir = 0;
        receive_len = 0;
        delay_ms(10);

        primary_servo_set_time_base_target_position_and_moving_time_analysis(receive_data);
        delay_ms(1000);

        //Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
        write_buffer[0] = 0;
        write_buffer[1] = 3000 & 0xff;
        write_buffer[2] = (3000 >> 8) & 0xff;
        write_buffer[3] = 1000 & 0xff;
        write_buffer[4] = (1000 >> 8) & 0xff;

        primary_servo_write(1, 0x3B, 5, write_buffer, order_buffer, &order_buffer_len);

        dir = 1;
        uart_send_buffer(order_buffer, order_buffer_len);

        dir = 0;
        receive_len = 0;
        delay_ms(1000);

        //In time base position control mode, let servo ID1 move to the 150° position at a velocity of 500ms, and let servo ID2 move to the 0° position at a constant velocity of 1s.
        servo.position[0] = 1500;
        servo.position[1] = 0;
        servo.time[0] = 500;
        servo.time[1] = 1000;

        dir = 1;
        primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_buffer_len);
        uart_send_buffer(order_buffer, order_buffer_len);
        delay_ms(1000);

        //In time base position control mode, let servo ID1 move to the 0° position at a velocity of 1s, and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
        servo.position[0] = 0;
        servo.position[1] = 3000;
        servo.time[0] = 1000;
        servo.time[1] = 500;

        dir = 1;
        primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_buffer_len);
        uart_send_buffer(order_buffer, order_buffer_len);
        delay_ms(1000);
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

