#include "stm32f10x.h"
#include "platform_config.h"
#include "PrimaryServo.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

uint8_t receive_data[20];               //Store the received status packet
uint8_t receive_len;                    //received Length
uint8_t ret;                            //Status Flag
uint8_t write_buffer[20] = {0};         //Write data to the memory table

extern __IO uint32_t TimingDelay;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Init(void);
void USART2_Init(void);
void USART1_Send(uint8_t *data, uint8_t data_len);
void SysTick_Configuration(void);
void Delay(__IO uint32_t nTime);

//Redirect printf
int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t) ch);

    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {}

    return ch;
}

int main(void)
{
    uint8_t order_buffer[20];       //Store Generated Instructions
    uint8_t order_len;              //Instruction Length

    SysTick_Configuration();
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Init();

    //Information print serial port initialization
    USART2_Init();

    struct primary_servo_sync_parameter servo;

    servo.id_counts = 2;            //Sync write two servos
    servo.id[0] = 1;                //Set the ID of the first servo to 1
    servo.id[1] = 2;                //Set the ID of the second servo to 2

    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    primary_servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    USART1_Send(order_buffer, order_len);
    PRINTF("sync write torque switch complete\r\n");
    Delay(1000);

    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    USART1_Send(order_buffer, order_len);
    PRINTF("sync write control mode complete\r\n");
    Delay(1000);

    while(1)
    {
        //Change the velocity base target position of servo ID1 to 150°.
        primary_servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);

        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        receive_len = 0x00;
        Delay(10);

        ret = primary_servo_set_velocity_base_target_position_analysis(receive_data);
        if(ret == PRIMARY_SUCCESS)
            PRINTF("write velocity base target position complete\r\n");
        Delay(1000);

        //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s.
        write_buffer[0] = 3000 & 0xff;
        write_buffer[1] = (3000 >> 8) & 0xff;
        write_buffer[2] = 3600 & 0xff;
        write_buffer[3] = (3600 >> 8) & 0xff;

        primary_servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);

        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        receive_len = 0x00;
        Delay(10);

        PRINTF("write velocity base target position and velocity status packet: ");
        for (uint8_t i = 0; i < receive_len; i++)
        {
            PRINTF("0x%x ", receive_data[i]);
        }
        PRINTF("\r\n");
        Delay(1000);

        //Change the velocity base target position, velocity base target velocity, velocity base target ACC,
        //and velocity base target DEC of servo ID1 to 0° position, 360°/s, 500°/s², and 50°/s², respectively.
        write_buffer[0] = 0 & 0xff;
        write_buffer[1] = (0 >> 8) & 0xff;
        write_buffer[2] = 3600 & 0xff;
        write_buffer[3] = (3600 >> 8) & 0xff;
        write_buffer[4] = 10 & 0xff;
        write_buffer[5] = 1 & 0xff;

        primary_servo_write(1, 0x35, 6, write_buffer, order_buffer, &order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);

        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        receive_len = 0x00;
        Delay(10);

        PRINTF("write velocity base target acc, dec, velocity and position status packet: ");
        for (uint8_t i = 0; i < receive_len; i++)
        {
            PRINTF("0x%x ", receive_data[i]);
        }
        PRINTF("\r\n");
        Delay(1000);

        //In velocity base position control mode, let servo ID1 move to the 150° midpoint and let servo ID2 move to the 0° position.
        servo.position[0] = 1500;
        servo.position[1] = 0;

        primary_servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);

        PRINTF("sync write velocity base target position complete\r\n");
        Delay(1000);

        //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s,
        //and let servo ID2 move to the 150° position at a velocity base target velocity of 720°/s.
        servo.velocity[0] = 3600;
        servo.velocity[1] = 7200;
        servo.position[0] = 3000;
        servo.position[1] = 1500;

        primary_servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);
        PRINTF("sync write velocity base target position and velocity complete\r\n");
        Delay(1000);

        //In velocity base position control mode, let servo ID1 move to the 0° position at a velocity base target velocity of 720°/s, a velocity base target ACC of 500°/s², and a velocity base target DEC of 50°/s².
        //Let servo ID2 move to the 300° position at a velocity base target velocity of 360°/s, a velocity base target ACC of 50°/s², and a velocity base target DEC of 500°/s².
        servo.velocity[0] = 7200;
        servo.velocity[1] = 3600;
        servo.position[0] = 0;
        servo.position[1] = 3000;
        servo.acc_velocity[0] = 10;
        servo.acc_velocity[1] = 1;
        servo.dec_velocity[0] = 1;
        servo.dec_velocity[1] = 10;

        primary_servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);
        PRINTF("sync write velocity base target acc, dec, velocity and position complete\r\n");
        Delay(1000);
    }
}

void RCC_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //USART1   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1   PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1 DIR PA.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART2_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART2_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void USART1_Init()
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    USART_InitStructure.USART_BaudRate = 1000000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    //Enable USART1
    USART_Cmd(USART1, ENABLE);
}

void USART2_Init()
{
    USART_InitTypeDef USART_InitStructure;

    USART_DeInit(USART2);

    USART_InitStructure.USART_BaudRate = 1000000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void USART1_Send(uint8_t *data, uint8_t data_len)
{
    for(uint8_t i = 0; i < data_len; i++)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, data[i]);
    }
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void SysTick_Configuration(void)
{
    /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1);
    }
    /* Set SysTick Priority to 3 */
    NVIC_SetPriority(SysTick_IRQn, 0x0C);
}

void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);
}
