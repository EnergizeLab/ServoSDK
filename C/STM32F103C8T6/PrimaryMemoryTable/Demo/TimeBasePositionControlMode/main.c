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


    //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    USART1_Send(order_buffer, order_len);
    PRINTF("sync write control mode complete\r\n");
    Delay(1000);

    while(1)
    {
        //Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
        primary_servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);

        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        receive_len = 0x00;
        Delay(10);

        ret = primary_servo_set_time_base_target_position_and_moving_time_analysis(receive_data);
        if(ret == PRIMARY_SUCCESS)
            PRINTF("write time base target position and moving time complete\r\n");
        Delay(1000);

        //Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
        write_buffer[0] = 0;
        write_buffer[1] = 3000 & 0xff;
        write_buffer[2] = (3000 >> 8) & 0xff;
        write_buffer[3] = 1000 & 0xff;
        write_buffer[4] = (1000 >> 8) & 0xff;

        primary_servo_write(1, 0x3B, 5, write_buffer, order_buffer, &order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);

        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        receive_len = 0x00;
        Delay(10);

        PRINTF("write time base target ACC, position and moving time status packet: ");
        for (uint8_t i = 0; i < receive_len; i++)
        {
            PRINTF("0x%x ", receive_data[i]);
        }
        PRINTF("\r\n");
        Delay(1000);

        //let servo ID1 move to the 150° position at a velocity of 500ms,
        //and let servo ID2 move to the 0° position at a constant velocity of 1s.
        servo.position[0] = 1500;
        servo.position[1] = 0;
        servo.time[0] = 500;
        servo.time[1] = 1000;

        primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);
        PRINTF("sync write time base target position and moving time complete\r\n");
        Delay(1000);

        //let servo ID1 move to the 0° position at a velocity of 1s,
        //and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
        servo.position[0] = 0;
        servo.position[1] = 3000;
        servo.time[0] = 1000;
        servo.time[1] = 500;

        primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        USART1_Send(order_buffer, order_len);
        PRINTF("sync write time base target position and moving time complete\r\n");
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
