#include "stm32f10x.h"
#include "platform_config.h"
#include "PrimaryServo.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

uint8_t receive_data[20];               //Store the received status packet
uint8_t receive_len;                    //received Length
uint8_t ret;                            //Status Flag

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

    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer,&order_len);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    USART1_Send(order_buffer, order_len);

    GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    receive_len = 0x00;
    Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive_data);
    if(ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");
    Delay(1000);

    //Change the control mode of servo ID1 to the current control mode.
    primary_servo_set_control_mode(1, 2, order_buffer,&order_len);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    USART1_Send(order_buffer, order_len);

    GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    receive_len = 0x00;
    Delay(10);

    ret = primary_servo_set_control_mode_analysis(receive_data);
    if(ret == PRIMARY_SUCCESS)
        PRINTF("write control mode complete\r\n");
    Delay(1000);

    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer,&order_len);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    USART1_Send(order_buffer, order_len);

    GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    receive_len = 0x00;
    Delay(10);

    ret = primary_servo_set_torque_switch_analysis(receive_data);
    if(ret == PRIMARY_SUCCESS)
        PRINTF("write torque switch complete\r\n");
    Delay(1000);

    //Change the target PWM of servo ID1 to 100mA.
    primary_servo_set_target_current(1, 100, order_buffer,&order_len);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    USART1_Send(order_buffer, order_len);

    GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    receive_len = 0x00;
    Delay(10);

    ret = primary_servo_set_target_pwm_analysis(receive_data);
    if(ret == PRIMARY_SUCCESS)
        PRINTF("write target current complete\r\n");
    Delay(3000);
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
