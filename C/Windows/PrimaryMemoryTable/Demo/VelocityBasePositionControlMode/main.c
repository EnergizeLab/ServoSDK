#include "PrimaryServo.h"
#include <windows.h>
#include <stdio.h>

//uart init
uint8_t uart_init(HANDLE hSerial)
{
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        PRINTF("failed to open serial port\n");
        return FALSE;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        PRINTF("failed to get serial port parameters\n");
        CloseHandle(hSerial);
        return FALSE;
    }

    dcbSerialParams.BaudRate = 1000000;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        PRINTF("Failed to set serial port parameters\n");
        CloseHandle(hSerial);
        return FALSE;
    }

    return TRUE;

}

//uart send
uint8_t order_send(HANDLE hSerial, uint8_t* order_buffer, uint8_t order_len)
{
    uint8_t ret;
    DWORD bytesWritten;

    ret = WriteFile(hSerial, order_buffer, order_len, &bytesWritten, NULL);

    if (ret != 0)
    {
        return TRUE;
    }
    else
    {
        PRINTF("send error!\r\n");
        return FALSE;
    }
}

//uart receiver
uint8_t order_receive(HANDLE hSerial, uint8_t pack[])
{
    uint8_t ret;
    DWORD bytesRead;
    DWORD errors;
    DWORD read_len;
    COMSTAT comstat;

    if (!ClearCommError(hSerial, &errors, &comstat)) {
        return FALSE;
    }

    read_len = comstat.cbInQue;

    ret = ReadFile(hSerial, pack, read_len, &bytesRead, NULL);

    if (ret != 0)
    {
        if (bytesRead > 0)
        {
            return bytesRead;
        }
        else
        {
            PRINTF("no response packet data!\r\n");
            return TRUE;
        }
    }
    else
    {
        PRINTF("read error!\r\n");
        return FALSE;
    }
}

int main() {

    uint8_t order_buffer[40] = { 0 };                                      //Store Generated Instructions
    uint8_t order_len = 0;                                                 //Instruction Length
    uint8_t pack[20] = { 0 };                                              //Store the received status packet
    uint8_t ret;                                                           //Status Flag
    uint8_t write_buffer[20] = { 0 };                                      //Write data to the memory table

    struct primary_servo_sync_parameter servo;

    servo.id_counts = 2;            //Sync write two servos
    servo.id[0] = 1;                //Set the ID of the first servo to 1
    servo.id[1] = 2;                //Set the ID of the second servo to 2

    //Open serial
    HANDLE hSerial = CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    //uart init
    ret = uart_init(hSerial);
    if (ret == FALSE)
    {
        return FALSE;
    }

    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    primary_servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("sync write torque witch complete\r\n");
    }
    Sleep(80);

    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("sync write control mode complete\r\n");
    }
    Sleep(80);

    //Change the velocity base target position of servo ID1 to 150°.
    primary_servo_set_velocity_base_target_position(1, 1500, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1000);

    ret = primary_servo_set_velocity_base_target_position_analysis(pack);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write velocity base target position complete\r\n");

    //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s.
    write_buffer[0] = 3000 & 0xff;
    write_buffer[1] = (3000 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;

    primary_servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    PRINTF("write velocity base target position and velocity status packet: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%02x ", pack[i]);
    }
    PRINTF("\r\n");
    Sleep(1000);

    //Change the velocity base target position, velocity base target velocity, velocity base target ACC, and velocity base target DEC of servo ID1 to 0° position, 360°/s, 500°/s², and 50°/s², respectively.
    write_buffer[0] = 0 & 0xff;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;
    write_buffer[4] = 10 & 0xff;
    write_buffer[5] = 1 & 0xff;

    primary_servo_write(1, 0x35, 6, write_buffer, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    PRINTF("write velocity base target acc, dec, velocity and position status packet: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%02x ", pack[i]);
    }
    PRINTF("\r\n");
    Sleep(1000);

    //In velocity base position control mode, let servo ID1 move to the 150° midpoint and let servo ID2 move to the 0° position.
    servo.position[0] = 1500;
    servo.position[1] = 0;

    primary_servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("sync write velocity base target position complete\r\n");
    }
    Sleep(1000);

    //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s,
    //and let servo ID2 move to the 150° position at a velocity base target velocity of 720°/s.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;
    servo.position[0] = 3000;
    servo.position[1] = 1500;

    primary_servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("sync write velocity base target position and velocity complete\r\n");
    }
    Sleep(1000);

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

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write velocity base target acc, dec, velocity and position complete\r\n");
    }
    Sleep(1000);

    //Close serial
    CloseHandle(hSerial);

    return 0;
}
