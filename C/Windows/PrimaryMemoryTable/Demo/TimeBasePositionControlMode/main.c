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
        PRINTF("failed to set serial port parameters\n");
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
            PRINTF("No response packet data!\r\n");
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

    uint8_t order_buffer[20] = { 0 };                                                                       //Store Generated Instructions
    uint8_t order_len = 0;                                                                                  //Instruction Length
    uint8_t pack[20] = { 0 };                                                                               //Store the received status packet
    uint8_t ret;                                                                                            //Status Flag
    uint8_t write_buffer[20] = { 0 };                                                                       //Write data to the memory table
    struct primary_servo_sync_parameter servo;

    //Open Serial
    HANDLE hSerial = CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    //uart init
    ret = uart_init(hSerial);
    if (ret == FALSE)
    {
        return FALSE;
    }

    servo.id_counts = 2;            //Sync write two servos
    servo.id[0] = 1;                //Set the ID of the first servo to 1
    servo.id[1] = 2;                //Set the ID of the second servo to 2

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

    //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
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


    //Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
    primary_servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);

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

    ret = primary_servo_set_time_base_target_position_and_moving_time_analysis(pack);
    if (ret == PRIMARY_SUCCESS)
        PRINTF("write time base target position and moving time complete\r\n");


    //Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
    write_buffer[0] = 0;
    write_buffer[1] = 3000 & 0xff;
    write_buffer[2] = (3000 >> 8) & 0xff;
    write_buffer[3] = 1000 & 0xff;
    write_buffer[4] = (1000 >> 8) & 0xff;

    primary_servo_write(1, 0x3B, 5, write_buffer, order_buffer, &order_len);

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
    PRINTF("write time base target ACC, position, and moving time status packet: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%02x ", pack[i]);
    }
    PRINTF("\r\n");
    Sleep(1000);

    //In time base position control mode, let servo ID1 move to the 150° position at a velocity of 500ms, and let servo ID2 move to the 0° position at a constant velocity of 1s.
    servo.position[0] = 1500;
    servo.position[1] = 0;
    servo.time[0] = 500;
    servo.time[1] = 1000;

    primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("sync write time base target position and moving time complete\r\n");
    }
    Sleep(1000);

    //In time base position control mode, let servo ID1 move to the 0° position at a velocity of 1s, and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
    servo.position[0] = 0;
    servo.position[1] = 3000;
    servo.time[0] = 1000;
    servo.time[1] = 500;

    primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("sync write time base target position and moving time complete\r\n");
    }
    Sleep(1000);

    //Close Serial
    CloseHandle(hSerial);

    return 0;
}
