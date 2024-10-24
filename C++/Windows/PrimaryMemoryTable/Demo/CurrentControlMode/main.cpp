#include <iostream>
#include "PrimaryServo.h"
#include "CSerialPort.h"

int main()
{
    uint8_t ret;                        //Status Flag
    uint8_t order_buffer[20] = { 0 };     //Store Generated Instructions
    uint8_t order_len = 0;              //Instruction Length
    uint8_t pack[20] = { 0 };             //Store the received status packet

    CSerialPort serialPort;             //Create a serial port class.

    DWORD bytesRead;                    //The actual number of bytes read from the serial port.
    DWORD bytesWritten;                 //The actual number of bytes written to the serial port.

    if (serialPort.Open(18, 1000000))
    {
        PRINTF("open serial complete\r\n");
    }
    else
    {
        PRINTF("failed to open serial port\r\n");
        return -1;
    }

    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the control mode of servo ID1 to the current control mode.
    primary_servo_set_control_mode(1, 2, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_control_mode_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write control mode complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the target PWM of servo ID1 to 100mA.
    primary_servo_set_target_current(1, 100, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_target_current_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write target current complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(3000);

    serialPort.Close();

    return 0;
}
