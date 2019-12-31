# Energize Servo<br>
Energize Servo 的SDK和Demo

## .NET3.5
- ServoLib<br>
基础库，主要包含Servo的整个内存表（Json文件中)、数据打包和解析
- EM3BaseDemo<br>
演示Ping指令、写指令、读指令

## MCU
### STM32F103C8T6<br>
 1. 演示Ping指令、写指令、读指令<br>
 2. 简单的接收数据（USART1 HALF-Duplex）（GPIO A9）<br>
 3. 接收到数据由USART2（GPIO A2）（BSP125000）发送出去<br>
