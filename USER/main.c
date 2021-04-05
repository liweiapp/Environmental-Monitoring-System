#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "rs485.h"


int main(void)
{
    u8 len = 11;
    u8 i = 0, t = 0;
    u16 humidity, temperature1, temperature2;
    u32 atmosphere,ppm;
    u8 rs485buf[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x03, 0xe1, 0xcb};
		u8 rs485buf1[8] = {0x01, 0x03, 0x00, 0x0b, 0x00, 0x01, 0xf5, 0xc8};
    u8 rs485buf0[11];
		u8 rs485buf2[7];
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
    delay_init(168);   //初始化延时函数
    uart_init(115200);	//初始化串口波特率为115200

    LED_Init();					//初始化LED
    LCD_Init();					//LCD初始化
    RS485_Init(9600);		//初始化RS485串口2
    POINT_COLOR = RED; //设置字体为红色
    LCD_ShowString(60, 50, 200, 16, 16, "Environmental Monitoring");
    POINT_COLOR = BLUE; //设置字体为蓝色
    LCD_ShowString(65, 110, 200, 16, 16, "Temperature:        C");
    LCD_ShowString(90, 150, 200, 16, 16, "Humidity:        %");
    LCD_ShowString(75, 190, 200, 16, 16, "Atmosphere:         Pa");
    LCD_ShowString(75, 230, 200, 16, 16, "Fire Alarm:         PPM");
    POINT_COLOR = BLACK; //设置字体为黑色
    LCD_ShowString(90, 400, 200, 16, 16, "Copyright (C) 2021");
    LCD_ShowString(40, 430, 300, 16, 16, "Wuhan Institute of Technology");

    while(1)
    {
        delay_ms(100);

//        for(i = 0; i < 8; i++)
//        {
//            LCD_ShowxNum(30 + i * 32, 270, rs485buf[i], 3, 16, 0X80);
//        }

        RS485_Send_Data(rs485buf, 8); //发送5个字节
        RS485_TX_EN = 0;				//设置为接收模式
        delay_ms(100);
        RS485_Receive_Data(rs485buf0, &len);
			
			
			  if(len)//接收到有数据
        {
            if(len > 11)
            {
                len = 11; //最大是5个数据.
            }

            humidity = Get_Humidity(rs485buf0);
            temperature1 = Get_Temperature1(rs485buf0);
            temperature2 = Get_Temperature2(rs485buf0);
            atmosphere = Get_Atmosphere(rs485buf0);
            LCD_ShowNum(170, 150, humidity, 4, 16);
            LCD_ShowNum(180, 110, temperature1, 2, 16);
            LCD_ShowString(200, 110, 8, 16, 16, ".");
            LCD_ShowNum(210, 110, temperature2, 1, 16);
            LCD_ShowNum(160, 190, atmosphere, 8, 16);

        }
				
			  delay_ms(100);
			
        RS485_Send_Data(rs485buf1, 8); //发送5个字节
        RS485_TX_EN = 0;				//设置为接收模式
        delay_ms(100);
        RS485_Receive_Data(rs485buf2, &len);
				
				if(len)//接收到有数据
        {
            if(len > 7)
            {
                len = 7; //最大是5个数据.
            }

            ppm = Get_FireAlarm(rs485buf2);
            LCD_ShowNum(160, 230, ppm, 8, 16);

        }


        t++;
        delay_ms(10);

        if(t == 10)
        {
            LED0 = !LED0; //提示系统正在运行
            t = 0;
        }
    }
}
