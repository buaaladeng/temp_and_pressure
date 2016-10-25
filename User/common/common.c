/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO-MINI STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "bsp_SysTick.h"
#include "string.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "common.h"
#include "gprs.h"

extern  u32  WorkingTime ;                 //设备运行使用的时间
extern struct DeviceSet  DeviceConfig;     //液位计配置信息结构体

/*******************************************************************************
* Function Name  : void gotoSleep(uint16_t UploadPeriod)
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void gotoSleep(uint16_t UploadPeriod)
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
  PWR_WakeUpPinCmd(ENABLE); //使能WAKE-UP管脚
	if(UploadPeriod<1) 
	{
		UploadPeriod =1;                       //数据上传周期，单位：分钟
	}
//	Delay_ms(800);  
//	PowerOFF_GPRS();                       //关闭GPRS模块电源
//	Delay_ms(100);
  PowerOFF_UltrasonicSensor();             //关闭超声波探头电源
	Delay_ms(100);
  PowerOFF_485();                          //关闭485电源
	Delay_ms(100);
  PowerOFF_Flash();                        //关闭Flash电源
	Delay_ms(100);
	GPIO_SetBits(GPIOC,GPIO_Pin_5);          //433模块SET管脚拉高
	Delay_ms(100);
  GPIO_SetBits(GPIOB,GPIO_Pin_0);          //433模块EN管脚拉高，进入休眠模式
	Delay_ms(100);
	WorkingTime =RTC_GetCounter()-WorkingTime;//大约56s
	if((60*UploadPeriod)<=WorkingTime)
	{
    RTC_SetAlarm(RTC_GetCounter()+(60*UploadPeriod));   //采集时间到开始唤醒
  }
	else
	{
    RTC_SetAlarm(RTC_GetCounter()+(60*UploadPeriod)-WorkingTime);   //采集时间到开始唤醒
  }
	RTC_WaitForLastTask();

  #if DEBUG_TEST	 
	printf("\r\nSLEEP OK!\r\n%dmin later wakeup!!", UploadPeriod);	        
	#endif
	PWR_EnterSTANDBYMode();	
	
}

