/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����жϽ��ղ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO-MINI STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
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

extern  u32  WorkingTime ;                 //�豸����ʹ�õ�ʱ��
extern struct DeviceSet  DeviceConfig;     //Һλ��������Ϣ�ṹ��

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
  PWR_WakeUpPinCmd(ENABLE); //ʹ��WAKE-UP�ܽ�
	if(UploadPeriod<1) 
	{
		UploadPeriod =1;                       //�����ϴ����ڣ���λ������
	}
//	Delay_ms(800);  
//	PowerOFF_GPRS();                       //�ر�GPRSģ���Դ
//	Delay_ms(100);
  PowerOFF_UltrasonicSensor();             //�رճ�����̽ͷ��Դ
	Delay_ms(100);
  PowerOFF_485();                          //�ر�485��Դ
	Delay_ms(100);
  PowerOFF_Flash();                        //�ر�Flash��Դ
	Delay_ms(100);
	GPIO_SetBits(GPIOC,GPIO_Pin_5);          //433ģ��SET�ܽ�����
	Delay_ms(100);
  GPIO_SetBits(GPIOB,GPIO_Pin_0);          //433ģ��EN�ܽ����ߣ���������ģʽ
	Delay_ms(100);
	WorkingTime =RTC_GetCounter()-WorkingTime;//��Լ56s
	if((60*UploadPeriod)<=WorkingTime)
	{
    RTC_SetAlarm(RTC_GetCounter()+(60*UploadPeriod));   //�ɼ�ʱ�䵽��ʼ����
  }
	else
	{
    RTC_SetAlarm(RTC_GetCounter()+(60*UploadPeriod)-WorkingTime);   //�ɼ�ʱ�䵽��ʼ����
  }
	RTC_WaitForLastTask();

  #if DEBUG_TEST	 
	printf("\r\nSLEEP OK!\r\n%dmin later wakeup!!", UploadPeriod);	        
	#endif
	PWR_EnterSTANDBYMode();	
	
}

