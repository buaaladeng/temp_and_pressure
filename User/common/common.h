#ifndef _COMMON_H_
#define _COMMON_H_

#include "stm32f10x.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define DEBUG_TEST 1   //屏蔽打印功能

void gotoSleep(uint16_t UploadPeriod);

#define  PowerON_GPRS()               GPIO_SetBits(GPIOA,GPIO_Pin_7)   //Power_GPRS(433)_EN引脚拉高
#define  PowerOFF_GPRS()              GPIO_ResetBits(GPIOA,GPIO_Pin_7) //Power_GPRS(433)_EN引脚拉低
#define  PowerON_UltrasonicSensor()   GPIO_SetBits(GPIOA,GPIO_Pin_8)   //Power_12V_EN引脚拉高
#define  PowerOFF_UltrasonicSensor()  GPIO_ResetBits(GPIOA,GPIO_Pin_8) //Power_12V_EN引脚拉低
#define  PowerON_485()                GPIO_SetBits(GPIOA,GPIO_Pin_11)  //Power_485_EN引脚拉高
#define  PowerOFF_485()               GPIO_ResetBits(GPIOA,GPIO_Pin_11)//Power_485_EN引脚拉低
#define  PowerON_Flash()              GPIO_SetBits(GPIOA,GPIO_Pin_12)  //Power_Flash_EN引脚拉高
#define  PowerOFF_Flash()             GPIO_ResetBits(GPIOA,GPIO_Pin_12)//Power_Flash_EN引脚拉低


#endif

