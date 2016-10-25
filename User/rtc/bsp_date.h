/******************** (C) COPYRIGHT 2009 www.armjishu.com ************************
* File Name          : date.h
* Author             : www.armjishu.com Team
* Version            : V1.0
* Date               : 12/1/2009
* Description        : 日期相关函数
*******************************************************************************/
#ifndef __DATE_H
#define __DATE_H

#include "stm32f10x.h"

struct rtc_time 
{
	u8  tm_sec;
	u8  tm_min;
	u8  tm_hour;
	u8  tm_mday;
	u8  tm_mon;
	u16 tm_year;
	u8  tm_wday;
};
    
void GregorianDay(struct rtc_time * tm);
uint32_t mktimev(struct rtc_time *tm);
void to_tm(uint32_t tim, struct rtc_time * tm);
#endif 
