/**
  ******************************************************************************
  * @file    main.c
  * @author  casic 203
  * @version V1.0
  * @date    2015-07-27
  * @brief   
  * @attention
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "modbus.h"
#include "bsp_SysTick.h"
#include "string.h"
#include "gprs.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "AiderProtocol.h"
#include "SPI_Flash.h"
#include "common.h"
#include "DS2780.h"
#include "433_Wiminet.h"


struct    rtc_time        systmtime;           //RTC时钟设置结构体
struct    DeviceSet       DeviceConfig ={0x00};//液位计配置信息结构体
struct    Config_RegPara  ConfigData   ={0x00};//定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
u16       WWDOG_Feed =0x1FFF;                  //窗口看门狗复位周期为：XX*1.8s = 7.6min
char      PowerOffReset =0;                    //掉电重启标志位
u8        LiquidDataSend_Flag =0;
u8        DataCollectBkCount =0;               //备份数据采集计数器
//float     LevelData_Float[FILTER_ORDER]={0.0}; //采集到的临时液位数据，浮点型
extern    uint8_t  LevelDataCount ;            //液位数据计数器，标示当前采集到数据数量
u8        DataCollectCount =1;                 //数据采集计数器
char      Usart1_recev_buff[100] ={'\0'};      //USART1接收缓存
u16       Usart1_recev_count =0;               //USART1发送计数器
//char      Usart2_recev_buff[50]={'\0'};      //USART2接收缓存
//uint8_t   Usart2_recev_count =0;             //USART2接收计数器
u32       WorkingTime =0;                      //设备运行使用的时间
u8        DMA_UART3_RECEV_FLAG =0;             //USART3 DMA接收标志变量
u8        Usart2_send_buff[SENDBUFF_SIZE]={'\0'};       //USART2发送缓存
u8        DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE] ={0x00};
static    struct DMA_USART3_RecevConfig  DMA_USART3_RecevIndicator; 

extern    char   Usart3_recev_buff[RECEIVEBUFF_SIZE];
extern    u16    Usart3_recev_count;
extern   volatile unsigned char Uart4_rev_count;        //RS485串口接收计数器
//uint32_t  time=0 ;                   // ms 计时变量  
//char      Usart1_send_buff[300]={'\0'};       //USART1发送缓存
//uint8_t   Usart1_send_count=0;                 //USART1发送计数器
//uint32_t  Tic_IWDG=0;                //独立看门狗喂狗时间设置
//extern  char  Usart3_send_buff[];
//extern  uint8_t  Usart3_send_count;                     
//extern  struct  Config_RegPara   ConfigData;  //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
//extern  float  Data_Liquid_Level;

extern void     Delay(uint32_t nCount);
//extern void     LSLIQUID_DataCollect( struct LiquidData* pLevel, u8* pDevID, u16 NodeAddr);          
extern void     RecvBuffInit_USART3(void);

u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3接收数据监测与数据解析
int  DMA_UART3_RecevDataGet(void);


/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : 从DMA接收存储器中提取有效数据，放入Usart3_recev_buff[],便于后续数据解析
* Input          : None
* Output         : None
* Return         : 接收数据长度
*******************************************************************************/
int  DMA_UART3_RecevDataGet(void)
{
   int i=0,j=0;
	 u16 DMA_RecevLength =0;
	
	 memset(Usart3_recev_buff, 0x00, sizeof(Usart3_recev_buff));
	 DMA_USART3_RecevIndicator.CurrentDataStartNum = DMA_USART3_RecevIndicator.NextDataStartNum ;
	  
	 i = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
	 if(DMA_USART3_RecevIndicator.DMA_RecevCount <i)
	 {
     DMA_RecevLength =i -DMA_USART3_RecevIndicator.DMA_RecevCount;
   }
	 else
	 {
     DMA_RecevLength = RECEIVEBUFF_SIZE -DMA_USART3_RecevIndicator.DMA_RecevCount + i;
   }
   DMA_USART3_RecevIndicator.DMA_RecevCount = i;
	
	 if((DMA_USART3_RecevIndicator.CurrentDataStartNum + DMA_RecevLength-1) < RECEIVEBUFF_SIZE)
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1;     
   }
	 else
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =(DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1) -RECEIVEBUFF_SIZE;  
   }
	 printf("\r\nDMA UART2 Recev Data Start Num:%d---End Num: %d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //测试使用
	 if(DMA_USART3_RecevIndicator.CurrentDataEndNum ==(RECEIVEBUFF_SIZE-1))
	 {
	   DMA_USART3_RecevIndicator.NextDataStartNum = 0;
   }
	 else
	 {
		 DMA_USART3_RecevIndicator.NextDataStartNum = DMA_USART3_RecevIndicator.CurrentDataEndNum + 1;
   }	
   //////////////////////////Data Copy///////////////////////////////////////////////////////////////////
   if(DMA_RecevLength !=0)
	 {
     j =DMA_USART3_RecevIndicator.CurrentDataStartNum;
		 if(DMA_USART3_RecevIndicator.CurrentDataEndNum >DMA_USART3_RecevIndicator.CurrentDataStartNum)
		 {
			 for(i=0; i<DMA_RecevLength; i++,j++)
			 {
					Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];	
			 }
		 }
		 else
		 {
			 for(i=0; i<DMA_RecevLength; i++)
			 {
					if( j<(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j++;				
					}
					else if( j==(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j =0;				 
					}
			  } 
      }
    }
	  return DMA_RecevLength;
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress)
{
	int DataLength =0;
	int i=0;
	u16 StateFlag =0;
	
  if(DMA_UART3_RECEV_FLAG==1)
  {
//		 DMA_Cmd(DMA1_Channel6, DISABLE);           //关闭DMA防止处理期间有数据
//		 USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);   //向USART2发送数据前，先打开USART2接收空闲中断，便于监测数据接收完成
		 DataLength = DMA_UART3_RecevDataGet();
//		 DMA_UART3_RECEV_FLAG =0;
//		 DMA_Cmd(DMA1_Channel6, ENABLE);            //开启DMA 
		 if(DataLength>0)
		 {
				printf("\r\nDataLength:%d\r\n", DataLength);             //测试使用
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);           //测试使用
			  for(i=0;i<DataLength;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);               //测试使用
        }
        StateFlag =Receive_Data_Analysis(pDeviceID, sNodeAddress);	      //服务器下发数据解析    //
        //对接收数据类型进行指示 
      	//			
		 }
		 else
		 {
        printf("\r\nNo data\r\n");
     }
		 DMA_Cmd(DMA1_Channel6, DISABLE);           //关闭DMA防止处理期间有数据
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //复位DMA数据接收BUFF
     DMA_UART3_RECEV_FLAG =0;
		 DMA_Cmd(DMA1_Channel6, ENABLE);            //开启DMA 
		 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //向USART2发送数据前，先打开USART2接收空闲中断，便于监测数据接收完成
  } 
	return StateFlag;
}

/*******************************************************************************
功能：去除最大最小值，对剩余数据求平均
输入：浮点数数组，处理数据数量
返回：处理后的平均值
编写：
编写日期：XX年XX月XX日
版本：v0.1
********************************************************************************/
float AverageLevelData(float* pLevelData, uint8_t LevelData_Count)
{
	float DataTemp[FILTER_ORDER] ={0.0};
  float Temp = 0.0;
  float AverageData = 0.0;
  float Dvalue = 0.0;          //最大值与最小值之差
  uint8_t  i=0,j=0;
  

  for(i=0;i<LevelData_Count;i++)
  {
    DataTemp[i] = pLevelData[i];
  }
	
  for(i=LevelData_Count-1;i>=1;i--)
	{ 
		for(j=LevelData_Count-1;j>=LevelData_Count-i;j--)    //从小到大顺序排序
		{
       if(DataTemp[j] < DataTemp[j-1])
			 {
          Temp = DataTemp[j-1];
				  DataTemp[j-1] = DataTemp[j];
				  DataTemp[j] =  Temp;
       }
    }
  }
	for(i=0;i<LevelData_Count;i++)
	{
     printf("##%f##",DataTemp[i]);    //测试使用
  }
	
	Temp =0;                            //复位累加器
	Dvalue =DataTemp[LevelData_Count-1]-DataTemp[0];
	if(Dvalue < 0.025)                   //当最大值与最小值之差在2.5cm以内时，去除4个偏差较大值
	{
		for(i=2;i<LevelData_Count-2;i++)
		{
			 Temp = Temp + DataTemp[i];
		}
		AverageData = Temp/(LevelData_Count-4);
		printf("\r\n**%f**\r\n##",Dvalue);    //测试使用
	}
	else                                   //当最大值与最小值之差大于2.5cm时，去除6个偏差较大值
	{
    for(i=3;i<LevelData_Count-3;i++)  
		{
			 Temp = Temp + DataTemp[i];
		}
		AverageData = Temp/(LevelData_Count-6);
  }
	return AverageData;
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void LSLIQUID_DataCollect( struct LiquidData* pLevel, u8* pDevID, u16 NodeAddr)
//{

//			u32       TimCount_Current =0;
//			float     LevelData_Smooth =0.0;
//			float     temp =0.0;
//			u8        RS485_Read_Cmd[6]={0x01,0x03,0x00,0x00,0x00,0x02};    //读液位数据指令（不包括CRC校验）
//			u8        i=0;   //测试使用
//			u8        InvalidData_cnt = 0;

//			#if DEBUG_TEST
//			printf("DataCollectCount:%d\r\n",DataCollectCount);  //测试使用
//			#endif
//			
//			while(LevelDataCount != 0)
//			{
//				memcpy(Uart4_send_buff, RS485_Read_Cmd,sizeof(RS485_Read_Cmd));					
//				Uart4_send_ready_flag = 1;
//				Crc_counter = 6;
//				RS485_Communication();                 //通过485与液位计进行通信	
//				Delay_ms(1000);                        //需要有一个适当的延时,大于1s	
//			  
//				if(Uart4_rev_comflag==1)
//				{
//					RS485_Communication();
//				}
//				InvalidData_cnt++;
//				//若10次收不到有效液位数据，则进入待机休眠，后期可考虑上报液位计故障帧再待机
//				if(InvalidData_cnt >= 25)                 //一次上传数据不会超过12组，因此，正常工作时在单片机休眠之前InvalidData_cnt不会超过20
//				{
//					InvalidData_cnt = 0;
//					gotoSleep(DeviceConfig.UploadCycle);
//				}
//				for(i=0;i<3;i++)                                 //适当增加延时，避免偶然的干扰
//				{
//					Delay_ms(100);                                 
//          if(DMA_UART3_RECEV_FLAG==1)                    //查询数据接收情况
//					{
//						DMA_UART3_RecevDetect(pDevID, NodeAddr);     //查询有无下发配置命令或者配置查询命令
//					}
//        }	
//			}
//			
//			if(DataCollectCount == DeviceConfig.CollectNum)    //第一次采集数据，记录采集时间信息，后续考虑每次是否采用真实时间
//			{
//				pLevel->DataCount =0;                            //对液位数据缓存计数器赋初值                       
////				USART_ITConfig(UART4, USART_IT_RXNE, DISABLE); //数据采集完成以后，关闭UART4接收中断
//				TimCount_Current = RTC_GetCounter();
//				Time_Display(TimCount_Current,&systmtime); 
//						
//			  DeviceConfig.Time_Sec  =systmtime.tm_sec;
//				DeviceConfig.Time_Min  =systmtime.tm_min;
//				DeviceConfig.Time_Hour =systmtime.tm_hour;
//			  DeviceConfig.Time_Mday =systmtime.tm_mday;		
//			  DeviceConfig.Time_Mon  =systmtime.tm_mon;
//			  DeviceConfig.Time_Year =systmtime.tm_year-2000; //对上传年份去基数修正				
////			#if DEBUG_TEST
//				printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //测试使用
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //测试使用
////			#endif
//			}		
////			printf("\r\nLevelDataCount:%d\r\n",LevelDataCount);      //测试使用
////			for(i=0;i<FILTER_ORDER;i++)
////			{
////         printf("--%f--",LevelData_Float[i]);                 //测试使用
////      }
//			LevelData_Smooth = AverageLevelData(LevelData_Float, FILTER_ORDER);  //滤除最大最小值，取得平均
//			////////根据探头安装高度参数，修正液位数据////////////
//			if(DeviceConfig.MountingHeight>0.5)                  //探头安装参数有效
//			{
//         if(DeviceConfig.MountingHeight >LevelData_Smooth)   
//				 {
//            LevelData_Smooth = DeviceConfig.MountingHeight -LevelData_Smooth;  //当探头距离液面高度小于井深，获取真实的被测液体深度，否则数据为探头距离液面的高度
//         }
//      }
//			temp =LevelData_Smooth*1000;
//			temp =(int)temp;                                   //精度控制在3位小数
//			LevelData_Smooth = temp/1000;
//			
//			LevelData_Float[0] =LevelData_Smooth;              //将平滑处理后的液位数据存放在平滑数组LevelData_Float[]的第一个单元，便于后续短信查询使用
//      printf("\r\nSoothData:%f\r\n",LevelData_Smooth);   //测试使用
////			Float2Hex_Aider(LevelData_Smooth);
//			i = pLevel->DataCount;
//			pLevel->CollectData[i] =LevelData_Smooth;
//			if(i==0)
//			{
//        pLevel->CollectTime[i] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);      //第一组数据采集时间
//      }
//		  else
//			{
//        pLevel->CollectTime[i] =(pLevel->CollectTime[i-1])+ DeviceConfig.CollectPeriod;    //后续每组数据采集时间为在前一组数据基础上增加采集间隔
//      }
//			pLevel->DataCount =(pLevel->DataCount)+1;
//     	DataCollectCount--;	
//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float Temperature_DataGet(void)
{
   u8     i=0; 	
   u8     TemperatureReadCmd[8]={0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};           //读温度数据指令,最后两位为CRC校验码
   u8     TemperatureReadResp[3]={0x01,0x03,0x02};   //数据应答指示:01,03,02,T_H,T_L,CrcL,CrcH
   u8*    pUart4Send =  TemperatureReadCmd;
   u8*    pDataRead  =NULL; 
   float  Temper =0;   //温度数据暂存

   Uart4_rev_comflag =0;   //接收标志变量复位
   Uart4_rev_count   =0;   //接收计数器清零
	 memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));
//多采集几次数据，求平均，保证数据稳定可靠

//			LevelData_Smooth = AverageLevelData(LevelData_Float, FILTER_ORDER);  //滤除最大最小值，取得平均
//			////////根据探头安装高度参数，修正液位数据////////////
//			if(DeviceConfig.MountingHeight>0.5)                  //探头安装参数有效
//			{
//         if(DeviceConfig.MountingHeight >LevelData_Smooth)   
//				 {
//            LevelData_Smooth = DeviceConfig.MountingHeight -LevelData_Smooth;  //当探头距离液面高度小于井深，获取真实的被测液体深度，否则数据为探头距离液面的高度
//         }
//      }
//			temp =LevelData_Smooth*1000;
//			temp =(int)temp;                                   //精度控制在3位小数
//			LevelData_Smooth = temp/1000;
//			printf("\r\nLevelDataCount:%d\r\n",LevelDataCount);      //测试使用
//			for(i=0;i<FILTER_ORDER;i++)
//			{
//         printf("--%f--",LevelData_Float[i]);                 //测试使用
//      }

printf("\r\n11\r\n");                 //测试使用

////////////////////////////////////////////////////

    DIR485_Send();                           //485发送使能
		Delay_ms(20);                            //原始3ms

//		for(i=0;i<sizeof(TemperatureReadCmd);i++)
	 for(i=0;i<8;i++)
		{  
			 USART_SendData(UART4, TemperatureReadCmd[i]);
			 pUart4Send++;
			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //等待发送完毕 
		}
		Delay_us(450);                  //在数据接收完成后，延迟一段时间再切换收发控制开关，默认延时350us		
		DIR485_Receive();
		Delay_ms(2000);
		
		for(i=0;i<15;i++) 
		{
      if(Uart4_rev_comflag ==1)
			{
          //解析
				 printf("\r\n22\r\n");                 //测试使用
				 for(i=0;i<Uart4_rev_count;i++)
					{  
						 printf("---%x--",Uart4_rev_buff[i]);                 //测试使用
					}
				
				pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //检查是否收到温度传感器应答
				if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
				{
           Temper =(pDataRead[3]*256 +pDataRead[4])/10;
        	 #if DEBUG_TEST
				   printf("\r\nPercept Temperature is :%f\r\n",Temper);                 //测试使用
     		   #endif
        }
				Uart4_rev_comflag =0;
				
				break;
      }
			else
			{
				Delay_ms(300);   
			}
   }
	 return  Temper;		
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float Pressure_DataGet(void)
{
   u8     i=0; 	
   u8     PressureReadCmd[8]={0x01,0x04,0x00,0x02,0x00,0x02,0xD0,0x0B};           //读温度数据指令,最后两位为CRC校验码
   u8     PressureReadResp[3]={0x01,0x04,0x04};   //数据应答指示:01,03,02,P_1,P_2,P_3,P_4,CrcL,CrcH    //浮点表示
   u8*    pUart4Send = PressureReadCmd;
   u8*    pDataRead  =NULL; 
   float  Pressure =0;   //压力数据暂存
   union  Hfloat  PressureTemp;

   Uart4_rev_comflag =0;   //接收标志变量复位
   Uart4_rev_count   =0;   //接收计数器清零
	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));
//多采集几次数据，求平均，保证数据稳定可靠

    DIR485_Send();                           //485发送使能
		Delay_ms(20);                            //原始3ms

		for(i=0;i<sizeof(PressureReadCmd);i++)
		{  
			 USART_SendData(UART4, *pUart4Send);
			 pUart4Send++;
			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //等待发送完毕 
		}
		Delay_us(650);                  //在数据接收完成后，延迟一段时间再切换收发控制开关，默认延时200us		
		DIR485_Receive();
		
		for(i=0;i<10;i++)
		{
      if(Uart4_rev_comflag ==1)
			{
          //解析
				pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)PressureReadResp, sizeof(Uart4_rev_buff), sizeof(PressureReadResp));  //检查是否收到温度传感器应答
				if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-10)))
				{
           for(i=0;i<4;i++)
					 {
              PressureTemp.Data_Hex[i] =pDataRead[3+i];
           }
					 Pressure =PressureTemp.Data_Float;
        	 #if DEBUG_TEST
				   printf("\r\nPercept Temperature is :%f\r\n", Pressure );                 //测试使用
     		   #endif
        }
				Uart4_rev_comflag =0;
				break;

      }
			else
			{
				Delay_ms(300);   
			}
   }
	 return  Pressure;		
}		
		
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SenserDataCollect(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
{

			u32       TimCount_Current =0;
//			float     LevelData_Smooth =0.0;
//			float     temp =0.0;
//			u8        RS485_Read_Cmd[6]={0x01,0x03,0x00,0x00,0x00,0x02};    //读液位数据指令（不包括CRC校验）
			u8        i=0;   //测试使用


			#if DEBUG_TEST
			printf("DataCollectCount:%d\r\n",DataCollectCount);  //测试使用
			#endif
			
	
			if(DataCollectCount == DeviceConfig.CollectNum)    //第一次采集数据，记录采集时间信息，后续考虑每次是否采用真实时间
			{
				pGetData->DataCount =0;                            //对液位数据缓存计数器赋初值                       
//				USART_ITConfig(UART4, USART_IT_RXNE, DISABLE); //数据采集完成以后，关闭UART4接收中断
				TimCount_Current = RTC_GetCounter();
				Time_Display(TimCount_Current,&systmtime); 
						
			  DeviceConfig.Time_Sec  =systmtime.tm_sec;
				DeviceConfig.Time_Min  =systmtime.tm_min;
				DeviceConfig.Time_Hour =systmtime.tm_hour;
			  DeviceConfig.Time_Mday =systmtime.tm_mday;		
			  DeviceConfig.Time_Mon  =systmtime.tm_mon;
			  DeviceConfig.Time_Year =systmtime.tm_year-2000; //对上传年份去基数修正				
//			#if DEBUG_TEST
				printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //测试使用
											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //测试使用
//			#endif
			}		

//			temp =LevelData_Smooth*1000;
//			temp =(int)temp;                                   //精度控制在3位小数
//			LevelData_Smooth = temp/1000;
//			
//			LevelData_Float[0] =LevelData_Smooth;              //将平滑处理后的液位数据存放在平滑数组LevelData_Float[]的第一个单元，便于后续短信查询使用
//      printf("\r\nSoothData:%f\r\n",LevelData_Smooth);   //测试使用
////			Float2Hex_Aider(LevelData_Smooth);
			Temperature_DataGet();
			Delay_ms(500);  
			Temperature_DataGet();
			Delay_ms(500);
			i = pGetData->DataCount;
    	pGetData->TemperatureData[i] = Temperature_DataGet();
			Delay_ms(500);
			pGetData->PressureData[i]    = Pressure_DataGet();
//			pGetData->TemperatureData[i] = 25.3;
//			pGetData->PressureData[i]    = 1.0;
			
			if(i==0)
			{
        pGetData->CollectTime[i] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);      //第一组数据采集时间
      }
		  else
			{
        pGetData->CollectTime[i] =(pGetData->CollectTime[i-1])+ DeviceConfig.CollectPeriod;    //后续每组数据采集时间为在前一组数据基础上增加采集间隔
      }
			pGetData->DataCount =(pGetData->DataCount)+1;
     	DataCollectCount--;	

}

		
	


/*******************************************************************************
* Function Name  :Power_SX1278_Init()
* Description    : 初始化433模块
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  Power_SX1278_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* config GPIOA clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
  /* Configure PowerEN_3.8V(PA.07) as output push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}


/*******************************************************************************
* Function Name  : void SX1287_Init(u16 sNodeID)
* Description    : 初始化433模块
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SX1287_Init(u16 sNodeID)
{
  u16  UartConfig   =0;
	u16  CentralFreq  =0;
	u16  NodeID       =0;
	u8   ExpFactor    =0;
	u8   ExpBW        =0;
	u8   WorkMode     =0;
	u8   NetID        =0;
	u8   PowerGrade   =0;
	u8   BreathPeriod =0;
	u8   WakeTime     =0;
  u8   BaudRate     =0;
	u8   VerifyType   =0; 
  u16  Temp =0;	
	
	Power_SX1278_Init();
	////////////////////配置433模块参数 //////////////////// 
//	SetNodeSerialPort(4, 0);                           //设置433模块串口波特率：9600,校验类型:无校验
	SetNodeCentralFrequency(434);                      //设置433模块载波中心频率：434MHz
	SetNodeFrequencyExpandFactor(11);                  //设置433模块扩频因子：2048
	SetNodeFrequencyExpandBandwidth(7);                //设置433模块扩频带宽：125K
	SetNodeWorkMode(2);                                //设置433模块工作模式：节点模式
	SetNodeID (sNodeID);                               //设置433模块ID：输入节点ID
	SetNodeNetworkID (1);                              //设置433模块网络ID：0
	SetNodeSendPowerGrade (7);                         //设置433模块发射功率等级：20dBm
	SetNodeBreathPeriod (0);                           //设置433模块呼吸周期:2s
	SetNodeBreathTime (5);                             //设置433模块呼吸时间：64ms
	
	PowerOFF_GPRS();                                   //重启433模块，使配置生效        
	Delay_ms(5000);
	PowerON_GPRS(); 
	////////////////////读取433模块参数 //////////////////// 
//u8  GetNodeReceiveSignalEnergy ();                 //获取433模块上一帧数据接收信号强度
  UartConfig =GetNodeSerialPortConfig();              //获取433模块串口配置参数
  BaudRate   =UartConfig>>8;
  VerifyType =UartConfig &0xFF;
  printf("\r\nBaud Rate:%d---Verify Type:%d\r\n",BaudRate,VerifyType);//测试使用
  CentralFreq =GetNodeCentralFrequency ();               //获取433模块载波频率参数
  printf("\r\nCentral Frequency :%d MHz\r\n",(CentralFreq+1));   //测试使用
  ExpFactor   =GetNodeFrequencyExpandFactor();           //获取433模块扩频因子参数
  switch(ExpFactor )
  {
			case 7:  
			{
				Temp=128;
				break;
			}
			case 8:  
			{
				Temp=256;
				break;
			}
			case 9:  
			{
				Temp=512;
				break;
			}		
			case 10:  
			{
				Temp=1024;
				break;
			}
			case 11:  
			{
				Temp=2048;
				break;
			}	
			case 12:  
			{
				Temp=4096;
				break;
			}
			default:
			{
				Temp=0;
				break;
			}		
  }
  printf("\r\nNode Frequency Expand Factor:%d \r\n",Temp);   //测试使用
  ExpBW =GetNodeFrequencyExpandBandwidth ();                 //获取433模块扩频带宽参数
	switch( ExpBW )
  {
			case 6:  
			{
				Temp=63;       //62.5约等于63
				break;
			}
			case 7:  
			{
				Temp=125;
				break;
			}
			case 8:  
			{
				Temp=256;
				break;
			}		
			case 9:  
			{
				Temp=512;
				break;
			}
			default:
			{
				Temp=0;
				break;
			}		
  }
	printf("\r\nNode Frequency Expand Bandwidth:%dKHz\r\n",Temp);   //测试使用
  WorkMode = GetNodeWorkMode ();                                  //获取433模块工作模式参数
	switch( WorkMode )
  {
			case 0:  
			{
	      printf("\r\n433 Module Work Mode is: Standard\r\n");   //测试使用
				break;
			}
			case 1:  
			{
				printf("\r\n433 Module Work Mode is: Center\r\n");    //测试使用
				break;
			}
			case 2:  
			{
				printf("\r\n433 Module Work Mode is: Node\r\n");    //测试使用
				break;
			}		
			default:
			{
				printf("\r\n433 Module Work Mode is: Unknown\r\n");    //测试使用
				break;
			}		
  }
  NodeID =GetNodeID ();                                 //获取433模块节点ID
	printf("\r\n433 Module Node ID is: %x\r\n",NodeID);   //测试使用
  NetID =GetNetworkID ();                               //获取433模块网络ID
	printf("\r\n433 Module Network ID is: %x\r\n",NetID); //测试使用
  PowerGrade = GetNodeSendPowerGrade ();                //获取433模块发射功率
	printf("\r\n433 Module Send Power Grade is: %d\r\n",PowerGrade); //测试使用
  BreathPeriod = GetNodeBreathPeriod ();                //获取433模块呼吸周期
	switch( BreathPeriod )
  {
			case 0:  
			{
				Temp=2;       
				break;
			}
			case 1:  
			{
				Temp=4;
				break;
			}
			case 2:  
			{
				Temp=6;
				break;
			}		
			case 3:  
			{
				Temp=8;
				break;
			}
			case 4:  
			{
				Temp=10;
				break;
			}
			default:
			{
				Temp=0;
				break;
			}		
  }
	printf("\r\nNode Breath Period:%d s\r\n",Temp );   //测试使用
  WakeTime  =  GetNodeBreathTime ();                 //获取433模块呼吸时间
	switch( WakeTime )
  {
			case 0:  
			{
				Temp=2;       
				break;
			}
			case 1:  
			{
				Temp=4;
				break;
			}
			case 2:  
			{
				Temp=8;
				break;
			}		
			case 3:  
			{
				Temp=16;
				break;
			}
			case 4:  
			{
				Temp=32;
				break;
			}
			case 5:  
			{
				Temp=64;
				break;
			}			
			default:
			{
				Temp=0;
				break;
			}		
  }
	printf("\r\nNode Wake Time:%d ms\r\n",Temp );   //测试使用

}
/*******************************************************************************
* Function Name  : void PeripheralInit( void )
* Description    : 初始化端口及外设
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PeripheralInit(void)
{
	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//首先开启窗口看门狗，计数器值为7f,窗口寄存器为5f,分频数为8	
	USART1_Config();      /* USART1 配置模式为 9600 8-N-1，  中断接收 */
	USART2_Config();      /* USART2 配置模式为 9600 8-N-1，中断接收 */
	USART3_Config();      /* USART3 配置模式为 9600 8-N-1，中断接收 */
	USART4_Config();      /* UART4  配置模式为 9600 8-N-1，  中断接收 */
  UART_NVIC_Configuration();
//	USART3_DMA_Config();
  USART2_DMA_Config();
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //配置串口DMA1接收
//  TIM2_Configuration();       /* 定时器TIM2参数配置 */	
//	TIM2_NVIC_Configuration();  /* 设置定时器TIM2的中断优先级 */
	TIM3_Configuration();       /* 定时器TIM3参数配置 */	
	TIM3_NVIC_Configuration();  /* 设置定时器TIM3的中断优先级 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //暂时关闭定时器TIM3
	
	RTC_NVIC_Config();                 /* 配置RTC秒中断优先级 */
	RTC_CheckAndConfig(&systmtime);
//	Time_Show(&systmtime);             /* Display time in infinite loop */
	WorkingTime =RTC_GetCounter();     //采集设备启动时间
  SysTick_Init();
  PowerON_Flash();                   //打开Flash电源 
  Delay_ms(100); 	
//	PowerON_GPRS();                  //打开GPRS模块电源，****必须打开，查明原因
//	Delay_ms(100);
	PowerON_UltrasonicSensor();        //打开超声波探头电源，在TCP连接正确建立以后再开启探头电源，后续如果需要备份历史数据，则需要做相应修改
	Delay_ms(500);   
  PowerON_485();                     //打开485电源
	Delay_ms(500);
  Uart4_rev_comflag=0;               //清除液位计刚启动时串口的乱码,考虑是否有必要
	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));
	
 	DMA_USART3_RecevIndicator.CurrentDataStartNum =0;  //初始化当前接收数据开始位置
	DMA_USART3_RecevIndicator.CurrentDataEndNum =0;    //初始化当前接收数据结束位置
	DMA_USART3_RecevIndicator.NextDataStartNum =0;     //初始化下一次接收数据开始位置
	DMA_USART3_RecevIndicator.DMA_RecevCount =0;
	///////////////////必须将模块配置成接收状态，否则串口接收不到数据
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                  //433模块SET管脚拉高，切换到接收模式
	Delay_ms(100);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                //433模块EN管脚拉低，切换到高速模式
	Delay_ms(100);
	USART_GetFlagStatus(USART2,USART_FLAG_TC);       //串口硬件复位之后，发送首字节之前，先读一下USART_SR,防止数据发送时首字节被覆盖
	Delay_ms(500);
	USART_GetFlagStatus(UART4,USART_FLAG_TC);       //串口硬件复位之后，发送首字节之前，先读一下USART_SR,防止数据发送时首字节被覆盖
	Delay_ms(500);
}


/*******************************************************************************
* Function Name  : int main(void)
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

int main( void )
{
//	#ifdef DEBUG
//  debug();
//  #endif	

	u8   i=0;
	u8  DeviceID[6] ={0x81,0x20,0x16,0x04,0x80,0x01}; //初始化设备ID号
  u16 NodeAddr =0x0000;
  struct SenserData   PerceptionData;         //传感器数据

	NodeAddr =DeviceID[4]*256 +DeviceID[5];     //提取设备ID号最后面两个字节作为节点地址
  PeripheralInit();                           //初始化外设
  ConfigData_Init(&DeviceConfig);             //初始化以后才能接收查询参数命令
  if (PowerOffReset ==1)          
  {
    printf("\r\n掉电重启，重新初始化库仑计\r\n");                 //测试使用
		Set_register_ds2780();    //掉电后对库仑计重新初始化
	  set_ACR(1000);            //掉电后对库仑计重新初始化
	  DS2780_CapacityInit();    //掉电后重新写电池容量
		DS2780_Test();            //掉电后将电池容量系数重新写入Flash
//		SX1287_Init(NodeAddr);
	  //////设备上电注册///////////
		Delay_ms(2000);   
		DeviceStartupRequest(Usart2_send_buff, DeviceID, NodeAddr);  
  }                
  PerceptionData.DataCount =0;                        //初始化液位数据计数器
	DataCollectCount = DeviceConfig.CollectNum;        //获取数据采集数量参数
	for(i=0;i<20;i++)
	{
		 Delay_ms(200);                                //参考探头上电后数据稳定时间，作相应调整
		 if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
		 {
				DMA_UART3_RecevDetect(DeviceID, NodeAddr);  
				break;
		 }
  }	
	while(1)
  {
		
		WWDOG_Feed =0x1FFF;                              //窗口看门狗喂狗,定时4分20秒，第二字节约1秒变化一次，即0x09AF变到0x099F约耗时1秒		
		for(i=0;i<5;i++)
		{
			 Delay_ms(200);                                //参考探头上电后数据稳定时间，作相应调整
			 if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			 {
				 DMA_UART3_RecevDetect(DeviceID, NodeAddr);  
				 break;
			 }
    }	
		if(DataCollectCount!=0)
		{
			 if(LevelDataCount == 0)
			 {
          LevelDataCount = FILTER_ORDER;             //对于需要一次上传多组数据时，需要给采集频率控制变量赋初值
       }
			 SenserDataCollect(&PerceptionData, DeviceID, NodeAddr);        //获取压力数据和温度数据
			 
		}
		else 
		{			
			 PowerOFF_UltrasonicSensor();                  //当液位数据采集完成时，关闭超声波探头电源
			 Delay_ms(100); 
       PowerOFF_485();                               //当液位数据采集完成时，关闭Uart转485模块电源
			 Delay_ms(100);
			 DeviceConfig.BatteryCapacity = DS2780_Test(); //监测电池电量
			 TrapRequest(Usart2_send_buff, DeviceID, NodeAddr, &PerceptionData);  //发送液位数据
			 LiquidDataSend_Flag=1;                        //液位数据成功发送到服务器标志变量
		}	
		if(LiquidDataSend_Flag ==1)            
		{ 
			 gotoSleep(DeviceConfig.UploadCycle);
		}		
	}  
}
#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number */
 
  printf("\n\r Wrong parameter value detected on\r\n");
  printf("       file  %s\r\n", file);
  printf("       line  %d\r\n", line);
    
  /* Infinite loop */
  /* while (1)
  {
  } */
}
#endif
/*********************************************END OF FILE***************************************/
