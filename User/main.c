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


struct    rtc_time        systmtime;           //RTCʱ�����ýṹ��
struct    DeviceSet       DeviceConfig ={0x00};//Һλ��������Ϣ�ṹ��
struct    Config_RegPara  ConfigData   ={0x00};//������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
u16       WWDOG_Feed =0x1FFF;                  //���ڿ��Ź���λ����Ϊ��XX*1.8s = 7.6min
char      PowerOffReset =0;                    //����������־λ
u8        LiquidDataSend_Flag =0;
u8        DataCollectBkCount =0;               //�������ݲɼ�������
//float     LevelData_Float[FILTER_ORDER]={0.0}; //�ɼ�������ʱҺλ���ݣ�������
extern    uint8_t  LevelDataCount ;            //Һλ���ݼ���������ʾ��ǰ�ɼ�����������
u8        DataCollectCount =1;                 //���ݲɼ�������
char      Usart1_recev_buff[100] ={'\0'};      //USART1���ջ���
u16       Usart1_recev_count =0;               //USART1���ͼ�����
//char      Usart2_recev_buff[50]={'\0'};      //USART2���ջ���
//uint8_t   Usart2_recev_count =0;             //USART2���ռ�����
u32       WorkingTime =0;                      //�豸����ʹ�õ�ʱ��
u8        DMA_UART3_RECEV_FLAG =0;             //USART3 DMA���ձ�־����
u8        Usart2_send_buff[SENDBUFF_SIZE]={'\0'};       //USART2���ͻ���
u8        DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE] ={0x00};
static    struct DMA_USART3_RecevConfig  DMA_USART3_RecevIndicator; 

extern    char   Usart3_recev_buff[RECEIVEBUFF_SIZE];
extern    u16    Usart3_recev_count;
extern   volatile unsigned char Uart4_rev_count;        //RS485���ڽ��ռ�����
//uint32_t  time=0 ;                   // ms ��ʱ����  
//char      Usart1_send_buff[300]={'\0'};       //USART1���ͻ���
//uint8_t   Usart1_send_count=0;                 //USART1���ͼ�����
//uint32_t  Tic_IWDG=0;                //�������Ź�ι��ʱ������
//extern  char  Usart3_send_buff[];
//extern  uint8_t  Usart3_send_count;                     
//extern  struct  Config_RegPara   ConfigData;  //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
//extern  float  Data_Liquid_Level;

extern void     Delay(uint32_t nCount);
//extern void     LSLIQUID_DataCollect( struct LiquidData* pLevel, u8* pDevID, u16 NodeAddr);          
extern void     RecvBuffInit_USART3(void);

u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3�������ݼ�������ݽ���
int  DMA_UART3_RecevDataGet(void);


/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : ��DMA���մ洢������ȡ��Ч���ݣ�����Usart3_recev_buff[],���ں������ݽ���
* Input          : None
* Output         : None
* Return         : �������ݳ���
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
	 printf("\r\nDMA UART2 Recev Data Start Num:%d---End Num: %d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //����ʹ��
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
//		 DMA_Cmd(DMA1_Channel6, DISABLE);           //�ر�DMA��ֹ�����ڼ�������
//		 USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);   //��USART2��������ǰ���ȴ�USART2���տ����жϣ����ڼ�����ݽ������
		 DataLength = DMA_UART3_RecevDataGet();
//		 DMA_UART3_RECEV_FLAG =0;
//		 DMA_Cmd(DMA1_Channel6, ENABLE);            //����DMA 
		 if(DataLength>0)
		 {
				printf("\r\nDataLength:%d\r\n", DataLength);             //����ʹ��
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);           //����ʹ��
			  for(i=0;i<DataLength;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);               //����ʹ��
        }
        StateFlag =Receive_Data_Analysis(pDeviceID, sNodeAddress);	      //�������·����ݽ���    //
        //�Խ����������ͽ���ָʾ 
      	//			
		 }
		 else
		 {
        printf("\r\nNo data\r\n");
     }
		 DMA_Cmd(DMA1_Channel6, DISABLE);           //�ر�DMA��ֹ�����ڼ�������
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //��λDMA���ݽ���BUFF
     DMA_UART3_RECEV_FLAG =0;
		 DMA_Cmd(DMA1_Channel6, ENABLE);            //����DMA 
		 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //��USART2��������ǰ���ȴ�USART2���տ����жϣ����ڼ�����ݽ������
  } 
	return StateFlag;
}

/*******************************************************************************
���ܣ�ȥ�������Сֵ����ʣ��������ƽ��
���룺���������飬������������
���أ�������ƽ��ֵ
��д��
��д���ڣ�XX��XX��XX��
�汾��v0.1
********************************************************************************/
float AverageLevelData(float* pLevelData, uint8_t LevelData_Count)
{
	float DataTemp[FILTER_ORDER] ={0.0};
  float Temp = 0.0;
  float AverageData = 0.0;
  float Dvalue = 0.0;          //���ֵ����Сֵ֮��
  uint8_t  i=0,j=0;
  

  for(i=0;i<LevelData_Count;i++)
  {
    DataTemp[i] = pLevelData[i];
  }
	
  for(i=LevelData_Count-1;i>=1;i--)
	{ 
		for(j=LevelData_Count-1;j>=LevelData_Count-i;j--)    //��С����˳������
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
     printf("##%f##",DataTemp[i]);    //����ʹ��
  }
	
	Temp =0;                            //��λ�ۼ���
	Dvalue =DataTemp[LevelData_Count-1]-DataTemp[0];
	if(Dvalue < 0.025)                   //�����ֵ����Сֵ֮����2.5cm����ʱ��ȥ��4��ƫ��ϴ�ֵ
	{
		for(i=2;i<LevelData_Count-2;i++)
		{
			 Temp = Temp + DataTemp[i];
		}
		AverageData = Temp/(LevelData_Count-4);
		printf("\r\n**%f**\r\n##",Dvalue);    //����ʹ��
	}
	else                                   //�����ֵ����Сֵ֮�����2.5cmʱ��ȥ��6��ƫ��ϴ�ֵ
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
//			u8        RS485_Read_Cmd[6]={0x01,0x03,0x00,0x00,0x00,0x02};    //��Һλ����ָ�������CRCУ�飩
//			u8        i=0;   //����ʹ��
//			u8        InvalidData_cnt = 0;

//			#if DEBUG_TEST
//			printf("DataCollectCount:%d\r\n",DataCollectCount);  //����ʹ��
//			#endif
//			
//			while(LevelDataCount != 0)
//			{
//				memcpy(Uart4_send_buff, RS485_Read_Cmd,sizeof(RS485_Read_Cmd));					
//				Uart4_send_ready_flag = 1;
//				Crc_counter = 6;
//				RS485_Communication();                 //ͨ��485��Һλ�ƽ���ͨ��	
//				Delay_ms(1000);                        //��Ҫ��һ���ʵ�����ʱ,����1s	
//			  
//				if(Uart4_rev_comflag==1)
//				{
//					RS485_Communication();
//				}
//				InvalidData_cnt++;
//				//��10���ղ�����ЧҺλ���ݣ������������ߣ����ڿɿ����ϱ�Һλ�ƹ���֡�ٴ���
//				if(InvalidData_cnt >= 25)                 //һ���ϴ����ݲ��ᳬ��12�飬��ˣ���������ʱ�ڵ�Ƭ������֮ǰInvalidData_cnt���ᳬ��20
//				{
//					InvalidData_cnt = 0;
//					gotoSleep(DeviceConfig.UploadCycle);
//				}
//				for(i=0;i<3;i++)                                 //�ʵ�������ʱ������żȻ�ĸ���
//				{
//					Delay_ms(100);                                 
//          if(DMA_UART3_RECEV_FLAG==1)                    //��ѯ���ݽ������
//					{
//						DMA_UART3_RecevDetect(pDevID, NodeAddr);     //��ѯ�����·���������������ò�ѯ����
//					}
//        }	
//			}
//			
//			if(DataCollectCount == DeviceConfig.CollectNum)    //��һ�βɼ����ݣ���¼�ɼ�ʱ����Ϣ����������ÿ���Ƿ������ʵʱ��
//			{
//				pLevel->DataCount =0;                            //��Һλ���ݻ������������ֵ                       
////				USART_ITConfig(UART4, USART_IT_RXNE, DISABLE); //���ݲɼ�����Ժ󣬹ر�UART4�����ж�
//				TimCount_Current = RTC_GetCounter();
//				Time_Display(TimCount_Current,&systmtime); 
//						
//			  DeviceConfig.Time_Sec  =systmtime.tm_sec;
//				DeviceConfig.Time_Min  =systmtime.tm_min;
//				DeviceConfig.Time_Hour =systmtime.tm_hour;
//			  DeviceConfig.Time_Mday =systmtime.tm_mday;		
//			  DeviceConfig.Time_Mon  =systmtime.tm_mon;
//			  DeviceConfig.Time_Year =systmtime.tm_year-2000; //���ϴ����ȥ��������				
////			#if DEBUG_TEST
//				printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //����ʹ��
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //����ʹ��
////			#endif
//			}		
////			printf("\r\nLevelDataCount:%d\r\n",LevelDataCount);      //����ʹ��
////			for(i=0;i<FILTER_ORDER;i++)
////			{
////         printf("--%f--",LevelData_Float[i]);                 //����ʹ��
////      }
//			LevelData_Smooth = AverageLevelData(LevelData_Float, FILTER_ORDER);  //�˳������Сֵ��ȡ��ƽ��
//			////////����̽ͷ��װ�߶Ȳ���������Һλ����////////////
//			if(DeviceConfig.MountingHeight>0.5)                  //̽ͷ��װ������Ч
//			{
//         if(DeviceConfig.MountingHeight >LevelData_Smooth)   
//				 {
//            LevelData_Smooth = DeviceConfig.MountingHeight -LevelData_Smooth;  //��̽ͷ����Һ��߶�С�ھ����ȡ��ʵ�ı���Һ����ȣ���������Ϊ̽ͷ����Һ��ĸ߶�
//         }
//      }
//			temp =LevelData_Smooth*1000;
//			temp =(int)temp;                                   //���ȿ�����3λС��
//			LevelData_Smooth = temp/1000;
//			
//			LevelData_Float[0] =LevelData_Smooth;              //��ƽ��������Һλ���ݴ����ƽ������LevelData_Float[]�ĵ�һ����Ԫ�����ں������Ų�ѯʹ��
//      printf("\r\nSoothData:%f\r\n",LevelData_Smooth);   //����ʹ��
////			Float2Hex_Aider(LevelData_Smooth);
//			i = pLevel->DataCount;
//			pLevel->CollectData[i] =LevelData_Smooth;
//			if(i==0)
//			{
//        pLevel->CollectTime[i] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);      //��һ�����ݲɼ�ʱ��
//      }
//		  else
//			{
//        pLevel->CollectTime[i] =(pLevel->CollectTime[i-1])+ DeviceConfig.CollectPeriod;    //����ÿ�����ݲɼ�ʱ��Ϊ��ǰһ�����ݻ��������Ӳɼ����
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
   u8     TemperatureReadCmd[8]={0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};           //���¶�����ָ��,�����λΪCRCУ����
   u8     TemperatureReadResp[3]={0x01,0x03,0x02};   //����Ӧ��ָʾ:01,03,02,T_H,T_L,CrcL,CrcH
   u8*    pUart4Send =  TemperatureReadCmd;
   u8*    pDataRead  =NULL; 
   float  Temper =0;   //�¶������ݴ�

   Uart4_rev_comflag =0;   //���ձ�־������λ
   Uart4_rev_count   =0;   //���ռ���������
	 memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));
//��ɼ��������ݣ���ƽ������֤�����ȶ��ɿ�

//			LevelData_Smooth = AverageLevelData(LevelData_Float, FILTER_ORDER);  //�˳������Сֵ��ȡ��ƽ��
//			////////����̽ͷ��װ�߶Ȳ���������Һλ����////////////
//			if(DeviceConfig.MountingHeight>0.5)                  //̽ͷ��װ������Ч
//			{
//         if(DeviceConfig.MountingHeight >LevelData_Smooth)   
//				 {
//            LevelData_Smooth = DeviceConfig.MountingHeight -LevelData_Smooth;  //��̽ͷ����Һ��߶�С�ھ����ȡ��ʵ�ı���Һ����ȣ���������Ϊ̽ͷ����Һ��ĸ߶�
//         }
//      }
//			temp =LevelData_Smooth*1000;
//			temp =(int)temp;                                   //���ȿ�����3λС��
//			LevelData_Smooth = temp/1000;
//			printf("\r\nLevelDataCount:%d\r\n",LevelDataCount);      //����ʹ��
//			for(i=0;i<FILTER_ORDER;i++)
//			{
//         printf("--%f--",LevelData_Float[i]);                 //����ʹ��
//      }

printf("\r\n11\r\n");                 //����ʹ��

////////////////////////////////////////////////////

    DIR485_Send();                           //485����ʹ��
		Delay_ms(20);                            //ԭʼ3ms

//		for(i=0;i<sizeof(TemperatureReadCmd);i++)
	 for(i=0;i<8;i++)
		{  
			 USART_SendData(UART4, TemperatureReadCmd[i]);
			 pUart4Send++;
			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //�ȴ�������� 
		}
		Delay_us(450);                  //�����ݽ�����ɺ��ӳ�һ��ʱ�����л��շ����ƿ��أ�Ĭ����ʱ350us		
		DIR485_Receive();
		Delay_ms(2000);
		
		for(i=0;i<15;i++) 
		{
      if(Uart4_rev_comflag ==1)
			{
          //����
				 printf("\r\n22\r\n");                 //����ʹ��
				 for(i=0;i<Uart4_rev_count;i++)
					{  
						 printf("---%x--",Uart4_rev_buff[i]);                 //����ʹ��
					}
				
				pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //����Ƿ��յ��¶ȴ�����Ӧ��
				if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
				{
           Temper =(pDataRead[3]*256 +pDataRead[4])/10;
        	 #if DEBUG_TEST
				   printf("\r\nPercept Temperature is :%f\r\n",Temper);                 //����ʹ��
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
   u8     PressureReadCmd[8]={0x01,0x04,0x00,0x02,0x00,0x02,0xD0,0x0B};           //���¶�����ָ��,�����λΪCRCУ����
   u8     PressureReadResp[3]={0x01,0x04,0x04};   //����Ӧ��ָʾ:01,03,02,P_1,P_2,P_3,P_4,CrcL,CrcH    //�����ʾ
   u8*    pUart4Send = PressureReadCmd;
   u8*    pDataRead  =NULL; 
   float  Pressure =0;   //ѹ�������ݴ�
   union  Hfloat  PressureTemp;

   Uart4_rev_comflag =0;   //���ձ�־������λ
   Uart4_rev_count   =0;   //���ռ���������
	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));
//��ɼ��������ݣ���ƽ������֤�����ȶ��ɿ�

    DIR485_Send();                           //485����ʹ��
		Delay_ms(20);                            //ԭʼ3ms

		for(i=0;i<sizeof(PressureReadCmd);i++)
		{  
			 USART_SendData(UART4, *pUart4Send);
			 pUart4Send++;
			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //�ȴ�������� 
		}
		Delay_us(650);                  //�����ݽ�����ɺ��ӳ�һ��ʱ�����л��շ����ƿ��أ�Ĭ����ʱ200us		
		DIR485_Receive();
		
		for(i=0;i<10;i++)
		{
      if(Uart4_rev_comflag ==1)
			{
          //����
				pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)PressureReadResp, sizeof(Uart4_rev_buff), sizeof(PressureReadResp));  //����Ƿ��յ��¶ȴ�����Ӧ��
				if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-10)))
				{
           for(i=0;i<4;i++)
					 {
              PressureTemp.Data_Hex[i] =pDataRead[3+i];
           }
					 Pressure =PressureTemp.Data_Float;
        	 #if DEBUG_TEST
				   printf("\r\nPercept Temperature is :%f\r\n", Pressure );                 //����ʹ��
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
//			u8        RS485_Read_Cmd[6]={0x01,0x03,0x00,0x00,0x00,0x02};    //��Һλ����ָ�������CRCУ�飩
			u8        i=0;   //����ʹ��


			#if DEBUG_TEST
			printf("DataCollectCount:%d\r\n",DataCollectCount);  //����ʹ��
			#endif
			
	
			if(DataCollectCount == DeviceConfig.CollectNum)    //��һ�βɼ����ݣ���¼�ɼ�ʱ����Ϣ����������ÿ���Ƿ������ʵʱ��
			{
				pGetData->DataCount =0;                            //��Һλ���ݻ������������ֵ                       
//				USART_ITConfig(UART4, USART_IT_RXNE, DISABLE); //���ݲɼ�����Ժ󣬹ر�UART4�����ж�
				TimCount_Current = RTC_GetCounter();
				Time_Display(TimCount_Current,&systmtime); 
						
			  DeviceConfig.Time_Sec  =systmtime.tm_sec;
				DeviceConfig.Time_Min  =systmtime.tm_min;
				DeviceConfig.Time_Hour =systmtime.tm_hour;
			  DeviceConfig.Time_Mday =systmtime.tm_mday;		
			  DeviceConfig.Time_Mon  =systmtime.tm_mon;
			  DeviceConfig.Time_Year =systmtime.tm_year-2000; //���ϴ����ȥ��������				
//			#if DEBUG_TEST
				printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //����ʹ��
											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //����ʹ��
//			#endif
			}		

//			temp =LevelData_Smooth*1000;
//			temp =(int)temp;                                   //���ȿ�����3λС��
//			LevelData_Smooth = temp/1000;
//			
//			LevelData_Float[0] =LevelData_Smooth;              //��ƽ��������Һλ���ݴ����ƽ������LevelData_Float[]�ĵ�һ����Ԫ�����ں������Ų�ѯʹ��
//      printf("\r\nSoothData:%f\r\n",LevelData_Smooth);   //����ʹ��
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
        pGetData->CollectTime[i] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);      //��һ�����ݲɼ�ʱ��
      }
		  else
			{
        pGetData->CollectTime[i] =(pGetData->CollectTime[i-1])+ DeviceConfig.CollectPeriod;    //����ÿ�����ݲɼ�ʱ��Ϊ��ǰһ�����ݻ��������Ӳɼ����
      }
			pGetData->DataCount =(pGetData->DataCount)+1;
     	DataCollectCount--;	

}

		
	


/*******************************************************************************
* Function Name  :Power_SX1278_Init()
* Description    : ��ʼ��433ģ��
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
* Description    : ��ʼ��433ģ��
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
	////////////////////����433ģ����� //////////////////// 
//	SetNodeSerialPort(4, 0);                           //����433ģ�鴮�ڲ����ʣ�9600,У������:��У��
	SetNodeCentralFrequency(434);                      //����433ģ���ز�����Ƶ�ʣ�434MHz
	SetNodeFrequencyExpandFactor(11);                  //����433ģ����Ƶ���ӣ�2048
	SetNodeFrequencyExpandBandwidth(7);                //����433ģ����Ƶ����125K
	SetNodeWorkMode(2);                                //����433ģ�鹤��ģʽ���ڵ�ģʽ
	SetNodeID (sNodeID);                               //����433ģ��ID������ڵ�ID
	SetNodeNetworkID (1);                              //����433ģ������ID��0
	SetNodeSendPowerGrade (7);                         //����433ģ�鷢�书�ʵȼ���20dBm
	SetNodeBreathPeriod (0);                           //����433ģ���������:2s
	SetNodeBreathTime (5);                             //����433ģ�����ʱ�䣺64ms
	
	PowerOFF_GPRS();                                   //����433ģ�飬ʹ������Ч        
	Delay_ms(5000);
	PowerON_GPRS(); 
	////////////////////��ȡ433ģ����� //////////////////// 
//u8  GetNodeReceiveSignalEnergy ();                 //��ȡ433ģ����һ֡���ݽ����ź�ǿ��
  UartConfig =GetNodeSerialPortConfig();              //��ȡ433ģ�鴮�����ò���
  BaudRate   =UartConfig>>8;
  VerifyType =UartConfig &0xFF;
  printf("\r\nBaud Rate:%d---Verify Type:%d\r\n",BaudRate,VerifyType);//����ʹ��
  CentralFreq =GetNodeCentralFrequency ();               //��ȡ433ģ���ز�Ƶ�ʲ���
  printf("\r\nCentral Frequency :%d MHz\r\n",(CentralFreq+1));   //����ʹ��
  ExpFactor   =GetNodeFrequencyExpandFactor();           //��ȡ433ģ����Ƶ���Ӳ���
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
  printf("\r\nNode Frequency Expand Factor:%d \r\n",Temp);   //����ʹ��
  ExpBW =GetNodeFrequencyExpandBandwidth ();                 //��ȡ433ģ����Ƶ�������
	switch( ExpBW )
  {
			case 6:  
			{
				Temp=63;       //62.5Լ����63
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
	printf("\r\nNode Frequency Expand Bandwidth:%dKHz\r\n",Temp);   //����ʹ��
  WorkMode = GetNodeWorkMode ();                                  //��ȡ433ģ�鹤��ģʽ����
	switch( WorkMode )
  {
			case 0:  
			{
	      printf("\r\n433 Module Work Mode is: Standard\r\n");   //����ʹ��
				break;
			}
			case 1:  
			{
				printf("\r\n433 Module Work Mode is: Center\r\n");    //����ʹ��
				break;
			}
			case 2:  
			{
				printf("\r\n433 Module Work Mode is: Node\r\n");    //����ʹ��
				break;
			}		
			default:
			{
				printf("\r\n433 Module Work Mode is: Unknown\r\n");    //����ʹ��
				break;
			}		
  }
  NodeID =GetNodeID ();                                 //��ȡ433ģ��ڵ�ID
	printf("\r\n433 Module Node ID is: %x\r\n",NodeID);   //����ʹ��
  NetID =GetNetworkID ();                               //��ȡ433ģ������ID
	printf("\r\n433 Module Network ID is: %x\r\n",NetID); //����ʹ��
  PowerGrade = GetNodeSendPowerGrade ();                //��ȡ433ģ�鷢�书��
	printf("\r\n433 Module Send Power Grade is: %d\r\n",PowerGrade); //����ʹ��
  BreathPeriod = GetNodeBreathPeriod ();                //��ȡ433ģ���������
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
	printf("\r\nNode Breath Period:%d s\r\n",Temp );   //����ʹ��
  WakeTime  =  GetNodeBreathTime ();                 //��ȡ433ģ�����ʱ��
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
	printf("\r\nNode Wake Time:%d ms\r\n",Temp );   //����ʹ��

}
/*******************************************************************************
* Function Name  : void PeripheralInit( void )
* Description    : ��ʼ���˿ڼ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PeripheralInit(void)
{
	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//���ȿ������ڿ��Ź���������ֵΪ7f,���ڼĴ���Ϊ5f,��Ƶ��Ϊ8	
	USART1_Config();      /* USART1 ����ģʽΪ 9600 8-N-1��  �жϽ��� */
	USART2_Config();      /* USART2 ����ģʽΪ 9600 8-N-1���жϽ��� */
	USART3_Config();      /* USART3 ����ģʽΪ 9600 8-N-1���жϽ��� */
	USART4_Config();      /* UART4  ����ģʽΪ 9600 8-N-1��  �жϽ��� */
  UART_NVIC_Configuration();
//	USART3_DMA_Config();
  USART2_DMA_Config();
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //���ô���DMA1����
//  TIM2_Configuration();       /* ��ʱ��TIM2�������� */	
//	TIM2_NVIC_Configuration();  /* ���ö�ʱ��TIM2���ж����ȼ� */
	TIM3_Configuration();       /* ��ʱ��TIM3�������� */	
	TIM3_NVIC_Configuration();  /* ���ö�ʱ��TIM3���ж����ȼ� */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //��ʱ�رն�ʱ��TIM3
	
	RTC_NVIC_Config();                 /* ����RTC���ж����ȼ� */
	RTC_CheckAndConfig(&systmtime);
//	Time_Show(&systmtime);             /* Display time in infinite loop */
	WorkingTime =RTC_GetCounter();     //�ɼ��豸����ʱ��
  SysTick_Init();
  PowerON_Flash();                   //��Flash��Դ 
  Delay_ms(100); 	
//	PowerON_GPRS();                  //��GPRSģ���Դ��****����򿪣�����ԭ��
//	Delay_ms(100);
	PowerON_UltrasonicSensor();        //�򿪳�����̽ͷ��Դ����TCP������ȷ�����Ժ��ٿ���̽ͷ��Դ�����������Ҫ������ʷ���ݣ�����Ҫ����Ӧ�޸�
	Delay_ms(500);   
  PowerON_485();                     //��485��Դ
	Delay_ms(500);
  Uart4_rev_comflag=0;               //���Һλ�Ƹ�����ʱ���ڵ�����,�����Ƿ��б�Ҫ
	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));
	
 	DMA_USART3_RecevIndicator.CurrentDataStartNum =0;  //��ʼ����ǰ�������ݿ�ʼλ��
	DMA_USART3_RecevIndicator.CurrentDataEndNum =0;    //��ʼ����ǰ�������ݽ���λ��
	DMA_USART3_RecevIndicator.NextDataStartNum =0;     //��ʼ����һ�ν������ݿ�ʼλ��
	DMA_USART3_RecevIndicator.DMA_RecevCount =0;
	///////////////////���뽫ģ�����óɽ���״̬�����򴮿ڽ��ղ�������
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ߣ��л�������ģʽ
	Delay_ms(100);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                //433ģ��EN�ܽ����ͣ��л�������ģʽ
	Delay_ms(100);
	USART_GetFlagStatus(USART2,USART_FLAG_TC);       //����Ӳ����λ֮�󣬷������ֽ�֮ǰ���ȶ�һ��USART_SR,��ֹ���ݷ���ʱ���ֽڱ�����
	Delay_ms(500);
	USART_GetFlagStatus(UART4,USART_FLAG_TC);       //����Ӳ����λ֮�󣬷������ֽ�֮ǰ���ȶ�һ��USART_SR,��ֹ���ݷ���ʱ���ֽڱ�����
	Delay_ms(500);
}


/*******************************************************************************
* Function Name  : int main(void)
* Description    : ������
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
	u8  DeviceID[6] ={0x81,0x20,0x16,0x04,0x80,0x01}; //��ʼ���豸ID��
  u16 NodeAddr =0x0000;
  struct SenserData   PerceptionData;         //����������

	NodeAddr =DeviceID[4]*256 +DeviceID[5];     //��ȡ�豸ID������������ֽ���Ϊ�ڵ��ַ
  PeripheralInit();                           //��ʼ������
  ConfigData_Init(&DeviceConfig);             //��ʼ���Ժ���ܽ��ղ�ѯ��������
  if (PowerOffReset ==1)          
  {
    printf("\r\n�������������³�ʼ�����ؼ�\r\n");                 //����ʹ��
		Set_register_ds2780();    //�����Կ��ؼ����³�ʼ��
	  set_ACR(1000);            //�����Կ��ؼ����³�ʼ��
	  DS2780_CapacityInit();    //���������д�������
		DS2780_Test();            //����󽫵������ϵ������д��Flash
//		SX1287_Init(NodeAddr);
	  //////�豸�ϵ�ע��///////////
		Delay_ms(2000);   
		DeviceStartupRequest(Usart2_send_buff, DeviceID, NodeAddr);  
  }                
  PerceptionData.DataCount =0;                        //��ʼ��Һλ���ݼ�����
	DataCollectCount = DeviceConfig.CollectNum;        //��ȡ���ݲɼ���������
	for(i=0;i<20;i++)
	{
		 Delay_ms(200);                                //�ο�̽ͷ�ϵ�������ȶ�ʱ�䣬����Ӧ����
		 if(DMA_UART3_RECEV_FLAG==1)                   //��ѯ���ݽ������
		 {
				DMA_UART3_RecevDetect(DeviceID, NodeAddr);  
				break;
		 }
  }	
	while(1)
  {
		
		WWDOG_Feed =0x1FFF;                              //���ڿ��Ź�ι��,��ʱ4��20�룬�ڶ��ֽ�Լ1��仯һ�Σ���0x09AF�䵽0x099FԼ��ʱ1��		
		for(i=0;i<5;i++)
		{
			 Delay_ms(200);                                //�ο�̽ͷ�ϵ�������ȶ�ʱ�䣬����Ӧ����
			 if(DMA_UART3_RECEV_FLAG==1)                   //��ѯ���ݽ������
			 {
				 DMA_UART3_RecevDetect(DeviceID, NodeAddr);  
				 break;
			 }
    }	
		if(DataCollectCount!=0)
		{
			 if(LevelDataCount == 0)
			 {
          LevelDataCount = FILTER_ORDER;             //������Ҫһ���ϴ���������ʱ����Ҫ���ɼ�Ƶ�ʿ��Ʊ�������ֵ
       }
			 SenserDataCollect(&PerceptionData, DeviceID, NodeAddr);        //��ȡѹ�����ݺ��¶�����
			 
		}
		else 
		{			
			 PowerOFF_UltrasonicSensor();                  //��Һλ���ݲɼ����ʱ���رճ�����̽ͷ��Դ
			 Delay_ms(100); 
       PowerOFF_485();                               //��Һλ���ݲɼ����ʱ���ر�Uartת485ģ���Դ
			 Delay_ms(100);
			 DeviceConfig.BatteryCapacity = DS2780_Test(); //����ص���
			 TrapRequest(Usart2_send_buff, DeviceID, NodeAddr, &PerceptionData);  //����Һλ����
			 LiquidDataSend_Flag=1;                        //Һλ���ݳɹ����͵���������־����
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
