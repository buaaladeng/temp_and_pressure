
// File Name: gprs.c
#include "string.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "AiderProtocol.h"
#include "bsp_rtc.h"
#include "433_Wiminet.h"
#include "common.h"
#include "math.h"
#include "SPI_Flash.h"
#include "DS2780.h"

char             Usart3_recev_buff[RECEIVEBUFF_SIZE]={0x00};     //USART3接收缓存
u16              Usart3_recev_count=0;           //USART3接收计数器
extern struct    Config_RegPara   ConfigData;    //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
extern struct    DeviceSet  DeviceConfig;        //液位计配置信息结构体
extern u8        DataCollectCount;               //数据采集计数器
extern u8        LiquidDataSend_Flag;
extern u8        DMA_UART3_RECEV_FLAG ;          //USART3 DMA接收标志变量
extern vu8       Uart4_rev_comflag;              //RS485串口接收完成标志变量
extern u8        Uart4_rev_buff[100];
extern vu8       Uart4_rev_count;                //RS485串口接收计数器
extern u8        Usart2_send_buff[SENDBUFF_SIZE];//USART2发送缓存

extern u16       DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3接收数据监测与数据解析
extern void      LevelDataMessageSend(void);     // 通过短信发送液位数据

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void delay(unsigned int dl_time)
//{
//   unsigned int i,y;
//	 for(i=0;i<5000;i++)
//	 {
//      for(y=0;y<dl_time;y++);
//   }
//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_DataBlock_Send(USART_TypeDef* USART_PORT,char* SendUartBuf,u16 SendLength)    //批量向串口发送数据
{
    u16 i;
        
    for(i=0;i<SendLength;i++)
    {
        USART_SendData(USART_PORT, *(SendUartBuf+i));
        while (USART_GetFlagStatus(USART_PORT, USART_FLAG_TC) == RESET);
    } 
}
void mput_mix(char *str,int length)                  //用于更新433模块配置参数，数据发送完成以后不能拉高SET引脚，否则模块功耗过高
{
	printf("length:%d\r\n",length);                    //测试使用
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //向USART2发送数据前，先打开USART2接收空闲中断，便于监测数据接收完成
//	USART_ClearFlag(USART2,USART_FLAG_TC);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433模块SET管脚拉低，切换到高速发送模式
	Delay_ms(200);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433模块EN管脚拉低，切换到高速发送模式
	Delay_ms(300);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
//	Delay_ms(1000);
//	USART_ClearFlag(USART2,USART_FLAG_TC);
}
void mput_mix_sx1278(char *str,int length)           //用于433模块发送数据，数据发送完成以后必须拉高SET引脚，否则模块经常接收不到数据
{
	printf("length:%d\r\n",length);                    //测试使用
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433模块SET管脚拉低，切换到高速发送模式
	Delay_ms(200);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433模块EN管脚拉低，切换到高速发送模式
	Delay_ms(500);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
	Delay_ms(800);                                     
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433模块SET管脚拉高，切换到接收模式
	Delay_ms(100);
//	USART_ClearFlag(USART2,USART_FLAG_TC);
}

void mput(char* str)
{
//	printf("length:%d\r\n",strlen(str));     //测试使用
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //向USART2发送数据前，先打开USART2接收空闲中断，便于监测数据接收完成
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433模块SET管脚拉低，切换到高速模式
	Delay_ms(500);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433模块EN管脚拉低，切换到高速模式
	Delay_ms(500);
	USART_DataBlock_Send(USART2,str,strlen(str));
	USART_DataBlock_Send(USART2,"\r\n",2);
	USART_DataBlock_Send(USART1,str,strlen(str));
	USART_DataBlock_Send(USART1,"\r\n",2);
	Delay_ms(800);
//	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433模块SET管脚拉高，切换到接收模式
//	Delay_ms(100);
}
//void mput_mix(char *str,int length)
//{
//	printf("length:%d\r\n",length);             //测试使用
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //向USART3发送数据前，先打开USART3接收空闲中断，便于监测数据接收完成
//	USART_DataBlock_Send(USART3,str,length);
//	USART_DataBlock_Send(USART3,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
//}
//void mput(char* str)
//{
////	printf("length:%d\r\n",strlen(str));     //测试使用
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //向USART3发送数据前，先打开USART3接收空闲中断，便于监测数据接收完成
//	USART_DataBlock_Send(USART3,str,strlen(str));
//	USART_DataBlock_Send(USART3,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,strlen(str));
//	USART_DataBlock_Send(USART1,"\r\n",2);
//}
/*******************************************************************************
* Function Name  : char* Find_String(char* Source, char* Object)
* Description    : 在目标字符串中发现一个指定的字符串
* Input          : 
* Output         : 
* Return         : 如果找到，则返回目标字符串在源字符串中的首地址
*******************************************************************************/
char* Find_String(char* Source, char* Object)
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   Length_Source =0;
	short   Length_Object =0;
	short   i=0,j=0;
	short   count=0;
	
	Length_Source = strlen(Source);
	Length_Object = strlen(Object);
	if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 return  &Ptemp1[i]; 
        }
				return NULL;
     }	  
  }
}
/***********函数功能：在特定序列中发现一个指定的序列****************/
/***********如果找到，则返回目标序列在源序列中的首地址**************/
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object)  
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   i=0,j=0;
	short   count=0;
	
	if((Length_Source < 0)||(Length_Object < 0))
	{
     return NULL;
  }
  if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  
			  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 {
							return  &Ptemp1[i]; 
					 }
				 
        }
				return NULL;
     }	  
  }
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8  UploadFlash(u8* pSetPara, u8 ParaLen, CommandType ParaRequest)
{    
  u16  TempShort =0x0000;        //短整型临时变量

	switch(ParaRequest)
	{
		case CLT1_ITRL1:               //存储液位数据采集间隔
		{
			 DataWrite_To_Flash(0,2,0,(u8*)pSetPara, ParaLen);                    //将数据采集间隔写入Flash ，低地址对应低字节
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //测试使用
       printf("\r\n CollectPeriod:%2x\r\n",TempShort);	                    //测试使用
			 return 1;
		}
		case CLT1_CNT1:                //存储单次数据上传，液位数据采集次数
		{
			 DataWrite_To_Flash(0,3,0,(u8*)pSetPara, ParaLen);                    //将数据采集次数写入Flash ，低地址对应低字节
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //测试使用
       printf("\r\n CollectNum:%2x\r\n",TempShort);	                        //测试使用		 
			 return 1;
		}
		case UPLOAD_CYCLE:             //存储液位数据上传周期
		{
       DataWrite_To_Flash(0,4,0,(u8*)pSetPara, ParaLen);                    //将数据采集次数写入Flash ，低地址对应低字节
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //测试使用
       printf("\r\n UploadCycle:%2x\r\n",TempShort);	                      //测试使用		 
			 return 1;
			 
		}
		case CLT1_STIME1:            //存储设备数据上传起始时间（即设备启动时间）
		{
			 DataWrite_To_Flash(0,5,0,(u8*)pSetPara, ParaLen);                    //将数据上传开始时间写入Flash ，低地址对应低字节
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //测试使用
			 printf("\r\n CLT1_STIME1:%2x\r\n",TempShort);	                      //测试使用	
			 return 1;
		}
		case DEF_NR:                  //存储数据发送错误重传次数
		{
       DataWrite_To_Flash(0,6,0,(u8*)pSetPara, ParaLen);                    //将数据采集次数写入Flash ，低地址对应低字节
			 TempShort =*(pSetPara);                                              //测试使用
       printf("\r\n UploadCycle:%x\r\n",TempShort);	                        //测试使用		 
			 return 1; 
		}
		default:
		{
			 printf("\r\nInstruct Code ERROR !!\r\n");
		   return 0;
		}
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : FunctionNum (0：恢复出厂设置；1：读取探头环境温度（℃）；2：设置温度补偿因子；3：读取温度补偿因子；);ParaData(仅当功能码为2时，参数数据才有效)
* Output         : None
* Return         : None
*******************************************************************************/
//void  UltrasonicParaConfig(u8 FunctionNum, char* ParaData)
//{

//   char  UltrasonicRecovery[21] ="AT+EEPROM=RECOVERY\r\n";      //超声波探头恢复出厂设置命令
//	 char  UltrasonicTempConsult[13] ="AT+MENU+01?\r\n";          //超声波探头环境温度查询命令
//   char  UltrasonicTempOffsetConfig[17] ={0x00};                //超声波探头温度补偿因子设置命令	
//   char  UltrasonicTempOffsetConsult[13]="AT+MENU+28?\r\n";     //超声波探头温度补偿因子查询命令
//   char  UltrasonicParaSave[17]="AT+EEPROM=WRITE\r\n";          //超声波探头参数保存命令
//   char  SmsSendTemp[20] ={0x00}; 
//   u8    RetryCount=3, i=0;
//   
//	  if((DataCollectCount!=0)||(LiquidDataSend_Flag==1))   //未完成数据采集或者数据已经上报完成，超声波探头和485电源均已关闭，需要重新开启探头和485电源
//	  {
//       PowerON_UltrasonicSensor();        //打开超声波探头电源，在TCP连接正确建立以后再开启探头电源，后续如果需要备份历史数据，则需要做相应修改
//	     Delay_ms(1000);   
//       PowerON_485();                     //打开485电源
//			 Delay_ms(7000); 
//    }	
//    Delay_ms(3000);		      //增加适当延时，防止开机时即操作探头，等待探头启动并稳定
//    switch(FunctionNum)
//		{
//        case 0:                   
//				{
//           printf("\r\nUltrasonic recovery is in process...\r\n");    //测试使用 		
//					 for(i=RetryCount;i>0;i--)
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485发送使能
//							Delay_ms(50);                           
//							USART_DataBlock_Send(UART4,UltrasonicRecovery,strlen(UltrasonicRecovery));
//							Delay_us(100);
//							DIR485_Receive();                        //485接收使能
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic recovery receive_%d:%s\r\n",i,Uart4_rev_buff); //测试使用
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //清空UART4接收BUFF
//								 Uart4_rev_count =0;                                       //复位UART4接收计数器
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					 break;
//        }	     
//				case 1:
//				{
//           printf("\r\nUltrasonic current temperature inquire is in process...\r\n");    //测试使用 		
//					 for(i=RetryCount;i>0;i--)
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485发送使能
//							Delay_ms(50);                            //原始3ms
//							USART_DataBlock_Send(UART4,UltrasonicTempConsult,strlen(UltrasonicTempConsult));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature receive_%d:%s\r\n",i,Uart4_rev_buff); //测试使用
//								 if(strlen((char*)Uart4_rev_buff) <20)
//								 {
//                    memcpy(SmsSendTemp,Uart4_rev_buff,strlen((char*)Uart4_rev_buff));
//                 }
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //清空UART4接收BUFF
//								 Uart4_rev_count =0;                                       //复位UART4接收计数器
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					 break;
//        }	
//       
//				case 2:
//				{
//           printf("\r\nUltrasonic temperature offset config is in process...\r\n");    //测试使用 		
//					 for(i=RetryCount;i>0;i--)                    //设置温度补偿因子
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485发送使能
//							Delay_ms(50);                            //原始3ms
//						  snprintf(UltrasonicTempOffsetConfig,sizeof(UltrasonicTempOffsetConfig),"AT+MENU+28=%s\r\n",ParaData); //设置探头温度补偿因子
//							USART_DataBlock_Send(UART4,UltrasonicTempOffsetConfig,strlen(UltrasonicTempOffsetConfig));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature offset config receive_%d:%s\r\n",i,Uart4_rev_buff); //测试使用
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //清空UART4接收BUFF
//								 Uart4_rev_count =0;                                       //复位UART4接收计数器
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           } 
//					 for(i=RetryCount;i>0;i--)                    //保存配置参数
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485发送使能
//							Delay_ms(50);                            //原始3ms
//							USART_DataBlock_Send(UART4,UltrasonicParaSave,strlen(UltrasonicParaSave));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature offset save ok_%d:%s\r\n",i,Uart4_rev_buff); //测试使用
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //清空UART4接收BUFF
//								 Uart4_rev_count =0;                                       //复位UART4接收计数器
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					break;  
//        }	
//				
//        case 3:
//				{
//           printf("\r\nUltrasonic temperature offset consult is in process...\r\n");    //测试使用 		
//					 for(i=RetryCount;i>0;i--)
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485发送使能
//							Delay_ms(50);                            
//							USART_DataBlock_Send(UART4,UltrasonicTempOffsetConsult,strlen(UltrasonicTempOffsetConsult));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature offset consult receive_%d:%s\r\n",i,Uart4_rev_buff); //测试使用
//								 if(strlen((char*)Uart4_rev_buff)<20)
//								 {
//                    memcpy(SmsSendTemp,Uart4_rev_buff,strlen((char*)Uart4_rev_buff));
//                 }
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //清空UART4接收BUFF
//								 Uart4_rev_count =0;                                       //复位UART4接收计数器
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					 break;
//        }	
//        default:
//				{
//           printf("\r\nWarning!!Input function number error!\r\n"); //测试使用
//					 break;
//        }
//    }	
//}

/*******************************************************************************
* Function Name  : 协议数据分析
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress)	
{
	u8   DataLen =0;          //Tag序列或者OID序列的长度
	u16  PduType =0;
  u8   ProbeReset =0;       //探头复位标志变量
	u8   i=0,j=0;
	u8*  pChar =NULL;
	u32* pOid  =NULL;
	union  Hfloat  UnionData1;
	struct CommandFrame ParaRequest;
	
  PduType =pTreatyBuff[13]*256 +pTreatyBuff[14];   //结算接收数据PDU编码
  switch(PduType)
	{
        case 0x0188:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Get Request Command from Server.\r\n");    //测试使用
						#endif
						*pFlag =PduType;
					 //////////////////////////////////////////
					  DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12; //接收到的OID序列的总长度
					  ParaRequest.OID_Count =DataLen/4;               //接收到的OID序列数量
					  if((ParaRequest.OID_Count >0)&&(ParaRequest.OID_Count <=20))   //限定OID长度，防止内存溢出
						{
							  pOid = (u32*)&pTreatyBuff[16];
							  j    =ParaRequest.OID_Count;
								for(i=0;i<ParaRequest.OID_Count;i++)
								{ 
									  ParaRequest.OID_List[i] =(CommandType)*pOid;                    //OID高低字节顺序有待确认    
									  ParaRequest.OID_List[i] =ntohl( ParaRequest.OID_List[i] );      //网络序转换为机器序，即将数据的低字节放在低地址
									  printf("\r\n--Cycle--%4x----.\r\n", ParaRequest.OID_List[i]);   //测试使用
										j--;
									  if(j==0)   
										{
                       break;
                    }
									  pOid++;
								}
								printf("\r\n--i:%d--j:%d--%4x----.\r\n",i,j, ParaRequest.OID_List[i]);    //测试使用
								
            }
					  else
						{
                #if DEBUG_TEST	 
							  printf("\r\nReceive Command OID not correct.\r\n");    //测试使用
							  #endif
            }
//						if(DMA_UART3_RECEV_FLAG==1)                      //查询数据接收情况
//						{
//							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //查询有无下发配置命令或者配置查询命令
//						}
						DMA_UART3_RECEV_FLAG =0;     //接收标志变量复位
					  GetResponse( ParaRequest, Usart2_send_buff, pDeviceID,  NodeAddress);
					  break;
        }	
        case 0x0388:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Set Request Command from Server.\r\n");    //测试使用
						#endif
						*pFlag =PduType;
					  DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12;    //接收到的Tag序列的总长度
					  if(DataLen >6)           //至少存在一个合法的配置参数
						{
                pOid = (u32*)(pTreatyBuff+16);
							  i=0;
							  while( DataLen >6 )
								{
									 printf("\r\n--Cycle--%4x----.\r\n",ntohl(*pOid));         //测试使用
									 pChar = (u8*)pOid;
                   switch(ntohl(*pOid))
									 {
											case DEF_NR:            //重传次数
											{
													DeviceConfig.RetryNum =*(pChar+6);
												  if(DeviceConfig.RetryNum >=1)                         //参数合法性判定
													{
                             UploadFlash(&(DeviceConfig.RetryNum), 1, DEF_NR);  //参数存入Flash
                          }
												  printf("\r\n-Retry Num-%x---.\r\n", DeviceConfig.RetryNum);   //测试使用
//												  else               //当配置参数不合法时，使用默认参数，同时不更新Flash数据
//													{
//                             DeviceConfig.RetryNum =1;             //为了反馈真实情况不发送默认参数
//                          }
												  ParaRequest.OID_List[i] =DEF_NR;           
												  pOid =(u32*)(pChar+7);                     //指针后移1个Tag
												  DataLen = DataLen-7; 
 	                        i++;               //合法OID计数器												
													break;
											}	
											case SYSTERM_TIME:     //系统时间
											{
													DeviceConfig.Time_Year =*(pChar+6);
												  DeviceConfig.Time_Mon  =*(pChar+7);
												  DeviceConfig.Time_Mday =*(pChar+8);
												  DeviceConfig.Time_Hour =*(pChar+9);
												  DeviceConfig.Time_Min  =*(pChar+10);
												  DeviceConfig.Time_Sec  =*(pChar+11);
												  printf("\r\n-年-月-日-时-分-秒：-%d--%d--%d--%d--%d--%d--.\r\n",DeviceConfig.Time_Year,DeviceConfig.Time_Mon,DeviceConfig.Time_Mday,
												                 DeviceConfig.Time_Hour,DeviceConfig.Time_Min, DeviceConfig.Time_Sec  );         //测试使用
												  if((DeviceConfig.Time_Mon<=12)&&(DeviceConfig.Time_Mday<=31)&&(DeviceConfig.Time_Hour<=23)&&(DeviceConfig.Time_Min<=60)&&(DeviceConfig.Time_Sec<=60)) //参数合法性判定
													{
                            Time_Auto_Regulate(&DeviceConfig);             //通过服务器下发参数进行RTC时钟校准，
                          }
												  ParaRequest.OID_List[i] =SYSTERM_TIME;            
												  pOid =(u32*)(pChar+12);                          //指针后移1个Tag
												  DataLen = DataLen-12;
												 	i++;        //合法OID计数器
													break;							
											}	
											case CLT1_ITRL1:       //一时区采集间隔
											{
													DeviceConfig.CollectPeriod =pChar[6]*256+pChar[7];
												  printf("\r\n-CollectPeriod:-%2x---.\r\n", DeviceConfig.CollectPeriod );         //测试使用
												  if(DeviceConfig.CollectPeriod<=1440)              //参数合法性判定
													{
                             UploadFlash((u8*)&(DeviceConfig.CollectPeriod), 2, CLT1_ITRL1);  //参数存入Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_ITRL1;            
												  pOid =(u32*)(pChar+8);                         //指针后移1个Tag
												  DataLen = DataLen-8;
												 	i++;        //合法OID计数器
													break;
											}	
											case CLT1_CNT1:        //一时区采集次数
											{
													DeviceConfig.CollectNum =pChar[6]*256+pChar[7];
													printf("\r\n-CollectNum:-%2x---.\r\n", DeviceConfig.CollectNum );         //测试使用
												  if(DeviceConfig.CollectNum<=1440)              //参数合法性判定
													{
                             UploadFlash((u8*)&(DeviceConfig.CollectNum), 2, CLT1_CNT1);  //参数存入Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_CNT1;            
												  pOid =(u32*)(pChar+8);                         //指针后移1个Tag
												  DataLen = DataLen-8;
												 	i++;        //合法OID计数器
													break;
											}
											case CLT1_STIME1  :    //一时区采集起始时间
											{
													
												  DeviceConfig.CollectStartTime =pChar[6]*256+pChar[7];
													printf("\r\n-Collect Start Time:-%2x---.\r\n", DeviceConfig.CollectStartTime );        //测试使用
												  if(DeviceConfig.CollectStartTime<=1440)              //参数合法性判定
													{
                             UploadFlash((u8*)&(DeviceConfig.CollectStartTime), 2, CLT1_STIME1);  //参数存入Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_STIME1;            
												  pOid =(u32*)(pChar+8);                              //指针后移1个Tag
												  DataLen = DataLen-8;
												 	i++;        //合法OID计数器
													break;
											}	
//											case RESET_PROBER:     //复位液位探头，后续可以考虑放到状态表中
//											{
//												  ProbeReset =*(pChar+6);                         //获取服务器复位探头命令参数
//												  printf("\r\n-RESET PROBER-%x---.\r\n", ProbeReset);   //测试使用
//												  if(ProbeReset==1)
//													{ 
//														UltrasonicParaConfig(0, 0);                     //复位超声波探头
//                          }	  
//												  ParaRequest.OID_List[i] =RESET_PROBER;           
//												  pOid =(u32*)(pChar+7);                          //指针后移1个Tag
//												  DataLen = DataLen-7;
//												 	i++;        //合法OID计数器
//													break;
//											}	
											case UPLOAD_CYCLE:     //液位计数据上报周期
											{
													DeviceConfig.UploadCycle =pChar[6]*256+pChar[7];
												  if(DeviceConfig.UploadCycle<=1440)              //参数合法性判定
													{
                             UploadFlash((u8*)&(DeviceConfig.UploadCycle), 2, UPLOAD_CYCLE);  //参数存入Flash
                          }
											  	printf("\r\n-UploadCycle:-%2x---.\r\n", DeviceConfig.UploadCycle ); //测试使用
												  ParaRequest.OID_List[i] =UPLOAD_CYCLE;          
												  pOid =(u32*)(pChar+8);                           //指针后移1个Tag
												  DataLen = DataLen-8;
											  	i++;        //合法OID计数器
													break;
											}					
											default:
											{
												 #if DEBUG_TEST	
												 printf("\r\nWarning!!Tag OID not recognition!\r\n"); //测试使用
												 #endif
												 pOid =(u32*)(pChar+1);  //指针后移一个字节，查询后续有无合法OID
												 DataLen = DataLen-1;    //指针后移一个字节，查询后续有无合法OID
												 break;
											}
									 }
                }
						}
						if((i>0)&&(i<=20))
						{
               ParaRequest.OID_Count =i;           //对OID序列进行计数	
            }
            else
						{
               ParaRequest.OID_Count =7;           //当OID计数器值超过20时,将计数器值重置成默认值（默认7）
            }	
//            if(DMA_UART3_RECEV_FLAG==1)                      //查询数据接收情况
//						{
//							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //查询有无下发配置命令或者配置查询命令
//						}			
            DMA_UART3_RECEV_FLAG =0;     //接收标志变量复位						
					  GetResponse( ParaRequest, Usart2_send_buff, pDeviceID,  NodeAddress);
					  break;
        }	
				case 0x0588:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Trap Response Command from Server.\r\n");    //测试使用
						#endif
					  /////////////////////////
					  //服务器接收完整性验证，对于一次上传一帧数据的情况，只需通过PDUType进行验证，接收到应答帧即代表接收完整
						*pFlag =PduType;
					  break;
        }	
				case 0x0988:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Startup Response Command from Server.\r\n");    //测试使用
						#endif
						*pFlag =PduType;
					  break;
        }	
				case 0x0A88:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Wakeup Request Command from Server.\r\n");    //测试使用
						#endif
						*pFlag =PduType;
						if(DMA_UART3_RECEV_FLAG==1)                      //查询数据接收情况
						{
							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //查询有无下发配置命令或者配置查询命令
						}
					  WakeupResponse(Usart2_send_buff, pDeviceID, NodeAddress);
					  break;
        }	
				case 0x0088:         //临时使用，上位机有待完善
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Get Response Receive OK by Server.\r\n");    //测试使用
						#endif
						*pFlag =PduType;
					  break;
        }	
        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!PDU Type not recognition!\r\n"); //测试使用
					 #endif
					 break;
        }
   }      
}
/*******************************************************************************
* Function Name  : XX
* Description    : 433远程配置参数命令解析
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterConfig_SX1278(char* pRecevCommand)
{
   u8   RecevCommandArry[5]={0x00};
	
	 u8   UseFlag   =0;    //配置参数用途标志变量，也用于指示配置参数的时效性，参数为0时立即生效，否则作为备份参数
	 u8   Opcode    =0;    //命令码字段，用于指示命令的操作对象
	 u8   DataLenth =0;    //参数长度
	 u16  TempLong  =0;
	 u8   TempShort =0;
	 u8   ConfigReply[12] ={0xAA,0xAA,0xAF,0xFF,0xAF,0xEE,0x00,0xAA,0xAA,0xAA,0xAA,0xAA}; //数组中0xAA字段为待定字段，含义分别是（从左向右）：
     
                                                    //节点地址高字节，节点地址低字节，参数用途，命令码，配置参数长度，配置参数第1字节，配置参数第2字节（可能不存在）
   memcpy(RecevCommandArry, pRecevCommand, sizeof(RecevCommandArry));
   UseFlag   =RecevCommandArry[0];
	 Opcode    =RecevCommandArry[1];
	 DataLenth =RecevCommandArry[2];
   TempLong  =GetNodeID (); //获取节点ID
   ConfigReply[0] =TempLong>>8;
   ConfigReply[1] =TempLong &0xFF;
	 ConfigReply[7] =UseFlag ;
   ConfigReply[8] =Opcode;
   Delay_ms(3000);
	 if(UseFlag ==0)
	 {
     switch(Opcode )
		 {
        case 0x09:                    //配置433模块串口波特率参数，谨慎修改!!!
				{
           if(DataLenth ==2)
					 {
//						 SetNodeSerialPort(RecevCommandArry[3], RecevCommandArry[4]);     //设置433模块串口波特率和校验类型
//						 PowerOFF_GPRS();                                           //重启433模块，使配置生效        
//						 Delay_ms(5000);
//						 PowerON_GPRS();
//						 TempLong =GetNodeSerialPortConfig();                      //读取433模块串口配置参数
//						 ConfigReply[9] =2;
//						 ConfigReply[10] =TempLong >>8;
//					   ConfigReply[11] =TempLong &0xFF;
//						 mput_mix_sx1278((char*)ConfigReply, 12);                  //参数配置应答帧
//						 //参数存入Flash，有待完善
						 #if DEBUG_TEST	 
						 printf("\r\nSerial Port Parameters Config not allowable !!\r\n"); //测试使用
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x0B:                                                    //配置433模块载波频率
				{
           if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];      //获得配置频率参数
					
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix_sx1278((char*)ConfigReply, 12);                //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);
						 mput_mix_sx1278((char*)ConfigReply, 12);                //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功,多发一次为了解决首次发送数据丢失BUG
						 Delay_ms(1000);
						 SetNodeCentralFrequency(TempLong);                      //设置433模块载波中心频率
						 TempLong =GetNodeCentralFrequency();                    //读取433模块载波频率参数
	           #if DEBUG_TEST	 
					   printf("\r\nCentral Frequency :%d MHz\r\n",(TempLong+1));   //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x0D:                                                    //配置433模块扩频因子参数
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置扩频因子参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                  //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                  //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);							 
						 SetNodeFrequencyExpandFactor(TempShort);                  //设置433模块扩频因子
						 TempShort  =GetNodeFrequencyExpandFactor();               //获取433模块扩频因子参数
			
						 #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Factor(Index Code):%d\r\n",TempShort); //测试使用
						 #endif
						//参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x0F:                                                        //配置433模块扩频带宽参数
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //获得配置扩频带宽参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);			
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);
						 SetNodeFrequencyExpandBandwidth(TempShort);                  //设置433模块扩频带宽
						 TempShort  =GetNodeFrequencyExpandBandwidth();               //获取433模块扩频带宽参数
	           #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Bandwidth(Index Code):%d\r\n",TempShort); //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x11:                                                     //配置433模块工作模式参数，谨慎修改!!!
				{
           if(DataLenth ==1)
					 {
//						 TempShort =RecevCommandArry[3];                           //获得配置工作模式参数
//						 SetNodeWorkMode(TempShort);                               //设置433模块工作模式
//						 TempShort  =GetNodeWorkMode();                            //获取433模块工作模式参数
//						 ConfigReply[9] =1;
//						 ConfigReply[10] =TempShort ;   
//						 mput_mix_sx1278((char*)ConfigReply, 11);                  //参数配置应答帧
//					 //参数存入Flash，有待完善
						 #if DEBUG_TEST	 
						 printf("\r\nNode Work Mode Config not allowable !!\r\n"); //测试使用
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x13:                                                     //配置433模块客户ID
				{
					 if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];  //获得配置客户ID参数
						 ConfigReply[0] =TempLong>>8;                              //重新更新节点ID字段
             ConfigReply[1] =TempLong &0xFF;                           //重新更新节点ID字段
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix_sx1278((char*)ConfigReply, 12);                  //参数配置应答帧
						 Delay_ms(2000);
						 mput_mix_sx1278((char*)ConfigReply, 12);                  //参数配置应答帧
						 Delay_ms(1000);
	
						 SetNodeID(TempLong);                                      //设置433模块客户ID
						 TempLong =GetNodeID();                                    //读取433模块客户ID参数				 
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Node ID is:%x\r\n",TempLong); //测试使用
						 #endif
						 	//参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }
				case 0x15:                                                        //配置433模块网络ID
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //获得配置网络ID参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);	
						 
						 SetNodeNetworkID(TempShort);                              //设置433模块网络ID
						 TempShort  = GetNetworkID();                              //获取433模块网络ID参数
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Network ID is:%x\r\n",TempShort);  //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				case 0x17:                                                     //配置433模块发射功率等级
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置发射功率等级参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);							 
						 
						 SetNodeSendPowerGrade(TempShort);                         //设置433模块发射功率等级
						 TempShort =GetNodeSendPowerGrade();                       //获取433模块发射功率等级参数
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Send Power Grade is:%x\r\n",TempShort);  //测试使用
						 #endif				 
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				case 0x19:                                                     //配置433模块呼吸周期
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置呼吸周期参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);	
						 
						 SetNodeBreathPeriod(TempShort);                           //设置433模块呼吸周期
						 TempShort =GetNodeBreathPeriod();                         //获取433模块呼吸周期参数
             #if DEBUG_TEST	 
					   printf("\r\nNode Breath Period is:%x\r\n",TempShort);  //测试使用
						 #endif	
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				case 0x1B:                                                     //配置433模块呼吸时间
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //获得配置呼吸时间参数
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //参数配置应答帧,先发送应答，再更新数据，否则无法通信成功
						 Delay_ms(1000);	
						 
						 SetNodeBreathTime(TempShort);                             //设置433模块呼吸时间
						 TempShort =GetNodeBreathTime();                           //获取433模块呼吸时间参数
						 #if DEBUG_TEST	 
					   printf("\r\nNode Wake Time is:%x\r\n",TempShort);         //测试使用
						 #endif
						 //参数存入Flash，有待完善
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
						 #endif
           }
					 break;
        }	
				default:
				{
					 #if DEBUG_TEST	
           printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
					 #endif
					 break;
        }
     }
   }
	 else if(UseFlag ==1)
	 {
		 //参数存入Flash，不立即更新模块参数,待完善
	 }
	 else
	 {
     //不做处理 ,待完善
   }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 433远程查询参数命令解析
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterInquire_SX1278(u8 InquireCommand)
{

   ;//待完善

}


/*******************************************************************************
* Function Name  : XX
* Description    : 433模块接收数据解析（第一层）
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u16 Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress)	      
{
	 u16    ReceiveFlag =0;     //接收数据正确性标志变量
	 char*  pRecevBuff =NULL;
	 char   RecevFromCollector[4]={0xAA,0x00,0x00,0x00};       //接收数据集中器数据标志序列
	 char   RecevFromConfig   [4]={0xAF,0xFF,0xAF,0xEE};       //接收参数配置器数据标志序列
   u8     PayloadLen =0;
   u16    CrcVerify =0x0000;
   u16    CrcRecev  =0x0000;

	 #if DEBUG_TEST
   printf("\r\n接收数据解析!!\r\n");          //测试使用
	 #endif
   RecevFromCollector[2] = sNodeAddress>>8;             //433节点地址高字节
   RecevFromCollector[3]  = sNodeAddress&0xff;           //433节点地址低字节

///////////////////////////////////////////////////////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff, RecevFromCollector ,sizeof(Usart3_recev_buff), sizeof(RecevFromCollector));  //检查是否收到集中器应答
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //防止指针越界          
   {	 
			PayloadLen =pRecevBuff[6];
		  if(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-10-PayloadLen))  //防止指针越界    
		  {
        ReceiveFlag = ReceiveMessageVerify( pRecevBuff );     //第一层校验
				if(ReceiveFlag==0)                                    //当前查找到的接收数据有误
				{
					pRecevBuff =NULL;
					#if DEBUG_TEST	 
					printf("\r\nReceive  data not correct!!\r\n");      //测试使用
					#endif
				}
				else                                                  //接收数据正确
				{				
				  if(((u8)pRecevBuff[9]==0xA3)&&(pRecevBuff[10]==0x20))//第二层校验
					{
              PayloadLen =pRecevBuff[11]*256+pRecevBuff[12]+4;  //结算CRC校验数据长度
						  // Validate the CRC of this message
              CrcVerify = CRC16((u8*)(pRecevBuff+9), PayloadLen);
						  CrcRecev  = pRecevBuff[PayloadLen+9]*256 +pRecevBuff[PayloadLen+10];  //上位机CRC高字节在前低字节在后，与上位机保持一致
						  if(CrcRecev ==CrcVerify)
							{
                  #if DEBUG_TEST	 
									printf("\r\nReceive  data right!!\r\n");      //测试使用
									#endif
									Treaty_Data_Analysis(((u8*)pRecevBuff+9), &ReceiveFlag, pDeviceID, sNodeAddress);  //解析接收数据
              }
							else
							{
									#if DEBUG_TEST	 
									printf("\r\nReceive  data not correct!!\r\n");      //测试使用
									#endif
              }
          }	
					Delay_ms(500);	
					pRecevBuff =NULL;
					return ReceiveFlag;
				}
      }
			else
			{
          pRecevBuff =NULL;
					#if DEBUG_TEST	 
					printf("\r\nReceive data not integrated!\rReceive buff overflow!\r\n"); //测试使用
					#endif
      }
	 }
	 
	 ///////////////////////////////////////////////////////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff, RecevFromConfig, sizeof(Usart3_recev_buff), sizeof(RecevFromConfig));  //检查是否收到参数配置器应答
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //防止指针越界          
   {
		  if(pRecevBuff[4] ==0x00)
			{
         ParameterConfig_SX1278(pRecevBuff+5);
      }
			else if(pRecevBuff[4] ==0x01)
			{
         ParameterInquire_SX1278(pRecevBuff[5]);
      }
			else
			{
					#if DEBUG_TEST	 
					printf("\r\nReceive 433 Config Command not correct!\r\n"); //测试使用
					#endif
      }
			pRecevBuff =NULL;
			return 0;
	 }
		 
	 #if DEBUG_TEST	 
	 printf("\r\n接收数据解析完成!!\r\n");                 //调试使用
   #endif
	 return 0;
}


/*******************************************************************************
* Function Name  : XX
* Description    : 初始化一些配置参数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ConfigData_Init(struct DeviceSet* Para )
{

		u16      Temp =0;
		float    fTemp=0;

		printf("\r\nConfigData_Init start...\r\n");  //测试使用
		BKP_TamperPinCmd(DISABLE);                   //
////////////////////////////////////////////////////////////////////////////////////////////
		DataRead_From_Flash(0,2,0, ConfigData.CollectPeriod_Byte,2); //从Flash中读取液位计采集间隔，单位分钟
		Temp =ConfigData.CollectPeriod_Byte[1]*256 + ConfigData.CollectPeriod_Byte[0];   //低地址对应低字节
		if((Temp>0)&&(Temp<=1440))  
		{
			Para->CollectPeriod = Temp;
		}
		else
		{
			Para->CollectPeriod =1440;           //默认配置，每隔1440分钟采集一次数据。
		}
////////////////////////////////////////////////////////////////////////////////////////////		
		DataRead_From_Flash(0,3,0, ConfigData.CollectNum_Byte,2);     //从Flash中读取液位计每次数据上报采集的数据个数
		Temp =ConfigData.CollectNum_Byte[1]*256 + ConfigData.CollectNum_Byte[0];    //低地址对应低字节
		if((Temp>0)&&(Temp<=5))             //由于433模块的限制，每次数据上报最多采集5组数据，一帧数据长度应控制在100字节以内。
		{
			Para->CollectNum = Temp;
		}
		else
		{    
			Para->CollectNum =1;               //默认配置，每次数据上报采集1组数据。
		}	
////////////////////////////////////////////////////////////////////////////////////////////		
		DataRead_From_Flash(0,4,0, ConfigData.UploadCycle_Byte,2);     //从Flash中读取液位监测仪数据上报周期
		Temp =ConfigData.UploadCycle_Byte[1]*256 + ConfigData.UploadCycle_Byte[0];
		if((Temp>0)&&(Temp<=1440))          //每次数据上报最多采集20组数据。
		{
			Para->UploadCycle = Temp;
		}
		else
		{    
			Para->UploadCycle =1440;           //默认配置，液位监测仪数据上报周期为24小时，即每天上传1次数据。
		}	
////////////////////////////////////////////////////////////////////////////////////////////		
		DataRead_From_Flash(0,5,0, ConfigData.CollectStartTime_Byte,2); 
    Temp =ConfigData.CollectStartTime_Byte[1]*256 + ConfigData.CollectStartTime_Byte[0];    //低地址对应低字节
		if((Temp>0)&&(Temp<=1440))            //
		{
			Para->CollectStartTime = Temp;
		}
		else
		{    
			Para->CollectStartTime =240;               //默认配置，数据采集起始时间为凌晨4点。
		}	
/////////////////////////////////////////////////////////////////////////////////////////////	
    DataRead_From_Flash(0,6,0, &(ConfigData.RetryNumSet),1);     //从Flash中读取液位监测仪数据传输时的故障重传次数
		Temp =ConfigData.RetryNumSet;
		if((Temp>0)&&(Temp<=10))         //重传次数上限为10
		{
			Para->RetryNum = Temp;
		}
		else
		{    
			Para->RetryNum =3;            //默认配置，数据丢失或者出错时最多重试3次。
		}	
/////////////////////////////////////////////////////////////////////////////////////////////	
//		DataRead_From_Flash(0,7,0, ConfigData.Threshold.Data_Hex,4);   //从Flash中读取预设报警阈值
//		fTemp = ConfigData.Threshold.Data_Float;
//		//	if((fTemp>0.01)&&(length<9.5))                   //报警阈值上限根据探头量程和盲区而定
//		if(fTemp>0.01)                                       //报警阈值上限根据探头量程和盲区而定
//		{
//			 Para->AlarmThreshold = fTemp;      
//		}
//		else                                                     
//		{ 
//			 Para->AlarmThreshold =0;                        //读取数据无效时，将报警阈值设为0，当报警阈值为0时，不会触发报警事件
//		}
/////////////////配置参数约束条件判断///////////////////////////////////
  if(DeviceConfig.CollectNum * DeviceConfig.CollectPeriod >DeviceConfig.UploadCycle)
	{
    if(DeviceConfig.UploadCycle <DeviceConfig.CollectPeriod)
		{
       DeviceConfig.UploadCycle =DeviceConfig.CollectPeriod;
    }	
		DeviceConfig.CollectNum =DeviceConfig.UploadCycle/DeviceConfig.CollectPeriod;           //对数据采集次数进行修正
  }
	if(DeviceConfig.CollectNum >3)
	{
    DeviceConfig.CollectNum =3;           //由于数据发送以及数据存储空间的限制，数据采集次数必须限制在3以内
  }
/////////////////////////////////////////////////////////////////////////////////////////////	
		Para->Time_Sec  =0x00;
		Para->Time_Min  =0x00;
		Para->Time_Hour =0x00;
		Para->Time_Mday =0x01;
		Para->Time_Mon  =0x03;
		Para->Time_Year =0x10;
		Para->BatteryCapacity =0x64;    //电池电量，暂定为100%  

}

/**********************************************END******************************************/




/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//uint8_t Char2Hex(uint8_t* HexArry, char* CharArry, uint8_t Length_CharArry)
//{
//	uint8_t   i=0,j=0;         
//	uint8_t   val=0;     
////	char*     pChar = CharArry;
//	
//  for(i=0; i<Length_CharArry; i++)
//	{
//		
////		val = *pChar;
//		val = CharArry[i];
//		if((val >= '0')&&(val <= '9'))
//		{
//			HexArry[j++] = val-'0';	
//		}
//		else if(val == '.')
//		{
//			HexArry[j++] = val;	
//		}
////		pChar++;
//	}	
//  return  j;	
//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void ParaUpdateCheck(void)
//{
//   printf("\r\nPhone Num Set OK!!%4x---%4x---%4x\r\n",ConfigData.PhoneNum[0],ConfigData.PhoneNum[1],ConfigData.PhoneNum[2]);
//	 if((ConfigData.PhoneNum[0])&&(ConfigData.PhoneNum[1])&&(ConfigData.PhoneNum[2]))
//		{
//      BKP_WriteBackupRegister(BKP_DR4, ConfigData.PhoneNum[0]);  
//		  RTC_WaitForLastTask();
//			BKP_WriteBackupRegister(BKP_DR5, ConfigData.PhoneNum[1]);  
//		  RTC_WaitForLastTask();
//			BKP_WriteBackupRegister(BKP_DR6, ConfigData.PhoneNum[2]);  
//		  RTC_WaitForLastTask();
//    }
//		if(ConfigData.Threshold != 0)
//		{
//      BKP_WriteBackupRegister(BKP_DR7, ConfigData.Threshold);  
//		  RTC_WaitForLastTask();
//    }
//		if((ConfigData.ServerIP[0])&&(ConfigData.ServerIP[1]))
//		{
//      BKP_WriteBackupRegister(BKP_DR8, ConfigData.ServerIP[0]);  
//		  RTC_WaitForLastTask();
//			BKP_WriteBackupRegister(BKP_DR9, ConfigData.ServerIP[1]);  
//		  RTC_WaitForLastTask();
//    }
//	  if(ConfigData.ServerPort != 0)
//		{
//      BKP_WriteBackupRegister(BKP_DR10, ConfigData.ServerPort);  
//		  RTC_WaitForLastTask();
//    }
//	  RCC_ClearFlag();                              /* Clear reset flags */
//}





