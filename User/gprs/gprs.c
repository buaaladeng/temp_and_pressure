
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

char             Usart3_recev_buff[RECEIVEBUFF_SIZE]={0x00};     //USART3���ջ���
u16              Usart3_recev_count=0;           //USART3���ռ�����
extern struct    Config_RegPara   ConfigData;    //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
extern struct    DeviceSet  DeviceConfig;        //Һλ��������Ϣ�ṹ��
extern u8        DataCollectCount;               //���ݲɼ�������
extern u8        LiquidDataSend_Flag;
extern u8        DMA_UART3_RECEV_FLAG ;          //USART3 DMA���ձ�־����
extern vu8       Uart4_rev_comflag;              //RS485���ڽ�����ɱ�־����
extern u8        Uart4_rev_buff[100];
extern vu8       Uart4_rev_count;                //RS485���ڽ��ռ�����
extern u8        Usart2_send_buff[SENDBUFF_SIZE];//USART2���ͻ���

extern u16       DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3�������ݼ�������ݽ���
extern void      LevelDataMessageSend(void);     // ͨ�����ŷ���Һλ����

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
void USART_DataBlock_Send(USART_TypeDef* USART_PORT,char* SendUartBuf,u16 SendLength)    //�����򴮿ڷ�������
{
    u16 i;
        
    for(i=0;i<SendLength;i++)
    {
        USART_SendData(USART_PORT, *(SendUartBuf+i));
        while (USART_GetFlagStatus(USART_PORT, USART_FLAG_TC) == RESET);
    } 
}
void mput_mix(char *str,int length)                  //���ڸ���433ģ�����ò��������ݷ�������Ժ�������SET���ţ�����ģ�鹦�Ĺ���
{
	printf("length:%d\r\n",length);                    //����ʹ��
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //��USART2��������ǰ���ȴ�USART2���տ����жϣ����ڼ�����ݽ������
//	USART_ClearFlag(USART2,USART_FLAG_TC);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(200);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433ģ��EN�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(300);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
//	Delay_ms(1000);
//	USART_ClearFlag(USART2,USART_FLAG_TC);
}
void mput_mix_sx1278(char *str,int length)           //����433ģ�鷢�����ݣ����ݷ�������Ժ��������SET���ţ�����ģ�龭�����ղ�������
{
	printf("length:%d\r\n",length);                    //����ʹ��
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(200);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433ģ��EN�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(500);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
	Delay_ms(800);                                     
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433ģ��SET�ܽ����ߣ��л�������ģʽ
	Delay_ms(100);
//	USART_ClearFlag(USART2,USART_FLAG_TC);
}

void mput(char* str)
{
//	printf("length:%d\r\n",strlen(str));     //����ʹ��
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //��USART2��������ǰ���ȴ�USART2���տ����жϣ����ڼ�����ݽ������
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ͣ��л�������ģʽ
	Delay_ms(500);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433ģ��EN�ܽ����ͣ��л�������ģʽ
	Delay_ms(500);
	USART_DataBlock_Send(USART2,str,strlen(str));
	USART_DataBlock_Send(USART2,"\r\n",2);
	USART_DataBlock_Send(USART1,str,strlen(str));
	USART_DataBlock_Send(USART1,"\r\n",2);
	Delay_ms(800);
//	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433ģ��SET�ܽ����ߣ��л�������ģʽ
//	Delay_ms(100);
}
//void mput_mix(char *str,int length)
//{
//	printf("length:%d\r\n",length);             //����ʹ��
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //��USART3��������ǰ���ȴ�USART3���տ����жϣ����ڼ�����ݽ������
//	USART_DataBlock_Send(USART3,str,length);
//	USART_DataBlock_Send(USART3,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
//}
//void mput(char* str)
//{
////	printf("length:%d\r\n",strlen(str));     //����ʹ��
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //��USART3��������ǰ���ȴ�USART3���տ����жϣ����ڼ�����ݽ������
//	USART_DataBlock_Send(USART3,str,strlen(str));
//	USART_DataBlock_Send(USART3,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,strlen(str));
//	USART_DataBlock_Send(USART1,"\r\n",2);
//}
/*******************************************************************************
* Function Name  : char* Find_String(char* Source, char* Object)
* Description    : ��Ŀ���ַ����з���һ��ָ�����ַ���
* Input          : 
* Output         : 
* Return         : ����ҵ����򷵻�Ŀ���ַ�����Դ�ַ����е��׵�ַ
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
/***********�������ܣ����ض������з���һ��ָ��������****************/
/***********����ҵ����򷵻�Ŀ��������Դ�����е��׵�ַ**************/
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
  u16  TempShort =0x0000;        //��������ʱ����

	switch(ParaRequest)
	{
		case CLT1_ITRL1:               //�洢Һλ���ݲɼ����
		{
			 DataWrite_To_Flash(0,2,0,(u8*)pSetPara, ParaLen);                    //�����ݲɼ����д��Flash ���͵�ַ��Ӧ���ֽ�
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //����ʹ��
       printf("\r\n CollectPeriod:%2x\r\n",TempShort);	                    //����ʹ��
			 return 1;
		}
		case CLT1_CNT1:                //�洢���������ϴ���Һλ���ݲɼ�����
		{
			 DataWrite_To_Flash(0,3,0,(u8*)pSetPara, ParaLen);                    //�����ݲɼ�����д��Flash ���͵�ַ��Ӧ���ֽ�
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //����ʹ��
       printf("\r\n CollectNum:%2x\r\n",TempShort);	                        //����ʹ��		 
			 return 1;
		}
		case UPLOAD_CYCLE:             //�洢Һλ�����ϴ�����
		{
       DataWrite_To_Flash(0,4,0,(u8*)pSetPara, ParaLen);                    //�����ݲɼ�����д��Flash ���͵�ַ��Ӧ���ֽ�
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //����ʹ��
       printf("\r\n UploadCycle:%2x\r\n",TempShort);	                      //����ʹ��		 
			 return 1;
			 
		}
		case CLT1_STIME1:            //�洢�豸�����ϴ���ʼʱ�䣨���豸����ʱ�䣩
		{
			 DataWrite_To_Flash(0,5,0,(u8*)pSetPara, ParaLen);                    //�������ϴ���ʼʱ��д��Flash ���͵�ַ��Ӧ���ֽ�
			 TempShort =*(pSetPara)+*(pSetPara+1)*256;                            //����ʹ��
			 printf("\r\n CLT1_STIME1:%2x\r\n",TempShort);	                      //����ʹ��	
			 return 1;
		}
		case DEF_NR:                  //�洢���ݷ��ʹ����ش�����
		{
       DataWrite_To_Flash(0,6,0,(u8*)pSetPara, ParaLen);                    //�����ݲɼ�����д��Flash ���͵�ַ��Ӧ���ֽ�
			 TempShort =*(pSetPara);                                              //����ʹ��
       printf("\r\n UploadCycle:%x\r\n",TempShort);	                        //����ʹ��		 
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
* Input          : FunctionNum (0���ָ��������ã�1����ȡ̽ͷ�����¶ȣ��棩��2�������¶Ȳ������ӣ�3����ȡ�¶Ȳ������ӣ�);ParaData(����������Ϊ2ʱ���������ݲ���Ч)
* Output         : None
* Return         : None
*******************************************************************************/
//void  UltrasonicParaConfig(u8 FunctionNum, char* ParaData)
//{

//   char  UltrasonicRecovery[21] ="AT+EEPROM=RECOVERY\r\n";      //������̽ͷ�ָ�������������
//	 char  UltrasonicTempConsult[13] ="AT+MENU+01?\r\n";          //������̽ͷ�����¶Ȳ�ѯ����
//   char  UltrasonicTempOffsetConfig[17] ={0x00};                //������̽ͷ�¶Ȳ���������������	
//   char  UltrasonicTempOffsetConsult[13]="AT+MENU+28?\r\n";     //������̽ͷ�¶Ȳ������Ӳ�ѯ����
//   char  UltrasonicParaSave[17]="AT+EEPROM=WRITE\r\n";          //������̽ͷ������������
//   char  SmsSendTemp[20] ={0x00}; 
//   u8    RetryCount=3, i=0;
//   
//	  if((DataCollectCount!=0)||(LiquidDataSend_Flag==1))   //δ������ݲɼ����������Ѿ��ϱ���ɣ�������̽ͷ��485��Դ���ѹرգ���Ҫ���¿���̽ͷ��485��Դ
//	  {
//       PowerON_UltrasonicSensor();        //�򿪳�����̽ͷ��Դ����TCP������ȷ�����Ժ��ٿ���̽ͷ��Դ�����������Ҫ������ʷ���ݣ�����Ҫ����Ӧ�޸�
//	     Delay_ms(1000);   
//       PowerON_485();                     //��485��Դ
//			 Delay_ms(7000); 
//    }	
//    Delay_ms(3000);		      //�����ʵ���ʱ����ֹ����ʱ������̽ͷ���ȴ�̽ͷ�������ȶ�
//    switch(FunctionNum)
//		{
//        case 0:                   
//				{
//           printf("\r\nUltrasonic recovery is in process...\r\n");    //����ʹ�� 		
//					 for(i=RetryCount;i>0;i--)
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485����ʹ��
//							Delay_ms(50);                           
//							USART_DataBlock_Send(UART4,UltrasonicRecovery,strlen(UltrasonicRecovery));
//							Delay_us(100);
//							DIR485_Receive();                        //485����ʹ��
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic recovery receive_%d:%s\r\n",i,Uart4_rev_buff); //����ʹ��
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //���UART4����BUFF
//								 Uart4_rev_count =0;                                       //��λUART4���ռ�����
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					 break;
//        }	     
//				case 1:
//				{
//           printf("\r\nUltrasonic current temperature inquire is in process...\r\n");    //����ʹ�� 		
//					 for(i=RetryCount;i>0;i--)
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485����ʹ��
//							Delay_ms(50);                            //ԭʼ3ms
//							USART_DataBlock_Send(UART4,UltrasonicTempConsult,strlen(UltrasonicTempConsult));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature receive_%d:%s\r\n",i,Uart4_rev_buff); //����ʹ��
//								 if(strlen((char*)Uart4_rev_buff) <20)
//								 {
//                    memcpy(SmsSendTemp,Uart4_rev_buff,strlen((char*)Uart4_rev_buff));
//                 }
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //���UART4����BUFF
//								 Uart4_rev_count =0;                                       //��λUART4���ռ�����
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					 break;
//        }	
//       
//				case 2:
//				{
//           printf("\r\nUltrasonic temperature offset config is in process...\r\n");    //����ʹ�� 		
//					 for(i=RetryCount;i>0;i--)                    //�����¶Ȳ�������
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485����ʹ��
//							Delay_ms(50);                            //ԭʼ3ms
//						  snprintf(UltrasonicTempOffsetConfig,sizeof(UltrasonicTempOffsetConfig),"AT+MENU+28=%s\r\n",ParaData); //����̽ͷ�¶Ȳ�������
//							USART_DataBlock_Send(UART4,UltrasonicTempOffsetConfig,strlen(UltrasonicTempOffsetConfig));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature offset config receive_%d:%s\r\n",i,Uart4_rev_buff); //����ʹ��
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //���UART4����BUFF
//								 Uart4_rev_count =0;                                       //��λUART4���ռ�����
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           } 
//					 for(i=RetryCount;i>0;i--)                    //�������ò���
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485����ʹ��
//							Delay_ms(50);                            //ԭʼ3ms
//							USART_DataBlock_Send(UART4,UltrasonicParaSave,strlen(UltrasonicParaSave));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature offset save ok_%d:%s\r\n",i,Uart4_rev_buff); //����ʹ��
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //���UART4����BUFF
//								 Uart4_rev_count =0;                                       //��λUART4���ռ�����
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					break;  
//        }	
//				
//        case 3:
//				{
//           printf("\r\nUltrasonic temperature offset consult is in process...\r\n");    //����ʹ�� 		
//					 for(i=RetryCount;i>0;i--)
//					 {
//              Uart4_rev_comflag=0;              
//							DIR485_Send();                           //485����ʹ��
//							Delay_ms(50);                            
//							USART_DataBlock_Send(UART4,UltrasonicTempOffsetConsult,strlen(UltrasonicTempOffsetConsult));
//							Delay_us(200);
//							DIR485_Receive();
//							Delay_ms(3000);
//							if(Uart4_rev_comflag ==1)
//							{
//								 printf("\r\nUltrasonic temperature offset consult receive_%d:%s\r\n",i,Uart4_rev_buff); //����ʹ��
//								 if(strlen((char*)Uart4_rev_buff)<20)
//								 {
//                    memcpy(SmsSendTemp,Uart4_rev_buff,strlen((char*)Uart4_rev_buff));
//                 }
//								 memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));       //���UART4����BUFF
//								 Uart4_rev_count =0;                                       //��λUART4���ռ�����
//								 Uart4_rev_comflag =0;
//								 break;
//              }		
//           }
//					 break;
//        }	
//        default:
//				{
//           printf("\r\nWarning!!Input function number error!\r\n"); //����ʹ��
//					 break;
//        }
//    }	
//}

/*******************************************************************************
* Function Name  : Э�����ݷ���
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress)	
{
	u8   DataLen =0;          //Tag���л���OID���еĳ���
	u16  PduType =0;
  u8   ProbeReset =0;       //̽ͷ��λ��־����
	u8   i=0,j=0;
	u8*  pChar =NULL;
	u32* pOid  =NULL;
	union  Hfloat  UnionData1;
	struct CommandFrame ParaRequest;
	
  PduType =pTreatyBuff[13]*256 +pTreatyBuff[14];   //�����������PDU����
  switch(PduType)
	{
        case 0x0188:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Get Request Command from Server.\r\n");    //����ʹ��
						#endif
						*pFlag =PduType;
					 //////////////////////////////////////////
					  DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12; //���յ���OID���е��ܳ���
					  ParaRequest.OID_Count =DataLen/4;               //���յ���OID��������
					  if((ParaRequest.OID_Count >0)&&(ParaRequest.OID_Count <=20))   //�޶�OID���ȣ���ֹ�ڴ����
						{
							  pOid = (u32*)&pTreatyBuff[16];
							  j    =ParaRequest.OID_Count;
								for(i=0;i<ParaRequest.OID_Count;i++)
								{ 
									  ParaRequest.OID_List[i] =(CommandType)*pOid;                    //OID�ߵ��ֽ�˳���д�ȷ��    
									  ParaRequest.OID_List[i] =ntohl( ParaRequest.OID_List[i] );      //������ת��Ϊ�����򣬼������ݵĵ��ֽڷ��ڵ͵�ַ
									  printf("\r\n--Cycle--%4x----.\r\n", ParaRequest.OID_List[i]);   //����ʹ��
										j--;
									  if(j==0)   
										{
                       break;
                    }
									  pOid++;
								}
								printf("\r\n--i:%d--j:%d--%4x----.\r\n",i,j, ParaRequest.OID_List[i]);    //����ʹ��
								
            }
					  else
						{
                #if DEBUG_TEST	 
							  printf("\r\nReceive Command OID not correct.\r\n");    //����ʹ��
							  #endif
            }
//						if(DMA_UART3_RECEV_FLAG==1)                      //��ѯ���ݽ������
//						{
//							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //��ѯ�����·���������������ò�ѯ����
//						}
						DMA_UART3_RECEV_FLAG =0;     //���ձ�־������λ
					  GetResponse( ParaRequest, Usart2_send_buff, pDeviceID,  NodeAddress);
					  break;
        }	
        case 0x0388:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Set Request Command from Server.\r\n");    //����ʹ��
						#endif
						*pFlag =PduType;
					  DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12;    //���յ���Tag���е��ܳ���
					  if(DataLen >6)           //���ٴ���һ���Ϸ������ò���
						{
                pOid = (u32*)(pTreatyBuff+16);
							  i=0;
							  while( DataLen >6 )
								{
									 printf("\r\n--Cycle--%4x----.\r\n",ntohl(*pOid));         //����ʹ��
									 pChar = (u8*)pOid;
                   switch(ntohl(*pOid))
									 {
											case DEF_NR:            //�ش�����
											{
													DeviceConfig.RetryNum =*(pChar+6);
												  if(DeviceConfig.RetryNum >=1)                         //�����Ϸ����ж�
													{
                             UploadFlash(&(DeviceConfig.RetryNum), 1, DEF_NR);  //��������Flash
                          }
												  printf("\r\n-Retry Num-%x---.\r\n", DeviceConfig.RetryNum);   //����ʹ��
//												  else               //�����ò������Ϸ�ʱ��ʹ��Ĭ�ϲ�����ͬʱ������Flash����
//													{
//                             DeviceConfig.RetryNum =1;             //Ϊ�˷�����ʵ���������Ĭ�ϲ���
//                          }
												  ParaRequest.OID_List[i] =DEF_NR;           
												  pOid =(u32*)(pChar+7);                     //ָ�����1��Tag
												  DataLen = DataLen-7; 
 	                        i++;               //�Ϸ�OID������												
													break;
											}	
											case SYSTERM_TIME:     //ϵͳʱ��
											{
													DeviceConfig.Time_Year =*(pChar+6);
												  DeviceConfig.Time_Mon  =*(pChar+7);
												  DeviceConfig.Time_Mday =*(pChar+8);
												  DeviceConfig.Time_Hour =*(pChar+9);
												  DeviceConfig.Time_Min  =*(pChar+10);
												  DeviceConfig.Time_Sec  =*(pChar+11);
												  printf("\r\n-��-��-��-ʱ-��-�룺-%d--%d--%d--%d--%d--%d--.\r\n",DeviceConfig.Time_Year,DeviceConfig.Time_Mon,DeviceConfig.Time_Mday,
												                 DeviceConfig.Time_Hour,DeviceConfig.Time_Min, DeviceConfig.Time_Sec  );         //����ʹ��
												  if((DeviceConfig.Time_Mon<=12)&&(DeviceConfig.Time_Mday<=31)&&(DeviceConfig.Time_Hour<=23)&&(DeviceConfig.Time_Min<=60)&&(DeviceConfig.Time_Sec<=60)) //�����Ϸ����ж�
													{
                            Time_Auto_Regulate(&DeviceConfig);             //ͨ���������·���������RTCʱ��У׼��
                          }
												  ParaRequest.OID_List[i] =SYSTERM_TIME;            
												  pOid =(u32*)(pChar+12);                          //ָ�����1��Tag
												  DataLen = DataLen-12;
												 	i++;        //�Ϸ�OID������
													break;							
											}	
											case CLT1_ITRL1:       //һʱ���ɼ����
											{
													DeviceConfig.CollectPeriod =pChar[6]*256+pChar[7];
												  printf("\r\n-CollectPeriod:-%2x---.\r\n", DeviceConfig.CollectPeriod );         //����ʹ��
												  if(DeviceConfig.CollectPeriod<=1440)              //�����Ϸ����ж�
													{
                             UploadFlash((u8*)&(DeviceConfig.CollectPeriod), 2, CLT1_ITRL1);  //��������Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_ITRL1;            
												  pOid =(u32*)(pChar+8);                         //ָ�����1��Tag
												  DataLen = DataLen-8;
												 	i++;        //�Ϸ�OID������
													break;
											}	
											case CLT1_CNT1:        //һʱ���ɼ�����
											{
													DeviceConfig.CollectNum =pChar[6]*256+pChar[7];
													printf("\r\n-CollectNum:-%2x---.\r\n", DeviceConfig.CollectNum );         //����ʹ��
												  if(DeviceConfig.CollectNum<=1440)              //�����Ϸ����ж�
													{
                             UploadFlash((u8*)&(DeviceConfig.CollectNum), 2, CLT1_CNT1);  //��������Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_CNT1;            
												  pOid =(u32*)(pChar+8);                         //ָ�����1��Tag
												  DataLen = DataLen-8;
												 	i++;        //�Ϸ�OID������
													break;
											}
											case CLT1_STIME1  :    //һʱ���ɼ���ʼʱ��
											{
													
												  DeviceConfig.CollectStartTime =pChar[6]*256+pChar[7];
													printf("\r\n-Collect Start Time:-%2x---.\r\n", DeviceConfig.CollectStartTime );        //����ʹ��
												  if(DeviceConfig.CollectStartTime<=1440)              //�����Ϸ����ж�
													{
                             UploadFlash((u8*)&(DeviceConfig.CollectStartTime), 2, CLT1_STIME1);  //��������Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_STIME1;            
												  pOid =(u32*)(pChar+8);                              //ָ�����1��Tag
												  DataLen = DataLen-8;
												 	i++;        //�Ϸ�OID������
													break;
											}	
//											case RESET_PROBER:     //��λҺλ̽ͷ���������Կ��Ƿŵ�״̬����
//											{
//												  ProbeReset =*(pChar+6);                         //��ȡ��������λ̽ͷ�������
//												  printf("\r\n-RESET PROBER-%x---.\r\n", ProbeReset);   //����ʹ��
//												  if(ProbeReset==1)
//													{ 
//														UltrasonicParaConfig(0, 0);                     //��λ������̽ͷ
//                          }	  
//												  ParaRequest.OID_List[i] =RESET_PROBER;           
//												  pOid =(u32*)(pChar+7);                          //ָ�����1��Tag
//												  DataLen = DataLen-7;
//												 	i++;        //�Ϸ�OID������
//													break;
//											}	
											case UPLOAD_CYCLE:     //Һλ�������ϱ�����
											{
													DeviceConfig.UploadCycle =pChar[6]*256+pChar[7];
												  if(DeviceConfig.UploadCycle<=1440)              //�����Ϸ����ж�
													{
                             UploadFlash((u8*)&(DeviceConfig.UploadCycle), 2, UPLOAD_CYCLE);  //��������Flash
                          }
											  	printf("\r\n-UploadCycle:-%2x---.\r\n", DeviceConfig.UploadCycle ); //����ʹ��
												  ParaRequest.OID_List[i] =UPLOAD_CYCLE;          
												  pOid =(u32*)(pChar+8);                           //ָ�����1��Tag
												  DataLen = DataLen-8;
											  	i++;        //�Ϸ�OID������
													break;
											}					
											default:
											{
												 #if DEBUG_TEST	
												 printf("\r\nWarning!!Tag OID not recognition!\r\n"); //����ʹ��
												 #endif
												 pOid =(u32*)(pChar+1);  //ָ�����һ���ֽڣ���ѯ�������޺Ϸ�OID
												 DataLen = DataLen-1;    //ָ�����һ���ֽڣ���ѯ�������޺Ϸ�OID
												 break;
											}
									 }
                }
						}
						if((i>0)&&(i<=20))
						{
               ParaRequest.OID_Count =i;           //��OID���н��м���	
            }
            else
						{
               ParaRequest.OID_Count =7;           //��OID������ֵ����20ʱ,��������ֵ���ó�Ĭ��ֵ��Ĭ��7��
            }	
//            if(DMA_UART3_RECEV_FLAG==1)                      //��ѯ���ݽ������
//						{
//							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //��ѯ�����·���������������ò�ѯ����
//						}			
            DMA_UART3_RECEV_FLAG =0;     //���ձ�־������λ						
					  GetResponse( ParaRequest, Usart2_send_buff, pDeviceID,  NodeAddress);
					  break;
        }	
				case 0x0588:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Trap Response Command from Server.\r\n");    //����ʹ��
						#endif
					  /////////////////////////
					  //������������������֤������һ���ϴ�һ֡���ݵ������ֻ��ͨ��PDUType������֤�����յ�Ӧ��֡�������������
						*pFlag =PduType;
					  break;
        }	
				case 0x0988:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Startup Response Command from Server.\r\n");    //����ʹ��
						#endif
						*pFlag =PduType;
					  break;
        }	
				case 0x0A88:
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Wakeup Request Command from Server.\r\n");    //����ʹ��
						#endif
						*pFlag =PduType;
						if(DMA_UART3_RECEV_FLAG==1)                      //��ѯ���ݽ������
						{
							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //��ѯ�����·���������������ò�ѯ����
						}
					  WakeupResponse(Usart2_send_buff, pDeviceID, NodeAddress);
					  break;
        }	
				case 0x0088:         //��ʱʹ�ã���λ���д�����
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Get Response Receive OK by Server.\r\n");    //����ʹ��
						#endif
						*pFlag =PduType;
					  break;
        }	
        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!PDU Type not recognition!\r\n"); //����ʹ��
					 #endif
					 break;
        }
   }      
}
/*******************************************************************************
* Function Name  : XX
* Description    : 433Զ�����ò����������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterConfig_SX1278(char* pRecevCommand)
{
   u8   RecevCommandArry[5]={0x00};
	
	 u8   UseFlag   =0;    //���ò�����;��־������Ҳ����ָʾ���ò�����ʱЧ�ԣ�����Ϊ0ʱ������Ч��������Ϊ���ݲ���
	 u8   Opcode    =0;    //�������ֶΣ�����ָʾ����Ĳ�������
	 u8   DataLenth =0;    //��������
	 u16  TempLong  =0;
	 u8   TempShort =0;
	 u8   ConfigReply[12] ={0xAA,0xAA,0xAF,0xFF,0xAF,0xEE,0x00,0xAA,0xAA,0xAA,0xAA,0xAA}; //������0xAA�ֶ�Ϊ�����ֶΣ�����ֱ��ǣ��������ң���
     
                                                    //�ڵ��ַ���ֽڣ��ڵ��ַ���ֽڣ�������;�������룬���ò������ȣ����ò�����1�ֽڣ����ò�����2�ֽڣ����ܲ����ڣ�
   memcpy(RecevCommandArry, pRecevCommand, sizeof(RecevCommandArry));
   UseFlag   =RecevCommandArry[0];
	 Opcode    =RecevCommandArry[1];
	 DataLenth =RecevCommandArry[2];
   TempLong  =GetNodeID (); //��ȡ�ڵ�ID
   ConfigReply[0] =TempLong>>8;
   ConfigReply[1] =TempLong &0xFF;
	 ConfigReply[7] =UseFlag ;
   ConfigReply[8] =Opcode;
   Delay_ms(3000);
	 if(UseFlag ==0)
	 {
     switch(Opcode )
		 {
        case 0x09:                    //����433ģ�鴮�ڲ����ʲ����������޸�!!!
				{
           if(DataLenth ==2)
					 {
//						 SetNodeSerialPort(RecevCommandArry[3], RecevCommandArry[4]);     //����433ģ�鴮�ڲ����ʺ�У������
//						 PowerOFF_GPRS();                                           //����433ģ�飬ʹ������Ч        
//						 Delay_ms(5000);
//						 PowerON_GPRS();
//						 TempLong =GetNodeSerialPortConfig();                      //��ȡ433ģ�鴮�����ò���
//						 ConfigReply[9] =2;
//						 ConfigReply[10] =TempLong >>8;
//					   ConfigReply[11] =TempLong &0xFF;
//						 mput_mix_sx1278((char*)ConfigReply, 12);                  //��������Ӧ��֡
//						 //��������Flash���д�����
						 #if DEBUG_TEST	 
						 printf("\r\nSerial Port Parameters Config not allowable !!\r\n"); //����ʹ��
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x0B:                                                    //����433ģ���ز�Ƶ��
				{
           if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];      //�������Ƶ�ʲ���
					
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix_sx1278((char*)ConfigReply, 12);                //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);
						 mput_mix_sx1278((char*)ConfigReply, 12);                //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�,�෢һ��Ϊ�˽���״η������ݶ�ʧBUG
						 Delay_ms(1000);
						 SetNodeCentralFrequency(TempLong);                      //����433ģ���ز�����Ƶ��
						 TempLong =GetNodeCentralFrequency();                    //��ȡ433ģ���ز�Ƶ�ʲ���
	           #if DEBUG_TEST	 
					   printf("\r\nCentral Frequency :%d MHz\r\n",(TempLong+1));   //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x0D:                                                    //����433ģ����Ƶ���Ӳ���
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //���������Ƶ���Ӳ���
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                  //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                  //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);							 
						 SetNodeFrequencyExpandFactor(TempShort);                  //����433ģ����Ƶ����
						 TempShort  =GetNodeFrequencyExpandFactor();               //��ȡ433ģ����Ƶ���Ӳ���
			
						 #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Factor(Index Code):%d\r\n",TempShort); //����ʹ��
						 #endif
						//��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x0F:                                                        //����433ģ����Ƶ�������
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //���������Ƶ�������
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);			
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);
						 SetNodeFrequencyExpandBandwidth(TempShort);                  //����433ģ����Ƶ����
						 TempShort  =GetNodeFrequencyExpandBandwidth();               //��ȡ433ģ����Ƶ�������
	           #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Bandwidth(Index Code):%d\r\n",TempShort); //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x11:                                                     //����433ģ�鹤��ģʽ�����������޸�!!!
				{
           if(DataLenth ==1)
					 {
//						 TempShort =RecevCommandArry[3];                           //������ù���ģʽ����
//						 SetNodeWorkMode(TempShort);                               //����433ģ�鹤��ģʽ
//						 TempShort  =GetNodeWorkMode();                            //��ȡ433ģ�鹤��ģʽ����
//						 ConfigReply[9] =1;
//						 ConfigReply[10] =TempShort ;   
//						 mput_mix_sx1278((char*)ConfigReply, 11);                  //��������Ӧ��֡
//					 //��������Flash���д�����
						 #if DEBUG_TEST	 
						 printf("\r\nNode Work Mode Config not allowable !!\r\n"); //����ʹ��
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x13:                                                     //����433ģ��ͻ�ID
				{
					 if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];  //������ÿͻ�ID����
						 ConfigReply[0] =TempLong>>8;                              //���¸��½ڵ�ID�ֶ�
             ConfigReply[1] =TempLong &0xFF;                           //���¸��½ڵ�ID�ֶ�
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix_sx1278((char*)ConfigReply, 12);                  //��������Ӧ��֡
						 Delay_ms(2000);
						 mput_mix_sx1278((char*)ConfigReply, 12);                  //��������Ӧ��֡
						 Delay_ms(1000);
	
						 SetNodeID(TempLong);                                      //����433ģ��ͻ�ID
						 TempLong =GetNodeID();                                    //��ȡ433ģ��ͻ�ID����				 
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Node ID is:%x\r\n",TempLong); //����ʹ��
						 #endif
						 	//��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x15:                                                        //����433ģ������ID
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //�����������ID����
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);	
						 
						 SetNodeNetworkID(TempShort);                              //����433ģ������ID
						 TempShort  = GetNetworkID();                              //��ȡ433ģ������ID����
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Network ID is:%x\r\n",TempShort);  //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				case 0x17:                                                     //����433ģ�鷢�书�ʵȼ�
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //������÷��书�ʵȼ�����
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);							 
						 
						 SetNodeSendPowerGrade(TempShort);                         //����433ģ�鷢�书�ʵȼ�
						 TempShort =GetNodeSendPowerGrade();                       //��ȡ433ģ�鷢�书�ʵȼ�����
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Send Power Grade is:%x\r\n",TempShort);  //����ʹ��
						 #endif				 
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				case 0x19:                                                     //����433ģ���������
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //������ú������ڲ���
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);	
						 
						 SetNodeBreathPeriod(TempShort);                           //����433ģ���������
						 TempShort =GetNodeBreathPeriod();                         //��ȡ433ģ��������ڲ���
             #if DEBUG_TEST	 
					   printf("\r\nNode Breath Period is:%x\r\n",TempShort);  //����ʹ��
						 #endif	
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				case 0x1B:                                                     //����433ģ�����ʱ��
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //������ú���ʱ�����
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix_sx1278((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);	
						 
						 SetNodeBreathTime(TempShort);                             //����433ģ�����ʱ��
						 TempShort =GetNodeBreathTime();                           //��ȡ433ģ�����ʱ�����
						 #if DEBUG_TEST	 
					   printf("\r\nNode Wake Time is:%x\r\n",TempShort);         //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				default:
				{
					 #if DEBUG_TEST	
           printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
					 #endif
					 break;
        }
     }
   }
	 else if(UseFlag ==1)
	 {
		 //��������Flash������������ģ�����,������
	 }
	 else
	 {
     //�������� ,������
   }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 433Զ�̲�ѯ�����������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterInquire_SX1278(u8 InquireCommand)
{

   ;//������

}


/*******************************************************************************
* Function Name  : XX
* Description    : 433ģ��������ݽ�������һ�㣩
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u16 Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress)	      
{
	 u16    ReceiveFlag =0;     //����������ȷ�Ա�־����
	 char*  pRecevBuff =NULL;
	 char   RecevFromCollector[4]={0xAA,0x00,0x00,0x00};       //�������ݼ��������ݱ�־����
	 char   RecevFromConfig   [4]={0xAF,0xFF,0xAF,0xEE};       //���ղ������������ݱ�־����
   u8     PayloadLen =0;
   u16    CrcVerify =0x0000;
   u16    CrcRecev  =0x0000;

	 #if DEBUG_TEST
   printf("\r\n�������ݽ���!!\r\n");          //����ʹ��
	 #endif
   RecevFromCollector[2] = sNodeAddress>>8;             //433�ڵ��ַ���ֽ�
   RecevFromCollector[3]  = sNodeAddress&0xff;           //433�ڵ��ַ���ֽ�

///////////////////////////////////////////////////////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff, RecevFromCollector ,sizeof(Usart3_recev_buff), sizeof(RecevFromCollector));  //����Ƿ��յ�������Ӧ��
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //��ָֹ��Խ��          
   {	 
			PayloadLen =pRecevBuff[6];
		  if(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-10-PayloadLen))  //��ָֹ��Խ��    
		  {
        ReceiveFlag = ReceiveMessageVerify( pRecevBuff );     //��һ��У��
				if(ReceiveFlag==0)                                    //��ǰ���ҵ��Ľ�����������
				{
					pRecevBuff =NULL;
					#if DEBUG_TEST	 
					printf("\r\nReceive  data not correct!!\r\n");      //����ʹ��
					#endif
				}
				else                                                  //����������ȷ
				{				
				  if(((u8)pRecevBuff[9]==0xA3)&&(pRecevBuff[10]==0x20))//�ڶ���У��
					{
              PayloadLen =pRecevBuff[11]*256+pRecevBuff[12]+4;  //����CRCУ�����ݳ���
						  // Validate the CRC of this message
              CrcVerify = CRC16((u8*)(pRecevBuff+9), PayloadLen);
						  CrcRecev  = pRecevBuff[PayloadLen+9]*256 +pRecevBuff[PayloadLen+10];  //��λ��CRC���ֽ���ǰ���ֽ��ں�����λ������һ��
						  if(CrcRecev ==CrcVerify)
							{
                  #if DEBUG_TEST	 
									printf("\r\nReceive  data right!!\r\n");      //����ʹ��
									#endif
									Treaty_Data_Analysis(((u8*)pRecevBuff+9), &ReceiveFlag, pDeviceID, sNodeAddress);  //������������
              }
							else
							{
									#if DEBUG_TEST	 
									printf("\r\nReceive  data not correct!!\r\n");      //����ʹ��
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
					printf("\r\nReceive data not integrated!\rReceive buff overflow!\r\n"); //����ʹ��
					#endif
      }
	 }
	 
	 ///////////////////////////////////////////////////////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff, RecevFromConfig, sizeof(Usart3_recev_buff), sizeof(RecevFromConfig));  //����Ƿ��յ�����������Ӧ��
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //��ָֹ��Խ��          
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
					printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
					#endif
      }
			pRecevBuff =NULL;
			return 0;
	 }
		 
	 #if DEBUG_TEST	 
	 printf("\r\n�������ݽ������!!\r\n");                 //����ʹ��
   #endif
	 return 0;
}


/*******************************************************************************
* Function Name  : XX
* Description    : ��ʼ��һЩ���ò���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ConfigData_Init(struct DeviceSet* Para )
{

		u16      Temp =0;
		float    fTemp=0;

		printf("\r\nConfigData_Init start...\r\n");  //����ʹ��
		BKP_TamperPinCmd(DISABLE);                   //
////////////////////////////////////////////////////////////////////////////////////////////
		DataRead_From_Flash(0,2,0, ConfigData.CollectPeriod_Byte,2); //��Flash�ж�ȡҺλ�Ʋɼ��������λ����
		Temp =ConfigData.CollectPeriod_Byte[1]*256 + ConfigData.CollectPeriod_Byte[0];   //�͵�ַ��Ӧ���ֽ�
		if((Temp>0)&&(Temp<=1440))  
		{
			Para->CollectPeriod = Temp;
		}
		else
		{
			Para->CollectPeriod =1440;           //Ĭ�����ã�ÿ��1440���Ӳɼ�һ�����ݡ�
		}
////////////////////////////////////////////////////////////////////////////////////////////		
		DataRead_From_Flash(0,3,0, ConfigData.CollectNum_Byte,2);     //��Flash�ж�ȡҺλ��ÿ�������ϱ��ɼ������ݸ���
		Temp =ConfigData.CollectNum_Byte[1]*256 + ConfigData.CollectNum_Byte[0];    //�͵�ַ��Ӧ���ֽ�
		if((Temp>0)&&(Temp<=5))             //����433ģ������ƣ�ÿ�������ϱ����ɼ�5�����ݣ�һ֡���ݳ���Ӧ������100�ֽ����ڡ�
		{
			Para->CollectNum = Temp;
		}
		else
		{    
			Para->CollectNum =1;               //Ĭ�����ã�ÿ�������ϱ��ɼ�1�����ݡ�
		}	
////////////////////////////////////////////////////////////////////////////////////////////		
		DataRead_From_Flash(0,4,0, ConfigData.UploadCycle_Byte,2);     //��Flash�ж�ȡҺλ����������ϱ�����
		Temp =ConfigData.UploadCycle_Byte[1]*256 + ConfigData.UploadCycle_Byte[0];
		if((Temp>0)&&(Temp<=1440))          //ÿ�������ϱ����ɼ�20�����ݡ�
		{
			Para->UploadCycle = Temp;
		}
		else
		{    
			Para->UploadCycle =1440;           //Ĭ�����ã�Һλ����������ϱ�����Ϊ24Сʱ����ÿ���ϴ�1�����ݡ�
		}	
////////////////////////////////////////////////////////////////////////////////////////////		
		DataRead_From_Flash(0,5,0, ConfigData.CollectStartTime_Byte,2); 
    Temp =ConfigData.CollectStartTime_Byte[1]*256 + ConfigData.CollectStartTime_Byte[0];    //�͵�ַ��Ӧ���ֽ�
		if((Temp>0)&&(Temp<=1440))            //
		{
			Para->CollectStartTime = Temp;
		}
		else
		{    
			Para->CollectStartTime =240;               //Ĭ�����ã����ݲɼ���ʼʱ��Ϊ�賿4�㡣
		}	
/////////////////////////////////////////////////////////////////////////////////////////////	
    DataRead_From_Flash(0,6,0, &(ConfigData.RetryNumSet),1);     //��Flash�ж�ȡҺλ��������ݴ���ʱ�Ĺ����ش�����
		Temp =ConfigData.RetryNumSet;
		if((Temp>0)&&(Temp<=10))         //�ش���������Ϊ10
		{
			Para->RetryNum = Temp;
		}
		else
		{    
			Para->RetryNum =3;            //Ĭ�����ã����ݶ�ʧ���߳���ʱ�������3�Ρ�
		}	
/////////////////////////////////////////////////////////////////////////////////////////////	
//		DataRead_From_Flash(0,7,0, ConfigData.Threshold.Data_Hex,4);   //��Flash�ж�ȡԤ�豨����ֵ
//		fTemp = ConfigData.Threshold.Data_Float;
//		//	if((fTemp>0.01)&&(length<9.5))                   //������ֵ���޸���̽ͷ���̺�ä������
//		if(fTemp>0.01)                                       //������ֵ���޸���̽ͷ���̺�ä������
//		{
//			 Para->AlarmThreshold = fTemp;      
//		}
//		else                                                     
//		{ 
//			 Para->AlarmThreshold =0;                        //��ȡ������Чʱ����������ֵ��Ϊ0����������ֵΪ0ʱ�����ᴥ�������¼�
//		}
/////////////////���ò���Լ�������ж�///////////////////////////////////
  if(DeviceConfig.CollectNum * DeviceConfig.CollectPeriod >DeviceConfig.UploadCycle)
	{
    if(DeviceConfig.UploadCycle <DeviceConfig.CollectPeriod)
		{
       DeviceConfig.UploadCycle =DeviceConfig.CollectPeriod;
    }	
		DeviceConfig.CollectNum =DeviceConfig.UploadCycle/DeviceConfig.CollectPeriod;           //�����ݲɼ�������������
  }
	if(DeviceConfig.CollectNum >3)
	{
    DeviceConfig.CollectNum =3;           //�������ݷ����Լ����ݴ洢�ռ�����ƣ����ݲɼ���������������3����
  }
/////////////////////////////////////////////////////////////////////////////////////////////	
		Para->Time_Sec  =0x00;
		Para->Time_Min  =0x00;
		Para->Time_Hour =0x00;
		Para->Time_Mday =0x01;
		Para->Time_Mon  =0x03;
		Para->Time_Year =0x10;
		Para->BatteryCapacity =0x64;    //��ص������ݶ�Ϊ100%  

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





