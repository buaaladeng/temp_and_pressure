#ifndef _GPRS_H_
#define _GPRS_H_
#include "stm32f10x.h"
#include "AiderProtocol.h"


//��������
void  mput(char *str);
void  mput_mix(char *str,int length);
void mput_mix_sx1278(char *str,int length);   //433ģ��SX1278��������
char* Find_String(char *Source, char *Object);
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object);
void  USART_DataBlock_Send(USART_TypeDef *USART_PORT,char *SendUartBuf,u16 SendLength);   //�����򴮿ڷ�������
void  ConfigData_Init(struct DeviceSet* Para);
u16   Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress);	         //����ԭʼ���ݽ���
void  Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress);	//Э�����ݽ�����ָ����9�ֽ�֮������ݲ��ֽ���
u8    UploadFlash(u8* pSetPara, u8 ParaLen, CommandType ParaRequest);  //����Flash�в���ֵ
void  UltrasonicParaConfig(u8 FunctionNum, char* ParaData);            //������̽ͷ����
#endif

