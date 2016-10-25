#ifndef _GPRS_H_
#define _GPRS_H_
#include "stm32f10x.h"
#include "AiderProtocol.h"


//声明函数
void  mput(char *str);
void  mput_mix(char *str,int length);
void mput_mix_sx1278(char *str,int length);   //433模块SX1278发送数据
char* Find_String(char *Source, char *Object);
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object);
void  USART_DataBlock_Send(USART_TypeDef *USART_PORT,char *SendUartBuf,u16 SendLength);   //批量向串口发送数据
void  ConfigData_Init(struct DeviceSet* Para);
u16   Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress);	         //接收原始数据解析
void  Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress);	//协议数据解析，指剥掉9字节之后的数据部分解析
u8    UploadFlash(u8* pSetPara, u8 ParaLen, CommandType ParaRequest);  //更新Flash中参数值
void  UltrasonicParaConfig(u8 FunctionNum, char* ParaData);            //超声波探头配置
#endif

