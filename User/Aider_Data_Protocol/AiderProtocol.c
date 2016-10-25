#include "stm32f10x.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "modbus.h"
#include "string.h"
#include "AiderProtocol.h"
#include "API-Platform.h"
#include "433_Wiminet.h"
#include "bsp_rtc.h"
#include "SPI_Flash.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
1、由于数据帧中存在回车换行符，因此通过3G模块AT命令上传数据的长度有待确认。
2、关于一帧上传数据的数据采集方式有两种方案，分别是：
一、数据采集时间在两次数据上传时间之间平均分配，此时需要采集一次数据进行休眠，同时将采集到的数据存入外部存储区，
并用外存或者备份寄存器记录数据采集次数。
二、数据采集间隔远小于数据上传时间间隔，定时唤醒，唤醒之后立即采集数据，N组数据采集完成之后立即上传到服务器，
上传完成，并完成其他相关处理后，再次进入待机模式。
目前默认采用方案二
*/
static	struct   DataFrame         StartupSend;
static	struct   DataFrame         GetRespSend;
static  struct   SpecialDataFrame  TrapSend;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern  struct  DeviceSet  DeviceConfig;          //液位计配置信息结构体
//extern  struct Config_RegPara   ConfigData;    //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器

//extern u8       DataCollectCache[13][4];            //浮点数HEX格式存储，低字节在前，高字节在后
extern u8       DataCollectCount;                   //数据采集计数器
//extern char     SetRev_OK ;                         //成功接收服务器配置
//extern char     DatRev_OK ;                         //成功正确接收液位数据
extern uint8_t  Usart2_send_buff[SENDBUFF_SIZE];    //USART2发送缓存

//extern void     LSLIQUID_DataCollect( struct LiquidData* pLevel);         //N次液位数据采集，以及最后一次采集时间记录更新
extern void     RecvBuffInit_USART3(void);


/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void Hex2Char(char* CharArry, uint8_t* HexArry, uint8_t Length_HexArry)
//{
//	uint8_t   i=0,j=0;
//	uint8_t   ch=0,val=0;
////  uint8_t*  pData =HexArry ;
//	
//	for(j=0;j<Length_HexArry;j++)
//	{
//		val = HexArry[j] & 0xf0;  //对高位进行转换
//		val>>=4;
//		if(val <= 9)
//		{
//			ch= val+'0';	
//		}
//		else  	
//		{
//			ch =val-10+'A';
//		}
//		CharArry[i++] = ch;
//				 
//		val = HexArry[j] & 0x0f; //对低位进行转换
//		if(val <= 9)
//		{
//			ch= val+'0';	
//		}
//		else  	
//		{
//			ch =val-10+'A';
//		}
//		CharArry[i++] = ch;				  
////		pData++;
//	}
//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void Hex2DecimalChar(char* CharArry, uint8_t* HexArry, uint8_t Length_HexArry)
//{
//	uint8_t   i=0,j=0;
//	uint8_t   val=0;
////  uint8_t*  pData =HexArry ;
//	
//	for(j=0;j<Length_HexArry;j++)    
//	{
//		val =HexArry[j]/100;       //取数据的百位数字
//		if(val != 0)
//		{
//      CharArry[i] = val+'0';
//			i++;
//    }
//		
//		val =HexArry[j]%100/10;   //取数据的十位数字
//		if((val!=0)||(i!=0))    //当百位数不为零，或者十位数不为零时，对该位进行转变
//		{
//      CharArry[i] = val+'0';
//			i++;
//    }
//		
//		val =HexArry[j]%10;       //取数据的个位数字
//		if((val!=0)||(i!=0))    //当百位数、十位数、个位数有一个不为零时，对该位进行转变
//		{
//      CharArry[i] = val+'0';
//			i++;
//    }
//		if(j<3)                 //IP地址最后一个字段转换完成以后不加"."
//		{
//		  CharArry[i] = '.';	
//			i++;
//		}		  
////		pData++;
//	}
//}

/*******************************************************************************
* Function Name  : XX
* Description    : 将hex格式数据转换为char型，返回转换后的字符个数，主要用于上传液位数据转换
* Input          : None
* Output         : None
* Return         : 转换后得到字符的数量
*******************************************************************************/
//int char_hextochar(char* dealbuf, char* databuf, u8 length)
//{
//	int i;
//	u8 ch=0,val=0;
////	char  head[9] ={'L','S','L','E','V','D','A','T','A',':'};
//  char* pData =databuf+10;
//	
//	for(i=0;i<10;i++)
//  {
//     dealbuf[i] =databuf[i];
//  }
//  while((pData-databuf)<length-2)  //最好使用数据长度计数器来判断
//	{

//		 if(((pData-databuf)==11)||((pData-databuf)==14)||((pData-databuf)==16)||((pData-databuf)==20)||((pData-databuf)==22)
//			 ||((pData-databuf)==29)||((pData-databuf)==34)||((pData-databuf)==36)||((pData-databuf)==38)||((pData-databuf)==44)
//		   ||((pData-databuf)==length-6)||((pData-databuf)==length-4))
//		 {
//        dealbuf[i++] =',' ;
//			  pData++;
//     } 
//		 else
//		 {
//				val =*pData&0xf0;
//				val>>=4;
//				if(val<=9) ch=val+'0';	
//				else  	ch=val-10+'A';
//				dealbuf[i++]=ch;
//			 
//				val =*pData&0x0f;
//				if(val<=9)  ch=val+'0';			
//				else  ch=val-10+'A';
//				dealbuf[i++]=ch;
//			  
//			  pData++;
//     } 
//  }
//  dealbuf[i++] ='\r';
//  dealbuf[i++] ='\n';
//	return i;
//}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void LSLIQUSET_Handle(char* pLiqudSet, struct LiquidSet* Parameter)   //指针pLiqudSet指向接收到的配置帧;
//                                                                       //指针Parameter指向系统配置参数，其根据服务器的设置实时更新。
//{
//	  u8 temp[2] ={0};

//	  #if DEBUG_TEST	 
//		printf("\r\n服务器下发校时帧!!\r\n");                              //测试使用	
//		#endif
//		Parameter->CollectPeriod =(pLiqudSet[8])*256+pLiqudSet[9];   //对网络序修正。（网络序高位在前，主机序低位在前）
//		Parameter->SendCount =(pLiqudSet[10])*256+pLiqudSet[11];     //对网络序修正。
//	  temp[0] = (Parameter->CollectPeriod)&0xFF;
//		temp[1] = (Parameter->CollectPeriod)>>8;
//		DataWrite_To_Flash(0,2,0, temp,2);      //将数据采集间隔写入Flash
//		
//		temp[0] = (Parameter->SendCount)&0xFF;
//		temp[1] = (Parameter->SendCount)>>8;
//		DataWrite_To_Flash(0,3,0, temp,2);      //将每日数据上传次数写入Flash

//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void LSTIMESET_Handle(char* pLiqudSet, struct LiquidSet* Parameter)//指针pLiqudSet指向接收到的配置帧;
//                                                                    //指针Parameter指向系统配置参数，其根据服务器的设置实时更新。
//{
//  	Parameter->Time_Sec  =pLiqudSet[12];
//		Parameter->Time_Min  =pLiqudSet[13];
//		Parameter->Time_Hour =pLiqudSet[14];
//		Parameter->Time_Mday =pLiqudSet[16];
//		Parameter->Time_Mon  =pLiqudSet[17];
//		Parameter->Time_Year =pLiqudSet[18];
//		#if DEBUG_TEST	 
//    printf("\r\n服务器下发校时帧!!\r\n");                          //测试使用		
//		#endif
//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void LSLIQUSET_Response(struct LiquidSet* Parameter)
//{
//	//	char  LSLiquidSet_Success[39] ={"LSLIQUSETRESP:50,0009,02,000034,21,03\r\n"};		
////	char  LSLiquidSet_Success[36] ={"LSSETRESP:50,0009,02,000034,21,03\r\n"};		

//////  RecvBuffInit_USART3();
////  SendMessage(LSLiquidSet_Success, strlen(LSLiquidSet_Success),Parameter->SendCount);

//}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void LSLIQUID_DataUpload(struct LiquidSet* Para)
//{
//	char  LSLiquidSend[150] ={'\0'}; //
//  char  LSLiquid_char[250]={'\0'}; //存放转换后的char格式数据
//	char  temp1[12] ={'L','S','L','E','V','D','A','T','A',':',0x50,','};
//  char  temp2[16] ={',',0x01,',',0x00,0x00,0x34,',',0x51,',',0x11,0x20,0x15,0x09,0x00,0x72,','};
//	u8    Batch_Num =0x01;                     //数据批次序号
//	u8    Batch_Sum =0x01;                     //数据批次总数
//  char* pSend = NULL;
//  char* convert = NULL;
//  int   Length_Frame = 0;
//  char  UploadCount =5;     //数据上传最大尝试次数
//  u16   i=0,j=0;
//  int   LiquidData_Num=0;   //一帧数据包含的采集液位信息的次数
//  
//  pSend =LSLiquidSend;
//  for(i=0;i<12;i++)
//	{
//		LSLiquidSend[i] =temp1[i];
//		pSend++;
//	}
//	i =Para->CollectPeriod;        
//	j =Para->SendCount;            
//  LiquidData_Num =(24*60/j)/i;        //对于方案二，此种计算不太合理，后续有待完善
////	LiquidData_Num =10;                          //测试使用
//	#if DEBUG_TEST	
//	printf("\r\n一帧数据包含液位数据组数：%d-%d-%d\r\n",LiquidData_Num,i,j);  //测试使用
//	#endif
//	
//	if(LiquidData_Num >12) 
//	{
//    LiquidData_Num =12;  //将一帧数据的最大液位数据采集次数限定在12次
//  }
//  (*pSend++) =0x00;
//  (*pSend++) =LiquidData_Num*4 + 27;
//	
//	for(i=0;i<16;i++)
//	{
//    (*pSend++) =temp2[i];
//  }
//	convert = (char*)&(Para->CollectPeriod); //液位配置信息：采集间隔
//  (*pSend++) = *(convert+1);   //调整为网络序
//	(*pSend++) = *(convert);     //调整为网络序
// 	convert = (char*)&(Para->SendCount);     //液位配置信息：发送次数（一天）
//  (*pSend++) = *(convert+1);   //调整为网络序
//	(*pSend++) = *(convert);     //调整为网络序
//  (*pSend++) =',';             //分隔符
//  (*pSend++) =Batch_Sum;       //发送批次总数
//	(*pSend++) =',';             //分隔符
//  (*pSend++) =Batch_Num;       //发送批次序号
//	(*pSend++) =',';             //分隔符
//	
//	(*pSend++) =Para->Time_Min;             //记录时间--分--
//  (*pSend++) =Para->Time_Hour;            //记录时间--时--
//	(*pSend++) =Para->Time_Mday;            //记录时间--日--
//  (*pSend++) =Para->Time_Mon;             //记录时间--月--
//	(*pSend++) =Para->Time_Year;            //记录时间--年--
//	(*pSend++) =',';                        //分隔符
//	for(i=0;i<LiquidData_Num;i++)
//	{
//    for(j=4;j>=1;j--)
//		{
//      (*pSend++) = DataCollectCache[i][j-1]; //低字节在前，高字节在后；最后采集到的数据存放于低字节
//    }
//  }
//	(*pSend++) =',';                        //分隔符
//	(*pSend++) =Para->BatteryCapacity;      //液位监测仪电量
//	(*pSend++) =',';                        //分隔符
//	
//	(*pSend++) =0x03;                       //结束字符
//	(*pSend++) =0x0D;                       //结尾
//	(*pSend++) =0x0A;                       //结尾
//	Length_Frame =pSend - LSLiquidSend;     //计算上报数据帧长度，测试使用
////////////////////////////Test Part////////////////////////////////////////
//	printf("\r\n");                     //测试使用
//	for(i=0;i<Length_Frame;i++)
//	{
//		if((i<9)||(LSLiquidSend[i]==','))  	
//		{
//			printf("%c",LSLiquidSend[i]);   //测试使用
//    }
//    else
//	  {
//			printf(" %x",LSLiquidSend[i]);  //测试使用
//    }
//  }
//	printf("\r\n");                     //测试使用
//	
//	Length_Frame =char_hextochar(LSLiquid_char, LSLiquidSend, Length_Frame);
//	
//	/////////////////////////Test Part////////////////////////////////////////
//	printf("After Convert:\r\n");                     //测试使用
//	for(i=0;i<Length_Frame;i++)
//	{
//		printf("%c",LSLiquid_char[i]);     //测试使用
//  }
//	printf("\r\n");                     //测试使用
//	
//	
//	#if DEBUG_TEST	
//  printf("\r\nSEND Length is%d\r\n",Length_Frame);  //测试使用
//	printf("\r\nSEND:%s\r\n",LSLiquid_char);          //测试使用
//	#endif
//		
//	while(UploadCount!=0)
//	{
//		UploadCount--;
////		TCP_StatusQuery();       //查询网络连接状态
////		SendMessage(LSLiquid_char,Length_Frame,Para->SendCount);
////		Delay_ms(1000);	
//		LSDataUpload_Finish(&DeviceConfig);
////		Delay_ms(1000);	
//		if(DatRev_OK ==1)
//		{
//      DatRev_OK =0;
//			DataUpload_TALK_OVER(&DeviceConfig);
//			break;
//    }
//	}
//	
////	for(i=0;i<13;i++)                       //一个唤醒周期仅上传一次数据，因此不需要复位缓冲区
////	{
////    for(j=0;j<4;j++)
////		{
////       DataCollectCache[i][j] ='\0';      //清空液位采集缓存
////    }
////  }
//}
///*******************************************************************************
//* Function Name  : XX
//* Description    : XX
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void LSDataUpload_Finish(struct LiquidSet* Parameter)
//{
//	
//	char  DataUpload_Finish[36] ="LSDATOVER:50,0009,02,000034,23,03\r\n";				
//  SendMessage(DataUpload_Finish, strlen(DataUpload_Finish),Parameter->SendCount);

//}
///*******************************************************************************
//* Function Name  : XX
//* Description    : XX
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void DataUpload_TALK_OVER(struct LiquidSet* Parameter)
//{
//			
//	char  DataUpload_TalkOver[36] ="STALKOVER:50,0009,02,000034,00,03\r\n";				
//  SendMessage(DataUpload_TalkOver, strlen(DataUpload_TalkOver),Parameter->SendCount);

//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WakeupResponse(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
			
	struct   DataFrame  WakeupSend;
	struct   DataFrame* pDataFrame =&WakeupSend;
	char*    pChar =NULL;
  u8       SendCounter =1;     //发送次数计数器
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除OID序列（或者Tag序列）后剩余数据的长度
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //初始化结构体
	WakeupSend.Preamble =0xA3;
	WakeupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    WakeupSend.DeviceID[i] =pDeviceID[i];
  }
	WakeupSend.RouteFlag =0x01;
	WakeupSend.NodeAddr  =ntohs( NodeAddress);               //调整为网络序，即高字节在前，低字节在后
	WakeupSend.PDU_Type  =(0x0B<<8)+(1<<7)+8;
	WakeupSend.PDU_Type  =ntohs(WakeupSend.PDU_Type);        //调整为网络序，即高字节在前，低字节在后
	WakeupSend.Seq       =1;
	WakeupSend.TagList[0].OID_Command =ntohl(DEVICE_WAKEUP); //调整为网络序，即高字节在前，低字节在后
	WakeupSend.TagList[0].Width =1;
	WakeupSend.TagList[0].Value[0]=1;
	WakeupSend.Tag_Count =1;
	WakeupSend.Length =CoreLength + WakeupSend.TagList[0].Width +6;  
	WakeupSend.Length =ntohs(WakeupSend.Length );
	WakeupSend.CrcCode=0xffff;                                        //CRC字段赋初值

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<WakeupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(WakeupSend.TagList[i].OID_Command);   
		ValidLength =WakeupSend.TagList[i].Width+6;            //计算1个Tag实际占用的字节空间
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //防止指针溢出
		{
       break;
    }
		WakeupSend.TagList[i].Width =ntohs(WakeupSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = WakeupSend.CrcCode &0xff;
	pSendBuff[j++] = WakeupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	WakeupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[j-1] = CrcData>>8;     //CRC高字节在后

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i< SendCounter;i++)
	{
     SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
//		 Delay_ms(2000); 
  }
	printf("\r\n----Length:%d----\r\n",j);   //测试使用
	for(i=0;i<j;i++)
	{
    printf("%x ",pSendBuff[i]);   //测试使用
  }
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DeviceStartupRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
			
//	struct   DataFrame  StartupSend;
	struct   DataFrame* pDataFrame =&StartupSend;
	char*    pChar =NULL;
	u16      RecevFlag =0;       //服务器数据接收标志变量
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       SendCounter =1;     //发送次数计数器
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除OID序列（或者Tag序列）后剩余数据的长度
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //初始化结构体
	StartupSend.Preamble =0xA3;
	StartupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    StartupSend.DeviceID[i] =pDeviceID[i];
  }
	StartupSend.RouteFlag =0x01;
	StartupSend.NodeAddr  =ntohs( NodeAddress);              //调整为网络序，即高字节在前，低字节在后
	StartupSend.PDU_Type  =(8<<8)+(1<<7)+8;
	StartupSend.PDU_Type  =ntohs(StartupSend.PDU_Type);      //调整为网络序，即高字节在前，低字节在后
	StartupSend.Seq       =1;
	StartupSend.TagList[0].OID_Command =ntohl(DEVICE_STATE); //调整为网络序，即高字节在前，低字节在后
	StartupSend.TagList[0].Width =1;
	StartupSend.TagList[0].Value[0]=1;
	StartupSend.Tag_Count =1;
	StartupSend.Length =CoreLength + StartupSend.TagList[0].Width +6;  
	StartupSend.Length =ntohs(StartupSend.Length );
	StartupSend.CrcCode=0xffff;                                        //CRC字段赋初值

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<StartupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(StartupSend.TagList[i].OID_Command);   
		ValidLength =StartupSend.TagList[i].Width+6;           //计算1个Tag实际占用的字节空间
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //防止指针溢出
		{
       break;
    }
		StartupSend.TagList[i].Width =ntohs(StartupSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = StartupSend.CrcCode &0xff;
	pSendBuff[j++] = StartupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	StartupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[j-1] = CrcData>>8;     //CRC高字节在后

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
		 if(RecevFlag ==0x0988)     //成功接收到TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //测试使用
				#endif
			  break;     
     }
//		 Delay_ms(2000); 
  }
	printf("\r\n----Length:%d----\r\n",j);   //测试使用
	for(i=0;i<j;i++)
	{
     printf("-%x-",pSendBuff[i]);   //测试使用
  }
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  TrapRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress, struct SenserData* pObtainData)
{
			
//	struct   SpecialDataFrame  TrapSend;
	struct   SpecialDataFrame* pDataFrame  =&TrapSend;
	char*    pChar =NULL;
	u16      RecevFlag =0;       //服务器数据接收标志变量
	u16      CrcData=0;
	u8       SendCounter =1;     //发送次数计数器
	u8       i=0,j=0;
	u8       Offset=0;           //发送缓存地址偏移变量
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	u8       LengthKey =0;       //数据净荷部分长度字段指示的数值
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除Tag序列后剩余数据的长度
	
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct SpecialDataFrame));   //初始化结构体
	TrapSend.Preamble =0xA3;
	TrapSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    TrapSend.DeviceID[i] =pDeviceID[i];
  }
	TrapSend.RouteFlag =0x01;
	TrapSend.NodeAddr  =ntohs( NodeAddress);              //调整为网络序，即高字节在前，低字节在后
	TrapSend.PDU_Type  =(4<<8)+(1<<7)+8;
	TrapSend.PDU_Type  =ntohs(TrapSend.PDU_Type);         //调整为网络序，即高字节在前，低字节在后
	TrapSend.Seq       =1;
	LengthKey = CoreLength;
	
	TrapSend.BattEnergy.OID_Command = ntohl(DEVICE_QTY);   //调整为网络序，即高字节在前，低字节在后
	TrapSend.BattEnergy.Width =1;                         //
	TrapSend.BattEnergy.Value[0] =DeviceConfig.BatteryCapacity;  //数据上传时的电池剩余电量
	LengthKey = LengthKey+6+TrapSend.BattEnergy.Width;
	
	TrapSend.SysTime.OID_Command= ntohl(SYSTERM_DATA);    //调整为网络序，即高字节在前，低字节在后
	TrapSend.SysTime.Width =3;                            //
	TrapSend.SysTime.Value[0] =DeviceConfig.Time_Year;           //系统日期，年
	TrapSend.SysTime.Value[1] =DeviceConfig.Time_Mon;            //系统日期，月
	TrapSend.SysTime.Value[2] =DeviceConfig.Time_Mday;           //系统日期，日
  LengthKey = LengthKey+6+TrapSend.SysTime.Width;
	
	for(i=0,TrapSend.Tag_Count=0,TrapSend.Length =LengthKey ;i<pObtainData->DataCount ;i++)    //对每一个温度数据Tag赋值
	{
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData->CollectTime[i] +(0xC8<<24);   
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //调整为网络序，即高字节在前，低字节在后
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F =pObtainData->TemperatureData[i];  //采集到的温度数据
    
		TrapSend.Tag_Count++ ;                                             //对温度数据Tag进行计数，不包括采集日期Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }
	i= TrapSend.Tag_Count;   //保存计数值，紧接着连接压力数据
	for(j=0; j<pObtainData->DataCount; i++,j++)    //对每一个压力数据Tag赋值
	{
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData->CollectTime[j] +(0xC9<<24);   
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //调整为网络序，即高字节在前，低字节在后
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F =pObtainData->PressureData[j];  //采集到的压力数据
    
		TrapSend.Tag_Count++ ;                                                    //对压力数据Tag进行计数，不包括采集日期Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }
	
	/////////////////////////////////////////////////////////////////////////////////////////////
	TrapSend.Length =ntohs(TrapSend.Length );              //计算长度字段
	memcpy(pSendBuff,pChar,PreLength);                     //复制Tag之前的数据到发送Buff
	Offset=PreLength;                                           //指针偏移地址
	pChar = (char*)&(TrapSend.BattEnergy.OID_Command);     //电池电量Tag
	ValidLength =TrapSend.BattEnergy.Width+6;                    //计算1个Tag实际占用的字节空间 
	TrapSend.BattEnergy.Width =ntohs(TrapSend.BattEnergy.Width); //调整为网络序，即高字节在前，低字节在后
	memcpy((pSendBuff+Offset),pChar,ValidLength);                     //复制电池电量Tag数据到发送Buff
	Offset = Offset+ValidLength;
	
	pChar = (char*)&(TrapSend.SysTime.OID_Command);        //系统日期Tag
	ValidLength =TrapSend.SysTime.Width+6;                 //计算1个Tag实际占用的字节空间 
	TrapSend.SysTime.Width =ntohs(TrapSend.SysTime.Width); //调整为网络序，即高字节在前，低字节在后
	memcpy((pSendBuff+Offset),pChar,ValidLength);       //复制系统时间Tag数据到发送Buff
	Offset = Offset+ValidLength;
	
	for(i=0;i<TrapSend.Tag_Count;i++)      //
	{
		pChar = (char*)&(TrapSend.TagList[i].OID_Data);   
		ValidLength =TrapSend.TagList[i].Width+6;            //计算1个Tag实际占用的字节空间
		if((Offset+ValidLength) >=(SENDBUFF_SIZE-3))              //防止指针溢出
		{
       break;
    }
		TrapSend.TagList[i].Width =ntohs(TrapSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+Offset),pChar,ValidLength);                //复制每一个数据Tag到发送Buff
		Offset = Offset+ValidLength;	
  }
	
	TrapSend.CrcCode=0xffff;                               //CRC字段赋初值
	pSendBuff[Offset++] = TrapSend.CrcCode &0xff;
	pSendBuff[Offset++] = TrapSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, Offset );   // Update the CRC value
	TrapSend.CrcCode =CrcData;
	pSendBuff[Offset-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[Offset-1] = CrcData>>8;     //CRC高字节在后

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0; i<SendCounter; i++)
	{
     RecevFlag =SendMessage(pSendBuff, Offset, pDeviceID, NodeAddress);
//		 printf("\r\nReceive TrapResponse success:%4x!\r\n", RecevFlag); //测试使用
		 if(RecevFlag ==0x0588)     //成功接收到TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //测试使用
				#endif
			  break;     
     }
//		 Delay_ms(2000); 
  }
	printf("\r\n----Length:%d----\r\n",Offset);   //测试使用
	for(i=0;i<Offset;i++)
	{
    printf("-%x-",pSendBuff[i]);   //测试使用
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GetResponse(struct CommandFrame RequestPara, u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
  struct   rtc_time   SystTime;           //RTC时钟设置结构体
//	struct   DataFrame  GetRespSend;
	struct   DataFrame* pDataFrame =&GetRespSend;
	u16      RecevFlag =0;       //服务器数据接收标志变量
	char*    pChar =NULL;
	union    Hfloat  UnionData;
	u8       SendCounter =1;     //发送次数计数器
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除OID序列（或者Tag序列）后剩余数据的长度
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //初始化结构体
	GetRespSend.Preamble =0xA3;
	GetRespSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    GetRespSend.DeviceID[i] =pDeviceID[i];
  }
	GetRespSend.RouteFlag =0x01;
	GetRespSend.NodeAddr  =ntohs( NodeAddress);              //调整为网络序，即高字节在前，低字节在后
	GetRespSend.PDU_Type  =(2<<8)+(1<<7)+8;
	GetRespSend.PDU_Type  =ntohs(GetRespSend.PDU_Type);      //调整为网络序，即高字节在前，低字节在后
	GetRespSend.Seq       =1;
	GetRespSend.Length    =CoreLength ;                       //
	for(i=0,GetRespSend.Tag_Count=0; i<RequestPara.OID_Count; i++)
	{
		 switch(RequestPara.OID_List[i])
	   {
        case DEF_NR:            //重传次数
				{
            GetRespSend.TagList[i].OID_Command =DEF_NR;
					  GetRespSend.TagList[i].Width =1;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.RetryNum;
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				case SYSTERM_TIME:     //系统时间
				{
      
						Time_Display(RTC_GetCounter(),&SystTime);     //获取系统当前时间			
						DeviceConfig.Time_Sec  =SystTime.tm_sec;
						DeviceConfig.Time_Min  =SystTime.tm_min;
						DeviceConfig.Time_Hour =SystTime.tm_hour;
						DeviceConfig.Time_Mday =SystTime.tm_mday;		
						DeviceConfig.Time_Mon  =SystTime.tm_mon;
						DeviceConfig.Time_Year =SystTime.tm_year-2000; //对上传年份去基数修正
					
					  GetRespSend.TagList[i].OID_Command =SYSTERM_TIME;
					  GetRespSend.TagList[i].Width =6;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.Time_Year;   //系统日期，年
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.Time_Mon;    //系统日期，月
					  GetRespSend.TagList[i].Value[2] =DeviceConfig.Time_Mday;   //系统日期，日
					  GetRespSend.TagList[i].Value[3] =DeviceConfig.Time_Hour;   //系统时间，小时
					  GetRespSend.TagList[i].Value[4] =DeviceConfig.Time_Min;    //系统时间，分
					  GetRespSend.TagList[i].Value[5] =DeviceConfig.Time_Sec;    //系统时间，秒
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
        case CLT1_ITRL1:       //一时区采集间隔
				{
            GetRespSend.TagList[i].OID_Command =CLT1_ITRL1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.CollectPeriod >>8;   //数据采集间隔，高字节在前
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.CollectPeriod &0xff; //数据采集间隔，低字节在后
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				case CLT1_CNT1:        //一时区采集次数
				{
            GetRespSend.TagList[i].OID_Command =CLT1_CNT1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.CollectNum >>8;   //数据采集间隔，高字节在前
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.CollectNum &0xff; //数据采集间隔，低字节在后
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }
				case CLT1_STIME1 :    //一时区采集开始时间
				{
            GetRespSend.TagList[i].OID_Command =CLT1_STIME1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] = DeviceConfig.CollectStartTime>>8;    //温度压力监测仪启动时间，高字节在前 
					  GetRespSend.TagList[i].Value[1] = DeviceConfig.CollectStartTime&0xff;  //温度压力监测仪启动时间，低字节在后  					
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
//				case RESET_PROBER:     //复位液位探头，后续可以考虑放到状态表中
//				{
//            GetRespSend.TagList[i].OID_Command =RESET_PROBER;
//					  GetRespSend.TagList[i].Width =1;
//					  GetRespSend.TagList[i].Value[0] =0;         //只有服务器下发复位命令时，Value值为1，其他均为0
//					  GetRespSend.Tag_Count++;
//					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
//					  break;
//        }	
				case UPLOAD_CYCLE:     //液位计数据上报周期
				{
            GetRespSend.TagList[i].OID_Command =UPLOAD_CYCLE;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.UploadCycle >>8;   //数据采集间隔，高字节在前
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.UploadCycle &0xff; //数据采集间隔，低字节在后
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }					
        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!Tag OID not recognition!\r\n"); //测试使用
					 #endif
					 break;
        }
		}
  }
	
	GetRespSend.Length =ntohs(GetRespSend.Length );                //调整为网络序，即高字节在前，低字节在后
	GetRespSend.CrcCode=0xffff;                                    //CRC字段赋初值
	memcpy(pSendBuff,pChar,PreLength);
	
	for(i=0,j=PreLength;i<GetRespSend.Tag_Count;i++)
	{
    GetRespSend.TagList[i].OID_Command =ntohl(GetRespSend.TagList[i].OID_Command);  //将OID序列调整为高字节在前，低字节在后  //有待测试
		pChar = (char*)&(GetRespSend.TagList[i].OID_Command);   
		ValidLength =GetRespSend.TagList[i].Width+6;           //计算1个Tag实际占用的字节空间
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //防止指针溢出
		{
       break;
    }
		GetRespSend.TagList[i].Width =ntohs(GetRespSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = GetRespSend.CrcCode &0xff;
	pSendBuff[j++] = GetRespSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	GetRespSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[j-1] = CrcData>>8;     //CRC高字节在后
  
	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
		 printf("\r\nReceive GetResponse success:%4x!\r\n", RecevFlag); //测试使用
		 if((RecevFlag ==0x0188)||(RecevFlag ==0x0388)||(RecevFlag ==0x0088))     //成功接收到GetResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive GetResponse success!\r\n"); //测试使用
				#endif
			  break;     
     }
//		 Delay_ms(3000); 
  }
  
	printf("\r\n----Length:%d----\r\n",j);   //测试使用
	for(i=0;i<j;i++)
	{
    printf(",0x%x",pSendBuff[i]);   //测试使用
  }

}
