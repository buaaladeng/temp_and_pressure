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
1����������֡�д��ڻس����з������ͨ��3Gģ��AT�����ϴ����ݵĳ����д�ȷ�ϡ�
2������һ֡�ϴ����ݵ����ݲɼ���ʽ�����ַ������ֱ��ǣ�
һ�����ݲɼ�ʱ�������������ϴ�ʱ��֮��ƽ�����䣬��ʱ��Ҫ�ɼ�һ�����ݽ������ߣ�ͬʱ���ɼ��������ݴ����ⲿ�洢����
���������߱��ݼĴ�����¼���ݲɼ�������
�������ݲɼ����ԶС�������ϴ�ʱ��������ʱ���ѣ�����֮�������ɼ����ݣ�N�����ݲɼ����֮�������ϴ�����������
�ϴ���ɣ������������ش�����ٴν������ģʽ��
ĿǰĬ�ϲ��÷�����
*/
static	struct   DataFrame         StartupSend;
static	struct   DataFrame         GetRespSend;
static  struct   SpecialDataFrame  TrapSend;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern  struct  DeviceSet  DeviceConfig;          //Һλ��������Ϣ�ṹ��
//extern  struct Config_RegPara   ConfigData;    //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���

//extern u8       DataCollectCache[13][4];            //������HEX��ʽ�洢�����ֽ���ǰ�����ֽ��ں�
extern u8       DataCollectCount;                   //���ݲɼ�������
//extern char     SetRev_OK ;                         //�ɹ����շ���������
//extern char     DatRev_OK ;                         //�ɹ���ȷ����Һλ����
extern uint8_t  Usart2_send_buff[SENDBUFF_SIZE];    //USART2���ͻ���

//extern void     LSLIQUID_DataCollect( struct LiquidData* pLevel);         //N��Һλ���ݲɼ����Լ����һ�βɼ�ʱ���¼����
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
//		val = HexArry[j] & 0xf0;  //�Ը�λ����ת��
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
//		val = HexArry[j] & 0x0f; //�Ե�λ����ת��
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
//		val =HexArry[j]/100;       //ȡ���ݵİ�λ����
//		if(val != 0)
//		{
//      CharArry[i] = val+'0';
//			i++;
//    }
//		
//		val =HexArry[j]%100/10;   //ȡ���ݵ�ʮλ����
//		if((val!=0)||(i!=0))    //����λ����Ϊ�㣬����ʮλ����Ϊ��ʱ���Ը�λ����ת��
//		{
//      CharArry[i] = val+'0';
//			i++;
//    }
//		
//		val =HexArry[j]%10;       //ȡ���ݵĸ�λ����
//		if((val!=0)||(i!=0))    //����λ����ʮλ������λ����һ����Ϊ��ʱ���Ը�λ����ת��
//		{
//      CharArry[i] = val+'0';
//			i++;
//    }
//		if(j<3)                 //IP��ַ���һ���ֶ�ת������Ժ󲻼�"."
//		{
//		  CharArry[i] = '.';	
//			i++;
//		}		  
////		pData++;
//	}
//}

/*******************************************************************************
* Function Name  : XX
* Description    : ��hex��ʽ����ת��Ϊchar�ͣ�����ת������ַ���������Ҫ�����ϴ�Һλ����ת��
* Input          : None
* Output         : None
* Return         : ת����õ��ַ�������
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
//  while((pData-databuf)<length-2)  //���ʹ�����ݳ��ȼ��������ж�
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
//void LSLIQUSET_Handle(char* pLiqudSet, struct LiquidSet* Parameter)   //ָ��pLiqudSetָ����յ�������֡;
//                                                                       //ָ��Parameterָ��ϵͳ���ò���������ݷ�����������ʵʱ���¡�
//{
//	  u8 temp[2] ={0};

//	  #if DEBUG_TEST	 
//		printf("\r\n�������·�Уʱ֡!!\r\n");                              //����ʹ��	
//		#endif
//		Parameter->CollectPeriod =(pLiqudSet[8])*256+pLiqudSet[9];   //�����������������������λ��ǰ���������λ��ǰ��
//		Parameter->SendCount =(pLiqudSet[10])*256+pLiqudSet[11];     //��������������
//	  temp[0] = (Parameter->CollectPeriod)&0xFF;
//		temp[1] = (Parameter->CollectPeriod)>>8;
//		DataWrite_To_Flash(0,2,0, temp,2);      //�����ݲɼ����д��Flash
//		
//		temp[0] = (Parameter->SendCount)&0xFF;
//		temp[1] = (Parameter->SendCount)>>8;
//		DataWrite_To_Flash(0,3,0, temp,2);      //��ÿ�������ϴ�����д��Flash

//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void LSTIMESET_Handle(char* pLiqudSet, struct LiquidSet* Parameter)//ָ��pLiqudSetָ����յ�������֡;
//                                                                    //ָ��Parameterָ��ϵͳ���ò���������ݷ�����������ʵʱ���¡�
//{
//  	Parameter->Time_Sec  =pLiqudSet[12];
//		Parameter->Time_Min  =pLiqudSet[13];
//		Parameter->Time_Hour =pLiqudSet[14];
//		Parameter->Time_Mday =pLiqudSet[16];
//		Parameter->Time_Mon  =pLiqudSet[17];
//		Parameter->Time_Year =pLiqudSet[18];
//		#if DEBUG_TEST	 
//    printf("\r\n�������·�Уʱ֡!!\r\n");                          //����ʹ��		
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
//  char  LSLiquid_char[250]={'\0'}; //���ת�����char��ʽ����
//	char  temp1[12] ={'L','S','L','E','V','D','A','T','A',':',0x50,','};
//  char  temp2[16] ={',',0x01,',',0x00,0x00,0x34,',',0x51,',',0x11,0x20,0x15,0x09,0x00,0x72,','};
//	u8    Batch_Num =0x01;                     //�����������
//	u8    Batch_Sum =0x01;                     //������������
//  char* pSend = NULL;
//  char* convert = NULL;
//  int   Length_Frame = 0;
//  char  UploadCount =5;     //�����ϴ�����Դ���
//  u16   i=0,j=0;
//  int   LiquidData_Num=0;   //һ֡���ݰ����Ĳɼ�Һλ��Ϣ�Ĵ���
//  
//  pSend =LSLiquidSend;
//  for(i=0;i<12;i++)
//	{
//		LSLiquidSend[i] =temp1[i];
//		pSend++;
//	}
//	i =Para->CollectPeriod;        
//	j =Para->SendCount;            
//  LiquidData_Num =(24*60/j)/i;        //���ڷ����������ּ��㲻̫���������д�����
////	LiquidData_Num =10;                          //����ʹ��
//	#if DEBUG_TEST	
//	printf("\r\nһ֡���ݰ���Һλ����������%d-%d-%d\r\n",LiquidData_Num,i,j);  //����ʹ��
//	#endif
//	
//	if(LiquidData_Num >12) 
//	{
//    LiquidData_Num =12;  //��һ֡���ݵ����Һλ���ݲɼ������޶���12��
//  }
//  (*pSend++) =0x00;
//  (*pSend++) =LiquidData_Num*4 + 27;
//	
//	for(i=0;i<16;i++)
//	{
//    (*pSend++) =temp2[i];
//  }
//	convert = (char*)&(Para->CollectPeriod); //Һλ������Ϣ���ɼ����
//  (*pSend++) = *(convert+1);   //����Ϊ������
//	(*pSend++) = *(convert);     //����Ϊ������
// 	convert = (char*)&(Para->SendCount);     //Һλ������Ϣ�����ʹ�����һ�죩
//  (*pSend++) = *(convert+1);   //����Ϊ������
//	(*pSend++) = *(convert);     //����Ϊ������
//  (*pSend++) =',';             //�ָ���
//  (*pSend++) =Batch_Sum;       //������������
//	(*pSend++) =',';             //�ָ���
//  (*pSend++) =Batch_Num;       //�����������
//	(*pSend++) =',';             //�ָ���
//	
//	(*pSend++) =Para->Time_Min;             //��¼ʱ��--��--
//  (*pSend++) =Para->Time_Hour;            //��¼ʱ��--ʱ--
//	(*pSend++) =Para->Time_Mday;            //��¼ʱ��--��--
//  (*pSend++) =Para->Time_Mon;             //��¼ʱ��--��--
//	(*pSend++) =Para->Time_Year;            //��¼ʱ��--��--
//	(*pSend++) =',';                        //�ָ���
//	for(i=0;i<LiquidData_Num;i++)
//	{
//    for(j=4;j>=1;j--)
//		{
//      (*pSend++) = DataCollectCache[i][j-1]; //���ֽ���ǰ�����ֽ��ں����ɼ��������ݴ���ڵ��ֽ�
//    }
//  }
//	(*pSend++) =',';                        //�ָ���
//	(*pSend++) =Para->BatteryCapacity;      //Һλ����ǵ���
//	(*pSend++) =',';                        //�ָ���
//	
//	(*pSend++) =0x03;                       //�����ַ�
//	(*pSend++) =0x0D;                       //��β
//	(*pSend++) =0x0A;                       //��β
//	Length_Frame =pSend - LSLiquidSend;     //�����ϱ�����֡���ȣ�����ʹ��
////////////////////////////Test Part////////////////////////////////////////
//	printf("\r\n");                     //����ʹ��
//	for(i=0;i<Length_Frame;i++)
//	{
//		if((i<9)||(LSLiquidSend[i]==','))  	
//		{
//			printf("%c",LSLiquidSend[i]);   //����ʹ��
//    }
//    else
//	  {
//			printf(" %x",LSLiquidSend[i]);  //����ʹ��
//    }
//  }
//	printf("\r\n");                     //����ʹ��
//	
//	Length_Frame =char_hextochar(LSLiquid_char, LSLiquidSend, Length_Frame);
//	
//	/////////////////////////Test Part////////////////////////////////////////
//	printf("After Convert:\r\n");                     //����ʹ��
//	for(i=0;i<Length_Frame;i++)
//	{
//		printf("%c",LSLiquid_char[i]);     //����ʹ��
//  }
//	printf("\r\n");                     //����ʹ��
//	
//	
//	#if DEBUG_TEST	
//  printf("\r\nSEND Length is%d\r\n",Length_Frame);  //����ʹ��
//	printf("\r\nSEND:%s\r\n",LSLiquid_char);          //����ʹ��
//	#endif
//		
//	while(UploadCount!=0)
//	{
//		UploadCount--;
////		TCP_StatusQuery();       //��ѯ��������״̬
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
////	for(i=0;i<13;i++)                       //һ���������ڽ��ϴ�һ�����ݣ���˲���Ҫ��λ������
////	{
////    for(j=0;j<4;j++)
////		{
////       DataCollectCache[i][j] ='\0';      //���Һλ�ɼ�����
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
  u8       SendCounter =1;     //���ʹ���������
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��OID���У�����Tag���У���ʣ�����ݵĳ���
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //��ʼ���ṹ��
	WakeupSend.Preamble =0xA3;
	WakeupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    WakeupSend.DeviceID[i] =pDeviceID[i];
  }
	WakeupSend.RouteFlag =0x01;
	WakeupSend.NodeAddr  =ntohs( NodeAddress);               //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	WakeupSend.PDU_Type  =(0x0B<<8)+(1<<7)+8;
	WakeupSend.PDU_Type  =ntohs(WakeupSend.PDU_Type);        //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	WakeupSend.Seq       =1;
	WakeupSend.TagList[0].OID_Command =ntohl(DEVICE_WAKEUP); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	WakeupSend.TagList[0].Width =1;
	WakeupSend.TagList[0].Value[0]=1;
	WakeupSend.Tag_Count =1;
	WakeupSend.Length =CoreLength + WakeupSend.TagList[0].Width +6;  
	WakeupSend.Length =ntohs(WakeupSend.Length );
	WakeupSend.CrcCode=0xffff;                                        //CRC�ֶθ���ֵ

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<WakeupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(WakeupSend.TagList[i].OID_Command);   
		ValidLength =WakeupSend.TagList[i].Width+6;            //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //��ָֹ�����
		{
       break;
    }
		WakeupSend.TagList[i].Width =ntohs(WakeupSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = WakeupSend.CrcCode &0xff;
	pSendBuff[j++] = WakeupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	WakeupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[j-1] = CrcData>>8;     //CRC���ֽ��ں�

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
	printf("\r\n----Length:%d----\r\n",j);   //����ʹ��
	for(i=0;i<j;i++)
	{
    printf("%x ",pSendBuff[i]);   //����ʹ��
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
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       SendCounter =1;     //���ʹ���������
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��OID���У�����Tag���У���ʣ�����ݵĳ���
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //��ʼ���ṹ��
	StartupSend.Preamble =0xA3;
	StartupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    StartupSend.DeviceID[i] =pDeviceID[i];
  }
	StartupSend.RouteFlag =0x01;
	StartupSend.NodeAddr  =ntohs( NodeAddress);              //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	StartupSend.PDU_Type  =(8<<8)+(1<<7)+8;
	StartupSend.PDU_Type  =ntohs(StartupSend.PDU_Type);      //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	StartupSend.Seq       =1;
	StartupSend.TagList[0].OID_Command =ntohl(DEVICE_STATE); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	StartupSend.TagList[0].Width =1;
	StartupSend.TagList[0].Value[0]=1;
	StartupSend.Tag_Count =1;
	StartupSend.Length =CoreLength + StartupSend.TagList[0].Width +6;  
	StartupSend.Length =ntohs(StartupSend.Length );
	StartupSend.CrcCode=0xffff;                                        //CRC�ֶθ���ֵ

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<StartupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(StartupSend.TagList[i].OID_Command);   
		ValidLength =StartupSend.TagList[i].Width+6;           //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //��ָֹ�����
		{
       break;
    }
		StartupSend.TagList[i].Width =ntohs(StartupSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = StartupSend.CrcCode &0xff;
	pSendBuff[j++] = StartupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	StartupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[j-1] = CrcData>>8;     //CRC���ֽ��ں�

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
		 if(RecevFlag ==0x0988)     //�ɹ����յ�TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //����ʹ��
				#endif
			  break;     
     }
//		 Delay_ms(2000); 
  }
	printf("\r\n----Length:%d----\r\n",j);   //����ʹ��
	for(i=0;i<j;i++)
	{
     printf("-%x-",pSendBuff[i]);   //����ʹ��
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
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	u16      CrcData=0;
	u8       SendCounter =1;     //���ʹ���������
	u8       i=0,j=0;
	u8       Offset=0;           //���ͻ����ַƫ�Ʊ���
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	u8       LengthKey =0;       //���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��Tag���к�ʣ�����ݵĳ���
	
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct SpecialDataFrame));   //��ʼ���ṹ��
	TrapSend.Preamble =0xA3;
	TrapSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    TrapSend.DeviceID[i] =pDeviceID[i];
  }
	TrapSend.RouteFlag =0x01;
	TrapSend.NodeAddr  =ntohs( NodeAddress);              //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.PDU_Type  =(4<<8)+(1<<7)+8;
	TrapSend.PDU_Type  =ntohs(TrapSend.PDU_Type);         //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.Seq       =1;
	LengthKey = CoreLength;
	
	TrapSend.BattEnergy.OID_Command = ntohl(DEVICE_QTY);   //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.BattEnergy.Width =1;                         //
	TrapSend.BattEnergy.Value[0] =DeviceConfig.BatteryCapacity;  //�����ϴ�ʱ�ĵ��ʣ�����
	LengthKey = LengthKey+6+TrapSend.BattEnergy.Width;
	
	TrapSend.SysTime.OID_Command= ntohl(SYSTERM_DATA);    //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.SysTime.Width =3;                            //
	TrapSend.SysTime.Value[0] =DeviceConfig.Time_Year;           //ϵͳ���ڣ���
	TrapSend.SysTime.Value[1] =DeviceConfig.Time_Mon;            //ϵͳ���ڣ���
	TrapSend.SysTime.Value[2] =DeviceConfig.Time_Mday;           //ϵͳ���ڣ���
  LengthKey = LengthKey+6+TrapSend.SysTime.Width;
	
	for(i=0,TrapSend.Tag_Count=0,TrapSend.Length =LengthKey ;i<pObtainData->DataCount ;i++)    //��ÿһ���¶�����Tag��ֵ
	{
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData->CollectTime[i] +(0xC8<<24);   
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F =pObtainData->TemperatureData[i];  //�ɼ������¶�����
    
		TrapSend.Tag_Count++ ;                                             //���¶�����Tag���м������������ɼ�����Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }
	i= TrapSend.Tag_Count;   //�������ֵ������������ѹ������
	for(j=0; j<pObtainData->DataCount; i++,j++)    //��ÿһ��ѹ������Tag��ֵ
	{
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData->CollectTime[j] +(0xC9<<24);   
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F =pObtainData->PressureData[j];  //�ɼ�����ѹ������
    
		TrapSend.Tag_Count++ ;                                                    //��ѹ������Tag���м������������ɼ�����Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }
	
	/////////////////////////////////////////////////////////////////////////////////////////////
	TrapSend.Length =ntohs(TrapSend.Length );              //���㳤���ֶ�
	memcpy(pSendBuff,pChar,PreLength);                     //����Tag֮ǰ�����ݵ�����Buff
	Offset=PreLength;                                           //ָ��ƫ�Ƶ�ַ
	pChar = (char*)&(TrapSend.BattEnergy.OID_Command);     //��ص���Tag
	ValidLength =TrapSend.BattEnergy.Width+6;                    //����1��Tagʵ��ռ�õ��ֽڿռ� 
	TrapSend.BattEnergy.Width =ntohs(TrapSend.BattEnergy.Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	memcpy((pSendBuff+Offset),pChar,ValidLength);                     //���Ƶ�ص���Tag���ݵ�����Buff
	Offset = Offset+ValidLength;
	
	pChar = (char*)&(TrapSend.SysTime.OID_Command);        //ϵͳ����Tag
	ValidLength =TrapSend.SysTime.Width+6;                 //����1��Tagʵ��ռ�õ��ֽڿռ� 
	TrapSend.SysTime.Width =ntohs(TrapSend.SysTime.Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	memcpy((pSendBuff+Offset),pChar,ValidLength);       //����ϵͳʱ��Tag���ݵ�����Buff
	Offset = Offset+ValidLength;
	
	for(i=0;i<TrapSend.Tag_Count;i++)      //
	{
		pChar = (char*)&(TrapSend.TagList[i].OID_Data);   
		ValidLength =TrapSend.TagList[i].Width+6;            //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((Offset+ValidLength) >=(SENDBUFF_SIZE-3))              //��ָֹ�����
		{
       break;
    }
		TrapSend.TagList[i].Width =ntohs(TrapSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+Offset),pChar,ValidLength);                //����ÿһ������Tag������Buff
		Offset = Offset+ValidLength;	
  }
	
	TrapSend.CrcCode=0xffff;                               //CRC�ֶθ���ֵ
	pSendBuff[Offset++] = TrapSend.CrcCode &0xff;
	pSendBuff[Offset++] = TrapSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, Offset );   // Update the CRC value
	TrapSend.CrcCode =CrcData;
	pSendBuff[Offset-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[Offset-1] = CrcData>>8;     //CRC���ֽ��ں�

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
//		 printf("\r\nReceive TrapResponse success:%4x!\r\n", RecevFlag); //����ʹ��
		 if(RecevFlag ==0x0588)     //�ɹ����յ�TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //����ʹ��
				#endif
			  break;     
     }
//		 Delay_ms(2000); 
  }
	printf("\r\n----Length:%d----\r\n",Offset);   //����ʹ��
	for(i=0;i<Offset;i++)
	{
    printf("-%x-",pSendBuff[i]);   //����ʹ��
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
  struct   rtc_time   SystTime;           //RTCʱ�����ýṹ��
//	struct   DataFrame  GetRespSend;
	struct   DataFrame* pDataFrame =&GetRespSend;
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	char*    pChar =NULL;
	union    Hfloat  UnionData;
	u8       SendCounter =1;     //���ʹ���������
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��OID���У�����Tag���У���ʣ�����ݵĳ���
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //��ʼ���ṹ��
	GetRespSend.Preamble =0xA3;
	GetRespSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    GetRespSend.DeviceID[i] =pDeviceID[i];
  }
	GetRespSend.RouteFlag =0x01;
	GetRespSend.NodeAddr  =ntohs( NodeAddress);              //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	GetRespSend.PDU_Type  =(2<<8)+(1<<7)+8;
	GetRespSend.PDU_Type  =ntohs(GetRespSend.PDU_Type);      //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	GetRespSend.Seq       =1;
	GetRespSend.Length    =CoreLength ;                       //
	for(i=0,GetRespSend.Tag_Count=0; i<RequestPara.OID_Count; i++)
	{
		 switch(RequestPara.OID_List[i])
	   {
        case DEF_NR:            //�ش�����
				{
            GetRespSend.TagList[i].OID_Command =DEF_NR;
					  GetRespSend.TagList[i].Width =1;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.RetryNum;
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				case SYSTERM_TIME:     //ϵͳʱ��
				{
      
						Time_Display(RTC_GetCounter(),&SystTime);     //��ȡϵͳ��ǰʱ��			
						DeviceConfig.Time_Sec  =SystTime.tm_sec;
						DeviceConfig.Time_Min  =SystTime.tm_min;
						DeviceConfig.Time_Hour =SystTime.tm_hour;
						DeviceConfig.Time_Mday =SystTime.tm_mday;		
						DeviceConfig.Time_Mon  =SystTime.tm_mon;
						DeviceConfig.Time_Year =SystTime.tm_year-2000; //���ϴ����ȥ��������
					
					  GetRespSend.TagList[i].OID_Command =SYSTERM_TIME;
					  GetRespSend.TagList[i].Width =6;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.Time_Year;   //ϵͳ���ڣ���
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.Time_Mon;    //ϵͳ���ڣ���
					  GetRespSend.TagList[i].Value[2] =DeviceConfig.Time_Mday;   //ϵͳ���ڣ���
					  GetRespSend.TagList[i].Value[3] =DeviceConfig.Time_Hour;   //ϵͳʱ�䣬Сʱ
					  GetRespSend.TagList[i].Value[4] =DeviceConfig.Time_Min;    //ϵͳʱ�䣬��
					  GetRespSend.TagList[i].Value[5] =DeviceConfig.Time_Sec;    //ϵͳʱ�䣬��
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
        case CLT1_ITRL1:       //һʱ���ɼ����
				{
            GetRespSend.TagList[i].OID_Command =CLT1_ITRL1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.CollectPeriod >>8;   //���ݲɼ���������ֽ���ǰ
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.CollectPeriod &0xff; //���ݲɼ���������ֽ��ں�
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				case CLT1_CNT1:        //һʱ���ɼ�����
				{
            GetRespSend.TagList[i].OID_Command =CLT1_CNT1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.CollectNum >>8;   //���ݲɼ���������ֽ���ǰ
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.CollectNum &0xff; //���ݲɼ���������ֽ��ں�
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }
				case CLT1_STIME1 :    //һʱ���ɼ���ʼʱ��
				{
            GetRespSend.TagList[i].OID_Command =CLT1_STIME1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] = DeviceConfig.CollectStartTime>>8;    //�¶�ѹ�����������ʱ�䣬���ֽ���ǰ 
					  GetRespSend.TagList[i].Value[1] = DeviceConfig.CollectStartTime&0xff;  //�¶�ѹ�����������ʱ�䣬���ֽ��ں�  					
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
//				case RESET_PROBER:     //��λҺλ̽ͷ���������Կ��Ƿŵ�״̬����
//				{
//            GetRespSend.TagList[i].OID_Command =RESET_PROBER;
//					  GetRespSend.TagList[i].Width =1;
//					  GetRespSend.TagList[i].Value[0] =0;         //ֻ�з������·���λ����ʱ��ValueֵΪ1��������Ϊ0
//					  GetRespSend.Tag_Count++;
//					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
//					  break;
//        }	
				case UPLOAD_CYCLE:     //Һλ�������ϱ�����
				{
            GetRespSend.TagList[i].OID_Command =UPLOAD_CYCLE;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.UploadCycle >>8;   //���ݲɼ���������ֽ���ǰ
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.UploadCycle &0xff; //���ݲɼ���������ֽ��ں�
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }					
        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!Tag OID not recognition!\r\n"); //����ʹ��
					 #endif
					 break;
        }
		}
  }
	
	GetRespSend.Length =ntohs(GetRespSend.Length );                //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	GetRespSend.CrcCode=0xffff;                                    //CRC�ֶθ���ֵ
	memcpy(pSendBuff,pChar,PreLength);
	
	for(i=0,j=PreLength;i<GetRespSend.Tag_Count;i++)
	{
    GetRespSend.TagList[i].OID_Command =ntohl(GetRespSend.TagList[i].OID_Command);  //��OID���е���Ϊ���ֽ���ǰ�����ֽ��ں�  //�д�����
		pChar = (char*)&(GetRespSend.TagList[i].OID_Command);   
		ValidLength =GetRespSend.TagList[i].Width+6;           //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //��ָֹ�����
		{
       break;
    }
		GetRespSend.TagList[i].Width =ntohs(GetRespSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = GetRespSend.CrcCode &0xff;
	pSendBuff[j++] = GetRespSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	GetRespSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[j-1] = CrcData>>8;     //CRC���ֽ��ں�
  
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
		 printf("\r\nReceive GetResponse success:%4x!\r\n", RecevFlag); //����ʹ��
		 if((RecevFlag ==0x0188)||(RecevFlag ==0x0388)||(RecevFlag ==0x0088))     //�ɹ����յ�GetResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive GetResponse success!\r\n"); //����ʹ��
				#endif
			  break;     
     }
//		 Delay_ms(3000); 
  }
  
	printf("\r\n----Length:%d----\r\n",j);   //����ʹ��
	for(i=0;i<j;i++)
	{
    printf(",0x%x",pSendBuff[i]);   //����ʹ��
  }

}
