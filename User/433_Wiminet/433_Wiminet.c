
// File Name: 433_Wiminet.c

#include "string.h"
#include "API-Platform.h"
#include "433_Wiminet.h"
#include "bsp_SysTick.h"
#include "gprs.h"
#include "bsp_usart.h"
#include "common.h"

extern  char Usart3_recev_buff[RECEIVEBUFF_SIZE];
extern  u16  Usart3_recev_count;
extern  u8   DMA_UART3_RECEV_FLAG ;       //USART3 DMA接收标志变量
//extern  uint32_t  time;                     // ms 计时变量 

extern u8   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
extern u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3接收数据监测与数据解析
extern int  DMA_UART3_RecevDataGet(void);

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char ReverseBitOrder08( unsigned char iSrc )
{
   unsigned char index;
   unsigned char iDst;
   
   iDst = iSrc & 0X01;
   for( index = 0X00; index < 0X07; index++ )
   {
      iDst <<= 0X01;
      iSrc >>= 0X01;
      iDst |= ( iSrc & 0X01 );
   }
   return iDst;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short ReverseBitOrder16( unsigned short iSrc )
{
   unsigned char index;
   unsigned short iDst;
   
   iDst = iSrc & 0X01;
   for( index = 0X00; index < 0X0F; index++ )
   {
      iDst <<= 0X01;
      iSrc >>= 0X01;
      iDst |= ( iSrc & 0X01 );
   }
   return iDst;
}

// *****************************************************************************
// Design Notes: CRC-16
// f(X)=X^16 + X^15 + X^2 + X^0
// POLYNOMIALS = 0X8005
// -----------------------------------------------------------------------------
unsigned short CRC16( unsigned char * pMsg, unsigned short iSize )
{
   unsigned char  index;
   unsigned short iCRC;
   
   // The default value
   iCRC = 0XFFFF;
   while ( iSize-- )
   {
      iCRC ^= ( ( ( unsigned short ) ReverseBitOrder08( *pMsg ) ) << 0X08 );
      for ( index = 0X00; index < 0X08; index++ )
      {
         if ( iCRC & 0X8000 )
         {
            iCRC = ( iCRC << 1 ) ^ 0X8005;
         }
         else
         {
            iCRC <<= 1;
         }
      }
      pMsg++;
   }
   return ReverseBitOrder16( iCRC );
}

//// -----------------------------------------------------------------------------
//// DESCRIPTION: CRC-16校验的高位字节表
//// -----------------------------------------------------------------------------
//static const unsigned char HiCRCTable[] = { 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40 };

//// -----------------------------------------------------------------------------
//// DESCRIPTION: CRC-16校验的低位字节表
//// -----------------------------------------------------------------------------
//static const unsigned char LoCRCTable[] = { 
//0X00, 0XC0, 0XC1, 0X01, 0XC3, 0X03, 0X02, 0XC2, 0XC6, 0X06, 0X07, 0XC7, 0X05, 0XC5, 0XC4, 0X04, 
//0XCC, 0X0C, 0X0D, 0XCD, 0X0F, 0XCF, 0XCE, 0X0E, 0X0A, 0XCA, 0XCB, 0X0B, 0XC9, 0X09, 0X08, 0XC8, 
//0XD8, 0X18, 0X19, 0XD9, 0X1B, 0XDB, 0XDA, 0X1A, 0X1E, 0XDE, 0XDF, 0X1F, 0XDD, 0X1D, 0X1C, 0XDC, 
//0X14, 0XD4, 0XD5, 0X15, 0XD7, 0X17, 0X16, 0XD6, 0XD2, 0X12, 0X13, 0XD3, 0X11, 0XD1, 0XD0, 0X10, 
//0XF0, 0X30, 0X31, 0XF1, 0X33, 0XF3, 0XF2, 0X32, 0X36, 0XF6, 0XF7, 0X37, 0XF5, 0X35, 0X34, 0XF4, 
//0X3C, 0XFC, 0XFD, 0X3D, 0XFF, 0X3F, 0X3E, 0XFE, 0XFA, 0X3A, 0X3B, 0XFB, 0X39, 0XF9, 0XF8, 0X38, 
//0X28, 0XE8, 0XE9, 0X29, 0XEB, 0X2B, 0X2A, 0XEA, 0XEE, 0X2E, 0X2F, 0XEF, 0X2D, 0XED, 0XEC, 0X2C, 
//0XE4, 0X24, 0X25, 0XE5, 0X27, 0XE7, 0XE6, 0X26, 0X22, 0XE2, 0XE3, 0X23, 0XE1, 0X21, 0X20, 0XE0, 
//0XA0, 0X60, 0X61, 0XA1, 0X63, 0XA3, 0XA2, 0X62, 0X66, 0XA6, 0XA7, 0X67, 0XA5, 0X65, 0X64, 0XA4, 
//0X6C, 0XAC, 0XAD, 0X6D, 0XAF, 0X6F, 0X6E, 0XAE, 0XAA, 0X6A, 0X6B, 0XAB, 0X69, 0XA9, 0XA8, 0X68, 
//0X78, 0XB8, 0XB9, 0X79, 0XBB, 0X7B, 0X7A, 0XBA, 0XBE, 0X7E, 0X7F, 0XBF, 0X7D, 0XBD, 0XBC, 0X7C, 
//0XB4, 0X74, 0X75, 0XB5, 0X77, 0XB7, 0XB6, 0X76, 0X72, 0XB2, 0XB3, 0X73, 0XB1, 0X71, 0X70, 0XB0, 
//0X50, 0X90, 0X91, 0X51, 0X93, 0X53, 0X52, 0X92, 0X96, 0X56, 0X57, 0X97, 0X55, 0X95, 0X94, 0X54, 
//0X9C, 0X5C, 0X5D, 0X9D, 0X5F, 0X9F, 0X9E, 0X5E, 0X5A, 0X9A, 0X9B, 0X5B, 0X99, 0X59, 0X58, 0X98, 
//0X88, 0X48, 0X49, 0X89, 0X4B, 0X8B, 0X8A, 0X4A, 0X4E, 0X8E, 0X8F, 0X4F, 0X8D, 0X4D, 0X4C, 0X8C, 
//0X44, 0X84, 0X85, 0X45, 0X87, 0X47, 0X46, 0X86, 0X82, 0X42, 0X43, 0X83, 0X41, 0X81, 0X80, 0X40 };


//// *****************************************************************************
//// Design Notes:  
//// -----------------------------------------------------------------------------
//unsigned short QuickCRC16( unsigned char * pMsg, unsigned short iSize )
//{
//   unsigned char iHiVal;                // high byte of CRC initialized
//   unsigned char iLoVal;                // low byte of CRC initialized
//   unsigned char index;                 // will index into CRC lookup table
//   
//   // Initial value for the CRC
//   iHiVal = 0XFF;
//   iLoVal = 0XFF;
//   
//   while ( iSize-- )
//   {
//      // Calculate the CRC
//      index = iLoVal ^ ( unsigned char )( *pMsg++ );
//      
//      iLoVal = iHiVal ^ HiCRCTable[index];
//      iHiVal = LoCRCTable[index];
//   }
//   return ( iHiVal << 8 | iLoVal );
//}
// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void UpdateNodeMsgCRC( NodeMsg * pMsg )
{
   unsigned char  iSize;
   unsigned short iCRC;
   
   // The message header
//   pMsg->m_iHeader = 0xAA;

   // The defualt CRC value
   pMsg->m_iCRCode = 0x0000;

   // The message size
   iSize = 0x09;
   iSize += pMsg->m_iAmount;

   // Update the CRC value
   iCRC = CRC16( ( unsigned char * )pMsg, iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if ( CPU_ENDIAN_MODE == LITTLE_ENDIAN_MODE )

   // Change the byte order
   iCRC = ntohs( iCRC );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif

   // Restore the CRC of this message
   pMsg->m_iCRCode = iCRC;
}

//unsigned short ReverseByteOrder (unsigned short OriginalData)
//{
//   char* pData = NULL;
//	 pData = (char*)&OriginalData;
//	 return  (*pData<<8) + *(pData+1);
//}

// *****************************************************************************
// Design Notes: 初始化结构体
// -----------------------------------------------------------------------------
void InitCommandMessage( NodeMsg * pMsg )
{
//   // Initialize the message
//   InitWiMinetMessage( pMsg, 0X01 );   
	 // Reset the message body
   memset( pMsg, 0x00, sizeof( NodeMsg ) );
}

//// *****************************************************************************
//// Design Notes:  对查询主站ID函数封包

//void IMP_GetCoordinatorID( NodeMsg * pMsg )
//{
//   // The command for this packet
//   pMsg->m_iHeader = 0xAA;
//	 // The command for this packet
////   pMsg->m_iOpCode = CMDMSG_GET_COORDINATOR_ID;
//   pMsg->m_iOpCode = 0x76;  
//}
//// *****************************************************************************
//// Design Notes: 查询主站ID
//// -----------------------------------------------------------------------------
// unsigned short GetCoordinatorID (void)
//{
//   NodeMsg  Msg;      //正式使用时不用
//	 NodeMsg* pMsg = &Msg;
//   unsigned short iSize;
////	 char  i=0;                   //调试使用
//	 char  receive_flag =0;
//	 //char  SendBuffer[12]={0xAA,0x1D,0x33,0x86,0x03,0x00,0x03,0x00,0x00,0x01,0x02,0x03};
//   char  ID_GetFlag[2]={0xAA,0xF6};
//   char*  pRecevBuff=NULL;
//   unsigned short  ID_Temp=0x0000;

//   // Initialize the message body
//   InitCommandMessage( pMsg );

//   // Construct the message
//   IMP_GetCoordinatorID( pMsg );
//   UpdateNodeMsgCRC( pMsg );
//	 // The total message size
//   iSize = pMsg->m_iAmount + 0x09;

//	 USART_DataBlock_Send(USART1,(char* )(pMsg),iSize);    //调试使用
//   USART_DataBlock_Send(USART1,"\r\n",2);                //调试使用
//	 USART_DataBlock_Send(USART3,(char* )(pMsg),iSize);
//	 
//	 Delay_ms(1000);
//   receive_flag = Receive_Monitor_433();
//	 if(receive_flag == 1)
//	 {
//		 pRecevBuff = Find_SpecialString(Usart3_recev_buff,ID_GetFlag,300,2);  //检查有无收到主站回复
//		 if(pRecevBuff!=NULL)                                     //有回复，提取地址信息   //暂时缺少CRC校验
//		 {   
//			
////			 printf("\r\nResponse from 433 is:");                   //调试使用
////			 for(i=0;i<Usart3_recev_count;i++)                      //调试使用
////			 {
////					printf(" %x",pRecevBuff[i]);                        //调试使用
////			 }
//			 ID_Temp = (pRecevBuff[9]*256) +pRecevBuff[10];
//			 pRecevBuff =NULL;
//			 receive_flag = 0;
//			 memset(Usart3_recev_buff,'\0',300);	
//			 Usart3_recev_count =0;                                 //清空USART3接收计数器
//			 time=0;	                                               //定时器复位
//			 return  ID_Temp;
//		 }
//		 else
//		 {	 
//				memset(Usart3_recev_buff,'\0',300);	
//				Usart3_recev_count =0;                                 //清空USART3接收计数器
//				time=0;	                                               //定时器复位
//		 }
//	 }
//	 return  0x0000;
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>----配置433模块参数----Start----<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块串口波特率和校验类型
* Input          : 波特率设置参数范围：1~ 7，参数含义：1=1200,2=2400,3=4800,4=9600,5=19200,6=38400,7=57600 ；校验类型参数范围：0~ 2，参数含义：0=无校验,1=奇校验,2=偶校验
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeSerialPort(u8 sBaudRate, u8 sVerifyType)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x09,0x02,0xFF,0xFF,0xFF,0x0D,0x0A};  //3个0xFF字段分别为模块串口速率、校验类型和校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  if((sBaudRate>=1 )&&(sBaudRate<=7 )&&((sVerifyType==0 )||(sVerifyType==1 )||(sVerifyType==2 )))  //参数有效性检验
	{
    SetCommand[8] =sBaudRate;
		SetCommand[9] =sVerifyType;
		for(i=0;i<10;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[10] =VerifySum %256;
		mput_mix((char*)SetCommand,13);
		for(i=0;i<10;i++)
		{
			Delay_ms(200);                        
			
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);              //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);     //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF	
				}
				DMA_UART3_RECEV_FLAG =0;                             //复位DMA数据接收标志变量       
				break;
			}	
		}
  }
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nBaudRate or VerifyType is out of limits!!\r\n");            //测试使用
		 #endif
  }
	
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块载波中心频率
* Input          : 16进制表示频率（单位MHz）,参数范围：421MHz ~ 500MHz（0x01A5 ~ 0x01F4）,应避开频点448MHz（0x01C0）和480MHz（0x01E0）
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeCentralFrequency(u16 sFreq)
{
	u8  SetCommand[14] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x0B,0x03,0xFF,0xFF,0xFF,0xFF,0x0D,0x0A};  //前3个0xFF字段表示设置的载波频率，最后一个0xFF字段表示校验和，分别需要输入和计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;
  u32 FreqTemp =0;

  if((sFreq>=421)&&(sFreq<=500))    //参数有效性检验
	{
		FreqTemp = (sFreq*1000000)/61.035;    //数据计算精度有待进一步测试
		SetCommand[8]  =(FreqTemp>>16)&0xFF;
		SetCommand[9]  =(FreqTemp>>8)&0xFF;
		SetCommand[10] =FreqTemp&0xFF;
		for(i=0;i<11;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[11] =VerifySum %256;
		mput_mix((char*)SetCommand,14);

		for(i=0;i<10;i++)
		{
			Delay_ms(200);                        
			
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);              //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);     //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF	
				}
				DMA_UART3_RECEV_FLAG =0;                             //复位DMA数据接收标志变量       
				break;
			}	
		}
  }
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nCentral Frequency is out of limits!!\r\n");            //测试使用
		 #endif
  }
}

/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块扩频因子
* Input          : 扩频因子参数范围：7~ 12，参数含义：7=128,8=256,9=512,10=1024,11=2048,12=4096
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeFrequencyExpandFactor(u8 sExpFactor)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x0D,0x02,0xFF,0x00,0xFF,0x0D,0x0A};  //2个0xFF字段分别为模块扩展因子编码、校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  if((sExpFactor>=7)&&(sExpFactor<=12))  //参数有效性检验
	{
		SetCommand[8] =sExpFactor;
		for(i=0;i<10;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[10] =VerifySum %256;
		mput_mix((char*)SetCommand,13);

		for(i=0;i<10;i++)
		{
			Delay_ms(200);                        
			
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);              //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);     //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF	
				}
				DMA_UART3_RECEV_FLAG =0;                             //复位DMA数据接收标志变量       
				break;
			}	
		}
	} 
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nFrequency Expand Factor is out of limits!!\r\n");            //测试使用
		 #endif
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块扩频带宽
* Input          : 扩频带宽参数范围：6~ 9，参数含义：6=62.5K,7=125K,8=256K,9=512K 
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeFrequencyExpandBandwidth(u8 sBandwidth)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x0F,0x02,0xFF,0x00,0xFF,0x0D,0x0A};  //2个0xFF字段分别为模块扩频带宽编码、校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  if((sBandwidth>=6)&&(sBandwidth<=9))  //参数有效性检验
	{
		
		SetCommand[8] =sBandwidth;
		for(i=0;i<10;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[10] =VerifySum %256;
		mput_mix((char*)SetCommand,13);

		for(i=0;i<10;i++)
		{
			Delay_ms(200);                        
			
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);              //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);     //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF	
				}
				DMA_UART3_RECEV_FLAG =0;                             //复位DMA数据接收标志变量       
				break;
			}	
		}
	}
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nFrequency Expand Bandwidth is out of limits!!\r\n");            //测试使用
		 #endif
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块工作模式
* Input          : 工作模式参数范围：0~ 2，参数含义：0正常，1中心，2节点
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeWorkMode(u8 sWorkMode)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x11,0x02,0xFF,0x00,0xFF,0x0D,0x0A};  //2个0xFF字段分别为模块工作模式编码、校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  if((sWorkMode==0)||(sWorkMode==1)||(sWorkMode==2))  //参数有效性检验
	{
		SetCommand[8] =sWorkMode;
		for(i=0;i<10;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[10] =VerifySum %256;
		mput_mix((char*)SetCommand,13);

		for(i=0;i<10;i++)
		{
			Delay_ms(200);                        
			
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);              //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);     //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF	
				}
				DMA_UART3_RECEV_FLAG =0;                             //复位DMA数据接收标志变量       
				break;
			}	
		}
	}
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nNode Work Mode is out of limits!!\r\n");            //测试使用
		 #endif
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块ID
* Input          : 模块ID参数范围：0 ~ 65535
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeID (u16 sNodeID)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x13,0x02,0xFF,0xFF,0xFF,0x0D,0x0A};  //3个0xFF字段分别为节点ID高字节、节点ID低字节和校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  SetCommand[8] =sNodeID >>8;
  SetCommand[9] =sNodeID &0xFF;
  for(i=0;i<10;i++)
  {
    VerifySum =VerifySum + SetCommand[i];
  }
	SetCommand[10] =VerifySum %256;
  mput_mix((char*)SetCommand,13);

  for(i=0;i<10;i++)
	{
		Delay_ms(200);                                
		if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
		{
			Len = DMA_UART3_RecevDataGet();
		  if(Len >0)
		  {
				printf("\r\nDataLength:%d\r\n", Len);             //测试使用
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);    //测试使用
			  for(i=0;i<Len;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
        }
        memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF
		  }
			DMA_UART3_RECEV_FLAG =0;                           //复位DMA数据接收标志变量      
			break;
    }	
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块网络ID
* Input          : 模块网络ID参数范围：0 ~ 255
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeNetworkID (u8 sNetID)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x15,0x02,0xFF,0x00,0xFF,0x0D,0x0A};  //2个0xFF字段分别为节点网络ID和校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  SetCommand[8] =sNetID;
  for(i=0;i<10;i++)
  {
    VerifySum =VerifySum + SetCommand[i];
  }
	SetCommand[10] =VerifySum %256;
  mput_mix((char*)SetCommand,13);

  for(i=0;i<10;i++)
	{
		Delay_ms(200);                                
		if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
		{
			Len = DMA_UART3_RecevDataGet();
		  if(Len >0)
		  {
				printf("\r\nDataLength:%d\r\n", Len);             //测试使用
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);    //测试使用
			  for(i=0;i<Len;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
        }
        memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF
		  }
			DMA_UART3_RECEV_FLAG =0;                           //复位DMA数据接收标志变量      
			break;
    }	
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块发射功率等级
* Input          : 发射功率等级参数范围：1~ 7，参数含义：1=4dBm,2=7dBm,3=10dBm,4=13dBm,5=14dBm,6=17dBm,7=20dBm
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeSendPowerGrade (u8 PowerGrade)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x17,0x02,0xFF,0x00,0xFF,0x0D,0x0A};  //2个0xFF字段分别为模块发射功率等级与校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  if((PowerGrade>=1)&&(PowerGrade<=7))  //参数有效性检验
	{
		SetCommand[8] =PowerGrade;
		for(i=0;i<10;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[10] =VerifySum %256;
		mput_mix((char*)SetCommand,13);

		for(i=0;i<10;i++)
		{
			Delay_ms(200);                                
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);             //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);    //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF
				}
				DMA_UART3_RECEV_FLAG =0;                           //复位DMA数据接收标志变量      
				break;
			}	
		}
	}
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nNode Send Power Grade is out of limits!!\r\n");            //测试使用
		 #endif
  }
	
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块呼吸周期
* Input          : 呼吸周期参数范围：0~ 4，参数含义：0=2S, 1=4S, 2=6S , 3=8S , 4=10S 
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeBreathPeriod (u8 sBreath)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x19,0x02,0xFF,0x00,0xFF,0x0D,0x0A};  //2个0xFF字段分别为模块呼吸周期与校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  if((sBreath==0)||(sBreath==1)||(sBreath==2)||(sBreath==3)||(sBreath==4))  //参数有效性检验
	{
		 SetCommand[8] =sBreath;
		for(i=0;i<10;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[10] =VerifySum %256;
		mput_mix((char*)SetCommand,13);

		for(i=0;i<10;i++)
		{
			Delay_ms(200);                                
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);             //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);    //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF
				}
				DMA_UART3_RECEV_FLAG =0;                           //复位DMA数据接收标志变量      
				break;
			}	
		}
	}
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nNode Breath Period is out of limits!!\r\n");            //测试使用
		 #endif
  }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 设置433模块呼吸时间
* Input          : 呼吸时间参数范围：0~ 5，参数含义：0=2ms,1=4ms, 2 =8ms, 3=16ms, 4=32ms,5=64ms
* Output         : None
* Return         : None
*******************************************************************************/
void SetNodeBreathTime (u8 sWakeTime)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x1B,0x02,0xFF,0x00,0xFF,0x0D,0x0A};  //2个0xFF字段分别为模块呼吸时间与校验和，需要输入或者计算得到
  u16 VerifySum =0x00;
  u8  i =0;
  u16 Len=0;

  if((sWakeTime==0)||(sWakeTime==1)||(sWakeTime==2)||(sWakeTime==3)||(sWakeTime==4)||(sWakeTime==5))  //参数有效性检验
	{
	  SetCommand[8] =sWakeTime;
		for(i=0;i<10;i++)
		{
			VerifySum =VerifySum + SetCommand[i];
		}
		SetCommand[10] =VerifySum %256;
		mput_mix((char*)SetCommand,13);

		for(i=0;i<10;i++)
		{
			Delay_ms(200);                                
			if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			{
				Len = DMA_UART3_RecevDataGet();
				if(Len >0)
				{
					printf("\r\nDataLength:%d\r\n", Len);             //测试使用
					printf("\r\nUart3:%s\r\n", Usart3_recev_buff);    //测试使用
					for(i=0;i<Len;i++)
					{
							 printf(" %.2x ",Usart3_recev_buff[i]);        //测试使用
					}
					memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);//复位DMA数据接收BUFF
				}
				DMA_UART3_RECEV_FLAG =0;                           //复位DMA数据接收标志变量      
				break;
			}	
		}
  }
	else
	{
     #if DEBUG_TEST	 
		 printf("\r\nNode Breath Time is out of limits!!\r\n");            //测试使用
		 #endif
  }
}
////////////////////////////////////////----配置433模块参数----End----///////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>----读取433模块参数----Start----<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块上一帧数据接收信号强度
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNodeReceiveSignalEnergy (void)
{
   char   ReadCommand[13]={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x06,0x02,0x00,0x00,0x95,0x0D,0x0A};
   char   GetNodeRSSI[8]  ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x06,0x02};
   char*  pRecevBuff =NULL;
   u8     NodeRSSI =0x00;         //接收信号强度绝对值（接收信号强度均为负值），经过-164修正

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetNodeRSSI,300,8);                    //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-14))    
	 {   
			NodeRSSI = pRecevBuff[8];
		  if(NodeRSSI <164)
			{
         NodeRSSI = 164 -NodeRSSI;
				 printf("\r\nNode RSSI:-%d\r\n",NodeRSSI);                   //调试使用
      }
			else
			{
         NodeRSSI = NodeRSSI -164;
				 printf("\r\nNode RSSI:+%d\r\n",NodeRSSI);                   //调试使用
      }
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  NodeRSSI;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块串口配置参数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u16 GetNodeSerialPortConfig (void)
{
   char   ReadCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x0A,0x02,0x00,0x00,0x99,0x0D,0x0A};
   char   GetNodeSerialConfig[8]  ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x0A,0x02};
   char*  pRecevBuff =NULL;
   u8     BaudRate =0;     //433模块串口波特率参数，参数范围：1~ 7，参数含义：1=1200,2=2400,3=4800,4=9600,5=19200,6=38400,7=57600 
   u8     VerifyType =0;   //433模块串口校验类型参数，参数范围：0~ 2，参数含义：0=无校验,1=奇校验,2=偶校验
   u16    UnionTemp  =0;   //组合串口波特率和串口校验类型     

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetNodeSerialConfig,300,8);         //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-14))    
	 {   
			BaudRate   = pRecevBuff[8];
		  VerifyType = pRecevBuff[9];
		  UnionTemp  =(BaudRate<<8)+ VerifyType;
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  UnionTemp;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFFFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块载波频率参数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u16 GetNodeCentralFrequency (void)
{
   char   ReadCommand[14] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x0C,0x03,0x00,0x00,0x00,0x9C,0x0D,0x0A};
   char   GetNodeCentralFreq[8]  ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x0C,0x03};
   char*  pRecevBuff =NULL;
   u16    CentralFreq =0;     //433模块通信载波频率（单位：MHz）
   u32    FreqTemp =0; 
   
   mput_mix(ReadCommand,14);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetNodeCentralFreq,300,8);               //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-15))            
	 {   
			FreqTemp    =(pRecevBuff[8]<<16)+(pRecevBuff[9]<<8)+pRecevBuff[10];
		  CentralFreq =FreqTemp*61.035/1000000;                                       
		 
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  CentralFreq;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFFFF;
}

/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块扩频因子参数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNodeFrequencyExpandFactor (void)
{
   char   ReadCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x0E,0x02,0x00,0x00,0x9D,0x0D,0x0A};
   char   GetNodeExpFactor[8]  ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x0E,0x02};
   char*  pRecevBuff =NULL;
   u8     ExpFactor  =0;    //433模块扩频因子，参数范围：7~ 12，参数含义：7=128,8=256,9=512,10=1024,11=2048,12=4096

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetNodeExpFactor,300,8);                    //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-14))      
	 {   
			ExpFactor  = pRecevBuff[8];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  ExpFactor;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块扩频带宽参数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNodeFrequencyExpandBandwidth (void)
{
   char   ReadCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x10,0x02,0x00,0x00,0x9F,0x0D,0x0A};
   char   GetNodeExpBW[8]  ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x10,0x02};
   char*  pRecevBuff =NULL;
   u8     ExpBandwidth  =0;    //433模块扩频因子，参数范围：6~ 9，参数含义：6=62.5K,7=125K,8=256K,9=512K 

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetNodeExpBW,300,8);                   //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-14))   
	 {   
			ExpBandwidth  = pRecevBuff[8];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  ExpBandwidth;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块工作模式参数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNodeWorkMode (void)
{
   char   ReadCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x12,0x02,0x00,0x00,0xA1,0x0D,0x0A};
   char   GetWorkMode[8]  ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x12,0x02};
   char*  pRecevBuff =NULL;
   u8     WorkMode  =0;    //433模块工作模式参数范围：0~ 2，参数含义：0正常，1中心，2节点

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetWorkMode,300,8);                   //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-14))   
	 {   
			WorkMode  = pRecevBuff[8];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  WorkMode;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块节点ID
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u16 GetNodeID (void)
{
   char   ReadCommand[13]={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x14,0x02,0x00,0x00,0xA3,0x0D,0x0A};
   char   GetNodeID[8]  ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x14,0x02};
   char*  pRecevBuff =NULL;
   u16    NodeID =0x0000;     //433模块节点ID

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetNodeID,300,8);                   //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-12)) 
	 {   
			NodeID = (pRecevBuff[8]*256) +pRecevBuff[9];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  NodeID;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFFFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块网络ID
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNetworkID (void)
{
   char   ReadCommand[13]={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x16,0x02,0x00,0x00,0xA5,0x0D,0x0A};
   char   GetNetID[8]    ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x16,0x02};
   char*  pRecevBuff =NULL;
   u8     NetID =0x00;     //433模块节点ID

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetNetID,300,8);                   //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-12)) 
	 {   
			NetID = pRecevBuff[8];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return   NetID;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块发射功率
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNodeSendPowerGrade (void)
{
   char   ReadCommand[13]  ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x18,0x02,0x00,0x00,0xA7,0x0D,0x0A};
   char   GetPowerGrade[8] ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x18,0x02};
   char*  pRecevBuff =NULL;
   u8     PowerGrade =0x00;     //433模块发射功率等级参数范围：1~ 7，参数含义：1=4dBm,2=7dBm,3=10dBm,4=13dBm,5=14dBm,6=17dBm,7=20dBm

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetPowerGrade,300,8);                   //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-12)) 
	 {   
			PowerGrade = pRecevBuff[8];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  PowerGrade;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块呼吸周期
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNodeBreathPeriod (void)
{
   char   ReadCommand[13]  ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x1A,0x02,0x00,0x00,0xA9,0x0D,0x0A};
   char   GetBreathPeriod[8] ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x1A,0x02};
   char*  pRecevBuff =NULL;
   u8     BreathPeriod =0x00;     //433模块呼吸周期参数范围：0~ 4，参数含义：0=2S, 1=4S, 2=6S , 3=8S , 4=10S 
//   u8     i=0;   //测试使用

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
		 
//		printf("\r\nUart3:%s\r\n", Usart3_recev_buff);           //测试使用
//		for(i=0;i<50;i++)
//		{
//         printf(" %.2x ",Usart3_recev_buff[i]);               //测试使用
//     } 
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetBreathPeriod,300,8);                   //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-12)) 
	 {   
			BreathPeriod = pRecevBuff[8];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  BreathPeriod;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}
/*******************************************************************************
* Function Name  : XX
* Description    : 获取433模块呼吸时间
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 GetNodeBreathTime (void)
{
   char   ReadCommand[13]  ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x1C,0x02,0x00,0x00,0xAB,0x0D,0x0A};
   char   GetBreathTime[8] ={0xAF,0xAF,0x00,0x00,0xAF,0x00,0x1C,0x02};
   char*  pRecevBuff =NULL;
   u8     WakeTime =0x00;     //433模块呼吸时间参数范围：0~ 5，参数含义：0=2ms,1=4ms, 2 =8ms, 3=16ms, 4=32ms,5=64ms

   mput_mix(ReadCommand,13);
	 Delay_ms(2000);
   if(DMA_UART3_RECEV_FLAG==1)
   {
		 DMA_UART3_RecevDataGet();
     DMA_UART3_RECEV_FLAG =0;
   } 
	 
	 pRecevBuff = Find_SpecialString(Usart3_recev_buff, GetBreathTime,300,8);                   //检查有无收到模块应答
	 if((pRecevBuff!=NULL) &&(pRecevBuff< Usart3_recev_buff +sizeof(Usart3_recev_buff)-12)) 
	 {   
			WakeTime = pRecevBuff[8];
			pRecevBuff =NULL;
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器
			return  WakeTime;
	 }
	 else
	 {	 
			memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));	
			Usart3_recev_count =0;                                 //清空USART3接收计数器                                        
	 } 
	 return  0xFF;
}

////////////////////////////////////////----读取433模块参数----End----///////////////////////////////////////////////////
// *****************************************************************************
// Design Notes:  发送数据封包
// -----------------------------------------------------------------------------
void IMP_SendMessage( NodeMsg * pMsg )
{ 
	 // The command for this packet
   pMsg->m_iHeader = 0xAA;
	 // The command for this packet
   pMsg->m_iOpCode = 0x00;  
	 pMsg->m_iValueC = 0x03;
	 pMsg->m_iValueD = 0x00;
}
// *****************************************************************************
// Design Notes:  发送数据主函数
// -----------------------------------------------------------------------------
u16  SendMessage(u8* Psend, unsigned short iSize, u8* pDeviceID, u16 Address_Node)
{
	 NodeMsg  Msg;
	 NodeMsg* pMsg = &Msg;
	 u16  RevFlag =0;
	 u8   i=0,j=0;

	 // Initialize the message body
   InitCommandMessage( pMsg );
	 IMP_SendMessage( pMsg );
	 pMsg->m_iValueB = Address_Node & 0xff;
	 pMsg->m_iValueA = Address_Node >>8;
   if(iSize>255)
	 {
     iSize =255;  //限定长度，防止缓存溢出
		 printf("\r\nSendBuff OverFlow!!\r\n");                   //调试使用
   }
	 memcpy(pMsg->m_pBuffer,Psend,iSize);
	 pMsg->m_iAmount = iSize;
	 UpdateNodeMsgCRC( pMsg );
	 iSize = pMsg->m_iAmount + 0x09;
    
   mput_mix_sx1278((char*)(pMsg), iSize); 
//	 Delay_ms(500);     //
// 	 GPIO_SetBits(GPIOB,GPIO_Pin_0);                  //433模块EN管脚拉高
//	 Delay_ms(500); 
//   mput_mix(End, 1);                                //需要在发送完数据以后发送结束符。
	 
	 srand(RTC_GetCounter());                           //测试使用
	 j= random(70); 
   printf("\r\nRandom Data:%d\r\n",j);                //测试使用
	 for(i=0; i<(40+j); i++)
	 {
     Delay_ms(200);                                   
		 if(DMA_UART3_RECEV_FLAG==1)                      //查询数据接收情况
		 {
				RevFlag =DMA_UART3_RecevDetect(pDeviceID, Address_Node);                 //接收数据解析处理
			  break;
		 }
   }
	 printf("\r\nSend Over!!\r\n");                     //调试使用
	 return RevFlag;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char IsValidNodeMsg( NodeMsg * pMsg )
{
   unsigned short iCRC1;
   unsigned short iCRC2;
   unsigned short iSize;
//   u8   i=0;     //测试使用
//	 u8*   pchar=NULL;     //测试使用 
   // Check the header
   if ( pMsg->m_iHeader != 0xAA )
   {
      return 0x00;
   }
   // The original CRC
   iCRC1 = pMsg->m_iCRCode;

   // Clear the CRC
   pMsg->m_iCRCode = 0x00;

   // The total message size
   iSize = 0x09;
   iSize += pMsg->m_iAmount;
	 
//   printf(" \r\nCRC body:");    //测试使用
//	 for(i=0,pchar=(u8*)pMsg;i<iSize;i++)
//	 {
//     printf(" %x",*pchar);    //测试使用
//		 pchar++;
//   }
   // Validate the CRC of this message
    iCRC2 = CRC16( ( unsigned char * )pMsg, iSize );
//    printf("\r\nCRC2:%4x\r\n",iCRC2);    //测试使用
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if ( CPU_ENDIAN_MODE == LITTLE_ENDIAN_MODE )

   // Change the byte order
   iCRC2 = ntohs( iCRC2 );
//	 printf("CRC is :%4x\r\n", iCRC2);    测试使用

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif

   // Restore the CRC of this message
   pMsg->m_iCRCode = iCRC1;

   // Check the CRC value
   return ( iCRC1 == iCRC2 );
}

// *****************************************************************************
// Design Notes: 接收数据解析函数
// -----------------------------------------------------------------------------
u8  ReceiveMessageVerify(char*  pRecevBuff)
{
	 NodeMsg  Msg;
	 NodeMsg* pMsg = &Msg;
   u16      i=0;
   u8       MessageRecv_Flag=0;
//   char*  pRecevBuff=NULL;

//	  #if DEBUG_TEST	  
//    printf("\r\nReceive from 433:");
//		for(i=0;i<150;i++)
//		{
//       printf(" %x",pRecevBuff[i]);
//    }
//		#endif
	 pMsg->m_iHeader = pRecevBuff[0];
	 pMsg->m_iOpCode = pRecevBuff[1];
	 pMsg->m_iValueA = pRecevBuff[2];
	 pMsg->m_iValueB = pRecevBuff[3];
	 pMsg->m_iValueC = pRecevBuff[4];
	 pMsg->m_iValueD = pRecevBuff[5];
	 pMsg->m_iAmount = pRecevBuff[6];
	 pMsg->m_iCRCode = (pRecevBuff[8]*256)+pRecevBuff[7];    //顺序有待进一步确认

   for(i=0;i<(pMsg->m_iAmount);i++)
	 {
      pMsg->m_pBuffer[i] = pRecevBuff[9+i];
   } 
	 MessageRecv_Flag = IsValidNodeMsg(pMsg);
	 if(MessageRecv_Flag ==1)                           //接收数据有效        
	 {
		 #if DEBUG_TEST	
		 printf("\r\nData receive from %4x is :",(pMsg->m_iValueA*256)+(pMsg->m_iValueB));  //测试使用
		 for(i=0;i<(pMsg->m_iAmount);i++)
		 {    
			  printf(" %x ",pMsg->m_pBuffer[i]);   //测试使用
		 } 
		 #endif
		 return 1;
	 }
	 return 0;
}




//// *****************************************************************************
//// Design Notes:  
//// -----------------------------------------------------------------------------
//void WriteNodeMsg( unsigned char hFile, NodeMsg * pMsg )
//{
//   unsigned char iSize;

//   // Update the message CRC value
//   UpdateNodeMsgCRC( pMsg );

//   // The packet whole size
//   iSize = 0X09 + pMsg->m_iAmount;

//   // Write out message header
////   WriteFile( hFile, ( char * )pMsg, iSize );   //暂时注释掉
//}











