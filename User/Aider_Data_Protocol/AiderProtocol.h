#ifndef _AIDERPROTOCOL_H_
#define _AIDERPROTOCOL_H_

#include "stm32f10x.h"
#include "API-Platform.h"
#include "stdio.h" 
#define   MAX   10              //����һ֡������������OID����    
 
 // -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if defined ( __CC_ARM ) 

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#pragma push

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#pragma pack( 1 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif
//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
union Hfloat
{
   u8       Data_Hex[4];           
	 float    Data_Float;
};	                        //���������ݸ�ʽת��

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct SenserData 
{

	u16    CollectTime[MAX];           //ÿ�����ݶ�Ӧ�Ĳɼ�ʱ�䣬��ÿ�����Ϊ�ο�����λ����
	float  PressureData[MAX];          //�ɼ�����ѹ������
	float  TemperatureData[MAX];       //�ɼ������¶�����	      
	u8     DataCount;                  //�ɼ�������������,�¶�ѹ�����ݳɶԳ���

};

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:����Flash���ݴ洢�ͽ���
// -----------------------------------------------------------------------------
struct Config_RegPara 
{
	union 
	{
     u8       Data_Hex[4];              //ѹ��������ֵ���洢������ΪFloat���Ͷ�Ӧ��Hex���ݣ��Է���Flash��д
		 float    Data_Float;
  }PressureAlarm;                       //Ԥ�����ܣ���δ����
	union 
	{
     u8       Data_Hex[4];              //�¶ȱ�����ֵ���洢������ΪFloat���Ͷ�Ӧ��Hex���ݣ��Է���Flash��д
		 float    Data_Float;
  }TemperatureAlarm;                    //Ԥ�����ܣ���δ����
	u8          CollectStartTime_Byte[2]; //���ݲɼ���ʼʱ�䣬ʹ������洢���Է���Flash��д
  u8          CollectPeriod_Byte[2];    //���ݲɼ������ʹ������洢���Է���Flash��д
	u8          CollectNum_Byte[2];       //�ɼ�������ʹ������洢���Է���Flash��д
	u8          UploadCycle_Byte[2];      //�����ϴ����ڣ�ʹ������洢���Է���Flash��д
  u8          RetryNumSet;              //�����ش�����
	
};

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
//struct LiquidSet       //���޸�
struct DeviceSet       //���޸�
{

	u16 CollectStartTime;         //���ݲɼ���ʼʱ��
	u16 CollectPeriod;            //���ݲɼ�������������������ݲɼ���ʱ��������λ����
	u16 CollectNum;               //�ɼ���������һ���ϴ�����������
	u16 UploadCycle;              //�����ϱ����ڣ����豸���������ڣ���λ����
	u8  RetryNum;                 //�����ش�����
	u8  BatteryCapacity;          //���������⣬����xx%����Ϊ�ٷֺ�ǰ��Ĳ��֡�//��Ȼ�������ò���������Ϊ��������֯���㣬�Է������ò����ṹ����
	u8  Time_Sec;                 //ϵͳʱ�䣬��
	u8  Time_Min;                 //ϵͳʱ�䣬��
	u8  Time_Hour;                //ϵͳʱ�䣬Сʱ
	u8  Time_Mday;                //ϵͳ���ڣ���
	u8  Time_Mon;                 //ϵͳ���ڣ���
	u8  Time_Year;                //ϵͳ���ڣ���
//	u8 Time_Wday;               //���ֶ���ʱ���ԣ���������
//	float AlarmThreshold;         //Һλ������ֵ
//	float MountingHeight;         //̽ͷ��װ�߶�
	
};
//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
typedef enum 
{
	DEF_NR       = 0x1000000A,    //�ش�����
	SYSTERM_DATA = 0x10000050,    //ϵͳ����
	SYSTERM_TIME = 0x10000051,    //ϵͳʱ��
	CLT1_ITRL1   = 0x10000105,    //һʱ���ɼ����
	CLT1_CNT1    = 0x10000106,    //һʱ���ɼ�����
	CLT1_STIME1  = 0x10000104,    //һʱ���ɼ���ʼʱ��
//	RESET_PROBER = 0x10000061,    //��λҺλ̽ͷ
	UPLOAD_CYCLE = 0x10000062,    //�����ϱ�����
	DEVICE_STATE = 0x60000100,    //�豸״ָ̬ʾ���ϵ磩
	DEVICE_QTY   = 0x60000020,    //��ص���
  DEVICE_WAKEUP= 0x60000200,    //�豸״ָ̬ʾ�����ѣ�
	FRAM_STATE   = 0x60000300     //���ݽ���״ָ̬ʾ
		
} CommandType;                  //�豸ͨ�Ź�����ʹ�õ���OID���ϣ�����ҵ���ϱ�OID��

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct TagStruct                         //���ڴ洢������������
{
   CommandType   OID_Command;            //����OID����
   u16           Width;                  //Value��ĳ���
   u8            Value[32];              //����ֵ����WidthΪ0ʱ��Value����Բ����ڣ���ǰӦ������Value�����ռ��32�ֽ�
};

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct SpecialTagStruct                  //���ڴ洢ҵ���ϱ�����
{
	 u32           OID_Data;               //����OID����
   u16           Width;                  //Value��ĳ���
	 union                                 //���������壬���ں������ض��ֽڲ���
	 {
      u8         Value[4];               //ieee754��ʽ��ʾ��Һλ����
		  float      Data_F;                 //��������ʽ��ʾ��Һλ����
   }PerceptionData;                      //�ɼ����Ĵ���������
	 u16           CollectTime;            //���ݲɼ�ʱ��
	
};

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct CommandFrame                      //��ѯ��������֡��ʽ
{
	u8           Preamble;                 //֡ǰ����
	u8           Version;                  //Э��汾��
	u16          Length;                   //��Ч���ݳ��ȣ�����Length�ֶε�CRC�ֶΰ�����ʵ�����ݳ��ȣ���λΪByte��������Length�ֶκ�CRC�ֶΣ�OID_List������ʵ��ʹ�õ����ݿռ�Ϊ׼��
	u8           DeviceID[6];              //�豸ID��
	u8           RouteFlag;                //·�ɱ�־
	u16          NodeAddr;                 //·�ɽڵ��ַ
	u16          PDU_Type;                 //����ָʾ
	u8           Seq;                      //������ţ�1~255��
	CommandType  OID_List[MAX];            //OID���У�һ֡���ݿ��԰������OID����಻����20��
	u16          CrcCode;                  //16λCRCУ���룬���㷶ΧΪ��Preamble��CrcCode��ȫ�����ݣ�����Preamble��CrcCode��CrcCode��ֵΪ0xFFFF��
	u8           OID_Count;                //OID���м�����
};

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct DataFrame                             //�·����ò�����֡��ʽ
{
	u8               Preamble;                 //֡ǰ����
	u8               Version;                  //Э��汾��
	u16              Length;                   //��Ч���ݳ��ȣ�����Length�ֶε�CRC�ֶΰ�����ʵ�����ݳ��ȣ���λΪByte��������Length�ֶκ�CRC�ֶΣ�OID_List������ʵ��ʹ�õ����ݿռ�Ϊ׼��
	u8               DeviceID[6];              //�豸ID��
	u8               RouteFlag;                //·�ɱ�־
	u16              NodeAddr;                 //·�ɽڵ��ַ
	u16              PDU_Type;                 //����ָʾ
	u8               Seq;                      //������ţ�1~255��
	struct TagStruct TagList[MAX];             //Tag���У�һ֡���ݿ��԰������Tag����಻����20��,����һ֡�����ܳ��ȣ�����9�ֽ�֡ͷ�����ܳ���256
	u16              CrcCode;                  //16λCRCУ���룬���㷶ΧΪ��Preamble��CrcCode��ȫ�����ݣ�����Preamble��CrcCode��CrcCode��ֵΪ0xFFFF��
	u8               Tag_Count;                //Tag���м�����
	
};

//�ṹ������
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct  SpecialDataFrame                     //ҵ�������ϴ�֡��ʽ
{
	u8                      Preamble;                 //֡ǰ����
	u8                      Version;                  //Э��汾��
	u16                     Length;                   //��Ч���ݳ��ȣ�����Length�ֶε�CRC�ֶΰ�����ʵ�����ݳ��ȣ���λΪByte��������Length�ֶκ�CRC�ֶΣ�OID_List������ʵ��ʹ�õ����ݿռ�Ϊ׼��
	u8                      DeviceID[6];              //�豸ID��
	u8                      RouteFlag;                //·�ɱ�־
	u16                     NodeAddr;                 //·�ɽڵ��ַ
	u16                     PDU_Type;                 //����ָʾ
	u8                      Seq;                      //������ţ�1~255��
	struct TagStruct        SysTime;                  //ϵͳʱ�䣬��ʾ��һ�����ݵĲɼ�ʱ��
	struct SpecialTagStruct TagList[MAX];             //Tag���У�һ֡���ݿ��԰������Tag����಻����20��,����һ֡�����ܳ��ȣ�����9�ֽ�֡ͷ�����ܳ���256
	struct TagStruct        BattEnergy;               //���ʣ�����������xx%����Ϊ�ٷֺ�ǰ��Ĳ��֡�
	u16                     CrcCode;                  //16λCRCУ���룬���㷶ΧΪ��Preamble��CrcCode��ȫ�����ݣ�����Preamble��CrcCode��CrcCode��ֵΪ0xFFFF��
	u8                      Tag_Count;                //Tag���м�����
	
};
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if defined ( __CC_ARM ) 

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#pragma pop

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif

//void ConfigData_Init(struct LiquidSet* Para);
////void LSLIQUSET_Handle(char* pLiqudSet, struct LiquidSet* Parameter);
//void LSLIQUSET_Response(struct LiquidSet* Parameter);
////void LSTIMESET_Handle(char* pLiqudSet, struct LiquidSet* Parameter);
//void LSDataUpload_Finish(struct LiquidSet* Parameter);
//void DataUpload_TALK_OVER(struct LiquidSet* Parameter);
//void Float2Hex_Aider( float DataSmooth );
////void LSLIQUID_DataUpload(struct LiquidSet* Para);
//void Section_Request(void);
//void Section_Handle(void);
//void mput_mix(char *str,u8 length);

void DeviceStartupRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress);
//void TrapRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress, struct LiquidData* pLevelData);
void TrapRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress, struct SenserData* pObtainData);
void WakeupResponse(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress);
void GetResponse(struct CommandFrame RequestPara, u8* pSendBuff, u8* pDeviceID, u16 NodeAddress);
#endif

