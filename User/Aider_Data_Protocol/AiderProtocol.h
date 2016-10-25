#ifndef _AIDERPROTOCOL_H_
#define _AIDERPROTOCOL_H_

#include "stm32f10x.h"
#include "API-Platform.h"
#include "stdio.h" 
#define   MAX   10              //定义一帧数据最多包含的OID数量    
 
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
//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
union Hfloat
{
   u8       Data_Hex[4];           
	 float    Data_Float;
};	                        //便于做数据格式转换

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct SenserData 
{

	u16    CollectTime[MAX];           //每组数据对应的采集时间，以每天零点为参考，单位分钟
	float  PressureData[MAX];          //采集到的压力数据
	float  TemperatureData[MAX];       //采集到的温度数据	      
	u8     DataCount;                  //采集到的数据总数,温度压力数据成对出现

};

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:便于Flash数据存储和交互
// -----------------------------------------------------------------------------
struct Config_RegPara 
{
	union 
	{
     u8       Data_Hex[4];              //压力报警阈值，存储的数据为Float类型对应的Hex数据，以方便Flash读写
		 float    Data_Float;
  }PressureAlarm;                       //预留功能，暂未开放
	union 
	{
     u8       Data_Hex[4];              //温度报警阈值，存储的数据为Float类型对应的Hex数据，以方便Flash读写
		 float    Data_Float;
  }TemperatureAlarm;                    //预留功能，暂未开放
	u8          CollectStartTime_Byte[2]; //数据采集起始时间，使用数组存储，以方便Flash读写
  u8          CollectPeriod_Byte[2];    //数据采集间隔，使用数组存储，以方便Flash读写
	u8          CollectNum_Byte[2];       //采集数量，使用数组存储，以方便Flash读写
	u8          UploadCycle_Byte[2];      //数据上传周期，使用数组存储，以方便Flash读写
  u8          RetryNumSet;              //数据重传次数
	
};

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
//struct LiquidSet       //待修改
struct DeviceSet       //待修改
{

	u16 CollectStartTime;         //数据采集起始时间
	u16 CollectPeriod;            //数据采集间隔，即相邻两组数据采集的时间间隔，单位分钟
	u16 CollectNum;               //采集数量，即一次上传多少组数据
	u16 UploadCycle;              //数据上报周期，即设备的休眠周期，单位分钟
	u8  RetryNum;                 //数据重传次数
	u8  BatteryCapacity;          //电池容量检测，例如xx%，仅为百分号前面的部分。//虽然不是配置参数，但是为了数据组织方便，仍放在配置参数结构体中
	u8  Time_Sec;                 //系统时间，秒
	u8  Time_Min;                 //系统时间，分
	u8  Time_Hour;                //系统时间，小时
	u8  Time_Mday;                //系统日期，日
	u8  Time_Mon;                 //系统日期，月
	u8  Time_Year;                //系统日期，年
//	u8 Time_Wday;               //本字段暂时忽略，不做处理
//	float AlarmThreshold;         //液位报警阈值
//	float MountingHeight;         //探头安装高度
	
};
//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
typedef enum 
{
	DEF_NR       = 0x1000000A,    //重传次数
	SYSTERM_DATA = 0x10000050,    //系统日期
	SYSTERM_TIME = 0x10000051,    //系统时间
	CLT1_ITRL1   = 0x10000105,    //一时区采集间隔
	CLT1_CNT1    = 0x10000106,    //一时区采集次数
	CLT1_STIME1  = 0x10000104,    //一时区采集开始时间
//	RESET_PROBER = 0x10000061,    //复位液位探头
	UPLOAD_CYCLE = 0x10000062,    //数据上报周期
	DEVICE_STATE = 0x60000100,    //设备状态指示（上电）
	DEVICE_QTY   = 0x60000020,    //电池电量
  DEVICE_WAKEUP= 0x60000200,    //设备状态指示（唤醒）
	FRAM_STATE   = 0x60000300     //数据接收状态指示
		
} CommandType;                  //设备通信过程中使用到的OID集合（除了业务上报OID）

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct TagStruct                         //用于存储参数命令数据
{
   CommandType   OID_Command;            //命令OID编码
   u16           Width;                  //Value域的长度
   u8            Value[32];              //参数值，当Width为0时，Value域可以不存在，当前应用需求Value域最多占用32字节
};

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct SpecialTagStruct                  //用于存储业务上报数据
{
	 u32           OID_Data;               //数据OID编码
   u16           Width;                  //Value域的长度
	 union                                 //声明共用体，便于后续对特定字节操作
	 {
      u8         Value[4];               //ieee754格式表示的液位数据
		  float      Data_F;                 //浮点数格式表示的液位数据
   }PerceptionData;                      //采集到的传感器数据
	 u16           CollectTime;            //数据采集时间
	
};

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct CommandFrame                      //查询命令数据帧格式
{
	u8           Preamble;                 //帧前导码
	u8           Version;                  //协议版本号
	u16          Length;                   //有效数据长度，即从Length字段到CRC字段包含的实际数据长度（单位为Byte，不包含Length字段和CRC字段，OID_List长度以实际使用的数据空间为准）
	u8           DeviceID[6];              //设备ID号
	u8           RouteFlag;                //路由标志
	u16          NodeAddr;                 //路由节点地址
	u16          PDU_Type;                 //操作指示
	u8           Seq;                      //报文序号（1~255）
	CommandType  OID_List[MAX];            //OID序列，一帧数据可以包含多个OID，最多不超过20个
	u16          CrcCode;                  //16位CRC校验码，计算范围为从Preamble到CrcCode的全部数据（包括Preamble和CrcCode，CrcCode初值为0xFFFF）
	u8           OID_Count;                //OID序列计数器
};

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct DataFrame                             //下发配置参数等帧格式
{
	u8               Preamble;                 //帧前导码
	u8               Version;                  //协议版本号
	u16              Length;                   //有效数据长度，即从Length字段到CRC字段包含的实际数据长度（单位为Byte，不包含Length字段和CRC字段，OID_List长度以实际使用的数据空间为准）
	u8               DeviceID[6];              //设备ID号
	u8               RouteFlag;                //路由标志
	u16              NodeAddr;                 //路由节点地址
	u16              PDU_Type;                 //操作指示
	u8               Seq;                      //报文序号（1~255）
	struct TagStruct TagList[MAX];             //Tag序列，一帧数据可以包含多个Tag，最多不超过20个,而且一帧数据总长度（包括9字节帧头）不能超过256
	u16              CrcCode;                  //16位CRC校验码，计算范围为从Preamble到CrcCode的全部数据（包括Preamble和CrcCode，CrcCode初值为0xFFFF）
	u8               Tag_Count;                //Tag序列计数器
	
};

//结构体声明
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
struct  SpecialDataFrame                     //业务数据上传帧格式
{
	u8                      Preamble;                 //帧前导码
	u8                      Version;                  //协议版本号
	u16                     Length;                   //有效数据长度，即从Length字段到CRC字段包含的实际数据长度（单位为Byte，不包含Length字段和CRC字段，OID_List长度以实际使用的数据空间为准）
	u8                      DeviceID[6];              //设备ID号
	u8                      RouteFlag;                //路由标志
	u16                     NodeAddr;                 //路由节点地址
	u16                     PDU_Type;                 //操作指示
	u8                      Seq;                      //报文序号（1~255）
	struct TagStruct        SysTime;                  //系统时间，表示第一个数据的采集时间
	struct SpecialTagStruct TagList[MAX];             //Tag序列，一帧数据可以包含多个Tag，最多不超过20个,而且一帧数据总长度（包括9字节帧头）不能超过256
	struct TagStruct        BattEnergy;               //电池剩余电量，例如xx%，仅为百分号前面的部分。
	u16                     CrcCode;                  //16位CRC校验码，计算范围为从Preamble到CrcCode的全部数据（包括Preamble和CrcCode，CrcCode初值为0xFFFF）
	u8                      Tag_Count;                //Tag序列计数器
	
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

