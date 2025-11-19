#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//===================================================================
//								Common
//===================================================================
int AdamIO_Open(int *pfd);
int AdamIO_Close(int fd);

unsigned int GetModuleName(int fd, char *o_szName);
unsigned int GetFirmwareVer(int fd, char *o_szVer);
unsigned int SetWDTTimeout(int fd, int timeout);
unsigned int GetWDTTimeout(int fd, int *timeout);

//===================================================================
//								DI/DO
//===================================================================
unsigned int DO_SetValue(int fd, int i_iChannel, unsigned char i_bValue);
unsigned int DO_SetValues(int fd, int i_iDOTotal, unsigned int i_dwDO);
unsigned int DIO_GetValues(int fd, int i_iDITotal, int i_iDOTotal, unsigned int *o_dwDI, unsigned int *o_dwDO);
unsigned int GetIOConfigs(int fd, int totalCh, unsigned char *o_byConfig);
void ParseDOConfig(unsigned char i_byConfig, unsigned char *o_byMode);
void ParseDIConfig(unsigned char i_byConfig, unsigned char *o_byMode, unsigned char *o_bRecordLastCount, unsigned char *o_bDigitalFilter, unsigned char *o_bInvert);
unsigned int SetIOConfigs(int fd, int totalCh, unsigned char *i_byConfig);
unsigned int GetDOConfig(int fd, int i_iChannel, unsigned char *o_byConfig);
unsigned int SetDOConfig(int fd, int i_iChannel, unsigned char i_byConfig);
unsigned int GetDIConfig(int fd, int i_iChannel, unsigned char *o_byConfig);
unsigned int SetDIConfig(int fd, int i_iChannel, unsigned char i_byConfig);
unsigned int DI_GetDiFilterMiniSignalWidth(int fd, int i_iChannel, unsigned int *o_lHigh, unsigned int *o_lLow);
unsigned int DI_SetDiFilterMiniSignalWidth(int fd, int i_iChannel, unsigned int i_lHigh, unsigned int i_lLow);
unsigned int DO_GetPulseOutputWidthAndDelayTime(int fd, int i_iChannel, unsigned int *o_lPulseHighWidth, unsigned int *o_lPulseLowWidth, unsigned int *o_lHighToLowDelay, unsigned int *o_lLowToHighDelay);
unsigned int DO_SetPulseOutputWidthAndDelayTime(int fd, int i_iChannel, unsigned int i_lPulseHighWidth, unsigned int i_lPulseLowWidth, unsigned int i_lHighToLowDelay, unsigned int i_lLowToHighDelay);
unsigned int DO_GetPulseOutputCount(int fd, int i_iChannel, unsigned char *o_bContinue, unsigned int *o_lPulseCount);
unsigned int DO_SetPulseOutputCount(int fd, int i_iChannel, unsigned char i_bContinue, unsigned int i_lPulseCount);
unsigned int CNT_GetStatus(int fd, int i_iChannel, unsigned char *o_bCounting);
unsigned int CNT_SetStatus(int fd, int i_iChannel, unsigned char i_bCounting);
unsigned int CNT_GetValue(int fd, int i_iChannel, unsigned int *o_lValue);
unsigned int CNT_Clear(int fd, int i_iChannel);
unsigned int ALM_GetLatchStatus(int fd, int i_iChannel, unsigned char *o_bLatchStatus);
unsigned int ALM_SetLatchClear(int fd, int i_iChannel);

//===================================================================
//								AI/AO
//===================================================================
unsigned int AI_GetAutoFilterEnabled(int fd, unsigned char *o_byFilterEnabledMask, int *o_iPercentIndex);
unsigned int AI_SetAutoFilterEnabled(int fd, unsigned char i_byFilterEnabledMask, int i_iPercentIndex);
unsigned int AI_GetAutoFilterSampleRate(int fd, int *o_iRate);
unsigned int AI_GetIntegrationMode(int fd, unsigned char *o_mode);
unsigned int AI_SetIntegrationMode(int fd, unsigned char i_mode);

unsigned int AI_GetBurnoutDetect(int fd, unsigned char *o_burnout);
unsigned int AI_SetBurnoutDetect(int fd, unsigned char i_burnout);
unsigned int AI_GetBurnoutValue(int fd, unsigned char *o_burnoutValue);
unsigned int AI_SetBurnoutValue(int fd, unsigned char i_burnoutValue);
unsigned int AI_GetInputRange(int fd, int i_iChannel, unsigned char *o_byRange);
unsigned int AI_SetInputRange(int fd, int i_iChannel, unsigned char i_byRange);
unsigned int AI_GetChannelEnabled(int fd, unsigned char *o_byEnabledMask);
unsigned int AI_SetChannelEnabled(int fd, unsigned char i_byEnabledMask);
unsigned int AI_GetFloatValue(int fd, int i_iChannel, float *o_fValue, unsigned char *o_status);
unsigned int AI_GetFloatValues(int fd, int i_iChannelTotal, float *o_fValues, unsigned char *o_status);
//===================================================================
//								Enum
//===================================================================

enum DeviceType
{
	DEV_6717 = 0x6717,
	DEV_6750 = 0x6750,
	DEV_Unknown = 0xFFFF
};


enum ErrorCode
{
	/// <summary>
	/// No error
	/// </summary>
	No_Error				= 0x00000000,
	/// <summary>
	/// ComPort error
	/// </summary>
	ComPort_Error			= 0x40000001,
	/// <summary>
	/// ComPort open fail
	/// </summary>
	ComPort_Open_Fail		= 0x40000002,
	/// <summary>
	/// ComPort send fail
	/// </summary>
	ComPort_Send_Fail		= 0x40000003,
	/// <summary>
	/// ComPort receive fail
	/// </summary>
	ComPort_Recv_Fail		= 0x40000004,
	/// <summary>
	/// Adam protocol response header is invalid
	/// </summary>
	Adam_Invalid_Head		= 0x40080001,
	/// <summary>
	/// Adam protocol response header is invalid
	/// </summary>
	Adam_Invalid_End		= 0x40080002,
	/// <summary>
	/// Adam protocol response length is invalid
	/// </summary>
	Adam_Invalid_Length		= 0x40080003,
	/// <summary>
	/// Adam protocol response data is invalid, which causes the convertion failed
	/// </summary>
	Adam_Invalid_Data		= 0x40080004,
	/// <summary>
	/// Adam protocol response checksum is invalid
	/// </summary>
	Adam_Invalid_Checksum	= 0x40080005,
	/// <summary>
	/// The parameter is invalid
	/// </summary>
	Adam_Invalid_Param		= 0x40080006,
	/// <summary>
	/// The password is invalid
	/// </summary>
	Adam_Invalid_Password	= 0x40080007,
	/// <summary>
	/// Parameter error
	/// </summary>
	API_Parameter_Error		= 0x400B0003,
	/// <summary>
	/// Internal Null detected
	/// </summary>
	Adam_Null_Error			= 0x400A0001
};

enum Adam_DOMode
{
	/// <summary>
	/// Normal DO mode
	/// </summary>
	DOMode_Do				= 0,
	/// <summary>
	/// Pulse output mode
	/// </summary>
	DOMode_PulseOutput		= 1,
	/// <summary>
	/// Low to high delay mode
	/// </summary>
	DOMode_LowToHighDelay	= 2,
	/// <summary>
	/// High to low delay mode
	/// </summary>
	DOMode_HighToLowDelay	= 3,
	/// <summary>
	/// Unknown
	/// </summary>
	DOMode_Unknown			= 16
};

enum Adam_DIMode
{
	/// <summary>
	/// Normal DI mode
	/// </summary>
	DIMode_Di				= 0,
	/// <summary>
	/// Counter mode
	/// </summary>
	DIMode_Counter			= 1,
	/// <summary>
	/// Low to high latch mode
	/// </summary>
	DIMode_LowToHighLatch	= 2,
	/// <summary>
	/// High to low latch mode
	/// </summary>
	DIMode_HighToLowLatch	= 3,
	/// <summary>
	/// Frequency mode
	/// </summary>
	DIMode_Frequency		= 4,
	/// <summary>
	/// Unknown
	/// </summary>
	DIMode_Unknown			= 16
};

enum Adam_DIConfiguration
{
	MODE_MASK				= 0x0F,
	RECORD_LAST_COUNT		= 0x20,
	ENABLE_DIGITAL_FILTER	= 0x40,
	ENABLE_INVERT_MODE		= 0x80
};

enum Adam_ChannelStatus
{
	/// <summary>
	/// Normal
	/// </summary>
	Adam_ChannelStatus_Normal		= 0,
	/// <summary>
	/// Over range
	/// </summary>
	Adam_ChannelStatus_Over			= 1,
	/// <summary>
	/// Under range
	/// </summary>
	Adam_ChannelStatus_Under		= 2,
	/// <summary>
	/// Burn out
	/// </summary>
	Adam_ChannelStatus_Burn			= 3,
	/// <summary>
	/// Disabled
	/// </summary>
	Adam_ChannelStatus_Disable		= 255
};

enum Adam6717_InputRange
{
	/// <summary>
	/// 4~20 mA
	/// </summary>
	Adam6717_InputRange_mA_4To20		= 0x07,
	/// <summary>
	/// +/- 10 V 
	/// </summary>
	Adam6717_InputRange_V_Neg10To10		= 0x08,
	/// <summary>
	///  +/- 5 V
	/// </summary>
	Adam6717_InputRange_V_Neg5To5		= 0x09,
	/// <summary>
	///   +/- 1 V
	/// </summary>
	Adam6717_InputRange_V_Neg1To1		= 0x0A,
	/// <summary>
	/// +/- 500 mV
	/// </summary>
	Adam6717_InputRange_mV_Neg500To500	= 0x0B,
	/// <summary>
	/// +/- 150 mV
	/// </summary>
	Adam6717_InputRange_mV_Neg150To150	= 0x0C,
	/// <summary>
	/// +/- 20 mA
	/// </summary>
	Adam6717_InputRange_mA_Neg20To20	= 0x0D,
	/// <summary>
	/// 0~10 V 
	/// </summary>
	Adam6717_InputRange_V_0To10			= 0x48,
	/// <summary>
	/// 0~5 V
	/// </summary>
	Adam6717_InputRange_V_0To5			= 0x49,
	/// <summary>
	/// 0~1 V
	/// </summary>
	Adam6717_InputRange_V_0To1			= 0x4A,
	/// <summary>
	/// 0~500 mV
	/// </summary>
	Adam6717_InputRange_mV_0To500		= 0x4B,
	/// <summary>
	/// 0~150 mV
	/// </summary>
	Adam6717_InputRange_mV_0To150		= 0x4C,
	/// <summary>
	/// 0~20 mA
	/// </summary>
	Adam6717_InputRange_mA_0To20		= 0x4D,
	/// <summary>
	/// Unknown range
	/// </summary>
	Adam6717_InputRange_Unknown			= 255
};

enum Adam_AIIntegrationMode
{
	MODE_50_60_HZ			= 0x0,
	MODE_USER_DEFINED		= 0x80,
	MODE_HIGH_SPEED			= 0xA0
};