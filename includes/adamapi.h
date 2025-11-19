#ifndef _ADAMAPI_H
#define _ADAMAPI_H
 

//-------------------
//#define DBG_PRINT	1
//#define DBG_PRINT_UART	1	// only show uart
//-------------------

#define ONESTOPBIT          0
#define ONE5STOPBITS        1
#define TWOSTOPBITS         2

#define NOPARITY            0
#define ODDPARITY           1
#define EVENPARITY          2
#define MARKPARITY          3
#define SPACEPARITY         4

#define MAX_CMD_SEND_SIZE 64
#define MAX_CMD_RECV_SIZE 64
/*
#define MODE_MASK 				0x0F
#define RECORD_LAST_COUNT 		0x20
#define ENABLE_DIGITAL_FILTER	0x40
#define ENABLE_INVERT_MODE		0x80
*/
#define FWDL_MAX_BINARY_FILE_SIZE_WISE2834 0x4F00	//20*1024-128 =>0x4f00
#define FWDL_MAX_BINARY_FILE_SIZE_ADAM6700 0x1B800	//110K

#define FWDL_FUN_SEND_BINARY_DATA 0
#define FWDL_FUN_SEND_TOTAL_CHKSUM 1

#define FWDL_HEADER_DEVID_OFFSET 0
#define FWDL_HEADER_FUNCTION_OFFSET 1
#define FWDL_HEADER_LENGTH_OFFSET 2

#define FWDL_SEND_BINARY_PKTCNT_OFFSET 3
#define FWDL_SEND_BINARY_DATA_OFFSET 5

#define FWDL_SEND_TOTAL_CHKSUM_VALUE_OFFSET 3

#define FWDL_RESP_DATA_OFFSET 3
#define FWDL_RESP_PKT_CHKSUM_OFFSET 4

#define FWDL_SEND_BINARY_HEADER_SIZE 5
#define FWDL_SEND_BINARY_MAX_DATA_SIZE 64
#define FWDL_SEND_TOTAL_CHKSUM_SIZE 5
#define FWDL_RECV_HEADER_SIZE 4
#define FWDL_PKT_CHKSUM_SIZE 2

#define FWDL_SEND_BINARY_SIZE_TOTAL FWDL_SEND_BINARY_HEADER_SIZE+FWDL_SEND_BINARY_MAX_DATA_SIZE+FWDL_PKT_CHKSUM_SIZE	//71
/////#define FWDL_SEND_BINARY_SIZE_TOTAL 100
#define FWDL_SEND_TOTAL_CHKSUM_TOTAL FWDL_SEND_TOTAL_CHKSUM_SIZE+FWDL_PKT_CHKSUM_SIZE	//7
#define FWDL_RECV_TOTAL 6

#define FWDL_RESP_FAIL 0x00
#define FWDL_RESP_SUCCESS 0x01
#define FWDL_RESP_END_PKT 0xFF

//=========================================================
//                    Ioctl Code   
//=========================================================

#define CMD_GetModuleName                     0x1001
#define CMD_GetFirmwareVer                    0x1002
#define CMD_GetModuleConfig                   0x1003
#define CMD_GetWDTTimeout                     0x1004
#define CMD_DIO_GetValues                     0x1005
#define CMD_DO_SetValues                      0x1006
#define CMD_SetModuleConfig                   0x1007
#define CMD_SetWDTTimeout                     0x1008
#define CMD_DO_SetValue                       0x1009
#define CMD_GetIOConfigs                      0x100a
#define CMD_SetIOConfigs                      0x100b
#define CMD_GetDIConfig                       0x100c
#define CMD_GetDOConfig                       0x100d
#define CMD_SetDIConfig                       0x100e
#define CMD_SetDOConfig                       0x100f
#define CMD_GetDIFilterMiniSignalWidth        0x1010
#define CMD_SetDIFilterMiniSignalWidth        0x1011
#define CMD_GetOutputPulseWidth               0x1012
#define CMD_SetOutputPulseWidth               0x1013
#define CMD_GetPulseOutputCount               0x1014
#define CMD_SetPulseOutputCount               0x1015
#define CMD_GetDODiagnostic                   0x1016
#define CMD_CNTGetStatus                      0x1017
#define CMD_CNTSetStatus                      0x1018
#define CMD_CNTGetValue                       0x1019
#define CMD_CNTClearValue                     0x101a
#define CMD_ALMGetLatchStatus                 0x101b
#define CMD_ALMClearLatchStatus               0x101c
#define CMD_AIGetAutoFilterEnabled            0x101d
#define CMD_AISetAutoFilterEnabled            0x101e
#define CMD_AIGetAutoFilterSampleTimeProgress 0x101f
#define CMD_AIGetAutoFilterSampleTimeResult   0x1020
#define CMD_AISetAutoFilterSampleTimeScan     0x1021
#define CMD_AIGetBurnoutDetectCtrl            0x1022
#define CMD_AISetBurnoutDetectCtrl            0x1023
#define CMD_AIGetBurnoutDetectValue           0x1024
#define CMD_AISetBurnoutDetectValue           0x1025
#define CMD_AIGetInputRange                   0x1026
#define CMD_AISetInputRange                   0x1027
#define CMD_AIGetChannelEnabled               0x1028
#define CMD_AISetChannelEnabled               0x1029
#define CMD_AIGetFloatValue                   0x102a
#define CMD_AIGetFloatValues                  0x102b
#define CMD_AIAutoCalibration                 0x102c
#define CMD_AIZeroCalibration                 0x102d
#define CMD_AISpanCalibration                 0x102e
#define CMD_DirectAsciiCommand                0x102f
#define CMD_FWDL_ChgToDownloadMode            0x1030
#define CMD_FWDL_ReadMode                     0x1031
#define CMD_FWDL_StartDownload                0x1032
#define CMD_FWDL_TransmitPacket               0x1033


//===================================================================
//								Common
//===================================================================
int AdamIO_Open(int *pfd);
int AdamIO_Close(int fd);

unsigned int GetModuleName(int fd, char *o_szName);
unsigned int GetFirmwareVer(int fd, char *o_szVer);
unsigned int GetModuleConfig(int fd, unsigned char *Address, unsigned char *TypeCode, unsigned char *Baudrate, unsigned char *Status);
//unsigned int SetModuleConfig(int fd);
unsigned int SetWDTTimeout(int fd, int timeout);
unsigned int GetWDTTimeout(int fd, int *timeout);
unsigned int DirectAsciiCommand(int fd, char *i_szAsciiCmd, char *o_szRespData, unsigned int *o_respLen);


//===================================================================
//								DI/DO
//===================================================================
unsigned int DO_SetValue(int fd, int i_iChannel, unsigned char i_bValue);
unsigned int DO_SetValues(int fd, int i_iDOTotal, unsigned int i_dwDO);
unsigned int DIO_GetValues(int fd, int i_iDITotal, int i_iDOTotal, unsigned int *o_dwDI, unsigned int *o_dwDO);
unsigned int GetIOConfigs(int fd, int totalCh, unsigned char *o_byConfig);
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
unsigned int DO_GetDiagnostic(int fd, int i_groupNum, unsigned char *o_sStatus);
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
unsigned int AI_GetAutoFilterProgress(int fd, int *o_iPercent);
unsigned int AI_GetAutoFilterSampleRate(int fd, int *o_iRate);
unsigned int AI_SetAutoFilter(int fd);
unsigned int AI_ScanAutoFilterRate(int fd, int *o_iRate);
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
//								Firmware download
//===================================================================
unsigned int FWDL_ChgToDownloadMode(int fd);
unsigned int FWDL_ReadMode(int fd, unsigned char *o_mode);
unsigned int FWDL_DownloadStart(int fd);
unsigned int FWDL_TransmitPacket(int fd, unsigned char * pktSend, unsigned int sendLen, unsigned char * pktRecv, unsigned int *pRecvLen);
unsigned int FWDL_SendBinaryData(int fd, char* filename);

//===================================================================
//								for nodered
//===================================================================
void ParseDIConfig(unsigned char i_byConfig, unsigned char *o_byMode, unsigned char *o_bRecordLastCount, unsigned char *o_bDigitalFilter, unsigned char *o_bInvert);
void ParseDOConfig(unsigned char i_byConfig, unsigned char *o_byMode);
unsigned int SetDIConfigAll(int fd, int totalCh, unsigned char *i_byConfig);
unsigned int SetDOConfigAll(int fd, int totalCh, unsigned char *i_byConfig);
unsigned int DI_GetDiFilterMiniSignalWidthAll(int fd, int totalCh, unsigned int *o_lHigh, unsigned int *o_lLow);
unsigned int DI_SetDiFilterMiniSignalWidthAll(int fd, int totalCh, unsigned int *i_lHigh, unsigned int *i_lLow);
unsigned int DO_GetPulseOutputWidthAndDelayTimeAll(int fd, int totalCh, unsigned int *o_lPulseHighWidth, unsigned int *o_lPulseLowWidth, unsigned int *o_lHighToLowDelay, unsigned int *o_lLowToHighDelay);
unsigned int DO_SetPulseOutputWidthAndDelayTimeAll(int fd, int totalCh, unsigned int *i_lPulseHighWidth, unsigned int *i_lPulseLowWidth, unsigned int *i_lHighToLowDelay, unsigned int *i_lLowToHighDelay);
unsigned int DO_GetPulseOutputCountAll(int fd, int totalCh, unsigned char *o_bContinue, unsigned int *o_lPulseCount);
unsigned int DO_SetPulseOutputCountAll(int fd, int totalCh, unsigned char *i_bContinue, unsigned int *i_lPulseCount);
unsigned int CNT_GetStatusAll(int fd, int totalCh, unsigned char *o_bCounting);
unsigned int CNT_SetStatusAll(int fd, int totalCh, unsigned char *i_bCounting);
unsigned int CNT_GetValueAll(int fd, int totalCh, unsigned int *o_lValue);
unsigned int CNT_ClearAll(int fd, int totalCh, unsigned char *i_bClear);
unsigned int ALM_GetLatchStatusAll(int fd, int totalCh, unsigned char *o_bLatchStatus);
unsigned int ALM_SetLatchClearAll(int fd, int totalCh, unsigned char *i_bClear);
unsigned int AI_GetInputRangeAll(int fd, int totalCh, unsigned char *o_byRange);
unsigned int AI_SetInputRangeAll(int fd, int totalCh, unsigned char *i_byRange);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










//===================================================================
//								Enum
//===================================================================

enum ENUM_PURGEFLAG
{
	TxClear     = 0x0004,	//Clear output buffer
	RxClear 	= 0x0008	//Clear input buffer
	
};


enum DeviceType
{
	DEV_6717    = 0x6717,
	DEV_6750    = 0x6750,
	DEV_6760    = 0x6760,  ///!!!+++ Yaojen+ Feb 2020
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
	API_Parameter_Error						= 0x400B0003,
	/// <summary>
	/// Internal Null detected
	/// </summary>
	Adam_Null_Error							= 0x400A0001,
	/// <summary>
	/// Command returning nack
	/// </summary>	
	FWDL_Exceed_File_Size					= 0x400C0001,
	FWDL_Open_File_Fail						= 0x400C0002,
	FWDL_Change_Boot_Mode_Fail				= 0x400C0003,
	FWDL_Invalid_CRC						= 0x400C0004,
	FWDL_Fail								= 0x400C0005
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


enum Adam4000_ChannelStatus
{
	/// <summary>
	/// Normal
	/// </summary>
	Adam4000_ChannelStatus_Normal		= 0,
	/// <summary>
	/// Over range
	/// </summary>
	Adam4000_ChannelStatus_Over			= 1,
	/// <summary>
	/// Under range
	/// </summary>
	Adam4000_ChannelStatus_Under		= 2,
	/// <summary>
	/// Burn out
	/// </summary>
	Adam4000_ChannelStatus_Burn			= 3,
	/// <summary>
	/// Disabled
	/// </summary>
	Adam4000_ChannelStatus_Disable		= 255
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

typedef struct _DIO_CHANNEL_CFG
{
    unsigned char mode               : 4;
    unsigned char reserved           : 1;	
    unsigned char en_record_last_cnt : 1;	
	  unsigned char en_digit_filter    : 1;
	  unsigned char en_invert          : 1;	
} DIO_CHANNEL_CFG;


#endif // _ADAMAPI_H