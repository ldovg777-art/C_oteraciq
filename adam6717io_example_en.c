#include "io_example.h"

#define TOTAL_DI 5
#define TOTAL_DO 4
#define TOTAL_AI 8

int main(void){
	int fd;
	int iCnt=0;
	int iDITotal = TOTAL_DI;
	int iDOTotal = TOTAL_DO;
	int iAITotal = TOTAL_AI;
	int OpenRet=0;
	//=====================================
	//	Initialize
	//=====================================
	OpenRet = AdamIO_Open(&fd);
	if(fd<0)
		printf( "failed to open i/o\n" );

	//=====================================
	//	Get module name
	//=====================================	
	printf("\n=========================================\n");
	printf("[Get module name]\n");

	unsigned int ret=0; 
	char resp[64] = {};
	ret= GetModuleName(fd,&resp[0]);
	printf("Module name is %s\n",resp);	

	//=====================================
	//	Get firmware version
	//=====================================		
	printf("\n=========================================\n");
	printf("[Get firmware version]\n");

	memset(resp, 0, sizeof(resp));

	ret= GetFirmwareVer(fd,&resp[0]);
	printf("Firmware version is %s\n",resp);

	
	//=====================================
	//	Get AI mutiple float values
	//=====================================		
	printf("\n=========================================\n");
	printf("[Get AI mutiple float values]\n");
	float fValues[TOTAL_AI] = {};
	unsigned char chstatusAll[TOTAL_AI] = {};
	ret = AI_GetFloatValues(fd,iAITotal,&fValues[0],&chstatusAll[0]);
	if(ret == No_Error)
	{	
		for(iCnt=0;iCnt<(TOTAL_AI);iCnt++)
		{
			printf("AI channel %d value is %f, status is %x\n",iCnt,fValues[iCnt],chstatusAll[iCnt]);
		}
	}
	else
	{
		printf("Fail to AI mutiple float values, error code = %d\n", ret);
	}
	
	//=====================================
	//	Get AI single float value
	//=====================================		
	printf("\n=========================================\n");
	printf("[Get AI single float value]\n");
	int iChannel = 1;
	float fValue = 0;
	unsigned char chstatus = 0;
	ret = AI_GetFloatValue(fd,iChannel,&fValue,&chstatus);
	if(ret == No_Error)
	{
		printf("AI channel %d value is %f, status is %x\n",iChannel,fValue,chstatus);
	}
	else
	{
		printf("Fail to AI single float values, error code = %d\n", ret);
	}		


	//=====================================
	//	Get AI input range
	//=====================================		
	printf("\n=========================================\n");
	printf("[Get AI input range]\n");
	for(iCnt=0;iCnt<TOTAL_AI;iCnt++)
	{	
		unsigned char byRange=0;
		ret = AI_GetInputRange(fd, iCnt, &byRange);
		if(ret == No_Error)
		{	
			printf("AI channel %d range is 0x%x\n",iCnt,byRange);
		}
		else
		{
			printf("Fail to get AI input range, error code = %d\n", ret);		
		}		
	}	
	
	//=====================================
	//	Set AI input range
	//=====================================		
	printf("\n=========================================\n");
	printf("[Set AI input range]\n");
	for(iCnt=0;iCnt<TOTAL_AI;iCnt++)
	{	
		unsigned char byRange=Adam6717_InputRange_V_Neg10To10;
		ret = AI_SetInputRange(fd, iCnt, byRange);
		if(ret == No_Error)
		{	
			printf("AI channel %d range is 0x%x\n",iCnt,byRange);
		}
		else
		{
			printf("Fail to set AI input range, error code = %d\n", ret);		
		}		
	}		

	
	//=====================================
	//	Get AI channel enable/disable mask
	//=====================================		
	printf("\n=========================================\n");
	printf("[Get AI channel enable/disable mask]\n");
	unsigned char byEnabledMask = 0;

	ret = AI_GetChannelEnabled(fd,&byEnabledMask);
	if(ret == No_Error)
	{
		printf("AI channel enable/disable mask is 0x%x\n",byEnabledMask);
	}
	else
	{
		printf("Fail to get AI channel enable/disable mask, error code = %d\n", ret);
	}			
	
	
	//=====================================
	//	Set AI channel enable/disable mask
	//=====================================		
	printf("\n=========================================\n");
	printf("[Set AI channel enable/disable mask]\n");
	byEnabledMask = 0xFF;

	ret = AI_SetChannelEnabled(fd,byEnabledMask);
	if(ret == No_Error)
	{
		printf("AI channel enable/disable mask is 0x%x\n",byEnabledMask);
	}
	else
	{
		printf("Fail to set AI channel enable/disable mask, error code = %d\n", ret);
	}			
	
	//==========================================================
	//	Get AI Auto-filter enable/disable mask and filter rate
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Get AI Auto-filter enable/disable mask and filter rate]\n");
	unsigned char FilterEnabledMask = 0;
	int PercentIndex = 0; //The percentage index. 1 means 10%, ..., 9 means 90%.

	ret = AI_GetAutoFilterEnabled(fd,&FilterEnabledMask, &PercentIndex );
	if(ret == No_Error)
	{
		printf("AI Auto-filter enable/disable mask is 0x%x, filter rate is %d%\n",FilterEnabledMask,PercentIndex*10);
	}
	else
	{
		printf("Fail to get Auto-filter enable/disable mask and filter rate, error code = %d\n", ret);
	}		

	
	//==========================================================
	//	Set AI Auto-filter enable/disable mask and filter rate
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Set AI Auto-filter enable/disable mask and filter rate]\n");
	FilterEnabledMask = 0xFF;
	PercentIndex = 7;	//The percentage index. 1 means 10%, ..., 9 means 90%.
	
	ret = AI_SetAutoFilterEnabled(fd,FilterEnabledMask, PercentIndex );
	if(ret == No_Error)
	{
		printf("AI Auto-filter enable/disable mask is 0x%x, filter rate is %d%\n",FilterEnabledMask,PercentIndex*10);
	}
	else
	{
		printf("Fail to set Auto-filter enable/disable mask and filter rate, error code = %d\n", ret);
	}			
	
	//=====================================
	//	Get Auto-filter sampling rate
	//=====================================		
	printf("\n=========================================\n");
	printf("[Get Auto-filter sampling rate]\n");
	int iRate = 0;

	ret = AI_GetAutoFilterSampleRate(fd,&iRate);
	if(ret == No_Error)
	{
		printf("Auto-filter sampling rate is %d\n",iRate);
	}
	else
	{
		printf("Fail to get Auto-filter sampling rate, error code = %d\n", ret);
	}			
	
	//==========================================================
	//	Get AI burnout detect mode
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Get AI burnout detect mode]\n");
	unsigned char burnout = 0;

	ret = AI_GetBurnoutDetect(fd,&burnout);
	if(ret == No_Error)
	{
		printf("AI burnout detect enable/disable is 0x%x\n",burnout);
	}
	else
	{
		printf("Fail to get AI burnout detect enable/disable, error code = %d\n", ret);
	}		
	
	//==========================================================
	//	Set AI burnout detect mode
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Set AI burnout detect mode]\n");
	burnout = 1;
	
	ret = AI_SetBurnoutDetect(fd,burnout );
	if(ret == No_Error)
	{
		printf("AI burnout detect enable/disable is 0x%x\n",burnout);
	}
	else
	{
		printf("Fail to set AI burnout detect enable/disable, error code = %d\n", ret);
	}	
	


	//==========================================================
	//	Get AI burnout value
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Get AI burnout value]\n");
	unsigned char burnoutValue = 0;

	ret = AI_GetBurnoutValue(fd,&burnoutValue);
	if(ret == No_Error)
	{
		printf("AI burnout value is 0x%x\n",burnoutValue);
	}
	else
	{
		printf("Fail to get AI burnout value, error code = %d\n", ret);
	}		
	
	//==========================================================
	//	Set AI burnout value
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Set AI burnout value]\n");
	burnoutValue = 1;
	
	ret = AI_SetBurnoutValue(fd,burnoutValue );
	if(ret == No_Error)
	{
		printf("AI burnout value is 0x%x\n",burnoutValue);
	}
	else
	{
		printf("Fail to set AI burnout value, error code = %d\n", ret);
	}		

	//==========================================================
	//	Get AI Integration Mode
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Get AI Integration Mode]\n");
	unsigned char integrationMode = 0;

	ret = AI_GetIntegrationMode(fd,&integrationMode);
	if(ret == No_Error)
	{
		printf("AI Integration Mode is 0x%x\n",integrationMode);
	}
	else
	{
		printf("Fail to get AI Integration Mode, error code = %d\n", ret);
	}		
	
	//==========================================================
	//	Set AI Integration Mode
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Set AI Integration Mode]\n");
	integrationMode = MODE_USER_DEFINED;
	
	ret = AI_SetIntegrationMode(fd,integrationMode );
	if(ret == No_Error)
	{
		printf("AI Integration Mode is 0x%x\n",integrationMode);
	}
	else
	{
		printf("Fail to set AI Integration Mode, error code = %d\n", ret);
	}	

	//==========================================================
	//	Get AI Auto Filter Sample Rate
	//==========================================================		
	printf("\n======================================================\n");
	printf("[Get AI Auto FilterSample Rate]\n");
	int sampleRate = 0;

	ret = AI_GetAutoFilterSampleRate(fd,&sampleRate);
	if(ret == No_Error)
	{
		printf("Get AI Auto Filter Sample Rate is %d\n",sampleRate);
	}
	else
	{
		printf("Fail to get AI auto filter sample rate, error code = %d\n", ret);
	}	
	
	//=====================================
	//	Get DI/DO multiple channel values
	//=====================================	
	printf("\n=========================================\n");
	printf("[Get DI/DO multiple channel values]\n");

	unsigned int dwDI=0, dwDO=0;	
	ret = DIO_GetValues(fd, iDITotal, iDOTotal, &dwDI, &dwDO);
	if(ret == No_Error)
	{
		printf("DI value in word is 0x%x\n",dwDI);
		printf("DO value in word is 0x%x\n\n",dwDO);
		for (iCnt = 0; iCnt < iDITotal ; iCnt++)
		{
			if (dwDI & (0x0001 << iCnt) ) 
				printf("DI Channel %d is true.\n", iCnt); //LED light on
			else
				printf("DI Channel %d is false.\n", iCnt); //LED light off
		}
		printf("\n");
		for (iCnt = 0; iCnt < iDOTotal ; iCnt++)
		{
			if (dwDO & (0x0001 << iCnt) ) 
				printf("DO Channel %d is true.\n", iCnt); //LED light on
			else
				printf("DO Channel %d is false.\n", iCnt); //LED light off
		}			
	}
	else
	{
		printf("Fail to get DI/DO multiple values, error code = %d\n", ret);
		
	}	
	
	//=====================================
	//	Set DO multiple channel values
	//=====================================	
	printf("\n=========================================\n");
	printf("[Set DO multiple channel values]\n");

	dwDO = 0x9;
	ret = DO_SetValues(fd, iDOTotal, dwDO);
	if(ret == No_Error)
	{	
		printf("Succeed to set DO value: 0x%x\n", dwDO);
	}
	else
	{
		printf("Fail to set DO value, error code = %d\n", ret);
		
	}
	
	
	//=====================================
	//	Set DO single channel value
	//=====================================		
	printf("\n=========================================\n");
	printf("[Set DO single channel value]\n");

	iChannel = 1;
	unsigned char DOValue=0;
	ret = DO_SetValue(fd,iChannel, DOValue);	
	if(ret == No_Error)
	{	
		printf("Succeed to set channel %d DO value: %x\n",iChannel, DOValue);
	}
	else
	{
		printf("Fail to set DO value, error code = %d\n", ret);
		
	}	
	
	
	//=============================
	//			End
	//=============================
	printf("\n=========================================\n");
	if(fd>0)
		AdamIO_Close(fd);
	return 0;

}
	