#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <ctype.h>

#include "Raf_Api.h"
#include "Utils.h"
#include "tlv.h"
#include "ver.h"

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////
/////////////	 	API Control Section		//////////////
//////////////////////////////////////////////////////////

TParamStr gaucApiCommandDescTable[LastInternalCmdType - ApiCmdNone] = {
    {"ApiCmdNone"},
    {"SetTime"},
    {"KeepAlive"},
    {"Status"},
    {"Start_Tx"},
    {"Stop_Tx"},
    {"RecordRawData"},
    {"ConfigurePC"},
    {"SetThresholds"},
    {"SelectSeq"},
    {"RficOperation"},
    {"Memory Op"},
    {"Debug Op"},
    {"VaractorTable"},
    {"SetHistogram"},
    {"CfgInjectPC"},
    {"CfgRD_Rec"},
    {"CfgCalibFrame"},
    {"GetNxPool"},
    {"FreeNxPool"},
    {"DataByRef"},
    {"SetDspParams"},
    {"FileLocation"},
    {"GetMapFile"},
    {"SetDspKrnlChain"},
    {"SetProcessingLimits"},
    {"ConfigureSeq"},
    {"EnableFramePcOutput"},
    {"GetNoiseVector"},
    {"ConfigPacketFormat"},
    {"SetFrameControlData"},
    {"SetLogCfg"},
    {"CfarMode"},
    {"NtcMode"},
    {"SetLocalMax"},
    {"TxSetAutocal"},
    {"SetAdt"},
    {"SetDspProfiler"},
    {"RpuCalibReady"},
    {"BootOperationCmd"},
    {"AzCdfSetThrLevels"},
    {"SetRangeHistConfig"},
    {"EnableHistogramOutput"}};

//////////////////////////////////////////////////////////
/////////////	 	API Data Section		//////////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////
/////////		  Set Threshold API			//////////
//////////////////////////////////////////////////////

TParamStr gatSetThresholdOpcode = {"Opcode"};

/* TParamStr */
/*     gatSetThresholdParamStr[LastSetThresholdApiOpcode][API_SET_TH_OPCODE_NUM]
 * = */
/*         {{"Static Threshold 3D", "Static Threshold 4D", "NULL", "NULL"}, */
/*          {"Dynamic Azimuth Threshold", "NULL", "NULL", "NULL"}, */
/*          {"Dynamic Elevation Threshold 4D", "NULL", "NULL", "NULL"}, */
/*          {"Static Threshold 3D", "Static Threshold 4D", */
/*           "Dynamic Azimuth Threshold", "Dynamic Elevation Threshold 4D"}}; */

TParamStr
    gatSetThresholdParamStr[LastSetThresholdApiOpcode][API_SET_TH_OPCODE_NUM] =
        {{{"Static Threshold 3D"}, {"Static Threshold 4D"}, {"NULL"}, {"NULL"}},
         {{"Dynamic Azimuth Threshold"}, {"NULL"}, {"NULL"}, {"NULL"}},
         {{"Dynamic Elevation Threshold 4D"}, {"NULL"}, {"NULL"}, {"NULL"}},
         {{"Static Threshold 3D"},
          {"Static Threshold 4D"},
          {"Dynamic Azimuth Threshold"},
          {"Dynamic Elevation Threshold 4D"}}};

//////////////////////////////////////////////////////
/////////	  SetProcessingLimits API		//////////
//////////////////////////////////////////////////////

/* TParamStr gatSetProcessingLimitsStr[API_PARAMS_NUM] = { */
/*     {"FrameType", "Min Range", "Max Range", "Min Doppler", "Max Doppler"}};
 */

TParamStr gatSetProcessingLimitsStr[API_PARAMS_NUM] = {{"FrameType"},
                                                       {"Min Range"},
                                                       {"Max Range"},
                                                       {"Min Doppler"},
                                                       {"Max Doppler"}};

//////////////////////////////////////////////////////
/////////	  ConfigreSeq API				//////////
//////////////////////////////////////////////////////

TParamStr gatConfigureSeqNumOfFrames = {"NumOfFrames"};

/* TParamStr gatConfigureSeqNumOfFrameTypes[API_PARAMS_NUM] = { */
/*     {"FrameType0", "FrameType1", "FrameType2", "FrameType3", "FrameType4"}};
 */

TParamStr gatConfigureSeqNumOfFrameTypes[API_PARAMS_NUM] = {{"FrameType0"},
                                                            {"FrameType1"},
                                                            {"FrameType2"},
                                                            {"FrameType3"},
                                                            {"FrameType4"}};

//////////////////////////////////////////////////////
/////////	  EnableFaramePcOutput API	//////////
//////////////////////////////////////////////////////

TParamStr gatEnableFramePcOutputType = {"FrameType"};

TParamStr gatEnableFramePcOutputValue = {"Enable"};

//////////////////////////////////////////////////////
/////////	  Select Active Sequence		//////////
//////////////////////////////////////////////////////

TParamStr gatSelActiveSeqStr[API_SET_SEQUENCE_NUM] = {
    {"Sequence Type"},
};

//////////////////////////////////////////////////////
/////////	  		Set Time //////////
//////////////////////////////////////////////////////

TParamStr gatSetTimeStr[API_SET_TIME_PARAMS_NUM] = {
    {"Initiate Time LSB"},
    {"Initiate Time MSB"},
};

//////////////////////////////////////////////////////
/////////	 	Record Raw Data Frame		//////////
//////////////////////////////////////////////////////

TParamStr gatRecRawDataFrameStr[API_REC_RAW_DATA_FRAME_PARAMS_NUM] = {
    {"Sequence Type"},
    {"Range Output"},
    {"Buffer Size"},
    {"Buffer Dest Address"},
};

//////////////////////////////////////////////////////
/////////	 	Conf Point Cloud			//////////
//////////////////////////////////////////////////////

TParamStr gatConfPointCloudStr[API_CONF_POINT_CLOUD_PARAMS_NUM] = {
    {"Buffer Size"},
    {"Param1"},
    {"Param2"},
    {"Buffer Dest Address"},
};

TParamStr gatConfPacketFormatStr[1] = {
    {"Packet Format"},
};

//////////////////////////////////////////////////////
/////////	 		Debug Operation			//////////
//////////////////////////////////////////////////////

TParamStr gatDebugOperStr[API_DEBUG_OPER_PARAMS_NUM] = {
    {"Debug Opcode"}, {"Param1"}, {"Param2"}, {"Param3"}, {"Param4"},
};

TParamStr gatPllVaractorCalibParamsStr[DEBUG_OPER_PARAMS_NUM] = {
    {"Chip Id"},
    {"Varactor #"},
    {"RSVD"},
    {"RSVD"},
};

TParamStr gatAnechoicChamberRtsModeParamsStr[DEBUG_OPER_PARAMS_NUM] = {
    {"On/OFF"},
    {"RSVD"},
    {"RSVD"},
    {"RSVD"},
};

//////////////////////////////////////////////////////
/////////	 		Mem Operation			//////////
//////////////////////////////////////////////////////

TParamStr gatMemOperStr[API_MEM_OPER_PARAMS_NUM] = {
    {"Peripheral Device"}, {"Chip Id"}, {"Opcode"}, {"Address"}, {"Value"},
};

//////////////////////////////////////////////////////
/////////	 		RFIC Operation			//////////
//////////////////////////////////////////////////////

TParamStr gatRficOperStr[API_RFIC_OPER_PARAMS_NUM] = {
    {"RFIC Operation"}, {"Peripheral Device"},
    {"Chip Id"},        {"Param1"},
    {"Param2"},         {"Param3"},
    {"Param4"},         {"Param5"},
    {"Param6"},
};

TParamStr gatConfigureCW[RFIC_OPER_PARAMS_NUM] = {
    {"Active"}, {"Antenna Index"}, {"Frequency"}, {"RSVD"}, {"RSVD"}, {"RSVD"},
};

//////////////////////////////////////////////////////
/////////	  Configure(R,D) Recording		//////////
//////////////////////////////////////////////////////

TParamStr gatConf_RD_RecStr[API_CONF_RD_REC_PARAMS_NUM] = {
    {"FrameType"},
    {"RangeBin"},
    {"DopplerBin"},
    {"Zoom"},
};

//////////////////////////////////////////////////////////
//////////		Internal API Control Section	//////////
//////////////////////////////////////////////////////////

TParamStr
    gaucInternalCommandDescTable[LastUserExtendedCmdType - InternalCmdNone] = {
        {"InternalCmdNone"},
        {"AtcMode"},
        {"DspJob"},
};

//////////////////////////////////////////////////////////
/////////////	Internal API Data Section	//////////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////
/////////	  		Atc Mode //////////
//////////////////////////////////////////////////////

TParamStr gatAtcModeStr[API_CONF_ATC_MODE_PARAMS_NUM] = {
    {"Enable ATC"},
    {"Sensitivity"},
};

//////////////////////////////////////////////////////
/////////	  		Ntc Mode //////////
//////////////////////////////////////////////////////

TParamStr gatNtcModeStr[API_CONF_NTC_MODE_PARAMS_NUM] = {{"Enable coarse NTC"},
                                                         {"Enable fine NTC"},
                                                         {"NTC percentage"},
                                                         {"Send Metadata"}};

//////////////////////////////////////////////////////
/////////	  		Cfar Mode //////////
//////////////////////////////////////////////////////

TParamStr gatCfarModeStr[API_CONF_CFAR_MODE_PARAMS_NUM] = {
    {"Enable coarse CFAR"}, {"Enable fine CFAR"}, {"Send Metadata"}};

//////////////////////////////////////////////////////
/////////	  		Set Histogram			//////////
//////////////////////////////////////////////////////

TParamStr gatSetHistogramStr[API_SET_HISTOGRAM_PARAMS_NUM] = {
    {"HistogranOper"},
    {"FrameTypesMask"},
    {"ThrConfigAddress"},
};

//////////////////////////////////////////////////////
/////////			Set TX Autocal			//////////
//////////////////////////////////////////////////////

TParamStr gatTxSetAutocalStr[API_TX_SET_AUTOCAL] = {
    {"TxChipIndex"},
    {"AutocalStartFreqKHz"},
    {"AutocalSpanEnum"},
    {"IsRunAutoCal"},
};

//////////////////////////////////////////////////////
/////////	  		Get Map File			//////////
//////////////////////////////////////////////////////

TParamStr gatGetFileLocationStr[API_GET_FILE_LOCATION_NUM] = {
    {"FileType"},
    {"SubType"},
    {"FileAddress"},
    {"Size"},
};

//////////////////////////////////////////////////////
/////////	  flash write  (boot cmd)   	//////////
//////////////////////////////////////////////////////

TParamStr gatBootOperationApiStr[API_TX_BOOT_OPERATION_API] = {
    {"boot_cmd"},       {"file_idx_countdown"}, {"crc16"},
    {"total_frgments"}, {"fragment_number"},
};

TParamStr gatBootOperationIpcStr[API_TX_BOOT_OPERATION_IPC] = {
    {"boot_cmd"}, {"file_idx_countdown"}, {"crc16"}, {"file_ptr"}, {"file_len"},
};

TParamStr gatDspSpotRslOnOff[API_TX_BOOT_OPERATION_IPC] = {
    {"Spot On"},
    {"RSL On"},
};

TParamStr gatDspSetRslParams[API_TX_BOOT_OPERATION_IPC] = {
    {"Rsl Back Off"}, {"Rsl param 0"}, {"Rsl param 1"},
    {"Rsl param 2"},  {"Rsl param 3"},
};

void RAF_API_SysCfg_ConfigurePcOutput(ptArbeApiMailBox ptMailboxHandler,
                                      TConfigurePointCloudsInfo *info) {
  TConfigurePointClouds tConfigurePointClouds;
  int _size = sizeof(TConfigurePointClouds);
  memset(&tConfigurePointClouds, 0, _size);
  BuildHeader(ptMailboxHandler, ConfigurePointCloud, _size,
              &tConfigurePointClouds.tHeader);

  tConfigurePointClouds.tConfigurePointCloudsInfo.pucDestinationAddress =
      info->pucDestinationAddress;
  tConfigurePointClouds.tConfigurePointCloudsInfo.unDestinationBufferSize =
      info->unDestinationBufferSize;
  tConfigurePointClouds.tConfigurePointCloudsInfo.unPacketFormat =
      info->unPacketFormat;
  tConfigurePointClouds.tConfigurePointCloudsInfo.unByteAlignment =
      info->unByteAlignment;

  ptMailboxHandler->sendCmd(ConfigurePointCloud,
                            (uint8_t *)&tConfigurePointClouds, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_ConfigPacketFormat(ptArbeApiMailBox ptMailboxHandler,
                                       TConfigPacketFormatInfo *info) {
  TConfigPacketFormat tConfigPacketFormats;
  int _size = sizeof(TConfigPacketFormat);
  memset(&tConfigPacketFormats, 0, _size);
  BuildHeader(ptMailboxHandler, ConfigPacketFormat, _size,
              &tConfigPacketFormats.tHeader);

  tConfigPacketFormats.tConfigPacketFormatInfo.unPacketFormat =
      info->unPacketFormat;
  ptMailboxHandler->sendCmd(ConfigPacketFormat,
                            (uint8_t *)&tConfigPacketFormats, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_SetTime(ptArbeApiMailBox ptMailboxHandler,
                            TSetTimeInfo info) {
  TSetTime tSetTime;

  int _size = sizeof(TSetTime);

  memset(&tSetTime, 0, _size);

  tSetTime.tSetTimeInfo.unInitateTimeLsb = info.unInitateTimeLsb;

  tSetTime.tSetTimeInfo.unInitateTimeMsb = info.unInitateTimeMsb;

  BuildHeader(ptMailboxHandler, SetTime, _size, &tSetTime.tHeader);

  ptMailboxHandler->sendCmd(SetTime, (uint8_t *)&tSetTime, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_FileLocation(ptArbeApiMailBox ptMailboxHandler,
                                 TFileLocationInfo *info) {
  TFileLocation tFileLocation;

  int _size = sizeof(tFileLocation);

  memset(&tFileLocation, 0, _size);

  BuildHeader(ptMailboxHandler, FileLocation, _size, &tFileLocation.tHeader);

  tFileLocation.tFileLocationInfo.eFileType = info->eFileType;
  tFileLocation.tFileLocationInfo.unSubType = info->unSubType;
  tFileLocation.tFileLocationInfo.unFileAddress = info->unFileAddress;
  tFileLocation.tFileLocationInfo.unSize = info->unSize;

  ptMailboxHandler->sendCmd(FileLocation, (uint8_t *)&tFileLocation, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_GetMapFile(ptArbeApiMailBox ptMailboxHandler,
                               TFileLocationInfo *info) {
  TFileLocation tFileLocation;

  int _size = sizeof(tFileLocation);

  memset(&tFileLocation, 0, _size);

  BuildHeader(ptMailboxHandler, GetMapFile, _size, &tFileLocation.tHeader);

  tFileLocation.tFileLocationInfo.eFileType = info->eFileType;
  tFileLocation.tFileLocationInfo.unFileAddress = info->unFileAddress;

  ptMailboxHandler->sendCmd(GetMapFile, (uint8_t *)&tFileLocation, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_CalUtil_RecordFrameRawData(ptArbeApiMailBox ptMailboxHandler,
                                        TRecordFrameRawDataInfo *info) {

  TRecordFrameRawData tRecordFrameRawData;

  int _size = sizeof(TRecordFrameRawData);

  memset(&tRecordFrameRawData, 0, _size);

  BuildHeader(ptMailboxHandler, RecordFrameRawData, _size,
              &tRecordFrameRawData.tHeader);

  tRecordFrameRawData.tRecordFrameRawDataInfo.eSequenceType =
      info->eSequenceType;
  tRecordFrameRawData.tRecordFrameRawDataInfo.pucDestinationAddress =
      info->pucDestinationAddress;
  tRecordFrameRawData.tRecordFrameRawDataInfo.unDestinationBufferSize =
      info->unDestinationBufferSize;

  tRecordFrameRawData.tRecordFrameRawDataInfo.unRangeOutput =
      info->unRangeOutput;

  ptMailboxHandler->sendCmd(RecordFrameRawData, (uint8_t *)&tRecordFrameRawData,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_StartTx(ptArbeApiMailBox ptMailboxHandler,
                             TStartTxInfo *info) {

  (void)info; // Unused

  TStartTx startCmd;

  int _size = sizeof(TStartTx);

  memset(&startCmd, 0, _size);

  BuildHeader(ptMailboxHandler, Start_Tx, _size, &startCmd.tHeader);

  ptMailboxHandler->sendCmd(Start_Tx, (uint8_t *)&startCmd, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_StopTx(ptArbeApiMailBox ptMailboxHandler,
                            TStopInfo *info) {
  TStopTx stopCmd;
  int _size = sizeof(TStopTx);
  memset(&stopCmd, 0, _size);
  BuildHeader(ptMailboxHandler, Stop_Tx, _size, &stopCmd.tHeader);
  stopCmd.tStopTxInfo.unReserved = info->unReserved;

  ptMailboxHandler->sendCmd(Stop_Tx, (uint8_t *)&stopCmd, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_DspCmd(ptArbeApiMailBox ptMailboxHandler,
                            TDspCmdInfo *ptDspData, uint32_t unDataLength) {
  TDspCmd tDspCmd;
  uint32_t unLength = sizeof(TRAF_API_Header) + unDataLength;

  memset(&tDspCmd, 0, sizeof(tDspCmd));
  memcpy((uint8_t *)tDspCmd.tDspCmdInfo.aunParams, ptDspData->aunParams,
         unDataLength);
  BuildHeader(ptMailboxHandler, SetDspParams, unLength, &tDspCmd.tHeader);
  ptMailboxHandler->sendCmd(SetDspParams, (uint8_t *)&tDspCmd, unLength,
                            ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_SetThresholds(ptArbeApiMailBox ptMailboxHandler,
                                   TSetThresholdsInfo *info) {
  TSetThresholds thresholdsCmd;

  int _size = sizeof(TSetThresholds);
  memset(&thresholdsCmd, 0, _size);
  BuildHeader(ptMailboxHandler, SetThresholds, _size, &thresholdsCmd.tHeader);

  thresholdsCmd.tSetThresholdsInfo.opcode = info->opcode;
  thresholdsCmd.tSetThresholdsInfo.aunParams[0] = info->aunParams[0];
  thresholdsCmd.tSetThresholdsInfo.aunParams[1] = info->aunParams[1];
  thresholdsCmd.tSetThresholdsInfo.aunParams[2] = info->aunParams[2];
  thresholdsCmd.tSetThresholdsInfo.aunParams[3] = info->aunParams[3];

  ptMailboxHandler->sendCmd(SetThresholds, (uint8_t *)&thresholdsCmd, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_SetProcessingLimits(ptArbeApiMailBox ptMailboxHandler,
                                         TSetProcessingLimitsInfo *info) {
  TSetProcessingLimits processingLimitsCmd;

  int _size = sizeof(TSetProcessingLimits);
  memset(&processingLimitsCmd, 0, _size);
  BuildHeader(ptMailboxHandler, SetProcessingLimits, _size,
              &processingLimitsCmd.tHeader);

  processingLimitsCmd.tSetProcessingLimitsInfo.unFrameType = info->unFrameType;
  processingLimitsCmd.tSetProcessingLimitsInfo.aunParams[0] =
      info->aunParams[0];
  processingLimitsCmd.tSetProcessingLimitsInfo.aunParams[1] =
      info->aunParams[1];
  processingLimitsCmd.tSetProcessingLimitsInfo.aunParams[2] =
      info->aunParams[2];
  processingLimitsCmd.tSetProcessingLimitsInfo.aunParams[3] =
      info->aunParams[3];

  ptMailboxHandler->sendCmd(SetProcessingLimits,
                            (uint8_t *)&processingLimitsCmd, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_ConfigureSeq(ptArbeApiMailBox ptMailboxHandler,
                                  TConfigureSeqInfo *info) {
  TConfigureSeq configureSeqCmd;

  int _size = sizeof(TConfigureSeq);
  memset(&configureSeqCmd, 0, _size);
  BuildHeader(ptMailboxHandler, ConfigureSeq, _size, &configureSeqCmd.tHeader);

  configureSeqCmd.tConfigureSeqInfo.unNumOfFrames = info->unNumOfFrames;
  memcpy(configureSeqCmd.tConfigureSeqInfo.aucParams, info->aucParams,
         sizeof(uint8_t) * info->unNumOfFrames);

  ptMailboxHandler->sendCmd(ConfigureSeq, (uint8_t *)&configureSeqCmd, _size,
                            ptMailboxHandler->user_data);
}
void RAF_API_RdrCtrl_EnableFramePcOutput(ptArbeApiMailBox ptMailboxHandler,
                                         TConfigureParamInfo *info) {
  TConfigureParam configureParamCmd;

  int _size = sizeof(TConfigureParam);
  memset(&configureParamCmd, 0, _size);
  BuildHeader(ptMailboxHandler, EnableFramePcOutput, _size,
              &configureParamCmd.tHeader);

  configureParamCmd.tConfigureParamInfo.opcode = info->opcode;
  configureParamCmd.tConfigureParamInfo.unParamType = info->unParamType;
  configureParamCmd.tConfigureParamInfo.unParamValue = info->unParamValue;

  ptMailboxHandler->sendCmd(EnableFramePcOutput, (uint8_t *)&configureParamCmd,
                            _size, ptMailboxHandler->user_data);
}
void RAF_API_RdrCtrl_SetActiveSeq(ptArbeApiMailBox ptMailboxHandler,
                                  TSelectActiveSeqInfo *info) {
  TSelectActiveSeq selectActiveSeqCmd;

  int _size = sizeof(selectActiveSeqCmd);

  memset(&selectActiveSeqCmd, 0, _size);

  BuildHeader(ptMailboxHandler, SelectActiveSeq, _size,
              &selectActiveSeqCmd.tHeader);

  selectActiveSeqCmd.tSelectActiveSeqInfo.eSequenceType = info->eSequenceType;

  ptMailboxHandler->sendCmd(SelectActiveSeq, (uint8_t *)&selectActiveSeqCmd,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_SetHistogram(ptArbeApiMailBox ptMailboxHandler,
                                  TSetHistogramInfo *info) {
  TSetHistogram setHistogram;
  int i;
  int _size = sizeof(setHistogram);

  memset(&setHistogram, 0, _size);

  BuildHeader(ptMailboxHandler, SetHistogram, _size, &setHistogram.tHeader);

  for (i = 0; i < HistogramTypeLast; i++) {
    setHistogram.tSetHistogramInfo[i].eHistogramOpr = info[i].eHistogramOpr;
    setHistogram.tSetHistogramInfo[i].unFrameTypesMask =
        info[i].unFrameTypesMask;
    setHistogram.tSetHistogramInfo[i].unThrConfigAddress =
        info[i].unThrConfigAddress;
  }

  ptMailboxHandler->sendCmd(SetHistogram, (uint8_t *)&setHistogram, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_GetStatus(ptArbeApiMailBox ptMailboxHandler,
                               TStatusInfo *info) {
  TStatus getStatusCmd;
  int _size = sizeof(getStatusCmd);

  memset(&getStatusCmd, 0, _size);

  BuildHeader(ptMailboxHandler, Status, _size, &getStatusCmd.tHeader);

  getStatusCmd.tStatusInfo.opcode = info->opcode;

  ptMailboxHandler->sendCmd(Status, (uint8_t *)&getStatusCmd, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_Dbg_GetVaractorTable(ptArbeApiMailBox ptMailboxHandler,
                                  TVarctorTableInfo *info) {

  (void)info; // Unused

  TVarctorTable tVarctorTable;

  int _size = sizeof(tVarctorTable);

  memset(&tVarctorTable, 0, _size);

  BuildHeader(ptMailboxHandler, VaractorTable, _size, &tVarctorTable.tHeader);

  ptMailboxHandler->sendCmd(VaractorTable, (uint8_t *)&tVarctorTable, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_Dbg_MemoryOperation(ptArbeApiMailBox ptMailboxHandler,
                                 TMemoryOperationInfo *info) {
  TMemoryOperation tMemoryOperation;

  int _size = sizeof(TMemoryOperation);

  memset(&tMemoryOperation, 0, _size);

  BuildHeader(ptMailboxHandler, MemoryOperation_Input, _size,
              &tMemoryOperation.tHeader);

  tMemoryOperation.tMemoryOperationInfo.eMemOperPeripheralDevice =
      info->eMemOperPeripheralDevice;

  tMemoryOperation.tMemoryOperationInfo.eMemoryOperation =
      info->eMemoryOperation;

  tMemoryOperation.tMemoryOperationInfo.unChipID = info->unChipID;

  tMemoryOperation.tMemoryOperationInfo.unAddress = info->unAddress;

  tMemoryOperation.tMemoryOperationInfo.unValueOrTotalLength =
      info->unValueOrTotalLength;

  tMemoryOperation.tMemoryOperationInfo.eSpecialOpcode = info->eSpecialOpcode;

  ptMailboxHandler->sendCmd(MemoryOperation_Input, (uint8_t *)&tMemoryOperation,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_Dbg_TestOperation(ptArbeApiMailBox ptMailboxHandler,
                               TDebugOperationInfo *info) {
  TDebugOperation tDebugOperation;

  int _size = sizeof(TDebugOperation);

  memset(&tDebugOperation, 0, _size);

  BuildHeader(ptMailboxHandler, DebugOperation_Input, _size,
              &tDebugOperation.tHeader);

  tDebugOperation.tDebugOperationInfo.eDebugOperation = info->eDebugOperation;

  tDebugOperation.tDebugOperationInfo.aunParams[0] = info->aunParams[0];

  tDebugOperation.tDebugOperationInfo.aunParams[1] = info->aunParams[1];

  tDebugOperation.tDebugOperationInfo.aunParams[2] = info->aunParams[2];

  tDebugOperation.tDebugOperationInfo.aunParams[3] = info->aunParams[3];

  ptMailboxHandler->sendCmd(DebugOperation_Input, (uint8_t *)&tDebugOperation,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_Dbg_RficOperation(ptArbeApiMailBox ptMailboxHandler,
                               TRficOperationInfo *info) {
  TRficOperation tRficOperation;

  int _size = sizeof(TRficOperation);

  memset(&tRficOperation, 0, _size);

  BuildHeader(ptMailboxHandler, RficOperation_Input, _size,
              &tRficOperation.tHeader);

  tRficOperation.tRficOperationInfo.eRficPeripheralDevice =
      info->eRficPeripheralDevice;

  tRficOperation.tRficOperationInfo.eRficOperation = info->eRficOperation;

  tRficOperation.tRficOperationInfo.unChipID = info->unChipID;

  tRficOperation.tRficOperationInfo.aunParams[0] = info->aunParams[0];

  tRficOperation.tRficOperationInfo.aunParams[1] = info->aunParams[1];

  tRficOperation.tRficOperationInfo.aunParams[2] = info->aunParams[2];

  tRficOperation.tRficOperationInfo.aunParams[3] = info->aunParams[3];

  tRficOperation.tRficOperationInfo.aunParams[4] = info->aunParams[4];

  tRficOperation.tRficOperationInfo.aunParams[5] = info->aunParams[5];

  ptMailboxHandler->sendCmd(RficOperation_Input, (uint8_t *)&tRficOperation,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_Dbg_ConfigureInjectPC(ptArbeApiMailBox ptMailboxHandler,
                                   TConfigureInjectPCInfo *info) {
  TConfigureInjectPC tConfigureInjectPC;

  int _size = sizeof(TConfigureInjectPC);

  memset(&tConfigureInjectPC, 0, _size);

  BuildHeader(ptMailboxHandler, ConfigureInjectPC, _size,
              &tConfigureInjectPC.tHeader);

  tConfigureInjectPC.tConfigureInjectPCInfo.unEnable = info->unEnable;

  tConfigureInjectPC.tConfigureInjectPCInfo.aunParams[0] = info->aunParams[0];

  tConfigureInjectPC.tConfigureInjectPCInfo.aunParams[1] = info->aunParams[1];

  ptMailboxHandler->sendCmd(ConfigureInjectPC, (uint8_t *)&tConfigureInjectPC,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_Dbg_ConfigureRDrecording(ptArbeApiMailBox ptMailboxHandler,
                                      TConfigureRDrecordingInfo *info) {
  TConfigureRDrecording tConfigureRDrecording;

  int _size = sizeof(TConfigureRDrecording);

  memset(&tConfigureRDrecording, 0, _size);

  BuildHeader(ptMailboxHandler, ConfigureRDrecording, _size,
              &tConfigureRDrecording.tHeader);

  tConfigureRDrecording.tConfigureRDrecordingInfo.unFrameType =
      info->unFrameType;

  tConfigureRDrecording.tConfigureRDrecordingInfo.unRangeBin = info->unRangeBin;

  tConfigureRDrecording.tConfigureRDrecordingInfo.unDopplerBin =
      info->unDopplerBin;

  tConfigureRDrecording.tConfigureRDrecordingInfo.unZoom = info->unZoom;

  ptMailboxHandler->sendCmd(ConfigureRDrecording,
                            (uint8_t *)&tConfigureRDrecording, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_Dbg_ConfigureCalibrationFrame(
    ptArbeApiMailBox ptMailboxHandler, TConfigureCalibrationFrameInfo *info) {
  TConfigureCalibrationFrame tConfigureCalibrationFrame;

  int _size = sizeof(tConfigureCalibrationFrame);

  memset(&tConfigureCalibrationFrame, 0, _size);

  BuildHeader(ptMailboxHandler, ConfigureCalibrationFrame, _size,
              &tConfigureCalibrationFrame.tHeader);

  tConfigureCalibrationFrame.tConfigureCalibrationFrameInfo.unNumberOfElement =
      info->unNumberOfElement;

  tConfigureCalibrationFrame.tConfigureCalibrationFrameInfo.unChirpsListIndex =
      info->unChirpsListIndex;

  ptMailboxHandler->sendCmd(ConfigureCalibrationFrame,
                            (uint8_t *)&tConfigureCalibrationFrame, _size,
                            ptMailboxHandler->user_data);
}

void BuildHeader(ptArbeApiMailBox ptMailboxHandler, uint32_t unCmdType,
                 uint32_t unMessageSize, TRAF_API_Header *tHeader) {
  tHeader->usPrefix = PACKET_PREFIX;
  tHeader->unLength = unMessageSize;
  tHeader->usType = unCmdType;
  tHeader->unMessageNumber = ptMailboxHandler->unMessageNumber++;
  if (ptMailboxHandler->sysCfg_GetSystemTimeMs != NULL) {

    uint64_t ulSystemTime = ptMailboxHandler->sysCfg_GetSystemTimeMs();
    tHeader->unTimeLsb = (uint32_t)LSB_TIME(ulSystemTime);
    tHeader->unTimeMsb = (uint32_t)MSB_TIME(ulSystemTime);
  } else {
    tHeader->unTimeLsb = 0;
    tHeader->unTimeMsb = 0;
  }
}

//////////////////////////////////////////////////////
/////////	  		Get Noise Vector		//////////
//////////////////////////////////////////////////////

TParamStr gatGetNoiseVectorStr = {"Address"};

void RAF_API_SysCfg_GetNoiseVector(ptArbeApiMailBox ptMailboxHandler,
                                   TNoiseVectorInfo *info) {

  (void)info; // Unused

  TNoiseVector tNoiseVector;

  int _size = sizeof(tNoiseVector);

  memset(&tNoiseVector, 0, _size);

  BuildHeader(ptMailboxHandler, GetNoiseVector, _size, &tNoiseVector.tHeader);

  ptMailboxHandler->sendCmd(GetNoiseVector, (uint8_t *)&tNoiseVector, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_CFAR(ptArbeApiMailBox ptMailboxHandler, TCFARInfo *info) {
  TCFARMode tCFARMode;

  int _size = sizeof(TCFARMode);

  memset(&tCFARMode, 0, _size);

  BuildHeader(ptMailboxHandler, CfarMode, _size, &tCFARMode.tHeader);

  tCFARMode.tCFARInfo.unCFARCoarseOn = info->unCFARCoarseOn;
  tCFARMode.tCFARInfo.unCFARFineOn = info->unCFARFineOn;
  tCFARMode.tCFARInfo.unSendMetadata = info->unSendMetadata;

  ptMailboxHandler->sendCmd(CfarMode, (uint8_t *)&tCFARMode, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_LocalMax(ptArbeApiMailBox ptMailboxHandler,
                             TSetLocalMaxInfo *info) {
  TLocalMax tLocalMax;

  int _size = sizeof(TLocalMax);

  memset(&tLocalMax, 0, _size);

  BuildHeader(ptMailboxHandler, SetLocalMax, _size, &tLocalMax.tHeader);

  tLocalMax.tSetLocalMaxInfo.unSetReset = info->unSetReset;

  ptMailboxHandler->sendCmd(SetLocalMax, (uint8_t *)&tLocalMax, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_SetAdt(ptArbeApiMailBox ptMailboxHandler,
                           TSetAdtInfo *info) {
  TSetAdt tSetAdt;

  int _size = sizeof(TSetAdt);

  memset(&tSetAdt, 0, _size);

  BuildHeader(ptMailboxHandler, SetAdt, _size, &tSetAdt.tHeader);

  tSetAdt.tSetAdtInfo.unAdtSet = info->unAdtSet;

  ptMailboxHandler->sendCmd(SetAdt, (uint8_t *)&tSetAdt, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_DspSetSpotRslOnOff(ptArbeApiMailBox ptMailboxHandler,
                                       TDspSpotRslOnOffInfo *info) {
  TDspSpotRslOnOff tDspSpotRslOnOff;

  int _size = sizeof(TDspSpotRslOnOff);

  memset(&tDspSpotRslOnOff, 0, _size);

  BuildHeader(ptMailboxHandler, DspSpotRslOnOff, _size,
              &tDspSpotRslOnOff.tHeader);

  tDspSpotRslOnOff.tDspSpotRslOnOffInfo.unIsSpotOn = info->unIsSpotOn;
  tDspSpotRslOnOff.tDspSpotRslOnOffInfo.unIsRslOn = info->unIsRslOn;

  ptMailboxHandler->sendCmd(DspSpotRslOnOff, (uint8_t *)&tDspSpotRslOnOff,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_DspSetRslParams(ptArbeApiMailBox ptMailboxHandler,
                                    TDspSetRslParamsInfo *info) {
  TDspSetRslParams tDspSetRslParams;

  int _size = sizeof(TDspSetRslParams);

  memset(&tDspSetRslParams, 0, _size);

  BuildHeader(ptMailboxHandler, DspSetRslParams, _size,
              &tDspSetRslParams.tHeader);

  tDspSetRslParams.tDspSetRslParamsInfo.unRslBackOff = info->unRslBackOff;
  memcpy(tDspSetRslParams.tDspSetRslParamsInfo.pucRslPrams, info->pucRslPrams,
         15 * sizeof(uint32_t));

  ptMailboxHandler->sendCmd(DspSpotRslOnOff, (uint8_t *)&tDspSetRslParams,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_NTC(ptArbeApiMailBox ptMailboxHandler, TNtcInfo *info) {
  TNtcMode tNtcMode;

  int _size = sizeof(TNtcMode);

  memset(&tNtcMode, 0, _size);

  BuildHeader(ptMailboxHandler, NtcMode, _size, &tNtcMode.tHeader);

  tNtcMode.tNtcInfo.unNTCCoarseOn = info->unNTCCoarseOn;
  tNtcMode.tNtcInfo.unNTCFineOn = info->unNTCFineOn;
  tNtcMode.tNtcInfo.unNTCPercentage = info->unNTCPercentage;
  tNtcMode.tNtcInfo.unSendMetadata = info->unSendMetadata;

  ptMailboxHandler->sendCmd(NtcMode, (uint8_t *)&tNtcMode, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_RdrCtrl_SetAutocal(ptArbeApiMailBox ptMailboxHandler,
                                TTxAutoCalInfo *info) {
  TSetTxAutocal tSetAutoCal;

  int _size = sizeof(TSetTxAutocal);

  memset(&tSetAutoCal, 0, _size);

  BuildHeader(ptMailboxHandler, TxSetAutocal, _size, &tSetAutoCal.tHeader);

  tSetAutoCal.tAutocalInfo.unChipIndex = info->unChipIndex;
  tSetAutoCal.tAutocalInfo.unAutocalStartFreqKHz = info->unAutocalStartFreqKHz;
  tSetAutoCal.tAutocalInfo.unAutoCalSpan = info->unAutoCalSpan;
  tSetAutoCal.tAutocalInfo.unIsRunAutocal = info->unIsRunAutocal;

  ptMailboxHandler->sendCmd(TxSetAutocal, (uint8_t *)&tSetAutoCal, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_SetAzCdfThrLvls(ptArbeApiMailBox ptMailboxHandler,
                                    TAzCdfThrInfo *info) {
  TSetAzCdfThr tSetAzCdfThr;

  int _size = sizeof(TSetAzCdfThr);

  memset(&tSetAzCdfThr, 0, _size);

  BuildHeader(ptMailboxHandler, AzCdfSetThrLevels, _size,
              &tSetAzCdfThr.tHeader);

  tSetAzCdfThr.tAzCdfThrLevels.unFrameType = info->unFrameType;
  memcpy(tSetAzCdfThr.tAzCdfThrLevels.pusThrLevels, info->pusThrLevels,
         AZ_CDF_NUM_OF_THR_LEVELS * sizeof(uint16_t));

  ptMailboxHandler->sendCmd(AzCdfSetThrLevels, (uint8_t *)&tSetAzCdfThr, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_SetRangeHistConfig(ptArbeApiMailBox ptMailboxHandler,
                                       TRangeHistConfigInfo *info) {
  TRangeHistCfg tRangeHistConfig;

  int _size = sizeof(TRangeHistCfg);

  memset(&tRangeHistConfig, 0, _size);

  BuildHeader(ptMailboxHandler, SetRangeHistConfig, _size,
              &tRangeHistConfig.tHeader);

  tRangeHistConfig.tRngHistCfgInfo.unThrStartOffset = info->unThrStartOffset;
  tRangeHistConfig.tRngHistCfgInfo.unPowerIncrementLevel =
      info->unPowerIncrementLevel;

  ptMailboxHandler->sendCmd(SetRangeHistConfig, (uint8_t *)&tRangeHistConfig,
                            _size, ptMailboxHandler->user_data);
}

void RAF_API_SysCfg_SetHistogramOutput(ptArbeApiMailBox ptMailboxHandler,
                                       THistOutputInfo *info) {
  THistEnableOutput tHistEnableOutput;

  int _size = sizeof(THistEnableOutput);

  memset(&tHistEnableOutput, 0, _size);

  BuildHeader(ptMailboxHandler, HistEnableOutput, _size,
              &tHistEnableOutput.tHeader);

  tHistEnableOutput.tHistOutputInfo.unAzimuthHistOutputEnable =
      info->unAzimuthHistOutputEnable;
  tHistEnableOutput.tHistOutputInfo.unRangeHistOutputEnable =
      info->unRangeHistOutputEnable;

  ptMailboxHandler->sendCmd(HistEnableOutput, (uint8_t *)&tHistEnableOutput,
                            _size, ptMailboxHandler->user_data);
}
/*###########################################################################*/
#define API_SET_FRAME_CONTROL_NUM_OF_PARAMAS 3

TParamStr gatSetFrameControlData[API_SET_FRAME_CONTROL_NUM_OF_PARAMAS] = {
    {"Frame Type"},
    {"Base Freq"},
    {"Bandwidth"},
};

uint32_t RAF_API_RdrCtrl_SetFrameControlData(ptArbeApiMailBox ptMailboxHandler,
                                             PTCtrlFrameControlInfo ptInfo) {
  TCtrlFrameControlData tFrameControlData;
  uint32_t unEntry;

  int _size = sizeof(tFrameControlData);

  memset(&tFrameControlData, 0, _size);

  BuildHeader(ptMailboxHandler, SetFrameControlData, _size,
              &tFrameControlData.tHeader);

  tFrameControlData.tFrameControlInfo.unFrameType = ptInfo->unFrameType;
  tFrameControlData.tFrameControlInfo.unBaseFreq = ptInfo->unBaseFreq;
  tFrameControlData.tFrameControlInfo.unBandwidth = ptInfo->unBandwidth;

  unEntry = ptMailboxHandler->sendCmd(SetFrameControlData,
                                      (uint8_t *)&tFrameControlData, _size,
                                      ptMailboxHandler->user_data);

  return unEntry;
}

// Leftovers from user extended

void RAF_API_EXT_AtcMode(ptArbeApiMailBox ptMailboxHandler, TAtcInfo *info) {
  TAtcMode tAtcMode;

  int _size = sizeof(TAtcMode);

  memset(&tAtcMode, 0, _size);

  BuildHeader(ptMailboxHandler, AtcMode, _size, &tAtcMode.tHeader);

  tAtcMode.tAtcInfo.unEnableAtc = info->unEnableAtc;
  tAtcMode.tAtcInfo.unsensitivity = info->unsensitivity;

  ptMailboxHandler->sendCmd(AtcMode, (uint8_t *)&tAtcMode, _size,
                            ptMailboxHandler->user_data);
}

/*###########################################################################*/

void RAF_API_BootOperation(ptArbeApiMailBox ptMailboxHandler,
                           st_boot_operation_ipc_data *pstCmdData) {
  st_boot_operation_ipc_msg stCmd;

  int _size = sizeof(st_boot_operation_ipc_msg);

  stCmd.data.boot_cmd = pstCmdData->boot_cmd;
  stCmd.data.file_idx_countdown = pstCmdData->file_idx_countdown;
  stCmd.data.crc16 = pstCmdData->crc16;
  stCmd.data.file_ptr = pstCmdData->file_ptr;
  stCmd.data.file_len = pstCmdData->file_len;

  BuildHeader(ptMailboxHandler, BootOperationCmd, _size, &stCmd.hdr);

  ptMailboxHandler->sendCmd(BootOperationCmd, (uint8_t *)&stCmd, _size,
                            ptMailboxHandler->user_data);
}

void RAF_API_GetRadarInfo(ptArbeApiMailBox ptMailboxHandler, TRadarInfo *info) {
  TRadarInfo_msg stCmd;

  int _size = sizeof(TRadarInfo_msg);

  memset(&stCmd, 0, _size);

  stCmd.tRadarInfo.raf_version_num = info->raf_version_num;
  stCmd.tRadarInfo.raf_commit_id = info->raf_commit_id;
  stCmd.tRadarInfo.user_core_version_num = info->user_core_version_num;
  stCmd.tRadarInfo.user_core_commit_id = info->user_core_commit_id;
  stCmd.tRadarInfo.dsp_commit_id = info->dsp_commit_id;
  stCmd.tRadarInfo.calibration_id = info->calibration_id;
  stCmd.tRadarInfo.antenna_id = info->antenna_id;
  stCmd.tRadarInfo.raf_params_commit_id = info->raf_params_commit_id;

  BuildHeader(ptMailboxHandler, GetRadarInfo, _size, &stCmd.tHeader);

  ptMailboxHandler->sendCmd(GetRadarInfo, (uint8_t *)&stCmd, _size,
                            ptMailboxHandler->user_data);
}
#ifdef __cplusplus
}
#endif
