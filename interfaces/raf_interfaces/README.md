Copyright (c) Sensrad 2023-2024

# Interfaces for ROS2 Hugin node

This ROS2 package provides implementation of the service message types for interfacing with the Hugin radar node. For additional information, refer to the hugin ROS2 package documentation.




## Implementation details

The following RAF API functions are implemented.


```
RAF_API_RdrCtrl_StartTx(ptArbeApiMailBox ptMailboxHandler,
                                        TStartTxInfo *info);

RAF_API_RdrCtrl_StopTx(ptArbeApiMailBox ptMailboxHandler,
                                       TStopInfo *info);


RAF_API_SysCfg_ConfigurePcOutput(ptArbeApiMailBox ptMailboxHandler,
                                 TConfigurePointCloudsInfo *info);

RAF_API_SysCfg_ConfigPacketFormat(ptArbeApiMailBox ptMailboxHandler,
                                  TConfigPacketFormatInfo *info);

RAF_API_SysCfg_SetTime(ptArbeApiMailBox ptMailboxHandler,
                                       TSetTimeInfo SystemTime);

RAF_API_SysCfg_FileLocation(ptArbeApiMailBox ptMailboxHandler,
                                            TFileLocationInfo *info);

RAF_API_SysCfg_GetMapFile(ptArbeApiMailBox ptMailboxHandler,
                                          TFileLocationInfo *info);

RAF_API_RdrCtrl_StartTx(ptArbeApiMailBox ptMailboxHandler,
                                        TStartTxInfo *info);

RAF_API_RdrCtrl_StopTx(ptArbeApiMailBox ptMailboxHandler,
                                       TStopInfo *info);

RAF_API_RdrCtrl_DspCmd(ptArbeApiMailBox ptMailboxHandler,
                                       TDspCmdInfo *info,
                                       uint32_t unDataLength);

RAF_API_RdrCtrl_SetThresholds(ptArbeApiMailBox ptMailboxHandler,
                                              TSetThresholdsInfo *info);

RAF_API_RdrCtrl_SetActiveSeq(ptArbeApiMailBox ptMailboxHandler,
                                             TSelectActiveSeqInfo *info);

RAF_API_RdrCtrl_SetHistogram(ptArbeApiMailBox ptMailboxHandler,
                                             TSetHistogramInfo *info);

RAF_API_CalUtil_RecordFrameRawData(ptArbeApiMailBox ptMailboxHandler,
                                   TRecordFrameRawDataInfo *info);

RAF_API_Dbg_RficOperation(ptArbeApiMailBox ptMailboxHandler,
                                          TRficOperationInfo *info);

RAF_API_Dbg_GetVaractorTable(ptArbeApiMailBox ptMailboxHandler,
                                             TVarctorTableInfo *info);

RAF_API_Dbg_MemoryOperation(ptArbeApiMailBox ptMailboxHandler,
                                            TMemoryOperationInfo *info);

RAF_API_Dbg_TestOperation(ptArbeApiMailBox ptMailboxHandler,
                                          TDebugOperationInfo *info);

RAF_API_Dbg_ConfigureInjectPC(ptArbeApiMailBox ptMailboxHandler,
                                              TConfigureInjectPCInfo *info);

RAF_API_Dbg_ConfigureRDrecording(ptArbeApiMailBox ptMailboxHandler,
                                 TConfigureRDrecordingInfo *info);

RAF_API_Dbg_ConfigureCalibrationFrame(ptArbeApiMailBox ptMailboxHandler,
                                      TConfigureCalibrationFrameInfo *info);

RAF_API_RdrCtrl_SetProcessingLimits(ptArbeApiMailBox ptMailboxHandler,
                                    TSetProcessingLimitsInfo *info);

RAF_API_RdrCtrl_ConfigureSeq(ptArbeApiMailBox ptMailboxHandler,
                                             TConfigureSeqInfo *info);

RAF_API_RdrCtrl_EnableFramePcOutput(ptArbeApiMailBox ptMailboxHandler,
                                    TConfigureParamInfo *info);

RAF_API_SysCfg_GetNoiseVector(ptArbeApiMailBox ptMailboxHandler,
                                              TNoiseVectorInfo *info);

void RAF_API_SysCfg_CFAR(ptArbeApiMailBox ptMailboxHandler,
                                    TCFARInfo *info);

void RAF_API_SysCfg_NTC(ptArbeApiMailBox ptMailboxHandler,
                                   TNtcInfo *info);
```
