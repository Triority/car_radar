/*
 * Raf_Api.h
 *
 *  Created on: feb 02, 2021
 *      Author: Dror
 */

#ifndef _INC_RAF_API_CTRL_H_
#define _INC_RAF_API_CTRL_H_

#include "CommonBase.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RAW_METADATA_VERSION                                                   \
  0x1 // should be incremented in every change of RawMetadataHeader or
      // RawMetadataFrame
#define POINT_CLOUD_METADATA_VER 1
#define MAX_FRAME_PER_SEQUENCE 50

#define CONFIGURE_SEQUENCE_MAX_SIZE 32
#define USER_API_CMD_TYPE_START 0xF000
#define USER_API_CMD_TYPE_END 0xFFFE

#define FRAME_CONTROL_IGNORE_VALUE 0xFFFFFFFF
#define NUMBER_OF_RFIC_CHIPS 6
#define NUMBER_OF_MONITORING_TRANSCATION_PER_RFIC 32

#define MAX_NUMBER_OF_CHIRPS 3072
#define MAX_TX_ELEMENTS_SUPPORTED 48
#define MAX_RX_ELEMENTS_SUPPORTED 48
// callibration ver 0 Size
#define NUMBER_OF_BEAM_FORMMING_FACTOR (3)
#define R_NUMER_OF_TX_CONFIGURATION_TABLE_V3 (3)
#define R_TX_TOTAL_NUM_OF_ELEMENTS (132)
#define NUM_OF_ELEMENTS_IN_ROW 2304
#define R_N_RX (48)
#define R_CAL_NOISE_SIZE (128)
#define R_PRE_VIRTUAL_ARRAY_NUM_OF_RANKS (4)
#define R_AZ_FFT_SIZE (128)
#define R_EL_FFT_SIZE (32)
#define R_EL_NUM_OF_RANKS (2)
#define CAL_TX_BUF_SIZE (ANT_TX_MAX * FREQ_TRACKER_NUM_OF_FREQS)
#define CAL_RX_BUF_SIZE                                                        \
  ((ANT_RX_MAX + RXC_ZERO_PAD_VALS) * FREQ_TRACKER_NUM_OF_FREQS)
#define R_CAL_PRE_AZ_BUF_SIZE                                                  \
  (2 * NUM_OF_ELEMENTS_IN_ROW * R_PRE_VIRTUAL_ARRAY_NUM_OF_RANKS)
#define R_CAL_POST_AZ_BUF_SIZE                                                 \
  ((2 * R_FINE_N_ROWS + 2) * R_AZ_FFT_SIZE * R_PRE_VIRTUAL_ARRAY_NUM_OF_RANKS)
#define R_CAL_PRE_EL_BUF_SIZE                                                  \
  ((2 * R_FINE_N_ROWS - 2) * R_AZ_FFT_SIZE * R_EL_NUM_OF_RANKS)
#define R_CAL_POST_EL_BUF_SIZE                                                 \
  (2 * R_EL_FFT_SIZE * R_AZ_FFT_SIZE * R_EL_NUM_OF_RANKS)
#define R_ARBE_NUMBER_OF_RX_CHANNELS (48)
#define R_ARBE_NUMBER_OF_RX_CHIPS (4)
#define R_ARBE_NUMBER_OF_TX_CHIPS (2)
#define T_PHASE_SHIFTER_CONFIGURATION_INFO_SIZE                                \
  (sizeof(uint16_t) * NUMBER_OF_BEAM_FORMMING_FACTOR)
#define T_ELEMENT_PHASE_SIZE                                                   \
  (sizeof(uint16_t) + (T_PHASE_SHIFTER_CONFIGURATION_INFO_SIZE *               \
                       R_NUMER_OF_TX_CONFIGURATION_TABLE_V3))
#define FRAME_TYPES_MAX (24)
#define R_N_TX_ROWS (12)
#define R_N_CTX (4)
#define R_N_TX (R_N_CTX * R_N_TX_ROWS)
#define R_FINE_N_ROWS 27
#define R_N_COLUMNS (R_N_CTX * R_N_RX / 2)
#define CALIBRATION_CALHEADER_SIZE (sizeof(uint32_t) * 7)
#define CALIBRATION_TX_GAINS_SIZE                                              \
  (sizeof(uint32_t) * R_TX_TOTAL_NUM_OF_ELEMENTS)
#define CALIBRATION_TX_DELAYS_SIZE                                             \
  (sizeof(uint32_t) * R_TX_TOTAL_NUM_OF_ELEMENTS)
#define CALIBRATION_RX_GAINS_SIZE (sizeof(uint32_t) * R_N_RX)
#define CALIBRATION_RX_DELAYS_SIZE (sizeof(uint32_t) * R_N_RX)
#define CALIBRATION_NOISE_SIZE (sizeof(uint32_t) * R_CAL_NOISE_SIZE)
#define CALIBRATION_PRE_AZ_SIZE (sizeof(uint32_t) * R_CAL_PRE_AZ_BUF_SIZE)
#define CALIBRATION_POST_AZ_SIZE (sizeof(uint32_t) * R_CAL_POST_AZ_BUF_SIZE)
#define CALIBRATION_PRE_EL_SIZE (sizeof(uint32_t) * R_CAL_PRE_EL_BUF_SIZE)
#define CALIBRATION_POST_EL_SIZE (sizeof(uint32_t) * R_CAL_POST_EL_BUF_SIZE)
#define CALIBRATION_AZIMUT_SL_SIZE (sizeof(uint32_t))
#define CALIBRATION_ELEVATION_SL_SIZE (sizeof(uint32_t))
#define CALIBRATION_STATIC_THRESHOLD_LIMIT_SIZE (sizeof(uint32_t))
#define CALIBRATION_UN_FREQ_TRACKER_BASE_OFFSET_SIZE (sizeof(uint32_t))
#define CALIBRATION_UN_FREQ_TRACKER_SPAN_SIZE (sizeof(uint32_t))
#define CALIBRATION_AF_RX_DELAYS_SIZE                                          \
  (sizeof(float) * R_ARBE_NUMBER_OF_RX_CHANNELS * 3) // X2+X+C
#define CALIBRATION_AF_CALIBRATION_RX_PTAT_SIZE                                \
  (sizeof(float) * R_ARBE_NUMBER_OF_RX_CHIPS) // X2+X+C
#define CALIBRATION_AF_TX_DELAYS_SIZE                                          \
  (sizeof(float) * R_TX_TOTAL_NUM_OF_ELEMENTS * 3) // X2+X+C
#define CALIBRATION_AF_CALIBRATION_TX_PTAT_SIZE                                \
  (sizeof(float) * R_ARBE_NUMBER_OF_TX_CHIPS) // X2+X+C
#define CALIBRATION_AS_PHASE_SHIFTER_CALIBRATION_SIZE                          \
  ((T_ELEMENT_PHASE_SIZE) * (R_TX_TOTAL_NUM_OF_ELEMENTS - R_N_TX))
#define CALIBRATION_AU_LRX_TX_PAIR_MASKING_SIZE                                \
  (sizeof(uint64_t) * R_TX_TOTAL_NUM_OF_ELEMENTS)
#define CALIBRATION_A_STAT_SHIFT_SIZE (sizeof(float) * FRAME_TYPES_MAX)
#define R_RD_REC_BUFFER_SIZE (R_FINE_N_ROWS * R_N_COLUMNS)
#define CALIBRATION_FLASH_DATA_SIZE                                            \
  (CALIBRATION_CALHEADER_SIZE + CALIBRATION_TX_GAINS_SIZE +                    \
   CALIBRATION_TX_DELAYS_SIZE + CALIBRATION_RX_GAINS_SIZE +                    \
   CALIBRATION_RX_DELAYS_SIZE + CALIBRATION_NOISE_SIZE +                       \
   CALIBRATION_PRE_AZ_SIZE + CALIBRATION_POST_AZ_SIZE +                        \
   CALIBRATION_PRE_EL_SIZE + CALIBRATION_POST_EL_SIZE +                        \
   CALIBRATION_AZIMUT_SL_SIZE + CALIBRATION_ELEVATION_SL_SIZE +                \
   CALIBRATION_STATIC_THRESHOLD_LIMIT_SIZE +                                   \
   CALIBRATION_UN_FREQ_TRACKER_BASE_OFFSET_SIZE +                              \
   CALIBRATION_UN_FREQ_TRACKER_SPAN_SIZE + CALIBRATION_AF_RX_DELAYS_SIZE +     \
   CALIBRATION_AF_CALIBRATION_RX_PTAT_SIZE + CALIBRATION_AF_TX_DELAYS_SIZE +   \
   CALIBRATION_AF_CALIBRATION_TX_PTAT_SIZE +                                   \
   CALIBRATION_AS_PHASE_SHIFTER_CALIBRATION_SIZE +                             \
   CALIBRATION_AU_LRX_TX_PAIR_MASKING_SIZE + CALIBRATION_A_STAT_SHIFT_SIZE)
//////////////////////////////////////////////////////////
/////////////	 	API Control	Section		 /////////////
//////////////////////////////////////////////////////////

typedef enum EApiCmdType {
  ApiCmdNone = 0x1000,
  SetTime,
  KeepAlive,
  Status,
  Start_Tx,
  Stop_Tx,
  RecordFrameRawData,
  ConfigurePointCloud,
  SetThresholds,
  SelectActiveSeq,
  RficOperation_Input,
  MemoryOperation_Input,
  DebugOperation_Input,
  VaractorTable,
  SetHistogram,
  ConfigureInjectPC,
  ConfigureRDrecording,
  ConfigureCalibrationFrame,
  GetNxPool,
  FreeNxPool,
  DataByReference,
  SetDspParams,
  FileLocation,
  GetMapFile,
  SetDspKernelsChain,
  SetProcessingLimits,
  ConfigureSeq,
  EnableFramePcOutput,
  GetNoiseVector,
  ConfigPacketFormat,
  SetFrameControlData,
  SetLogCfg,
  CfarMode,
  NtcMode,
  SetLocalMax,
  TxSetAutocal,
  SetAdt,
  SetDspProfiler,
  RpuCalibReady,
  BootOperationCmd, // Include write to flash
  AzCdfSetThrLevels,
  SetRangeHistConfig,
  HistEnableOutput,
  DspSpotRslOnOff,
  DspSetRslParams,
  GetRadarInfo,
  LastInternalCmdType,

  /********************** User Allocated
     ****************************************/
  UserCmd001 = USER_API_CMD_TYPE_START,
  UserCmd002,
  UserCmdLast = USER_API_CMD_TYPE_END,
  /********************** End of User ****************************************/
  // Only 16 LSBits of EApiCmdType are used. Do not add commands here.
  LastApiCmdType // 0xFFFF
} EApiCmdType;

extern TParamStr gaucApiCommandDescTable[];

//////////////////////////////////////////////////////////
/////////////	 	API Data Section		//////////////
//////////////////////////////////////////////////////////

#define API_TYPE_OFFSET 0x1000

typedef enum EParseErrs {

  PErr_OK = 0,
  PErr_CRC,
  PErr_LENGTH,
  PErr_TYPE,
  PErr_CONFLICT, /*In case there was a request that conflicts with the mode we
                    are working at this time : raw was requested while we are
                    running p.Cloude without asking for stop*/
  PErr_OVERFLOW,
  PErr_DATALOSS,
  PErr_DONT_SEND_RESP,
  PErr_FAILED,
  Perr_UNSUPPORTED

} EParseErrs;

typedef enum EOutputType {

  Raw_Output = 1, // bypass
  PointCloud_Output,
  RegOutput,
  MemRdOutput,
  Status_Output = 6,
  Response_Output,
  Slam_Output = 10,
  DspResponse_Output,
  PointCloudAndSlam,
  PointCloud_Output_V1_1,
  Slam_Output_V1_1,
  Range_Output,
  MemoryOperation_Output,
  RficOperation_Output,
  Histogram_Output,
  RangeThreshold_output,
  RdPairRec_Output,
  FileLocation_Output,
  NoiseVector_Output,
  AlgoDetectorThr_Output,
  Extended_PointCloud_Output = 0x18,
  BootOperationReport_Output,
  RadarInfo_Output,
  last_output_type

} EOutputType;

typedef enum EFrameTypeUser {

  FrameType0 = 0,
  FrameType1,
  FrameType2,
  FrameType3,
  FrameType4,
  FrameType5,
  FrameType6,
  FrameType7,
  FrameType8,
  FrameType9,
  FrameType10,
  FrameType11,
  FrameType12,
  FrameType13,
  FrameType14,
  FrameType15,
  FrameType16,
  FrameType17,
  FrameType18,
  FrameType19,
  FrameType20,
  FrameType21,
  FrameType22,
  FrameType23,
  FrameTypeLast

} EFrameTypeUser;

typedef enum EResponseType {

  etResponseTypeUnknown = -1,
  etResponseTypeRaw,
  etResponseTypeStatus,
  etResponseTypePointCloud,
  etResponseTypeRegister,

} EResponseType;

typedef enum ETX_VoltageMeasurement {

  Ptat,
  Pll_Ptat,
  Vcnt,
  Vtune,
  SupplyMonitor

} ETX_VoltageMeasurement;

typedef enum ESupplyMonitorVoltage {

  Vdd_Synth,
  Vdd_0V9Lo,
  Vss1,
  Vss2,
  Vss3,
  Vddd2,
  Vddd,
  Vdda,
  Vdd_1V5bb,
  Vdd1V5Bgr,
  Vdd_HVco,
  Vdd_HPll,
  Vdd_3VPa,
  Vss4,
  Vss5,
  Vss6,

} ESupplyMonitorVoltage;

typedef enum ERadarError {

  OK,
  MEM_OVERFLOW,
  MEM_ALLOC,
  CRC,

} ERadarError;

typedef enum {
  Eboot_cmd_from_flash,
  Eboot_cmd_from_eth,
  Eboot_cmd_save_to_flash,
  Eboot_cmd_save_and_boot,
  Eboot_cmd_erase_fw_from_flash,
  Eboot_cmd_non = 0xFF
} Eboot_cmd;

typedef struct TRangeFields {
  uint32_t fraction : 16;
  uint32_t integral : 10;
  uint32_t range_zoom : 6;

} TRangeFields;

typedef struct TRangeCoef {

  union {
    TRangeFields tFields;
    uint32_t value;
  };

} TRangeCoef;

typedef struct TDopplerFields {

  uint32_t fraction : 16;
  uint32_t integral : 14;
  uint32_t isDynamic : 1;
  uint32_t isFine : 1;

} TDopplerFields;

typedef struct TDopplerCoef {
  union {
    TDopplerFields tFields;
    uint32_t value;
  };

} TDopplerCoef;

typedef struct TAzimuthFields {

  uint32_t fraction : 16;
  uint32_t padding : 6;
  uint32_t fftRange : 10;

} TAzimuthFields;

typedef struct TAzimuthCoef {

  union {
    TAzimuthFields tFields;
    uint32_t value;
  };

} TAzimuthCoef;

typedef struct TElevationFields {

  uint32_t fraction : 16;
  uint32_t padding : 6;
  uint32_t fftRange : 10;

} TElevationFields;

typedef struct TElevationCoef {

  union {

    TElevationFields tFields;
    uint32_t value;
  };

} TElevationCoef;

typedef struct TRawOutput {
  TRAF_API_Header tHeader;
  uint32_t usTotalSamples;
  uint32_t unSampleNumber;
  uint8_t ucIsLastPacket;
  uint8_t ucParams[3];
  uint32_t unCrc;

} TRawOutput;

/* RAW_METADATA_VERSION    above should be incremented if making changes in
 * following structs*/

typedef struct TRawMetadataHeader {

  uint32_t unMagicNum;
  uint32_t usHeaderVersion;
  uint32_t unSampleDataOffset;
  uint32_t unNumberOfSamples;
  uint32_t unSwVersion;
  uint32_t unBitstreamVersion;
  uint32_t unRadarNum;
  uint32_t unCalibrationVersion;
  uint32_t unTimeUtc;
  uint16_t usFramesInSeq;
  uint16_t reserverd;
  uint32_t unMetadataOffset[MAX_FRAME_PER_SEQUENCE];
  uint32_t unMetadataSize[MAX_FRAME_PER_SEQUENCE];
  float afPtatResults[2 * 3 + 4 * 2 + 16]; // NEED TO USE A DEFINE FOR SIZE!!

} TRawMetadataHeader;

typedef struct TRawMetadataFrame {

  uint32_t unFrameTypeNumber;
  uint32_t unChirpBandwidth;
  uint32_t unRepetition;
  uint32_t unNumOfChirps;
  uint32_t unNumOfRxChannels;
  uint32_t unTChirp;
  uint32_t unDelayBetweenChirps;

} TRawMetadataFrame;
/* RAW_METADATA_VERSION    above should be incremented if making changes in
 * structs above*/

typedef struct TPcMetadata {
  TRangeCoef range;
  TDopplerCoef doppler;
  TAzimuthCoef azimuth;
  TElevationCoef elevation;

} TPcMetadata;

typedef struct TPointCloud {
  uint16_t usPrefix;
  uint16_t usType;
  uint16_t usLength;
  uint16_t usTime_lsb;
  uint16_t usTime_msb;
  uint16_t usFrameCounter;
  uint16_t usMessageNumber;
  uint8_t ucLastPacket;
  uint8_t ucTrgtFmtFrmType;
  TPcMetadata tMetadata;
  uint16_t usHeaderCrc;
  uint16_t usPayloadCrc;

} TPointCloud;

typedef struct TPointCloud_V1_1 {

  uint16_t usPrefix;
  uint16_t usType;
  uint32_t unLength;
  uint32_t unTimeLsb;
  uint32_t unTimeMsb;
  uint16_t usFrameCounter;
  uint16_t usMessageNumber;
  uint8_t ucLastPacket;
  uint8_t ucTrgtFmtFrmType;
  uint16_t usCrdCount;
  TPcMetadata tMetadata;
  uint16_t usHeaderCrc;
  uint16_t usPayloadCrc;

} TPointCloud_V1_1;

typedef struct TFrameHandlerInfo {
  uint32_t unFrameNumber;
  uint32_t unNumberOfPoints;
  uint32_t unFrameType;

} TFrameHandlerInfo;

typedef struct THistogramOutput {
  TRAF_API_Header tHeader;
  TFrameHandlerInfo tFrameInfo;
  uint32_t ucIsLastPacket;
  uint32_t unCrc;

} THistogramOutput;

typedef struct TRangeThresholdInfo {
  uint32_t unFrameNumber;
  uint32_t unRangePartition;

} TRangeThresholdInfo;

typedef struct TRangeThreseholdOutput {
  TRAF_API_Header tHeader;
  TRangeThresholdInfo tFrameInfo;
  uint32_t unIsLastPacket;
  uint32_t unCrc;

} TRangeThreseholdOutput;

typedef enum EAlgoOutput {
  AlgoNTC,
  AlgoCFAR,
} EAlgoOutput;

typedef enum EAlgoThresholdType {
  AlgoThresholdRange,
  AlgoThresholdAzimuth
} EAlgoThresholdType;

typedef enum EAlgoFrameType { AlgoFrameCoarse, AlgoFrameFine } EAlgoFrameType;

typedef struct TAlgoDetectorThrInfo {
  uint32_t unFrameNumber;
  EAlgoOutput eAlgoOutput;
  EAlgoThresholdType eThresholdType;
  EAlgoFrameType eFrameType;
} TAlgoDetectorThrInfo;

typedef struct TAlgoDetectorThrOutput {
  TRAF_API_Header tHeader;
  TAlgoDetectorThrInfo tInfo;

} TAlgoDetectorThrOutput;

typedef struct TSlamMetadata {
  uint16_t usFrameNumber;
  uint8_t RFU1[10];
  uint16_t usHostHeading;
  uint16_t usHostHeadingConf;
  uint16_t usHostVelocity;
  uint16_t usHostVelocityConf;
  uint16_t usHost_w;
  uint8_t RFU2[4];

} TSlamMetadata;

typedef struct TSlamHeader {

  uint16_t prefix;
  uint16_t type;
  uint32_t length;
  uint32_t time;
  uint16_t frame_counter;
  uint16_t packet_counter; // in frame
  uint8_t last_packet;
  uint8_t format;
  TSlamMetadata meta_data;
  uint16_t header_crc;
  uint16_t payload_crc;

} TSlamHeader;

typedef struct TSlamHeader_V1_1 {
  uint16_t prefix;
  uint16_t type;
  uint32_t length;
  uint32_t unTimeLsb;
  uint32_t unTimeMsb;
  uint16_t frame_counter;
  uint16_t packet_counter; // in frame
  uint8_t last_packet;
  uint8_t format;
  TSlamMetadata meta_data;
  uint16_t header_crc;
  uint16_t payload_crc;

} TSlamHeader_V1_1;

typedef struct TSlamPos {
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t dx;
  uint8_t dy;
  uint8_t dz;
  uint8_t rfu1;

} TSlamPos;

typedef struct TSlamVelocity {
  uint16_t x_dot;
  uint16_t y_dot;
  uint8_t dx_dot;
  uint8_t dy_dot;

} TSlamVelocity;

typedef struct TSlamBox {
  // Cartesian size
  uint16_t scale_x;
  uint16_t scale_y;
  uint16_t scale_z;
  uint8_t scale_x_conf;
  uint8_t scale_y_conf;
  uint8_t scale_z_conf;
  uint8_t orientation_conf;
  uint16_t orientation;

} TSlamBox;

typedef struct TSlamObj {
  uint16_t ID;
  uint16_t obj_class;
  uint16_t class_conf;
  TSlamPos position;
  TSlamVelocity velocity;
  TSlamBox bounding_box;
  uint8_t age;
  uint8_t status;
  uint8_t ast_frame_update;
  uint8_t side_lobe_conf;
  uint8_t rfu2;
  uint8_t rfu3;

} TSlamObj;

typedef struct TRdPairBinding {
  uint32_t unRangeBin;
  int32_t unDopplerBin;
  uint32_t unZoom;
  uint32_t unFrameTypeNumber;

} TRdPairBinding;

typedef struct TRdCellFrameConfig {
  uint32_t unRangeBin;
  int32_t unDopplerBin;
  uint32_t unZoomFactor;
  uint32_t unFrameTypeNumber;
} TRdCellFrameConfig;

typedef struct TFrameMetadata {
  uint32_t unChirpBandwidth;
  uint32_t unRepetition;
  uint32_t unNumOfChirps;
  uint32_t unNumOfRxChannels;
  uint32_t unTChirp;
  uint32_t unDelayBetweenChirps;
  uint32_t unNumberOfSamplesInChirp;
} TFrameMetadata;

typedef struct {
  uint32_t unCalibrationVersion;
  uint8_t aucTCalibrationFlashData[CALIBRATION_FLASH_DATA_SIZE];
} TCalibrationFileCfg;

typedef struct {
  uint32_t unApiVersion;
  uint32_t unSwVersion;
  TRdCellFrameConfig tRdCellFrameConfig;
  TFrameMetadata tFrameMetadata;
  TCalibrationFileCfg tCalibrationFileCfg;
  uint32_t aunRficMonitoring
      [NUMBER_OF_RFIC_CHIPS]
      [NUMBER_OF_MONITORING_TRANSCATION_PER_RFIC]; //	//TX_0_1-RX-0_3
  uint16_t ausTxOrder[MAX_NUMBER_OF_CHIRPS];       // matlab
  uint16_t ausFreqOrder[MAX_NUMBER_OF_CHIRPS];     // freq
  uint8_t
      ucRdCellTxColumnsOrdering[MAX_TX_ELEMENTS_SUPPORTED];  // TBD currently 0
  uint8_t ucRdCellRxRowsOrdering[MAX_RX_ELEMENTS_SUPPORTED]; // TBD currently 0
  uint32_t unRdCellData
      [R_RD_REC_BUFFER_SIZE]; // 2,592 in 1.2
                              // -->[MAX_RX_ELEMENTS_SUPPORTED][MAX_TX_ELEMENTS_SUPPORTED]=2304;
  uint32_t unCrc;
} TRdCellInfo;

typedef struct {
  TRAF_API_Header tRafHeader;
  TRdCellInfo tRdCellInfo;
} TRdCellInfoOutput;

typedef enum EHistogramType {
  HistogramTypeRange = 0,
  HistogramTypeAzimuth,
  HistogramTypeLast

} EHistogramType;

typedef enum EHistogramOpr {

  HistogramDisable,
  HistogramEnableOneShot,
  HistogramEnablePeriodic

} EHistogramOpr;

typedef struct TResponseInfo {

  uint32_t usStatus; // 0 - Success or error number

} TResponseInfo;

typedef struct TResponseMsg {

  TRAF_API_Header tHeader;
  TResponseInfo tResponseInfo;

} TResponseMsg;

//////////////////////////////////////////////////////
/////////		 	Start TX //////////
//////////////////////////////////////////////////////

typedef struct TStartTxInfo {

  uint32_t unReserved;

} TStartTxInfo, *PTStartTxInfo;

typedef struct TStartTx {

  TRAF_API_Header tHeader;
  TStartTxInfo tStartTxInfo;

} TStartTx, *PTStartTx;

//////////////////////////////////////////////////////
/////////		 	Stop TX //////////
//////////////////////////////////////////////////////

typedef struct TStopInfo {

  uint32_t unReserved;

} TStopInfo, *PTStopInfo;

typedef struct TStopTx {

  TRAF_API_Header tHeader;
  TStopInfo tStopTxInfo;

} TStopTx, *PTStopTx;

//////////////////////////////////////////////////////
/////////		 Select Active Sequence		//////////
//////////////////////////////////////////////////////

#define API_SET_SEQUENCE_NUM 1

typedef enum ESeuqenceType {

  IdleSeq = 0,
  CoarseShortSeq,
  CoarseMidSeq,
  CoarseLongSeq,
  FineShortSeq,
  FineMidSeq,
  FineLongSeq,
  CalibrationSeq,
  LabSeq,
  DelayCalibrationSeq,
  CoarseUltraLongSeq,
  FineUltraLongSeq,
  UserConfigureSeq1,
  UserConfigureSeq2,
  CalibrationFastSeq,
  LastSeq // add new seq before LastSeq
} ESeuqenceType;

typedef struct TSelectActiveSeqInfo {

  ESeuqenceType eSequenceType;

} TSelectActiveSeqInfo, *PTSelectActiveSeqInfo;

typedef struct TSelectActiveSeq {

  TRAF_API_Header tHeader;
  TSelectActiveSeqInfo tSelectActiveSeqInfo;

} TSelectActiveSeq, *PTSelectActiveSeq;

extern TParamStr gatSelActiveSeqStr[];

//////////////////////////////////////////////////////
/////////	  		Set Time //////////
//////////////////////////////////////////////////////

#define API_SET_TIME_PARAMS_NUM 2

typedef struct TSetTimeInfo {

  uint32_t unInitateTimeLsb;
  uint32_t unInitateTimeMsb;

} TSetTimeInfo, *PTSetTimeInfo;

typedef struct TSetTime {

  TRAF_API_Header tHeader;
  TSetTimeInfo tSetTimeInfo;

} TSetTime, *PTSetTime;

extern TParamStr gatSetTimeStr[];

//////////////////////////////////////////////////////
/////////	 	Record Raw Data Frame		//////////
//////////////////////////////////////////////////////

#define API_REC_RAW_DATA_FRAME_PARAMS_NUM 4

typedef struct TRecordFrameRawDataInfo {

  ESeuqenceType eSequenceType;
  uint32_t unRangeOutput;
  uint32_t unDestinationBufferSize;
  uint8_t *pucDestinationAddress;

} TRecordFrameRawDataInfo, *PTRecordFrameRawDataInfo;

typedef struct TRecordFrameRawData {

  TRAF_API_Header tHeader;
  TRecordFrameRawDataInfo tRecordFrameRawDataInfo;

} TRecordFrameRawData, *PTRecordFrameRawData;

extern TParamStr gatRecRawDataFrameStr[];

//////////////////////////////////////////////////////
/////////	 	Conf Point Cloud			//////////
//////////////////////////////////////////////////////

#define API_CONF_POINT_CLOUD_PARAMS_NUM 4

typedef struct TConfigurePointCloudsInfo {
  uint32_t unDestinationBufferSize;
  uint32_t unByteAlignment;
  uint32_t unPacketFormat;
  uint8_t *pucDestinationAddress;

} TConfigurePointCloudsInfo, *PTConfigurePointCloudsInfo;

typedef struct TConfigurePointClouds {

  TRAF_API_Header tHeader;
  TConfigurePointCloudsInfo tConfigurePointCloudsInfo;

} TConfigurePointClouds, *PTConfigurePointClouds;

extern TParamStr gatConfPointCloudStr[];

typedef struct TConfigPacketFormatInfo {
  uint32_t unPacketFormat;
} TConfigPacketFormatInfo;

typedef struct TConfigPacketFormat {
  TRAF_API_Header tHeader;
  TConfigPacketFormatInfo tConfigPacketFormatInfo;
} TConfigPacketFormat;

typedef struct TConfigPCInfo {
  uint32_t unDestAdrLsb;
  uint32_t unDestAdrMsb;
  uint32_t unDestBufSize;
} TConfigPCInfo;

typedef struct TConfigPCInfos {
  TRAF_API_Header tHeader;
  TConfigPCInfo tConfigPCInfo;
} TConfigPCInfos;

extern TParamStr gatConfPacketFormatStr[];

//////////////////////////////////////////////////////
/////////		  Set Threshold API			//////////
//////////////////////////////////////////////////////

#define API_SET_TH_PARAMS_NUM 4
#define API_SET_TH_OPCODE_NUM 4

typedef enum EThresholdOpcodes {

  SetStaticThresholdCoarseAndFine = 0x0001,
  SetDynamicAzimuteThresholdCoarseFine = 0x0002,
  SetDynamicElevationThresholdFine = 0x0003,
  SetStaticAndDynamicThresholds = 0x0004,
  LastSetThresholdApiOpcode

} EThresholdOpcodes;

typedef struct TSetThresholdsInfo {

  EThresholdOpcodes opcode;
  uint32_t aunParams[API_SET_TH_PARAMS_NUM];

} TSetThresholdsInfo, *PTSetThresholdsInfo;

typedef struct TSetThresholds {

  TRAF_API_Header tHeader;
  TSetThresholdsInfo tSetThresholdsInfo;

} TSetThresholds, *PTSetThresholds;

extern TParamStr gatSetThresholdOpcode;

extern TParamStr gatSetThresholdParamStr[LastSetThresholdApiOpcode]
                                        [API_SET_TH_OPCODE_NUM];

//////////////////////////////////////////////////////
/////////	  ConfigreSeq API				//////////
//////////////////////////////////////////////////////

typedef struct {
  uint32_t unNumOfFrames;
  uint8_t aucParams[CONFIGURE_SEQUENCE_MAX_SIZE];
} TConfigureSeqInfo;

typedef struct {
  TRAF_API_Header tHeader;
  TConfigureSeqInfo tConfigureSeqInfo;
} TConfigureSeq, *PTConfigureSeq;

extern TParamStr gatConfigureSeqNumOfFrames;
extern TParamStr gatConfigureSeqNumOfFrameTypes[API_PARAMS_NUM];

//////////////////////////////////////////////////////
/////////	  SetProcessingLimits Sequence	//////////
//////////////////////////////////////////////////////

#define SET_PROCESSING_LIMITS_PARAMS 4

typedef struct {
  uint32_t unFrameType;
  uint32_t aunParams[SET_PROCESSING_LIMITS_PARAMS];
} TSetProcessingLimitsInfo;

typedef struct {
  TRAF_API_Header tHeader;
  TSetProcessingLimitsInfo tSetProcessingLimitsInfo;
} TSetProcessingLimits, *PTSetProcessingLimits;

extern TParamStr gatSetProcessingLimitsFrameType;
extern TParamStr gatSetProcessingLimitsStr[API_PARAMS_NUM];

//////////////////////////////////////////////////////
/////////	  EnableFramePcOutput API	//////////
//////////////////////////////////////////////////////

typedef enum EConfigureParamOpcodes {

  DaeEnableFramePcOutput = 0x0001,

} EConfigureParamOpcodes;

typedef struct {
  EConfigureParamOpcodes opcode;
  uint32_t unParamType;
  uint32_t unParamValue;
} TConfigureParamInfo;

typedef struct {
  TRAF_API_Header tHeader;
  TConfigureParamInfo tConfigureParamInfo;
} TConfigureParam, *PTConfigureParam;

extern TParamStr gatEnableFramePcOutputType;
extern TParamStr gatEnableFramePcOutputValue;

//////////////////////////////////////////////////////
/////////		  	Status API //////////
//////////////////////////////////////////////////////

#define API_STATUS_PARAMS_NUM 4

typedef enum EStatusOpcode {

  SystemTime = 0,
  FwVersion,
  FpgaVersion,
  SequenceNumber,
  RadarErrorNum,
  TaskState,
  ApiInterfaceErrorNum,
  NucStatus,
  VerifyApiCommonVersion,
  GetThreshold // wHAT???

} EStatusOpcode;

typedef struct TStatusInfo {

  EStatusOpcode opcode;
  uint32_t aunParams[API_STATUS_PARAMS_NUM];

} TStatusInfo, *PTStatusInfo;

typedef struct TStatus {

  TRAF_API_Header tHeader;
  TStatusInfo tStatusInfo;

} TStatus, *PTStatus;

//////////////////////////////////////////////////////
/////////		  	Varactor API			//////////
//////////////////////////////////////////////////////

typedef struct TVarctorTableInfo {

  uint32_t unResereved;

} TVarctorTableInfo, *PTVarctorTableInfo;

typedef struct TVarctorTable {

  TRAF_API_Header tHeader;
  TVarctorTableInfo tVarctorTableInfo;

} TVarctorTable, *PTVarctorTable;

//////////////////////////////////////////////////////
/////////	 		Mem Operation			//////////
//////////////////////////////////////////////////////

#define API_MEM_OPER_PARAMS_NUM 5

typedef enum EArbePeripheralDevice {

  Arbe_Tx,
  Arbe_Rx,
  Arbe_Adc,
  Arbe_AntAdc,
  Arbe_Memory

} EArbePeripheralDevice;

typedef enum EMemoryOperation {

  WriteMemory,
  ReadMemory,
  WriteRegister,
  ReadRegister,
  WriteAutoMemory,
  ReadAutoMemory,

} EMemoryOperation;

typedef enum ESpecialOpcode {

  NONE,
  TRACE,
  TRACE_PREVIOUS,
  SMART,
  PC_INJECT,
  RD_PAIRS,
  DEBUG_INFO,
  FILE_INJECT_CALIBRAION,
  FILE_INJECT_PLL,
  FLASH_OPERATION /*FILE_INJECT_RAF_PARAMS*/,

} ESpecialOpcode;

typedef struct TMemoryOperationInfo {

  EArbePeripheralDevice eMemOperPeripheralDevice;
  uint32_t unChipID;
  EMemoryOperation eMemoryOperation;
  uint32_t unAddress;
  uint32_t unValueOrTotalLength;
  ESpecialOpcode eSpecialOpcode;

} TMemoryOperationInfo, *PTMemoryOperationInfo;

typedef struct TMemoryOperation {

  TRAF_API_Header tHeader;
  TMemoryOperationInfo tMemoryOperationInfo;

} TMemoryOperation, *PTMemoryOperation;

extern TParamStr gatMemOperStr[];

//////////////////////////////////////////////////////
/////////		  	RFIC Operation			//////////
//////////////////////////////////////////////////////

#define API_RFIC_OPER_PARAMS_NUM 9
#define RFIC_OPER_PARAMS_NUM 6

typedef enum ERficOperation {

  ConfigureCw = 0x0000,
  ConfigurePhaseShifter = 0x0001,
  TemperatureMeasurement = 0x0002,
  SetChirpsFromSingleElement = 0x0003,
} ERficOperation;

typedef struct TRficOperationInfo {

  ERficOperation eRficOperation;
  EArbePeripheralDevice eRficPeripheralDevice;
  uint32_t unChipID;
  uint32_t aunParams[RFIC_OPER_PARAMS_NUM];

} TRficOperationInfo, *PTRficOperationInfo;

typedef struct TRficOperation {

  TRAF_API_Header tHeader;
  TRficOperationInfo tRficOperationInfo;

} TRficOperation, *PTRficOperation;

extern TParamStr gatRficOperStr[];

extern TParamStr gatConfigureCW[];

//////////////////////////////////////////////////////
/////////	 		Debug Operation			//////////
//////////////////////////////////////////////////////

#define API_DEBUG_OPER_PARAMS_NUM 5
#define DEBUG_OPER_PARAMS_NUM 4

typedef enum EDebugOperation {

  TxInitTest = 0x0000,
  RxInitTest = 0x0001,
  RxCommulativeCurrentTest = 0x0002,
  TxTransmitTest = 0x0003,
  PllCalTest_Freq = 0x0004,
  PllCalTest_Var = 0x0005,
  AnechoicChamberRTS = 0x0006,
  HistogramTest = 0x0007

} EDebugOperation;

typedef struct TDebugOperationInfo {

  EDebugOperation eDebugOperation;
  uint32_t aunParams[DEBUG_OPER_PARAMS_NUM];

} TDebugOperationInfo, *PTDebugOperationInfo;

typedef struct TDebugOperation {
  TRAF_API_Header tHeader;
  TDebugOperationInfo tDebugOperationInfo;

} TDebugOperation, *PTDebugOperation;

extern TParamStr gatDebugOperStr[];

extern TParamStr gatPllVaractorCalibParamsStr[];

extern TParamStr gatAnechoicChamberRtsModeParamsStr[];

//////////////////////////////////////////////////////
/////////  Configure Inject PointCloud Info //////////
//////////////////////////////////////////////////////

#define CONF_INJECT_PC_INFO_PARAMS_NUM 2

typedef struct TConfigureInjectPCInfo {

  uint32_t unEnable;
  uint32_t aunParams[CONF_INJECT_PC_INFO_PARAMS_NUM];

} TConfigureInjectPCInfo, *PTConfigureInjectPCInfo;

typedef struct TConfigureInjectPC {

  TRAF_API_Header tHeader;
  TConfigureInjectPCInfo tConfigureInjectPCInfo;

} TConfigureInjectPC, *PTConfigureInjectPC;

//////////////////////////////////////////////////////
/////////	  Configure (R,D) Recording		//////////
//////////////////////////////////////////////////////

#define API_CONF_RD_REC_PARAMS_NUM 4

typedef struct TConfigureRDrecordingInfo {

  uint32_t unFrameType;
  uint32_t unRangeBin;
  uint32_t unDopplerBin;
  uint32_t unZoom;

} TConfigureRDrecordingInfo, *PTConfigureRDrecordingInfo;

typedef struct TConfigureRDrecording {

  TRAF_API_Header tHeader;
  TConfigureRDrecordingInfo tConfigureRDrecordingInfo;

} TConfigureRDrecording, *PTConfigureRDrecording;

extern TParamStr gatConf_RD_RecStr[];

//////////////////////////////////////////////////////
/////////	    Calibration Frame Info    	//////////
//////////////////////////////////////////////////////

typedef struct TConfigureCalibrationFrameInfo {

  uint32_t unNumberOfElement;
  uint32_t unChirpsListIndex;

} TConfigureCalibrationFrameInfo, *PTConfigureCalibrationFrameInfo;

typedef struct TConfigureCalibrationFrame {

  TRAF_API_Header tHeader;
  TConfigureCalibrationFrameInfo tConfigureCalibrationFrameInfo;

} TConfigureCalibrationFrame, *PTConfigureCalibrationFrame;

//////////////////////////////////////////////////////
/////////	  		File Location Info   	//////////
//////////////////////////////////////////////////////

#define API_GET_FILE_LOCATION_NUM 4

typedef enum EFileType {

  FileTypeMap,
  FileTypeCalibration,
  FileTypePllData,
  FileTypeCfgData,
  FileTypeHwModel,
  FileTypeRafParams,
  FileTypeLast

} EFileType;

typedef struct TFileLocationInfo {

  EFileType eFileType;
  uint32_t unSubType;
  uint32_t unFileAddress;
  uint32_t unSize;

} TFileLocationInfo, *PTFileLocationInfo;

typedef struct TFileLocation {

  TRAF_API_Header tHeader;
  TFileLocationInfo tFileLocationInfo;
  uint32_t unStatus;

} TFileLocation, *PTFileLocation;

extern TParamStr gatGetFileLocationStr[API_GET_FILE_LOCATION_NUM];

//////////////////////////////////////////////////////
/////////	  		Map File Header 	   	//////////
//////////////////////////////////////////////////////

#define MAPFILE_FILELIST_MAX_SIZE 64

typedef struct TMapFileHeader {

  uint32_t unPrefix;
  uint32_t unNumberOfFiles;
  uint32_t unVersion;

} TMapFileHeader, *PTMapFileHeader;

typedef struct TMapFile {

  TMapFileHeader tMapFileHeader;
  TFileLocationInfo tFileList[MAPFILE_FILELIST_MAX_SIZE];

} TMapFile, *PTMapFile;

//////////////////////////////////////////////////////////
//////////		Internal API Control Section	//////////
//////////////////////////////////////////////////////////

typedef enum EUserExtendedCmdType {
  InternalCmdNone = 0x2000,
  AtcMode,
  DspJob,
  LastUserExtendedCmdType
} EUserExtendedCmdType;

extern TParamStr gaucInternalCommandDescTable[];

//////////////////////////////////////////////////////////
/////////////	Internal API Data Section	//////////////
//////////////////////////////////////////////////////////

#define API_USER_EXTENDED_TYPE_OFFSET 0x2000

//////////////////////////////////////////////////////
/////////	  		Atc Mode //////////
//////////////////////////////////////////////////////

#define API_CONF_ATC_MODE_PARAMS_NUM 2

typedef struct TAtcInfo {

  uint32_t unEnableAtc;
  uint32_t unsensitivity;

} TAtcInfo, *PTAtcInfo;

typedef struct TAtcMode {

  TRAF_API_Header tHeader;
  TAtcInfo tAtcInfo;

} TAtcMode, *PTAtcMode;

extern TParamStr gatAtcModeStr[];

//////////////////////////////////////////////////////
/////////	  		Ntc Mode //////////
//////////////////////////////////////////////////////

#define API_CONF_NTC_MODE_PARAMS_NUM 4

typedef struct TNtcInfo {

  uint32_t unNTCCoarseOn;
  uint32_t unNTCFineOn;
  uint32_t unNTCPercentage;
  uint32_t unSendMetadata;

} TNtcInfo, *PTNtcInfo;

typedef struct TNtcMode {

  TRAF_API_Header tHeader;
  TNtcInfo tNtcInfo;

} TNtcMode, *PTNtcMode;

extern TParamStr gatNtcModeStr[];

//////////////////////////////////////////////////////
/////////	  		Cfar Mode //////////
//////////////////////////////////////////////////////

#define API_CONF_CFAR_MODE_PARAMS_NUM 3

typedef struct TCFARInfo {

  uint32_t unCFARCoarseOn;
  uint32_t unCFARFineOn;
  uint32_t unSendMetadata;

} TCFARInfo, *PTCFARInfo;

typedef struct TCFARMode {

  TRAF_API_Header tHeader;
  TCFARInfo tCFARInfo;

} TCFARMode, *PTCFARMode;

extern TParamStr gatCfarModeStr[];

//////////////////////////////////////////////////////
/////////	  		Set Histogram			//////////
//////////////////////////////////////////////////////

#define API_SET_HISTOGRAM_PARAMS_NUM 3

typedef struct TSetHistogramInfo {

  EHistogramOpr eHistogramOpr;
  uint32_t unFrameTypesMask;
  uint32_t unThrConfigAddress;

} TSetHistogramInfo, *PTSetHistogramInfo;

typedef struct TSetHistogram {

  TRAF_API_Header tHeader;
  TSetHistogramInfo tSetHistogramInfo[HistogramTypeLast];

} TSetHistogram, *PTSetHistogram;

extern TParamStr gatSetHistogramStr[];

//////////////////////////////////////////////////////
/////////			Set TX Autocal			//////////
//////////////////////////////////////////////////////
#define API_TX_SET_AUTOCAL 4

typedef struct TTxAutoCalInfo {
  uint32_t unChipIndex;
  uint32_t unAutocalStartFreqKHz;
  uint32_t unAutoCalSpan;
  uint32_t unIsRunAutocal;
} TTxAutoCalInfo, *PTTxAutocalInfo;

typedef struct TSetTxAutocal {
  TRAF_API_Header tHeader;
  TTxAutoCalInfo tAutocalInfo;
} TSetTxAutocal, *PTSetTxAutocal;

extern TParamStr gatTxSetAutocalStr[];
//////////////////////////////////////////////////////
/////////			Set Az Cdf Thr Levels	//////////
//////////////////////////////////////////////////////
#define AZ_CDF_NUM_OF_THR_LEVELS 16

typedef struct TAzCdfThrInfo {
  uint32_t unFrameType;
  uint16_t pusThrLevels[AZ_CDF_NUM_OF_THR_LEVELS];
} TAzCdfThrInfo, *PTAzCdfThrInfo;

typedef struct TSetAzCdfThr {
  TRAF_API_Header tHeader;
  TAzCdfThrInfo tAzCdfThrLevels;
} TSetAzCdfThr, *PTSetAzCdfThr;

//////////////////////////////////////////////////////
/////////			Set Range Hist Config	//////////
//////////////////////////////////////////////////////
typedef struct TRangeHistConfigInfo {
  uint32_t unThrStartOffset;
  uint32_t unPowerIncrementLevel;
} TRangeHistConfigInfo, *PTRangeHistConfigInfo;

typedef struct TRangeHistCfg {
  TRAF_API_Header tHeader;
  TRangeHistConfigInfo tRngHistCfgInfo;
} TRangeHistCfg, *PTRangeHistCfg;

//////////////////////////////////////////////////////
/////////			enable Hist output		//////////
//////////////////////////////////////////////////////

typedef struct THistOutputInfo {
  uint32_t unAzimuthHistOutputEnable;
  uint32_t unRangeHistOutputEnable;
} THistOutputInfo, *PTHistOutputInfo;

typedef struct THistEnableOutput {
  TRAF_API_Header tHeader;
  THistOutputInfo tHistOutputInfo;
} THistEnableOutput, *PTHistEnableOutput;

//////////////////////////////////////////////////////
/////////	  		Get Map File			//////////
//////////////////////////////////////////////////////

typedef struct TGetMapFileInfo {
  uint32_t unFileType;
  uint32_t unSubType;
  uint32_t unFileAddress;
  uint32_t unSize;

} TGetMapFileInfo, *PTGetMapFileInfo;

//////////////////////////////////////////////////////
/////////	  	Memory Data Info			//////////
//////////////////////////////////////////////////////

typedef struct TMemoryDataOpInfo {
  TMemoryOperationInfo tMemoryOperationInfo;
  uint32_t unCurrentDataLength;
  uint32_t unSliceNumber;
  uint32_t unCrc;
  uint8_t pucData[TCP_PACKET_LENGTH];
} TMemoryDataOpInfo, *PTMemoryDataOpInfo;

typedef struct TMemoryDataOp {

  TRAF_API_Header tHeader;
  TMemoryDataOpInfo tMemoryOperation;

} TMemoryDataOp, *PTMemoryDataOp;

typedef struct {
  uint32_t uart_level;
  uint32_t eth_level;
  uint32_t mem_level;
} s_LevelMsk;

typedef struct TLogCoreCfg {
  uint32_t unLogLevel;
  s_LevelMsk tLogMask;
} TLogCoreCfg;

typedef struct TLogCfgInfo {
  TLogCoreCfg tLogCfg;
} TLogCfgInfo, *PLogCfgInfo;

typedef struct TDspProfilerCfg {
  uint32_t unDspProfCfg;
} TDspProfilerCfg;

typedef struct TDspProfilerInfo {
  TDspProfilerCfg tDspProfilerCfg;
} TDspProfilerInfo, *PDspProfilerInfo;

typedef struct TSetLocalMaxInfo {
  uint32_t unSetReset;
} TSetLocalMaxInfo, *PSetLocalMaxInfo;

typedef struct TLocalMax {

  TRAF_API_Header tHeader;
  TSetLocalMaxInfo tSetLocalMaxInfo;

} TLocalMax, *PLocalMax;

typedef struct TSetAdtInfo {
  uint32_t unAdtSet;
} TSetAdtInfo;

typedef struct TSetAdt {
  TRAF_API_Header tHeader;
  TSetAdtInfo tSetAdtInfo;
} TSetAdt;

typedef struct TDspSpotRslOnOffInfo {
  uint32_t unIsSpotOn;
  uint32_t unIsRslOn;
} TDspSpotRslOnOffInfo;

typedef struct TDspSpotRslOnOff {
  TRAF_API_Header tHeader;
  TDspSpotRslOnOffInfo tDspSpotRslOnOffInfo;
} TDspSpotRslOnOff;

extern TParamStr gatDspSpotRslOnOff[];

typedef struct TDspSetRslParamsInfo {
  uint32_t unRslBackOff;
  uint32_t pucRslPrams[15]; // spare params for future use
} TDspSetRslParamsInfo;

typedef struct TDspSetRslParams {
  TRAF_API_Header tHeader;
  TDspSetRslParamsInfo tDspSetRslParamsInfo;
} TDspSetRslParams;

extern TParamStr gatDspSetRslParams[];

//////////////////////////////////////////////////////
/////////		 	GetNoiseVector		//////////
//////////////////////////////////////////////////////

extern TParamStr gatGetNoiseVectorStr;

typedef struct TNoiseVectorInfo {
  uint32_t unDataSize;
  uint32_t unAddress;

} TNoiseVectorInfo, *PTNoiseVectorInfo;

typedef struct TNoiseVector {

  TRAF_API_Header tHeader;
  TNoiseVectorInfo tNoiseVectorInfo;

} TNoiseVector, *PTNoiseVector;

typedef struct TCtrlFrameControlInfo {
  uint32_t unFrameType;
  uint32_t unBaseFreq;
  uint32_t unBandwidth;
} TCtrlFrameControlInfo, *PTCtrlFrameControlInfo;

typedef struct TCtrlFrameControlData {
  TRAF_API_Header tHeader;
  TCtrlFrameControlInfo tFrameControlInfo;
} TCtrlFrameControlData, *PTCtrlFrameControlData;

extern TParamStr gatSetFrameControlData[];

//////////////////////////////////////////////////////
/////////		 	Boot Operation  		//////////
//////////////////////////////////////////////////////

#define API_TX_BOOT_OPERATION_API 5
// Host to Radar
typedef struct {
  uint8_t boot_cmd;
  uint8_t file_idx_countdown;
  uint16_t crc16;
  uint16_t total_frgments;
  uint16_t fragment_number; // 1 - total_fragments
} st_boot_operation_data;

typedef struct {
  TRAF_API_Header hdr;
  st_boot_operation_data data;
} st_boot_operation_api_msg;

extern TParamStr gatBootOperationApiStr[];

//////////////////

#define API_TX_BOOT_OPERATION_IPC 5

// Internal- USER to RAF
typedef struct {
  uint8_t boot_cmd;
  uint8_t file_idx_countdown;
  uint16_t crc16;
  uint32_t file_ptr;
  uint32_t file_len;
} st_boot_operation_ipc_data;

typedef struct {
  TRAF_API_Header hdr;
  st_boot_operation_ipc_data data;
} st_boot_operation_ipc_msg;

extern TParamStr gatBootOperationIpcStr[];

//////////////////////////////////////////////////////
/////////	Boot Operation  response        //////////
//////////////////////////////////////////////////////

typedef enum {
  Eboot_status_unsupported_file,
  Eboot_status_in_progress,
  Eboot_status_ok,
  Eboot_status_fail_flash_write,
  Eboot_status_fail_flash_validate,
  Eboot_status_fail_crc,
  Eboot_status_fail_boot_from_eth
} Eboot_status;

typedef struct {
  uint8_t num_of_files;
  uint8_t actual_num_of_files;
  uint16_t status; // Eboot_status
} st_boot_file_status_data;

typedef struct {
  TRAF_API_Header hdr;
  st_boot_file_status_data data;
} st_boot_file_status_api_msg;

//////////////////////////////////////////////////////
/////////	Get Radar Info			        //////////
//////////////////////////////////////////////////////

typedef struct {
  uint32_t raf_version_num;
  uint32_t raf_commit_id;
  uint32_t user_core_version_num;
  uint32_t user_core_commit_id;
  uint32_t dsp_commit_id;
  uint32_t calibration_id;
  uint32_t antenna_id;
  uint32_t raf_params_commit_id;
} TRadarInfo;

typedef struct {
  TRAF_API_Header tHeader;
  TRadarInfo tRadarInfo;
} TRadarInfo_msg;

/*###########################################################################*/

DLL_EXPIMP void
RAF_API_SysCfg_ConfigurePcOutput(ptArbeApiMailBox ptMailboxHandler,
                                 TConfigurePointCloudsInfo *info);

DLL_EXPIMP void
RAF_API_SysCfg_ConfigPacketFormat(ptArbeApiMailBox ptMailboxHandler,
                                  TConfigPacketFormatInfo *info);

DLL_EXPIMP void RAF_API_SysCfg_SetTime(ptArbeApiMailBox ptMailboxHandler,
                                       TSetTimeInfo SystemTime);

DLL_EXPIMP void RAF_API_SysCfg_FileLocation(ptArbeApiMailBox ptMailboxHandler,
                                            TFileLocationInfo *info);

DLL_EXPIMP void RAF_API_SysCfg_GetMapFile(ptArbeApiMailBox ptMailboxHandler,
                                          TFileLocationInfo *info);

DLL_EXPIMP void RAF_API_RdrCtrl_StartTx(ptArbeApiMailBox ptMailboxHandler,
                                        TStartTxInfo *info);

DLL_EXPIMP void RAF_API_RdrCtrl_StopTx(ptArbeApiMailBox ptMailboxHandler,
                                       TStopInfo *info);

DLL_EXPIMP void RAF_API_RdrCtrl_DspCmd(ptArbeApiMailBox ptMailboxHandler,
                                       TDspCmdInfo *info,
                                       uint32_t unDataLength);

DLL_EXPIMP void RAF_API_RdrCtrl_SetThresholds(ptArbeApiMailBox ptMailboxHandler,
                                              TSetThresholdsInfo *info);

DLL_EXPIMP void RAF_API_RdrCtrl_SetActiveSeq(ptArbeApiMailBox ptMailboxHandler,
                                             TSelectActiveSeqInfo *info);

DLL_EXPIMP void RAF_API_RdrCtrl_SetHistogram(ptArbeApiMailBox ptMailboxHandler,
                                             TSetHistogramInfo *info);

DLL_EXPIMP void
RAF_API_CalUtil_RecordFrameRawData(ptArbeApiMailBox ptMailboxHandler,
                                   TRecordFrameRawDataInfo *info);

DLL_EXPIMP void RAF_API_Dbg_RficOperation(ptArbeApiMailBox ptMailboxHandler,
                                          TRficOperationInfo *info);

DLL_EXPIMP void RAF_API_Dbg_GetVaractorTable(ptArbeApiMailBox ptMailboxHandler,
                                             TVarctorTableInfo *info);

DLL_EXPIMP void RAF_API_Dbg_MemoryOperation(ptArbeApiMailBox ptMailboxHandler,
                                            TMemoryOperationInfo *info);

DLL_EXPIMP void RAF_API_Dbg_TestOperation(ptArbeApiMailBox ptMailboxHandler,
                                          TDebugOperationInfo *info);

DLL_EXPIMP void RAF_API_Dbg_ConfigureInjectPC(ptArbeApiMailBox ptMailboxHandler,
                                              TConfigureInjectPCInfo *info);

DLL_EXPIMP void
RAF_API_Dbg_ConfigureRDrecording(ptArbeApiMailBox ptMailboxHandler,
                                 TConfigureRDrecordingInfo *info);

DLL_EXPIMP void
RAF_API_Dbg_ConfigureCalibrationFrame(ptArbeApiMailBox ptMailboxHandler,
                                      TConfigureCalibrationFrameInfo *info);

DLL_EXPIMP void
RAF_API_RdrCtrl_SetProcessingLimits(ptArbeApiMailBox ptMailboxHandler,
                                    TSetProcessingLimitsInfo *info);

DLL_EXPIMP void RAF_API_RdrCtrl_ConfigureSeq(ptArbeApiMailBox ptMailboxHandler,
                                             TConfigureSeqInfo *info);

DLL_EXPIMP void
RAF_API_RdrCtrl_EnableFramePcOutput(ptArbeApiMailBox ptMailboxHandler,
                                    TConfigureParamInfo *info);

DLL_EXPIMP void RAF_API_SysCfg_GetNoiseVector(ptArbeApiMailBox ptMailboxHandler,
                                              TNoiseVectorInfo *info);

DLL_EXPIMP uint32_t RAF_API_RdrCtrl_SetFrameControlData(
    ptArbeApiMailBox ptMailboxHandler, PTCtrlFrameControlInfo ptInfo);

void BuildHeader(ptArbeApiMailBox ptMailboxHandler, uint32_t unCmdType,
                 uint32_t unMessageSize, TRAF_API_Header *tHeader);

DLL_EXPIMP void RAF_API_SysCfg_CFAR(ptArbeApiMailBox ptMailboxHandler,
                                    TCFARInfo *info);

DLL_EXPIMP void RAF_API_SysCfg_LocalMax(ptArbeApiMailBox ptMailboxHandler,
                                        TSetLocalMaxInfo *info);

DLL_EXPIMP void RAF_API_SysCfg_SetAdt(ptArbeApiMailBox ptMailboxHandler,
                                      TSetAdtInfo *info);

DLL_EXPIMP void
RAF_API_SysCfg_DspSetSpotRslOnOff(ptArbeApiMailBox ptMailboxHandler,
                                  TDspSpotRslOnOffInfo *info);

DLL_EXPIMP void
RAF_API_SysCfg_DspSetRslParams(ptArbeApiMailBox ptMailboxHandler,
                               TDspSetRslParamsInfo *info);

DLL_EXPIMP void RAF_API_SysCfg_NTC(ptArbeApiMailBox ptMailboxHandler,
                                   TNtcInfo *info);

DLL_EXPIMP void RAF_API_RdrCtrl_SetAutocal(ptArbeApiMailBox ptMailboxHandler,
                                           TTxAutoCalInfo *info);

DLL_EXPIMP void
RAF_API_SysCfg_SetAzCdfThrLvls(ptArbeApiMailBox ptMailboxHandler,
                               TAzCdfThrInfo *info);

DLL_EXPIMP void
RAF_API_SysCfg_SetRangeHistConfig(ptArbeApiMailBox ptMailboxHandler,
                                  TRangeHistConfigInfo *info);

DLL_EXPIMP void
RAF_API_SysCfg_SetHistogramOutput(ptArbeApiMailBox ptMailboxHandler,
                                  THistOutputInfo *info);

/*###########################################################################*/

// Leftovers from user extended

DLL_EXPIMP void RAF_API_EXT_AtcMode(ptArbeApiMailBox ptMailboxHandler,
                                    TAtcInfo *info);

/*###########################################################################*/

DLL_EXPIMP void RAF_API_BootOperation(ptArbeApiMailBox ptMailboxHandler,
                                      st_boot_operation_ipc_data *pstCmdData);

DLL_EXPIMP void RAF_API_GetRadarInfo(ptArbeApiMailBox ptMailboxHandler,
                                     TRadarInfo *info);

#ifdef __cplusplus
}
#endif

#endif
