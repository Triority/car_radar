#ifndef _TLV_H_
#define _TLV_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TAG_DOPP_AMBIGUITY_FILTER_ON 0x9F03
#define TAG_EGO_VAL_CALIB 0x9F06
#define TAG_STATIONARY_TARGETS_REMOVE 0x9F07
#define TAG_AMBIGUITY_TARGETS_REMOVE 0x9F09
#define TAG_GPS_DATA 0x9F0A
#define TAG_IMU_DATA 0x9F0B
#define TAG_START_STOP_RECORD 0x9F10
#define TAG_RESET_EGOVEL_CALIB 0x9F12
#define TAG_MAG_CAL_PARAMS 0x9F13
#define TAG_IS_RECORDING 0x9F17

// BYPASS
#define TAG_BYPASS_SLAM 0x9F40
#define TAG_BYPASS_HORN_REMOVAL 0x9F41
#define TAG_EGO_VEL_BYPASS 0x9F05
#define TAG_BYPASS_SL_REMOVAL 0x9F42
#define TAG_BYPASS_GPS 0x9F43
#define TAG_BYPASS_IMU 0x9F44
#define TAG_BYPASS_MULTIPATH_FILTER 0x9F45

// METADATA
#define TAG_ANTENNA_TYPE 0x9F21
#define TAG_NUM_AZ_BINS 0x9F22
#define TAG_REC_PC_PRE_SLAM 0x9F23

// EGO VEL
#define TAG_LEFT_SIGMA_COEF 0x9F24
#define TAG_RIGHT_SIGMA_COEF 0x9F25
#define TAG_EGO_VEL_TRANSITION 0x9F26
#define TAG_LEFT_SIGMA_WITHIN_0ENV 0x9F0C
#define TAG_RIGHT_SIGMA_WITHIN_0ENV 0x9F0D
#define TAG_STATIONARY_SPUR_TH 0x9F14

// MAPPING
#define TAG_IDLE_BET_CALIBS_SEC 0x9F18
#define TAG_MAG_DEC_DEG 0x9F19
#define TAG_RST_MAG_CAL_W_EGO_V 0x9F1A

// AMBIGUITY
#define TAG_AMB_DOPP_STEP 0x9F27
#define TAG_AMB_SIN_AZ_STEP 0x9F28
#define TAG_ADD_AMB_TRACKS 0x9F29

// HORN REMOVAL
#define TAG_HORN_N_TH 0x9F2A
#define TAG_HORN_POW_BIN_SIZE_DB 0x9F2B
#define TAG_HORN_POW_DB_LEAP 0x9F2C

// CLUSTERING
#define TAG_EPSILON 0x9F01
#define TAG_CLSTR_METRIC_R 0x9F2D
#define TAG_CLSTR_METRIC_D 0x9F2E
#define TAG_CLSTR_METRIC_A 0x9F2F
#define TAG_CLSTR_METRIC_E 0x9F30
#define TAG_CLSTR_METRIC_POW 0x9F31

// MP REMOVAL
#define TAG_DR_TH 0x9F32
#define TAG_DV_TH 0x9F33
#define TAG_DPOW_TH 0x9F34
#define TAG_DRVTOT_TH 0x9F0F
#define TAG_DAZ_TH 0x9F11
#define TAG_MIRROR_RTH 0x9F15
#define TAG_MIN_MIRROR_FR 0x9F16

// MATCH
#define TAG_X_TH 0x9F36
#define TAG_Y_TH 0x9F37
#define TAG_VX_TH 0x9F38
#define TAG_VY_TH 0x9F39
#define TAG_RDOT_TH 0x9F3A
#define TAG_VTOT_SQ_TH 0x9F3B
#define TAG_RDOT_2_TH 0x9F0E

// OBJECT
#define TAG_VIS_TH 0x9F3C
#define TAG_MAX_AGE 0x9F3D
#define TAG_MERGE_TH 0x9F3E
#define TAG_ALPHA 0x9F02
#define TAG_ENLARGE_FCT 0x9F3F

// SPOT
#define TAG_SPOT_ENABLE 0x9F04
#define TAG_SPOT_WIN_1_AZ_SIZE 0x9F08
#define TAG_SPOT_WIN_1_R_SIZE 0x9F60
#define TAG_SPOT_WIN_1_BACKOFF 0x9F61
#define TAG_SPOT_WIN_2_AZ_SIZE 0x9F62
#define TAG_SPOT_WIN_2_R_SIZE 0x9F63
#define TAG_SPOT_WIN_2_BACKOFF 0x9F64
#define TAG_SPOT_LOWER_BOUND_BACKOFF 0x9F65

int TLV_DecodeTLVbyIndex(uint8_t *buffer, uint16_t index, uint16_t len,
                         uint16_t *usTag, uint16_t *usLength,
                         uint8_t **ucDataPtr);

int TLV_EncodeTLV(uint16_t usTag, uint8_t ucLength, uint8_t *data,
                  uint8_t *targetBuffer, uint16_t bufferLength,
                  uint8_t *ucTlvHeaderLength);

int TLV_DecodeTLbyIndex(uint8_t *buffer, uint16_t index, uint16_t len,
                        uint16_t *usTag, uint16_t *usLength);
int TLV_EncodeTL(uint16_t usTag, uint8_t ucLength, uint8_t *targetBuffer,
                 uint16_t bufferLength);

#ifdef __cplusplus
}
#endif
#endif //_TLV_H_
