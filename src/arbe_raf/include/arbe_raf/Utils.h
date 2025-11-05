#ifndef UTILS_H
#define UTILS_H

#include "CommonBase.h"
#include "Raf_Api.h"
#include "tlv.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRC_POLY_16 0xD175
#define CRC_START_16 0x0000

#define POW_TO_DB_RATIO 0.1881 //(float)(3 / 16.0f)

#define BIN_DATA_WITH_PHASE_SIZE_BYTES 12
#define BIN_DATA_NO_PHASE_SIZE_BYTES 10
#define BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES 7
#define BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES 6

#define UTILS_PC_IF_CFG_BITS_PER_TARGET_56 0x0
#define UTILS_PC_IF_CFG_BITS_PER_TARGET_48 0x1
#define UTILS_PC_IF_CFG_BITS_PER_TARGET_96 0x2
#define UTILS_PC_IF_CFG_BITS_PER_TARGET_80 0x3

/*
 * Prototype list of global functions
 */

typedef struct {
  union {
    uint8_t data[BIN_DATA_WITH_PHASE_SIZE_BYTES];
    struct {
      uint64_t range : 16;
      uint64_t doppler : 16;
      uint64_t azimuth : 16;
      uint64_t elevation : 16;
      uint64_t power : 16;
      uint64_t phase : 16;
    } Fields;
  };
} Target96bit;

typedef struct {
  union {
    uint8_t data[BIN_DATA_NO_PHASE_SIZE_BYTES];
    struct {
      uint64_t range : 16;
      uint64_t doppler : 16;
      uint64_t azimuth : 16;
      uint64_t elevation : 16;
      uint64_t power : 16;
    } Fields;
  };
} Target80bit;

typedef struct {
  union {
    uint8_t data[BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES];
    struct {
      uint64_t phase : 11;
      uint64_t power : 10;
      uint64_t elevation : 6;
      uint64_t azimuth : 8;
      uint64_t doppler : 12;
      uint64_t range : 9;
    } Fields;
  };
} Target56bit;

typedef struct {
  union {
    uint8_t data[BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES];
    struct {
      uint64_t power : 10;
      uint64_t elevation : 6;
      uint64_t azimuth : 8;
      uint64_t doppler : 12;
      uint64_t range : 9;
      uint64_t dummy : 3;
    } Fields;
  };
} Target48bit;

typedef struct {
  float range;
  float azimuthParam;
  int azimuthNumOfBins;
  float elevationParam;
  int elevationNumOfBins;
} TargetGenericResolution;

typedef struct {
  float phase;
  float power;
  float elevation;
  float azimuth;
  float doppler;
  float range;
} TargetGeneric;

typedef struct {
  double x;
  double y;
  double z;
} TargetCartesian;

/*****************************************************************************
*	RAF_COM_CALC_Bins2Phy - Convert binary point cloud package to list of
phy points with resolution
** @params
*		Input
*			pointCloudPackage - pointer on begin of the point cloud
pacakge (include header) *			TargetsBuffer - Allocate memory
for physical results, Max size is 160 * 24 + 24 bytes (Maximum points in pacakge
* TargetGenric size) + resulotion struct size

*
*		Output
*			TargetsBuffer - Contain resolution and all the physical
info , each point convert to TargetGeneric struct
                                * First:  Resolution represented by
TargetGenericResolution struct
                                * Second: All physical point clouds points
chained , represented by TargetGeneric struct *			TargetsLength -
TargetsBuffer return size
*
* @note		None.
*
******************************************************************************/
DLL_EXPIMP void RAF_COM_CALC_Binary2Phy(uint8_t *pointCloudPackage,
                                        uint8_t *TargetsBuffer,
                                        uint32_t *TargetsLength);

/*****************************************************************************
*	RAF_COM_CALC_Binary2Bins - Convert binary point cloud package to list of
bins points
** @params
*		Input
*			BinaryData - pointer on begin of the point cloud data
(not include header) *			BinaryDataLength - Length of BinaryData
buffer bytesPerTarget - Number of bytes to represent one point BinsBuffer -
Allocate memory for physical results, Max size is 160 * 24 bytes (Maximum points
in pacakge * TargetGenric size)
*
*		Output
*			BinsBuffer - All bins point clouds points chained ,
represented by TargetGeneric struct
*			BinsLength - BinsBuffer return size
*
* @note		None.
*
******************************************************************************/
DLL_EXPIMP void RAF_COM_CALC_Binary2Bins(uint8_t *pointCloudPackage,
                                         uint8_t *TargetsBuffer,
                                         uint32_t *TargetsLength);

DLL_EXPIMP void RAF_COM_CALC_Binary2PhysicalBins(uint8_t *pointCloudPackage,
                                                 uint8_t *TargetsBuffer,
                                                 uint32_t *TargetsLength);

DLL_EXPIMP void RAF_COM_Bin2cart(float range_bin, float azimuth_bin,
                                 float elevation_bin, TPcMetadata *tMetadata,
                                 TargetCartesian *taregtCartesian);

DLL_EXPIMP void RAF_COM_CALC_Binary2Cart(uint8_t *pointCloudPackage,
                                         uint8_t *TargetsBuffer,
                                         uint32_t *TargetsLength);

DLL_EXPIMP float RAF_COM_CALC_CalcRangeZoomBin(int rangeBin, TRangeCoef range,
                                               TDopplerCoef doppler);

DLL_EXPIMP float RAF_COM_CALC_CalcRangeCoefficient(TRangeCoef range);

DLL_EXPIMP uint32_t RAF_COM_CALC_CalcPhysicalRangeZoom(TRangeCoef range);

DLL_EXPIMP float RAF_COM_CALC_CalcRange(float bin, TPcMetadata *meta);

DLL_EXPIMP float RAF_COM_CALC_RangeOffset(TRangeCoef range);

DLL_EXPIMP int RAF_COM_CALC_ZoomFromOffset(float offset);

DLL_EXPIMP float RAF_COM_CALC_CalcDopplerCoefficient(TDopplerCoef doppler);

DLL_EXPIMP float RAF_COM_CALC_CalcDopplerSignBin(int bin, TDopplerCoef doppler);

DLL_EXPIMP float RAF_COM_CALC_CalcDoppler(int bin, TPcMetadata *meta);

DLL_EXPIMP float RAF_COM_CALC_CalcAzimuthCoefficient(TAzimuthCoef azimuth);

DLL_EXPIMP int32_t RAF_COM_CALC_CalcAzimuthSignBin(int bin,
                                                   TAzimuthCoef azimuth);

DLL_EXPIMP float RAF_COM_CALC_CalcSinAlpha(float bin, TPcMetadata *meta);

DLL_EXPIMP float RAF_COM_CALC_CalcAlpha(int bin, TPcMetadata *meta);

DLL_EXPIMP float RAF_COM_CALC_CalcAzimuth(int Azbin, int Elbin,
                                          TPcMetadata *meta);

DLL_EXPIMP int32_t RAF_COM_CALC_CalcElevationSignBin(int bin,
                                                     TElevationCoef elevation);

DLL_EXPIMP float
RAF_COM_CALC_CalcElevationCoefficient(TElevationCoef elevation);

DLL_EXPIMP float RAF_COM_CALC_CalcSinTheta(uint16_t el_bin,
                                           TPcMetadata *metadata);

DLL_EXPIMP float RAF_COM_CALC_CalcElevation(double bin, TPcMetadata *meta);

DLL_EXPIMP float RAF_COM_CALC_CalcPhase(int bin);

DLL_EXPIMP float RAF_COM_CALC_CalcPower(int power);

DLL_EXPIMP float RAF_COM_CALC_CalcPowerEqulizer(float power, int rangeBin,
                                                float rangeResInDb,
                                                float range_offset);

DLL_EXPIMP float RAF_COM_CALC_ClacRangeResInDb(TPcMetadata *meta);

DLL_EXPIMP uint8_t RAF_COM_CLAC_Is4d(TPcMetadata *ptMetadata);

DLL_EXPIMP uint8_t RAF_COM_CLAC_GetFrameNumber(uint8_t *pointCloudPackage);

DLL_EXPIMP uint32_t
RAF_COM_CALC_GetBytesLengthOfTraget(uint32_t unTargetFormat);

DLL_EXPIMP TargetGeneric GetPhysicalBin(uint8_t *data,
                                        uint32_t NumberOfbytesTarget,
                                        TPcMetadata *pcMetadata);

uint16_t Float2Fixed(float floatingNum, uint8_t fractionalBits);

uint16_t Crc16Ccitt(uint8_t *buf, int len);

float Fixed2Float(uint16_t fixedNum, uint8_t fractionalBits);

float Fixed32toFloat(int32_t fixedNum, uint8_t fractionalBits);

float Fixed64toFloat(int64_t fixedNum, uint8_t fractionalBits);

#ifdef __cplusplus
}
#endif
#endif
