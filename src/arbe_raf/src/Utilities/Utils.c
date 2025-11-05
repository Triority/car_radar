#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "CommonBase.h"
#include "Raf_Api.h"
#include "Utils.h"
#include "ver.h"

#ifdef __cplusplus
extern "C" {
#endif

/************ STATIC FILEDS AND FUNCTION *************/

static void init_crc16_tab(void);
static bool crc_tab16_init = false;
static uint16_t crc_tab16[256];
static uint8_t rangeBinToDb[128] = {
    0x0,  0xC,  0x13, 0x18, 0x1C, 0x1F, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2B,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x36, 0x37,
    0x38, 0x39, 0x39, 0x3A, 0x3A, 0x3B, 0x3C, 0x3C, 0x3D, 0x3D, 0x3E, 0x3E,
    0x3F, 0x3F, 0x40, 0x40, 0x41, 0x41, 0x41, 0x42, 0x42, 0x43, 0x43, 0x43,
    0x44, 0x44, 0x44, 0x45, 0x45, 0x45, 0x46, 0x46, 0x46, 0x47, 0x47, 0x47,
    0x47, 0x48, 0x48, 0x48, 0x49, 0x49, 0x49, 0x49, 0x4A, 0x4A, 0x4A, 0x4A,
    0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4C, 0x4C, 0x4C, 0x4C, 0x4D, 0x4D, 0x4D,
    0x4D, 0x4D, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F,
    0x4F, 0x50, 0x50, 0x50, 0x50, 0x50, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51,
    0x51, 0x52, 0x52, 0x52, 0x52, 0x52, 0x52, 0x53, 0x53, 0x53, 0x53, 0x53,
    0x53, 0x53, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54,
};

#define EQUALIZATION_UNIT 12.04 // 40 log10(2)

/*****************************************/

/************ LOCAL FUNCTION *************/

void InnerGetBinsFromPointClouds_1_1(uint8_t *pointCloudBinData,
                                     uint8_t *Results, uint32_t *ResultsLength);

void InnerGetTargetsFromPointClouds_1_1(uint8_t *pointCloudBinData,
                                        uint8_t *Results,
                                        uint32_t *ResultsLength);

TargetGeneric GetTarget(uint8_t *data, uint32_t NumberOfbytesTarget,
                        TPcMetadata *pcMetadata);

TargetGeneric GetBin(uint8_t *data, uint32_t NumberOfbytesTarget);

/*****************************************/
float RAF_COM_CALC_CalcRangeCoefficient(TRangeCoef range) {
  uint16_t complete = range.tFields.integral;

  uint16_t fraction = range.tFields.fraction;

  return complete + fraction / (float)(1 << 16);
}

uint32_t RAF_COM_CALC_CalcPhysicalRangeZoom(TRangeCoef range) {
  if (range.tFields.range_zoom == 0x0) {
    return 1;
  } else if (range.tFields.range_zoom == 0x1) {
    return 2;
  } else if (range.tFields.range_zoom == 0x3) {
    return 4;
  } else if (range.tFields.range_zoom == 0x7) {
    return 8;
  } else if (range.tFields.range_zoom == 0xF) {
    return 16;
  } else {
    return 0;
  }
}

float RAF_COM_CALC_CalcRangeZoomBin(int rangeBin, TRangeCoef range,
                                    TDopplerCoef doppler) {
  uint32_t zoom_factor;
  if (doppler.tFields.isFine == 1) {
    zoom_factor = RAF_COM_CALC_CalcPhysicalRangeZoom(range);
    if ((uint32_t)rangeBin > ((zoom_factor - 2) / 2)) {
      return (rangeBin - ((zoom_factor - 1) / 2));
    } else {
      return 0;
    }
  }

  return rangeBin;
}

float RAF_COM_CALC_RangeOffset(TRangeCoef range) {
  uint32_t zoom_factor = RAF_COM_CALC_CalcPhysicalRangeZoom(range);

  return (((float)zoom_factor - 1) / 2);
}

int RAF_COM_CALC_ZoomFromOffset(float offset) { return (int)(2 * offset + 1); }

float RAF_COM_CALC_CalcRange(float bin, TPcMetadata *meta) {

  float coef = RAF_COM_CALC_CalcRangeCoefficient(meta->range);

  if (meta->doppler.tFields.isFine == 1) {
    uint32_t zoom = RAF_COM_CALC_CalcPhysicalRangeZoom(meta->range);
    if (bin > zoom / 2 - 1) {
      return (bin - RAF_COM_CALC_RangeOffset(meta->range)) * coef;
    } else {
      return 0;
    }
  }

  return bin * coef;
}

float RAF_COM_CALC_CalcDopplerCoefficient(TDopplerCoef doppler) {
  return doppler.tFields.integral +
         (doppler.tFields.fraction / (float)(1 << 16));
}

float RAF_COM_CALC_CalcDopplerSignBin(int bin, TDopplerCoef doppler) {
  float _doppler;

  if (doppler.tFields.isFine) {
    _doppler = (float)((bin + 0x800) % 0x1000);

    _doppler -= 0x800;
  }

  else

  {

    _doppler = (float)((bin + 0x200) % 0x400);

    _doppler -= 0x200;
  }

  return (-1) * _doppler;
}

float RAF_COM_CALC_CalcDoppler(int bin, TPcMetadata *meta) {

  float doppler = RAF_COM_CALC_CalcDopplerSignBin(bin, meta->doppler);

  doppler *= RAF_COM_CALC_CalcDopplerCoefficient(meta->doppler);

  return doppler;
}

float RAF_COM_CALC_CalcAzimuthCoefficient(TAzimuthCoef azimuth) {
  return azimuth.tFields.fraction / (float)(1 << 16);
}

int32_t RAF_COM_CALC_CalcAzimuthSignBin(int bin, TAzimuthCoef azimuth) {
  return (-1) *
         (bin - (azimuth.tFields.fftRange / 2) + azimuth.tFields.padding);
}

float RAF_COM_CALC_CalcSinAlpha(float bin, TPcMetadata *meta) {
  float signedBin = RAF_COM_CALC_CalcAzimuthSignBin(bin, meta->azimuth);
  float coef = RAF_COM_CALC_CalcAzimuthCoefficient(meta->azimuth);
  return signedBin * coef;
}

float RAF_COM_CALC_CalcAzimuth(int Azbin, int Elbin, TPcMetadata *metadata) {
  float azimuth = 0;

  float SinAlpha = RAF_COM_CALC_CalcSinAlpha(Azbin, metadata);

  float sinEl = RAF_COM_CALC_CalcSinTheta(Elbin, metadata);
  azimuth = asin(SinAlpha / sqrt(1 - sinEl * sinEl));

  return azimuth;
}

float RAF_COM_CALC_CalcAlpha(int bin, TPcMetadata *meta) {
  float SinAlpha = RAF_COM_CALC_CalcSinAlpha(bin, meta);

  return (float)asin(SinAlpha);
}

int32_t RAF_COM_CALC_CalcElevationSignBin(int bin, TElevationCoef elevation) {
  return (-1) *
         (bin - (elevation.tFields.fftRange) / 2 + elevation.tFields.padding);
}

float RAF_COM_CALC_CalcElevationCoefficient(TElevationCoef elevation) {
  return (elevation.tFields.fraction / (float)(1 << 16));
}

float RAF_COM_CALC_CalcSinTheta(uint16_t el_bin, TPcMetadata *metadata) {

  int32_t elfixedBin =
      RAF_COM_CALC_CalcElevationSignBin(el_bin, metadata->elevation);
  float SinEl = RAF_COM_CALC_CalcElevationCoefficient(metadata->elevation);
  return SinEl * elfixedBin;
}

float RAF_COM_CALC_CalcElevation(double bin, TPcMetadata *meta) {
  float SinEl = RAF_COM_CALC_CalcSinTheta(bin, meta);
  return (float)asin(SinEl);
}

float RAF_COM_CALC_CalcPhase(int bin) { return (float)bin; }

float RAF_COM_CALC_CalcPower(int power) {
  return (float)(power * POW_TO_DB_RATIO);
}
float RAF_COM_CALC_CalcPowerEqulizer(float power, int rangeBin,
                                     float rangeResInDb, float range_offset) {
  int zoom = RAF_COM_CALC_ZoomFromOffset(range_offset);

  if ((rangeBin / zoom) > 128) // This should not happen but causes a crash if a
                               // bad frame is received
    return 0;

  float eq_p = power + rangeResInDb + rangeBinToDb[(int)(rangeBin / zoom)];

  switch (zoom) {
  case 2:
    eq_p += EQUALIZATION_UNIT;
    break;
  case 4:
    eq_p += 2 * EQUALIZATION_UNIT;
    break;
  case 8:
    eq_p += 3 * EQUALIZATION_UNIT;
    break;
  default: // 3d mode or no zoom
    break;
  }
  return eq_p;
}

float RAF_COM_CALC_ClacRangeResInDb(TPcMetadata *meta) {
  uint16_t complete = meta->range.tFields.integral;

  uint16_t fraction = meta->range.tFields.fraction;

  float coef = complete + fraction / (float)(1 << 16);

  return 40 * log10(coef);
}

uint8_t RAF_COM_CLAC_Is4d(TPcMetadata *ptMetadata) {

  return (ptMetadata->doppler.tFields.isFine);
}

uint8_t RAF_COM_CLAC_GetFrameNumber(uint8_t *pointCloudPackage) {
  return ((TPointCloud_V1_1 *)pointCloudPackage)->ucTrgtFmtFrmType & 0x3f;
}

uint16_t Crc16Ccitt(uint8_t *buf, int len) {
  int counter;

  if (!crc_tab16_init)
    init_crc16_tab();

  uint16_t crc = 0;
  for (counter = 0; counter < len; counter++)
    crc = (crc << 8) ^ crc_tab16[((crc >> 8) ^ *(char *)buf++) & 0x00FF];
  return crc;
}

static void init_crc16_tab(void) {
  uint16_t i;
  int j;
  int xor_flag;
  uint16_t result;

  for (i = 0; i < 256; i++) {
    result = i << 8;
    for (j = 0; j < 8; j++) {
      /* Flag for XOR if leftmost bit is set */
      xor_flag = result & 0x8000;

      /* Shift CRC */
      result <<= 1;

      /* Perform the XOR */
      if (xor_flag)
        result ^= 0xd175;
    }
    crc_tab16[i] = result;
  }
  crc_tab16_init = true;

} /* init_crc16_tab */

void RAF_COM_CALC_Binary2Phy(uint8_t *pointCloudPackage, uint8_t *TargetsBuffer,
                             uint32_t *TargetsLength) {
  *TargetsLength = 0;
  if ((((TRAF_API_Header *)pointCloudPackage)->usType) ==
      PointCloud_Output_V1_1) {
    InnerGetTargetsFromPointClouds_1_1(pointCloudPackage, TargetsBuffer,
                                       TargetsLength);
  }
}

void RAF_COM_CALC_Binary2Bins(uint8_t *pointCloudPackage,
                              uint8_t *TargetsBuffer, uint32_t *TargetsLength) {
  *TargetsLength = 0;
  if ((((TRAF_API_Header *)pointCloudPackage)->usType) ==
      PointCloud_Output_V1_1) {
    InnerGetBinsFromPointClouds_1_1(pointCloudPackage, TargetsBuffer,
                                    TargetsLength);
  }
}

void RAF_COM_CALC_Binary2PhysicalBins(uint8_t *pointCloudPackage,
                                      uint8_t *TargetsBuffer,
                                      uint32_t *TargetsLength) {
  uint32_t i = 0;
  TargetGeneric targetGeneric;
  uint32_t bytesPerTarget = RAF_COM_CALC_GetBytesLengthOfTraget(
      ((TPointCloud_V1_1 *)pointCloudPackage)->ucTrgtFmtFrmType);

  for (i = sizeof(TPointCloud_V1_1);
       i < ((TPointCloud_V1_1 *)pointCloudPackage)->unLength;
       i += bytesPerTarget) {
    targetGeneric =
        GetPhysicalBin(pointCloudPackage + i, bytesPerTarget,
                       &((TPointCloud_V1_1 *)pointCloudPackage)->tMetadata);
    memcpy(TargetsBuffer, &targetGeneric, sizeof(targetGeneric));
    TargetsBuffer = TargetsBuffer + sizeof(targetGeneric);
    *TargetsLength += sizeof(targetGeneric);
  }
}

void RAF_COM_Bin2cart(float range_bin, float azimuth_bin, float elevation_bin,
                      TPcMetadata *tMetadata,
                      TargetCartesian *taregtCartesian) {
  float range = RAF_COM_CALC_CalcRange(range_bin, tMetadata);

  taregtCartesian->x =
      range * RAF_COM_CALC_CalcSinAlpha(azimuth_bin, tMetadata);
  taregtCartesian->z =
      range * RAF_COM_CALC_CalcSinTheta(elevation_bin, tMetadata);
  taregtCartesian->y =
      sqrt(range * range - taregtCartesian->z * taregtCartesian->z -
           taregtCartesian->x * taregtCartesian->x);
}

void RAF_COM_CALC_Binary2Cart(uint8_t *pointCloudPackage,
                              uint8_t *TargetsBuffer, uint32_t *TargetsLength) {
  *TargetsLength = 0;

  uint32_t i = 0;
  TargetGeneric targetGeneric;
  TargetCartesian taregtCartesian;

  uint32_t unTargetFormat =
      ((TPointCloud_V1_1 *)pointCloudPackage)->ucTrgtFmtFrmType;

  uint32_t bytesPerTarget = RAF_COM_CALC_GetBytesLengthOfTraget(unTargetFormat);

  for (i = sizeof(TPointCloud_V1_1);
       i < ((TPointCloud_V1_1 *)pointCloudPackage)->unLength;
       i += bytesPerTarget) {
    targetGeneric = GetBin(pointCloudPackage + i, bytesPerTarget);
    RAF_COM_Bin2cart(
        targetGeneric.range, targetGeneric.azimuth, targetGeneric.elevation,
        &((TPointCloud_V1_1 *)pointCloudPackage)->tMetadata, &taregtCartesian);

    memcpy(TargetsBuffer, &taregtCartesian, sizeof(taregtCartesian));
    TargetsBuffer = TargetsBuffer + sizeof(taregtCartesian);
    *TargetsLength += sizeof(taregtCartesian);
  }
}

void InnerGetBinsFromPointClouds_1_1(uint8_t *pointCloudBinData,
                                     uint8_t *Results,
                                     uint32_t *ResultsLength) {
  uint32_t i = 0;
  TargetGeneric targetGeneric;

  uint32_t unTargetFormat =
      ((TPointCloud_V1_1 *)pointCloudBinData)->ucTrgtFmtFrmType;

  uint32_t bytesPerTarget = RAF_COM_CALC_GetBytesLengthOfTraget(unTargetFormat);

  for (i = sizeof(TPointCloud_V1_1);
       i < ((TPointCloud_V1_1 *)pointCloudBinData)->unLength;
       i += bytesPerTarget) {
    targetGeneric = GetBin(pointCloudBinData + i, bytesPerTarget);
    memcpy(Results, &targetGeneric, sizeof(targetGeneric));

    Results = Results + sizeof(targetGeneric);
    *ResultsLength += sizeof(targetGeneric);
  }
}

void InnerGetTargetsFromPointClouds_1_1(uint8_t *pointCloudBinData,
                                        uint8_t *Results,
                                        uint32_t *ResultsLength) {
  TargetGenericResolution resolution;

  uint32_t unTargetFormat =
      ((TPointCloud_V1_1 *)pointCloudBinData)->ucTrgtFmtFrmType;

  uint32_t bytesPerTarget = RAF_COM_CALC_GetBytesLengthOfTraget(unTargetFormat);

  // calculate resolution
  resolution.range =
      RAF_COM_CALC_CalcRange(
          1, &((TPointCloud_V1_1 *)pointCloudBinData)->tMetadata) -
      RAF_COM_CALC_CalcRange(
          0, &((TPointCloud_V1_1 *)pointCloudBinData)->tMetadata);
  resolution.azimuthParam = ((TPointCloud_V1_1 *)pointCloudBinData)
                                ->tMetadata.azimuth.tFields.fraction /
                            (float)(1 << 16);
  resolution.azimuthNumOfBins = ((TPointCloud_V1_1 *)pointCloudBinData)
                                    ->tMetadata.azimuth.tFields.fftRange;
  resolution.elevationParam = ((TPointCloud_V1_1 *)pointCloudBinData)
                                  ->tMetadata.elevation.tFields.fraction /
                              (float)(1 << 16);
  resolution.elevationNumOfBins = ((TPointCloud_V1_1 *)pointCloudBinData)
                                      ->tMetadata.elevation.tFields.fftRange;

  // insert resolution struct before physical points
  memcpy(Results, (uint8_t *)&resolution, sizeof(TargetGenericResolution));
  Results += sizeof(resolution);
  *ResultsLength += sizeof(resolution);

  uint32_t i = 0;
  TargetGeneric targetGeneric;

  for (i = 0; i < ((TPointCloud_V1_1 *)pointCloudBinData)->unLength -
                      sizeof(TPointCloud_V1_1);
       i += bytesPerTarget) {

    targetGeneric = GetTarget(
        pointCloudBinData + sizeof(TPointCloud_V1_1) + i, bytesPerTarget,
        &((TPointCloud_V1_1 *)pointCloudBinData)->tMetadata);
    memcpy(Results, &targetGeneric, sizeof(targetGeneric));

    Results = Results + sizeof(targetGeneric);
    *ResultsLength = *ResultsLength + sizeof(targetGeneric);
  }
}

TargetGeneric GetTarget(uint8_t *data, uint32_t NumberOfbytesTarget,
                        TPcMetadata *pcMetadata) {
  TargetGeneric targetGeneric = {0};

  if (NumberOfbytesTarget == BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES) {
    Target56bit *target56bit = (Target56bit *)(data);
    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuth(
        (double)target56bit->Fields.azimuth,
        (double)target56bit->Fields.elevation, pcMetadata);
    targetGeneric.doppler =
        RAF_COM_CALC_CalcDoppler((int)target56bit->Fields.doppler, pcMetadata);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevation(
        (double)target56bit->Fields.elevation, pcMetadata);
    targetGeneric.phase =
        RAF_COM_CALC_CalcPhase((int)target56bit->Fields.phase);
    targetGeneric.power =
        RAF_COM_CALC_CalcPower((int)target56bit->Fields.power);
    targetGeneric.range =
        RAF_COM_CALC_CalcRange((int)target56bit->Fields.range, pcMetadata);
  } else if (NumberOfbytesTarget == BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES) {
    Target48bit *target48bit = (Target48bit *)(data);
    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuth(
        (double)target48bit->Fields.azimuth,
        (double)target48bit->Fields.elevation, pcMetadata);
    targetGeneric.doppler =
        RAF_COM_CALC_CalcDoppler((int)target48bit->Fields.doppler, pcMetadata);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevation(
        (double)target48bit->Fields.elevation, pcMetadata);
    targetGeneric.phase = 0;
    targetGeneric.power =
        RAF_COM_CALC_CalcPower((int)target48bit->Fields.power);
    targetGeneric.range =
        RAF_COM_CALC_CalcRange((int)target48bit->Fields.range, pcMetadata);
  } else if (NumberOfbytesTarget == BIN_DATA_WITH_PHASE_SIZE_BYTES) {
    Target96bit *target96bit = (Target96bit *)(data);
    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuth(
        (double)target96bit->Fields.azimuth,
        (double)target96bit->Fields.elevation, pcMetadata);
    targetGeneric.doppler =
        RAF_COM_CALC_CalcDoppler((int)target96bit->Fields.doppler, pcMetadata);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevation(
        (double)target96bit->Fields.elevation, pcMetadata);
    targetGeneric.phase =
        RAF_COM_CALC_CalcPhase((int)target96bit->Fields.phase);
    targetGeneric.power =
        RAF_COM_CALC_CalcPower((int)target96bit->Fields.power);
    targetGeneric.range =
        RAF_COM_CALC_CalcRange((int)target96bit->Fields.range, pcMetadata);
  } else if (NumberOfbytesTarget == BIN_DATA_NO_PHASE_SIZE_BYTES) {
    Target80bit *target80bit = (Target80bit *)(data);
    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuth(
        (double)target80bit->Fields.azimuth,
        (double)target80bit->Fields.elevation, pcMetadata);
    targetGeneric.doppler =
        RAF_COM_CALC_CalcDoppler((int)target80bit->Fields.doppler, pcMetadata);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevation(
        (double)target80bit->Fields.elevation, pcMetadata);
    targetGeneric.phase = 0;
    targetGeneric.power =
        RAF_COM_CALC_CalcPower((int)target80bit->Fields.power);
    targetGeneric.range =
        RAF_COM_CALC_CalcRange((int)target80bit->Fields.range, pcMetadata);
  }
  return targetGeneric;
}

TargetGeneric GetBin(uint8_t *data, uint32_t NumberOfbytesTarget) {
  TargetGeneric targetGeneric = {0};

  if (NumberOfbytesTarget == BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES) {
    Target56bit *target56bit = (Target56bit *)(data);
    targetGeneric.azimuth = (float)target56bit->Fields.azimuth;
    targetGeneric.doppler = (float)target56bit->Fields.doppler;
    targetGeneric.elevation = (float)target56bit->Fields.elevation;
    targetGeneric.phase = (float)target56bit->Fields.phase;
    targetGeneric.power = (float)target56bit->Fields.power;
    targetGeneric.range = (float)target56bit->Fields.range;
  } else if (NumberOfbytesTarget == BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES) {
    Target48bit *target48bit = (Target48bit *)(data);
    targetGeneric.azimuth = (float)target48bit->Fields.azimuth;
    targetGeneric.doppler = (float)target48bit->Fields.doppler;
    targetGeneric.elevation = (float)target48bit->Fields.elevation;
    targetGeneric.phase = 0;
    targetGeneric.power = (float)target48bit->Fields.power;
    targetGeneric.range = (float)target48bit->Fields.range;
  } else if (NumberOfbytesTarget == BIN_DATA_WITH_PHASE_SIZE_BYTES) {
    Target96bit *target96bit = (Target96bit *)(data);
    targetGeneric.azimuth = (float)target96bit->Fields.azimuth;
    targetGeneric.doppler = (float)target96bit->Fields.doppler;
    targetGeneric.elevation = (float)target96bit->Fields.elevation;
    targetGeneric.phase = (float)target96bit->Fields.phase;
    targetGeneric.power = (float)target96bit->Fields.power;
    targetGeneric.range = (float)target96bit->Fields.range;
  } else if (NumberOfbytesTarget == BIN_DATA_NO_PHASE_SIZE_BYTES) {
    Target80bit *target80bit = (Target80bit *)(data);
    targetGeneric.azimuth = (float)target80bit->Fields.azimuth;
    targetGeneric.doppler = (float)target80bit->Fields.doppler;
    targetGeneric.elevation = (float)target80bit->Fields.elevation;
    targetGeneric.phase = 0;
    targetGeneric.power = (float)target80bit->Fields.power;
    targetGeneric.range = (float)target80bit->Fields.range;
  }

  return targetGeneric;
}

TargetGeneric GetPhysicalBin(uint8_t *data, uint32_t NumberOfbytesTarget,
                             TPcMetadata *pcMetadata) {
  TargetGeneric targetGeneric = {0};

  if (NumberOfbytesTarget == BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES) {
    Target56bit *target56bit = (Target56bit *)(data);
    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuthSignBin(
        (float)target56bit->Fields.azimuth, pcMetadata->azimuth);
    targetGeneric.doppler = RAF_COM_CALC_CalcDopplerSignBin(
        (float)target56bit->Fields.doppler, pcMetadata->doppler);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevationSignBin(
        (float)target56bit->Fields.elevation, pcMetadata->elevation);
    targetGeneric.phase = RAF_COM_CALC_CalcPhase(target56bit->Fields.phase);
    targetGeneric.power = (float)target56bit->Fields.power;
    targetGeneric.range = (float)target56bit->Fields.range;
  } else if (NumberOfbytesTarget == BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES) {
    Target48bit *target48bit = (Target48bit *)(data);

    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuthSignBin(
        (float)target48bit->Fields.azimuth, pcMetadata->azimuth);
    targetGeneric.doppler = RAF_COM_CALC_CalcDopplerSignBin(
        (float)target48bit->Fields.doppler, pcMetadata->doppler);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevationSignBin(
        (float)target48bit->Fields.elevation, pcMetadata->elevation);
    targetGeneric.phase = 0;
    targetGeneric.power = (float)target48bit->Fields.power;
    targetGeneric.range = (float)target48bit->Fields.range;

  } else if (NumberOfbytesTarget == BIN_DATA_WITH_PHASE_SIZE_BYTES) {
    Target96bit *target96bit = (Target96bit *)(data);
    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuthSignBin(
        (float)target96bit->Fields.azimuth, pcMetadata->azimuth);
    targetGeneric.doppler = RAF_COM_CALC_CalcDopplerSignBin(
        (float)target96bit->Fields.doppler, pcMetadata->doppler);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevationSignBin(
        (float)target96bit->Fields.elevation, pcMetadata->elevation);
    targetGeneric.phase = RAF_COM_CALC_CalcPhase(target96bit->Fields.phase);
    targetGeneric.power = (float)target96bit->Fields.power;
    targetGeneric.range = (float)target96bit->Fields.range;
  } else if (NumberOfbytesTarget == BIN_DATA_NO_PHASE_SIZE_BYTES) {
    Target80bit *target80bit = (Target80bit *)(data);
    targetGeneric.azimuth = RAF_COM_CALC_CalcAzimuthSignBin(
        (float)target80bit->Fields.azimuth, pcMetadata->azimuth);
    targetGeneric.doppler = RAF_COM_CALC_CalcDopplerSignBin(
        (float)target80bit->Fields.doppler, pcMetadata->doppler);
    targetGeneric.elevation = RAF_COM_CALC_CalcElevationSignBin(
        (float)target80bit->Fields.elevation, pcMetadata->elevation);
    targetGeneric.phase = 0;
    targetGeneric.power = (float)target80bit->Fields.power;
    targetGeneric.range = (float)target80bit->Fields.range;
  }

  return targetGeneric;
}

uint32_t RAF_COM_CALC_GetBytesLengthOfTraget(uint32_t unTargetFormat) {
  uint32_t bytesPerTarget = 0;

  switch ((((unTargetFormat) >> 6) & 0x3)) {
  case UTILS_PC_IF_CFG_BITS_PER_TARGET_56: {
    bytesPerTarget = BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES;
    break;
  }
  case UTILS_PC_IF_CFG_BITS_PER_TARGET_48: {
    bytesPerTarget = BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES;
    break;
  }
  case UTILS_PC_IF_CFG_BITS_PER_TARGET_96: {
    bytesPerTarget = BIN_DATA_WITH_PHASE_SIZE_BYTES;
    break;
  }
  case UTILS_PC_IF_CFG_BITS_PER_TARGET_80: {
    bytesPerTarget = BIN_DATA_NO_PHASE_SIZE_BYTES;
    break;
  }
  default: {
    break;
  }
  }

  return bytesPerTarget;
}

/*****************************************************************************
 *	Float2Fixed - convert floating to fixed point
 ** @params
 *		Input
 *			floatingNum
 *			fractionalBits - num of bits right of the fixed point
 *
 *		Output
 *			None
 *
 * @return		 fixed point value
 *matchedPCList
 * @note		None.
 *
 ******************************************************************************/
uint16_t Float2Fixed(float floatingNum, uint8_t fractionalBits) {
  float temp =
      floatingNum *
      (1 << fractionalBits); // floating num * 2^(num of bits on the right side)
  return (uint16_t)temp;
}

/*****************************************************************************
 *	Float2Fixed - convert floating to fixed point
 ** @params
 *		Input
 *			floatingNum
 *			fractionalBits - num of bits right of the fixed point
 *
 *		Output
 *			None
 *
 * @return		 fixed point value
 *matchedPCList
 * @note		None.
 *
 ******************************************************************************/
float Fixed2Float(uint16_t fixedNum, uint8_t fractionalBits) {
  uint8_t isNeg = 0;
  float result;
  if (fixedNum & 0x8000) {
    isNeg = 1;
    fixedNum = -fixedNum;
  }
  result = ((float)fixedNum) / (1 << fractionalBits);
  result = (isNeg) ? -result : result;
  return result;
}

/*****************************************************************************
 *	Float2Fixed - convert 32-bit fixed point to float
 ** @params
 *		Input
 *			fixedNum
 *			fractionalBits - num of bits right of the fixed point
 *
 *		Output
 *			None
 *
 * @return		 floating point value
 *matchedPCList
 * @note		None.
 *
 ******************************************************************************/
float Fixed32toFloat(int32_t fixedNum, uint8_t fractionalBits) {

  uint8_t isNeg = 0;
  float result;
  if (fixedNum & 0x80000000) {
    isNeg = 1;
    fixedNum = -fixedNum;
  }
  result = ((float)fixedNum) / (1 << fractionalBits);
  result = (isNeg) ? -result : result;
  return result;
}

/*****************************************************************************
 *	Float2Fixed - convert 64-bit fixed point to float
 ** @params
 *		Input
 *			fixedNum
 *			fractionalBits - num of bits right of the fixed point
 *
 *		Output
 *			None
 *
 * @return		 floating point value
 *matchedPCList
 * @note		None.
 *
 ******************************************************************************/
float Fixed64toFloat(int64_t fixedNum, uint8_t fractionalBits) {
  uint8_t isNeg = 0;
  float result;
  if (fixedNum & 0x8000000000000000) {
    isNeg = 1;
    fixedNum = -fixedNum;
  }
  result = ((float)fixedNum) / (1 << fractionalBits);
  result = (isNeg) ? -result : result;
  return result;
}

#ifdef __cplusplus
}
#endif
