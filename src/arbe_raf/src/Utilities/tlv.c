#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TLV_MINIMAL_RECORD_LENGTH 3
int TLV_DecodeTLVbyIndex(uint8_t *buffer, uint16_t index, uint16_t len,
                         uint16_t *usTag, uint16_t *usLength,
                         uint8_t **ucDataPtr) {
  uint16_t usCurrTag;
  uint16_t usCurrLength;
  uint16_t usNextTagIndex;
  uint8_t *startPtr = buffer + index;
  uint8_t *currPtr = buffer + index;
  uint8_t *endPtr = startPtr + len;

  // sanity check
  if (len < TLV_MINIMAL_RECORD_LENGTH) {
    return -1;
  }

  // decode tag
  usCurrTag = *currPtr++;
  if (usCurrTag & 0x1F) // 2-bytes tag
  {
    usCurrTag <<= 8;
    usCurrTag += *currPtr++;
  }

  // decode length
  //  assumming maximal length is 255
  usCurrLength = *currPtr++;
  if (usCurrLength > 0x7F) {
    usCurrLength = *currPtr++;
  }

  // check overflow
  if (currPtr + usCurrLength > endPtr) {
    return -1;
  }

#if 0
		// check length legallity
		if (!CheckTagLengthLegallity(usCurrTag, usCurrLength))
		{
			return -1;
		}
#endif

  // set results
  *usTag = usCurrTag;
  *usLength = usCurrLength;
  *ucDataPtr = currPtr;

  //    if ((currPtr + usCurrLength) == endPtr) //last tag
  //    {
  //        return 0;
  //    }
  usNextTagIndex = (uint16_t)((currPtr - startPtr) + usCurrLength);

  return usNextTagIndex;
}

int TLV_EncodeTLV(uint16_t usTag, uint8_t ucLength, uint8_t *data,
                  uint8_t *targetBuffer, uint16_t bufferLength,
                  uint8_t *ucTlvHeaderLength) {
  uint8_t *currPtr = targetBuffer;
  uint8_t tagFieldLength = ((usTag >> 8) & 0x1F) ? 2 : 1;
  uint8_t lengthFieldLength = (ucLength > 0x7F) ? 2 : 1;

  // sanity check
  if ((tagFieldLength + lengthFieldLength + ucLength) > bufferLength) {
    return -1;
  }

  // encode tag
  if (tagFieldLength == 2) // 2-bytes tag
  {
    *currPtr++ = usTag >> 8;
  }
  *ucTlvHeaderLength = tagFieldLength + lengthFieldLength;

  *currPtr++ = usTag & 0xFF;

  // encode length
  //  assumming maximal length is 255
  if (lengthFieldLength == 2) {
    *currPtr++ = 0x81;
  }
  *currPtr++ = ucLength;

  if (ucLength > 0x7F) {
    *currPtr++ = ucLength;
  }
  memcpy(currPtr, data, ucLength);

  return 0;
}

int TLV_DecodeTLbyIndex(uint8_t *buffer, uint16_t index, uint16_t len,
                        uint16_t *usTag, uint16_t *usLength) {
  uint16_t usCurrTag;
  uint16_t usCurrLength;
  uint16_t usNextTagIndex;
  uint8_t *startPtr = buffer + index;
  uint8_t *currPtr = buffer + index;
  uint8_t *endPtr = startPtr + len;

  // sanity check
  if (len < TLV_MINIMAL_RECORD_LENGTH) {
    return -1;
  }

  // decode tag
  usCurrTag = *currPtr++;
  if (usCurrTag & 0x1F) // 2-bytes tag
  {
    usCurrTag <<= 8;
    usCurrTag += *currPtr++;
  }

  // decode length
  //  assumming maximal length is 255
  usCurrLength = *currPtr++;
  if (usCurrLength > 0x7F) {
    usCurrLength = *currPtr++;
  }

#if 0
		// check length legallity
		if (!CheckTagLengthLegallity(usCurrTag, usCurrLength))
		{
			return -1;
		}
#endif

  // set results
  *usTag = usCurrTag;
  *usLength = usCurrLength;

  if ((currPtr + usCurrLength) == endPtr) // last tag
  {
    return 0;
  }

  usNextTagIndex = (uint16_t)(currPtr - startPtr);

  return usNextTagIndex;
}

int TLV_EncodeTL(uint16_t usTag, uint8_t ucLength, uint8_t *targetBuffer,
                 uint16_t bufferLength) {
  uint8_t *currPtr = targetBuffer;
  uint8_t tagFieldLength = ((usTag >> 8) & 0x1F) ? 2 : 1;
  uint8_t lengthFieldLength = (ucLength > 0x7F) ? 2 : 1;

  // sanity check
  if ((tagFieldLength + lengthFieldLength + ucLength) > bufferLength) {
    return -1;
  }

  // encode tag
  if (tagFieldLength == 2) // 2-bytes tag
  {
    *currPtr++ = usTag >> 8;
  }
  *currPtr++ = usTag & 0xFF;

  // encode length
  //  assumming maximal length is 255
  if (lengthFieldLength == 2) {
    *currPtr++ = 0x81;
  }
  *currPtr = ucLength;

  if (ucLength > 0x7F) {
    *currPtr++ = ucLength;
  }
  return 0;
}

#ifdef __cplusplus
extern "C"
}
#endif
