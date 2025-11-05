/*
 * CommonBase.h
 *
 *  Created on: Jun 11, 2018
 *      Author: User
 */

#ifndef INC_COMMON_BASE_H_
#define INC_COMMON_BASE_H_

#ifdef CSHARP_DLL
#define STD_CALL __stdcall
#define DLL_EXPIMP __declspec(dllexport)
#else
#ifdef _WIN32
#define STD_CALL __stdcall
#define DLL_EXPIMP
#else
// ********** IMPORTANT  **************** //
// Do not change empty defines below!
#define STD_CALL
#define DLL_EXPIMP
#endif
#endif

#include "stdint.h"
#include "ver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARBE_API_COMMON_VERSION_MASK 0xFF
#define ARBE_API_COMMON_MAJOR_SHIFT 8

#define ARBE_API_COMMON_MAJOR_VER 2
#define ARBE_API_COMMON_MINOR_VER 1

#define ETH_NUM_OF_SUPPORTED_PORTS 2
#define TCP_BASE_PORT 6001
#define TCP_PORT_PC (TCP_BASE_PORT)
#define TCP_PORT_DSP (TCP_BASE_PORT + 1)

#define UDP_BASE_PORT 6003

#define ETH_PORT_TO_IDX(usPort) (usPort - TCP_BASE_PORT)

#define ETHERNET_DEST_PC (uint16_t) TCP_PORT_PC
#define ETHERNET_DEST_DSP (uint16_t) TCP_PORT_DSP

#define TCP_PACKET_LENGTH 1400

#define ETH_SANITY_MAX_PACKET_LEN 5000

#define PACKET_PREFIX 0xA55A
#define PACKET_PREFIX_SHIFT 16

#define LSB_TIME(x) ((x)&0x00000000ffffffff)
#define MSB_TIME(x) (((x)&0xffffffff00000000) >> 32)

typedef struct TArbeApiMailBox *ptArbeApiMailBox;

typedef uint32_t (*SendCmd)(int32_t unMsgID, uint8_t *pucBuffer, uint32_t unLen,
                            void *user_data);

typedef unsigned long (*SysCfg_GetSystemTimeMs)();

//////////////////////////////////////////////////////
/////////	  	String Arrays Info 	   		//////////
//////////////////////////////////////////////////////

#define API_PARAMS_NUM 5
#define API_PARAMS_MAX_STR_LEN 40
#define TEMP_MAX 32

typedef struct TParamStr {

  uint8_t aucdesc[API_PARAMS_MAX_STR_LEN];

} TParamStr;

//////////////////////////////////////////////////////
/////////	  		API Header  			//////////
//////////////////////////////////////////////////////

typedef struct TRAF_API_Header {
  uint16_t usPrefix;
  uint16_t usType;
  uint32_t unLength;
  uint32_t unTimeLsb;
  uint32_t unTimeMsb;
  uint32_t unMessageNumber;

} TRAF_API_Header, *PTRAF_API_Header;

//////////////////////////////////////////////////////
/////////	  		DSP Command Info 	   	//////////
//////////////////////////////////////////////////////

#define DSP_CMD_INFO_PARAM_NUM 15

typedef struct {

  uint32_t aunParams[DSP_CMD_INFO_PARAM_NUM];

} TDspCmdInfo;

typedef struct {

  TRAF_API_Header tHeader;
  TDspCmdInfo tDspCmdInfo;

} TDspCmd;

//////////////////////////////////////////////////////
/////////	  	Arbe MailBox 	   		//////////
//////////////////////////////////////////////////////

struct TArbeApiMailBox {

  SendCmd sendCmd;
  SysCfg_GetSystemTimeMs sysCfg_GetSystemTimeMs;
  uint32_t unMessageNumber; // Read Only , RAF_API increment this filed.
  void *user_data;          // Added by Sensrad
};

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_BASE_H_ */
