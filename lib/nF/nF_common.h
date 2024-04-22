/****************************************************************************
 * Copyright (C) 2017 by nanoFaktur GmbH                                    *
 *                                                                          *
 * All right reserved.                                                      *
 *                                                                          *
 ****************************************************************************/

/**
 * @file nF_common.h
 * @author Cunman Zhang
 * @date 08 Sep. 2017
 * @brief common header for nF controllers.
 *
 * common header definition for nanoFaktur controllers  
 * @see http://www.nanoFaktur.com
 */
#ifndef _nF_COMMON_H_
#define _nF_COMMON_H_

#define BIT(n)                          (1<<(n))

//-----------------------------------------------------------------
// flags for command-package
//-----------------------------------------------------------------
#define CMD_PACKAGE_FLAG_RD             0       // when bit0 not set: get from controller
#define CMD_PACKAGE_FLAG_WR             BIT(0)  // when bit0 set: set to controller
#define CMD_PACKAGE_FLAG_RSP            BIT(4)  // Package is a response
#define CMD_PACKAGE_FLAG_ACK            BIT(5)  // for command: requires acknowledge, for response: means error (data[0] is error code)
#define CMD_PACKAGE_FLAG_FORWARD        BIT(6)  // Package will be forward to other interface
#define CMD_PACKAGE_FLAG_MORE           BIT(7)  // Package not end: more data follows (only for response)

//-----------------------------------------------------------------
// command data format
//-----------------------------------------------------------------
#define CMD_DATA_FMT_U8                 0   // unsigned char (Byte)
#define CMD_DATA_FMT_U32                1   // unsigned int (D-Word)
#define CMD_DATA_FMT_FLOAT              2   // float
#define CMD_DATA_FMT_DOUBLE             3   // double
#define CMD_DATA_FMT_STRING             4   // string
#define CMD_DATA_FMT_S32                5   // int
#define CMD_DATA_FMT_CHAR               6   // char
#define CMD_DATA_FMT_U16                7   // unsigned short (Word)
#define CMD_DATA_FMT_S16                8   // short
#define CMD_DATA_FMT_ARRAY              9   // array of data: (fmt_array, ArrSize, dataFmt, data[])
#define CMD_DATA_FMT_LF                 10  // for host software: show a line-feed
#define CMD_DATA_FMT_INVALID            0xFF

#ifndef u8
typedef    unsigned char     u8;
#endif

#ifndef u16
typedef   unsigned short     u16;
#endif

#ifndef u32
typedef   unsigned long      u32;
#endif

#ifndef _WINDOWS
#ifndef u64
typedef   unsigned long long u64;
#endif		   
#endif

// definition of command package
#ifndef _WINDOWS
typedef struct __attribute__((packed)) _CMD_PACKAGE_ {
#else
typedef struct _CMD_PACKAGE_ {
#endif
    u16         len;        // total package length
    u16         CmdId;      // command ID
    u16         CustomId;   // value will be just returned (can be used for software-routinue)
    u8          opt;        // option: e.g. slave should acknowledge, CRC or check-sum, etc.
    u8          seq;        // sequence number: set by controller for response
    u8          IntfId;     // interface ID: set by controller
    u8          ChkSumHeader; // check-sum for the header
    u8          *pParam;    // parameters
    u8          ChkSumData; // check-sum for parameters
}CMD_PACKAGE;

#endif // #ifndef _nF_COMMON_H_
