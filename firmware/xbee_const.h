/*
 * xbee_const.h - Header file that contains constants for doing 
 *   XBee serial communications.
 *
 * Author: Rick Coogle
 *
 */

#ifndef _XBEE_CONST_H_INCLUDED
#define _XBEE_CONST_H_INCLUDED

// Constants
// Packet buffer info - static buffer size,
// but we know from the XBee API definitions that
// we won't get things this large.
#define XBEE_BUF_SIZE        512
#define XBEE_MIN_READ_SIZE   3

// XBee stuff
#define XBEE_MAGIC           0x7E
#define XBEE_BROADCAST_ADDR  0xFFFF

// API identifiers
#define XBEE_FN_STATUS            0x8A
#define XBEE_FN_ATCMD             0x08
#define XBEE_FN_QUEUE_PARAM       0x09
#define XBEE_FN_ATCMD_RESP        0x88
#define XBEE_FN_REMOTE_ATCMD      0x17
#define XBEE_FN_REMOTE_ATCMD_RESP 0x97
#define XBEE_FN_XMIT_ADDR64       0x00
#define XBEE_FN_XMIT_ADDR16       0x01
#define XBEE_FN_XMIT_STATUS       0x89
#define XBEE_FN_RECV_ADDR64       0x80
#define XBEE_FN_RECV_ADDR16       0x81

// Packet options
#define XBEE_PAK_OPT_RESERVED     0x01
#define XBEE_PAK_OPT_ADDR_BCAST   0x02
#define XBEE_PAK_OPT_PAN_BCAST    0x04
// (other bits are reserved)

// Macros
#define XBEE_PAK_LEN(buffer)      (((buffer[1] << 8) | buffer[2]) + 4)

// Typedefs

// Definition of an XBee API packet
typedef struct _XBEE_API_PAK {
	unsigned char uMagic;        // Magic number (byte 1)
	unsigned short uLength;      // Total packet length
	unsigned char uRssi;         // RSSI value (if applicable)
	unsigned char uOptions;      // Option flags (if applicable)
	unsigned short uAddr16;      // Address (if applicable)
	unsigned short uStartOfData; // Index that starts the actual data.
	unsigned char uFuncId;       // API function identifier
	unsigned char uHeaderLen;    // Header buffer length
	unsigned char* puHeaderBuf;  // Header data buffer
	unsigned char uDataLen;      // Data buffer length
	unsigned char* puDataBuf;    // Data buffer
	unsigned char uChecksum;     // Checksum
} XBEE_API_PAK;


#endif /* _XBEE_CONST_H_INCLUDED */
