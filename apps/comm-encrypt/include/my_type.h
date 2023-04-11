#ifndef __MY_TYPE_H__
#define __MY_TYPE_H__

#include <stdint.h>
#include "../GEC/include/gec.h"


#define UART_PORT_NUMBER        PS_SERIAL5
#define TELEMETRY_PORT_NUMBER   PS_SERIAL3


#define RING_BUFFER_SIZE 2048
typedef struct ring_buffer {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
} ring_buffer_t;


#define MAVLINK_FRAME_LEN 280

typedef struct MAVLink_msg_frame {
    uint8_t frame[MAVLINK_FRAME_LEN];
    uint32_t len;
} MAVLink_msg_frame_t;

/**
 * GEC related
 */
#define GEC_CIPHERTEXT_FRAME_MAGIC 0x7e
#define GEC_CIPHERTEXT_FRAME_TAG 0
typedef struct CipherTextFrame {
    uint8_t magic;
    uint8_t tag;
    uint8_t ct[GEC_CT_LEN];
} CipherTextFrame_t;

#endif