#pragma once

#ifndef __MY_TYPE_H__
#define __MY_TYPE_H__

#include <stdint.h>
#include "gec.h"
#include <utils/util.h>


#define TELEMETRY_PORT_NUMBER   PS_SERIAL3
#define UART_PORT_NUMBER        PS_SERIAL5


#define RING_BUFFER_SIZE (2048)
typedef struct ring_buffer {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
} ring_buffer_t;


#define MAVLINK_MAX_FRAME_LEN 280


typedef struct pixhawk_buf {
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
} pixhawk_buf_t;

typedef struct telemetry_buf {
    uint8_t buf[2 + GEC_CT_LEN];
} telemetry_buf_t;


/**
 * GEC related
 */
#define GEC_CIPHERTEXT_FRAME_MAGIC 0x7e
#define GEC_CIPHERTEXT_FRAME_TAG 0

typedef struct CipherTextFrame {
    uint8_t magic;
    uint8_t tag;
    uint8_t ciphertext[GEC_CT_LEN];
} PACKED CipherTextFrame_t;

/*
 * Queue
 */
#define MAX_QUEUE_SIZE 4096
typedef struct _queue {
    uint8_t raw_queue[MAX_QUEUE_SIZE];
    uint32_t head;
    uint32_t size;
} queue_t;

#define queue_init(q) \
    ({ \
        memset((q)->raw_queue, -1, MAX_QUEUE_SIZE); \
        (q)->head = 0; \
        (q)->size = 0; \
    })

#define queue_full(q) \
    ({ \
        (q)->size == MAX_QUEUE_SIZE; \
    })

#define queue_empty(q) \
    ({ \
        !(q)->size; \
    })

#define enqueue(q, x) \
    ({ \
        int _ret; \
        if (queue_full(q)) { \
            LOG_ERROR("Cannot enquque"); \
            _ret = -1; \
        } else {\
        uint32_t _index = ((q)->head + (q)->size) % MAX_QUEUE_SIZE; \
        (q)->raw_queue[_index] = x; \
        (q)->size++; \
        _ret = 0; \
        } \
        _ret; \
    })

#define dequeue(q, ret) \
    ({ \
        int _ret; \
        if (queue_empty(q)) { \
            LOG_ERROR("Cannot dequeue"); \
            _ret = -1; \
        } else { \
        *(ret) = (q)->raw_queue[(q)->head]; \
        (q)->head = ((q)->head + 1) % MAX_QUEUE_SIZE; \
        (q)->size--; \
        _ret = 0; \
        } \
        _ret; \
    })

#define print_queue(q) \
    ({ \
        char _str[MAX_QUEUE_SIZE * 3 + 1]; \
        char *_p = _str; \
        LOG_ERROR("Print queue:"); \
        uint32_t size = (q)->size; \
        for (uint32_t h = (q)->head, i = 0; i < size; i++, h = (h+1) % MAX_QUEUE_SIZE) { \
            sprintf(_p, "%02X ", (q)->raw_queue[h]); \
            _p += 3; \
        } \
        LOG_ERROR("%s", _str); \
    })

#define print_serial(s) { \
    for (int i=0; s[i]; i++) { \
        ps_cdev_putchar(serial, s[i]); \
    } \
}


#endif
