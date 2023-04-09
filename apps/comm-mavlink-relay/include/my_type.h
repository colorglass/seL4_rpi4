#ifndef __MY_TYPE_H__
#define __MY_TYPE_H__

#include <stdint.h>

#define RING_BUFFER_SIZE 2048
typedef struct ring_buffer {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
} ring_buffer_t;

#endif
