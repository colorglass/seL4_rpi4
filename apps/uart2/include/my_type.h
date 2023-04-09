#ifndef __MY_TYPE_H__
#define __MY_TYPE_H__

#include <stdint.h>
typedef struct ring_buffer {
    uint8_t buffer[2048];
    uint16_t head;
    uint16_t tail;
} ring_buffer_t;

#endif
