#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <platsupport/io.h>
#include <utils/util.h>

#include <top_types.h>

#define send_wait() \
    do { \
        if (uart_send_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define send_post() \
    do { \
        if (uart_send_post()) { \
            LOG_ERROR("[%s] failed to post\n", get_instance_name()); \
        } \
    } while (0)

#define recv_wait() \
    do { \
        if (uart_recv_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_post() \
    do { \
        if (uart_recv_post()) { \
            LOG_ERROR("[%s] failed to post\n", get_instance_name()); \
        } \
    } while (0)

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

void pre_init() {
    int error;
    ring_buffer_t *rb;

    LOG_ERROR("Starting UART");
    LOG_ERROR("In pre_init");

    error = camkes_io_ops(&io_ops);
    ZF_LOGF_IF(error, "Failed to initialise IO ops");

    serial = ps_cdev_init(BCM2xxx_UART5, &io_ops, &serial_device);
    ZF_LOGF_IF(!serial, "Failed to initialise char device");
    
    rb = (ring_buffer_t *) ring_buffer;
    rb->head = 0;
    ring_buffer_release();
    rb->tail = 0;
    ring_buffer_release();

    LOG_ERROR("Out pre_init");
}

static inline void uart_rx_poll(void) {
    int c;
    uint8_t tail;
    ring_buffer_t *rb = (ring_buffer_t *) ring_buffer;

    while ((c = ps_cdev_getchar(serial)) == EOF) {

    }
    tail = rb->tail;
    ring_buffer_acquire();

    rb->buffer[tail] = c;
    ring_buffer_release();

    // tail = (tail + 1) % RING_BUFFER_SIZE;
    tail++; // uint8_t saves modular operation
    rb->tail = tail;
    ring_buffer_release();
}

int run(void) {
    LOG_ERROR("In run");

    while (1) {
        uart_rx_poll();
    }
    
    return 0;
}
