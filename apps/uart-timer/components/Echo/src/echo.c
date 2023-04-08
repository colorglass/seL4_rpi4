#include "utils/util.h"
#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

#include <stdint.h>
typedef struct ring_buffer {
    uint8_t buffer[0x10000];
    uint16_t head;
    uint16_t tail;
} ring_buffer_t;

static ring_buffer_t ringbuffer;

static void handle_char(uint8_t c) {
    ZF_LOGE("In handle_char %c", c);
    // char buf[8];
    // sprintf(buf, "3:%c\n\r", c);
    // ps_cdev_write(serial, buf, strlen(buf), NULL, NULL);
}

void serial_irq_handle(void *data, ps_irq_acknowledge_fn_t acknowledge_fn, void *ack_data) {
    int error;

    if (serial) {
        int c = 0;
        ps_cdev_handle_irq(serial, 0);
        while (c != EOF) {
            c = ps_cdev_getchar(serial);
            if (c != EOF) {
                handle_char((uint8_t) c);
            }
        }
    }

    error = acknowledge_fn(ack_data);
    ZF_LOGF_IF(error, "Failed to acknowledge IRQ");
}

// static ps_io_ops_t io_ops2;
// static ps_chardevice_t serial_device2;
// static ps_chardevice_t *serial2 = NULL;

// static void handle_char2(uint8_t c) {
//     ZF_LOGE("In handle_char2 %c", c);
// }

// void serial_irq_handle2(void *data, ps_irq_acknowledge_fn_t acknowledge_fn, void *ack_data) {
//     int error;

//     if (serial) {
//         int c = 0;
//         ps_cdev_handle_irq(serial, 0);
//         while (c != EOF) {
//             c = ps_cdev_getchar(serial);
//             if (c != EOF) {
//                 handle_char2((uint8_t) c);
//             }
//         }
//     }

//     error = acknowledge_fn(ack_data);
//     ZF_LOGF_IF(error, "Failed to acknowledge IRQ");
// }


void timer_complete_callback(void *arg) {
    // static uint32_t counter = 0;
    // LOG_ERROR("Counter: %u", counter++);
    int c = EOF;
    if ((c = ps_cdev_getchar(serial)) != EOF) {
        ringbuffer.buffer[ringbuffer.tail++] = c;
    }
    // timer_complete_reg_callback(timer_complete_callback, NULL);
}

void pre_init() {
    ZF_LOGE("In pre_init");
    int error;
    error = camkes_io_ops(&io_ops);
    ZF_LOGF_IF(error, "Failed to initialise IO ops");

    serial = ps_cdev_init(BCM2xxx_UART5, &io_ops, &serial_device);
    if (serial == NULL) {
        ZF_LOGE("Failed to initialise char device");
    } else {
        ZF_LOGI("Initialised char device");
    }

    // ps_irq_t irq_info = {
    //     .type = PS_INTERRUPT,
    //     .irq = {
    //         .number = UART5_IRQ,
    //     }
    // };
    // irq_id_t serial_irq_id = ps_irq_register(&io_ops.irq_ops, irq_info, serial_irq_handle, NULL);
    // ZF_LOGF_IF(serial_irq_id < 0, "Failed to register irq");

    ////
    // ZF_LOGE("In pre_init 2");
    // error = camkes_io_ops(&io_ops2);
    // ZF_LOGF_IF(error, "Failed to initialise IO ops");

    // serial2 = ps_cdev_init(BCM2xxx_UART5, &io_ops2, &serial_device2);
    // if (serial2 == NULL) {
    //     ZF_LOGE("Failed to initialise char device");
    // } else {
    //     ZF_LOGI("Initialised char device");
    // }

    // ps_irq_t irq_info2 = {
    //     .type = PS_INTERRUPT,
    //     .irq = {
    //         .number = UART5_IRQ,
    //     }
    // };
    // irq_id_t serial_irq_id2 = ps_irq_register(&io_ops2.irq_ops, irq_info2, serial_irq_handle2, NULL);
    // ZF_LOGF_IF(serial_irq_id2 < 0, "Failed to register irq");

    ringbuffer.head = ringbuffer.tail = 0;

    timer_periodic(0, 1);
}

// void timer_complete__init() {
//     timer_complete_reg_callback(timer_complete_callback, NULL);
// }

int run(void) {
    ZF_LOGE("In run");

    while (1) {
        if (ringbuffer.head != ringbuffer.tail) {
            LOG_ERROR("%02X", ringbuffer.buffer[ringbuffer.head++]);
        }
    }
    return 0;
}
