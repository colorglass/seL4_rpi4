#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

static void handle_char(uint8_t c) {
    ZF_LOGE("%02X", c);
    // ps_cdev_putchar(serial, '0');
    // ps_cdev_putchar(serial, ':');
    // ps_cdev_putchar(serial, c);
    // ps_cdev_putchar(serial, '\n');
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
    //         .number = UART0_IRQ,
    //     }
    // };
    // irq_id_t serial_irq_id = ps_irq_register(&io_ops.irq_ops, irq_info, serial_irq_handle, NULL);
    // ZF_LOGF_IF(serial_irq_id < 0, "Failed to register irq");
}

int run(void) {
    ZF_LOGE("In run");

    while (1) {
        // int c = EOF;
        // while ((c = ps_cdev_getchar(serial)) == EOF) {

        // }
        // ps_cdev_putchar(serial, c);
        // ps_cdev_putchar(serial, '\n');
        ring_buffer_t *ringbuffer = (ring_buffer_t *) rb;
        uint8_t head, tail;
        head = ringbuffer->head;
        rb_acquire();
        tail = ringbuffer->tail;
        rb_acquire();
        if (head != tail) {
            handle_char(ringbuffer->buffer[head]);
            rb_acquire();
            head = (head + 1) % sizeof(ringbuffer->buffer);
            ringbuffer->head = head;
            rb_release();
        }
    }
    return 0;
}
