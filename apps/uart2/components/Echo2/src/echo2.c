#include "mavlink/v2.0/mavlink_types.h"
#include <camkes.h>
#include <camkes/io.h>
#include <camkes/dma.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "my_type.h"
#include "utils/attribute.h"
#include "utils/util.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

#include "mavlink/v2.0/common/mavlink.h"
static mavlink_message_t g_mavlink_message_rx_buffer;
static mavlink_status_t g_mavlink_status;

static uint8_t my_mavlink_parse_char(uint8_t c,
						   mavlink_message_t *r_message,
						   mavlink_status_t *r_mavlink_status)
{
	uint8_t msg_received = mavlink_frame_char_buffer(&g_mavlink_message_rx_buffer,
								&g_mavlink_status,
								c,
								r_message,
								r_mavlink_status);
	if (msg_received == MAVLINK_FRAMING_BAD_CRC) {
		LOG_ERROR("MAVLink message parse error: Bad CRC");
	} else if (msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
		LOG_ERROR("MAVLink message parse error: Bad signature");
	}
	
	return msg_received;
}

typedef volatile struct pl011_regs_s {
    uint32_t dr;            // 0x00: data register
    uint32_t rsrecr;        // 0x04: receive status/error clear register
    uint64_t unused0[2];    // 0x08
    uint32_t fr;            // 0x18: flag register
    uint32_t unused1;       // 0x1c
    uint32_t ilpr;          // 0x20: not in use
    uint32_t ibrd;          // 0x24: integer baud rate divisor
    uint32_t fbrd;          // 0x28: fractional baud rate divisor
    uint32_t lcrh;          // 0x2c: line control register
    uint32_t cr;            // 0x30: control register
    uint32_t ifls;          // 0x34: interrupt FIFO level select register
    uint32_t imsc;          // 0x38: interrupt mask set clear register
    uint32_t ris;           // 0x3c: raw interrupt status register
    uint32_t mis;           // 0x40: masked interrupt status register
    uint32_t icr;           // 0x44: interrupt clear register
    uint32_t dmacr;         // 0x48: DMA control register
}
pl011_regs_t;

static inline pl011_regs_t *pl011_uart_get_priv(ps_chardevice_t *dev)
{
    return (pl011_regs_t *)(dev->vaddr);
}

static pl011_regs_t *regs = NULL;

static void handle_char(uint8_t c) {
    ZF_LOGE("%02X", c);
    // ring_buffer_t *ringbuffer = (ring_buffer_t *) rb;
    // uint8_t tail = ringbuffer->tail;
    // rb_acquire();
    // ringbuffer->buffer[tail++] = c;
    // rb_release();
    // ringbuffer->tail = tail;
    // rb_release();
    // mavlink_message_t msg;
    // mavlink_status_t status;
    // int result = my_mavlink_parse_char(c, &msg, &status);
    // if (result) {
    //     LOG_ERROR("Message");
    // }
}

void serial_irq_handle(void *data, ps_irq_acknowledge_fn_t acknowledge_fn, void *ack_data) {
    int error;

    if (serial) {
        int c = 0;
        ps_cdev_handle_irq(serial, 0);
        // while (c != EOF) {
            uint32_t buf[300];
            uint32_t len = ps_cdev_read(serial, buf, sizeof(buf), NULL, NULL);
            for (uint32_t i=0; i<len; i++) {
                handle_char(buf[i]);
            }
        // }
    }

    error = acknowledge_fn(ack_data);
    ZF_LOGF_IF(error, "Failed to acknowledge IRQ");
}

// uint8_t my_dma_pool[4096] ALIGN(4096);

void pre_init() {
    ZF_LOGE("In pre_init2");
    int error;
    error = camkes_io_ops(&io_ops);
    ZF_LOGF_IF(error, "Failed to initialise IO ops");

    serial = ps_cdev_init(BCM2xxx_UART5, &io_ops, &serial_device);
    if (serial == NULL) {
        ZF_LOGE("Failed to initialise char device");
    } else {
        ZF_LOGI("Initialised char device");
    }

    regs = pl011_uart_get_priv(serial);

    ps_irq_t irq_info = {
        .type = PS_INTERRUPT,
        .irq = {
            .number = UART5_IRQ,
        }
    };
    irq_id_t serial_irq_id = ps_irq_register(&io_ops.irq_ops, irq_info, serial_irq_handle, NULL);
    ZF_LOGF_IF(serial_irq_id < 0, "Failed to register irq");

    // ring_buffer_t *rb = (ring_buffer_t *) rb;
    // rb->head = 0;
    // rb_release();
    // rb->tail = 0;
    // rb_release();

    // if (camkes_dma_init(my_dma_pool, 4096, 4096, 0)) {
    //     ZF_LOGF("DMA init failed");
    // }
    // void *dma_ptr = camkes_dma_alloc(0x90, 0, 0);
    // if (!dma_ptr) {
    //     ZF_LOGF("DMA alloc failed");
    // }
    // for (int i=0; i<0x90; i++) {
    //     LOG_ERROR("%02X", ((char*)dma_ptr)[i]);
    // }
}

int run(void) {
    ZF_LOGE("In run");

    // pl011_regs_t *r = pl011_uart_get_priv(serial);
    while (1) {
        // int c = EOF;
        // while ((c = ps_cdev_getchar(serial)) == EOF) {
            
        // }
        // handle_char(c);
    //     // while ((c = ps_cdev_getchar(serial)) != EOF) {
    //     //     handle_char(c);
    //     // }
        // int ch = EOF;
        // while (!(r->fr & BIT(4))) {
        //     ch = (int)(r->dr);

        //     if (ch & 0xff00) {
        //         LOG_ERROR("ERROR: %04X", ch);
        //     }

        //     ch &= MASK(8);
        //     handle_char(ch);
        // }
    }
    return 0;
}
