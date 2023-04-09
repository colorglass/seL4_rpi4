#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "mavlink/v2.0/common/mavlink.h"
#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

static mavlink_message_t mavlink_message_rx_buffer;
static mavlink_status_t mavlink_status;

static uint8_t my_mavlink_parse_char(uint8_t c,
						   mavlink_message_t *r_message,
						   mavlink_status_t *r_mavlink_status)
{
	uint8_t msg_received = mavlink_frame_char_buffer(&mavlink_message_rx_buffer,
								&mavlink_status,
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

static void handle_char(uint8_t c) {
    // ZF_LOGE("%02X", c);
    mavlink_message_t msg;
    mavlink_status_t status;
    int result = my_mavlink_parse_char(c, &msg, &status);
    // int result = mavlink_parse_char(0, c, &msg, &status);
    if (result) {
        LOG_ERROR("Message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d", 
            msg.seq, msg.msgid, msg.sysid, msg.compid);
    }
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
        uint32_t head, tail;
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
