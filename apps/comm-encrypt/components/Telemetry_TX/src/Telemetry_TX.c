#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"
#include "mavlink/v2.0/mavlink_types.h"
#include "my_type.h"
#include "utils/util.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

static mavlink_message_t mavlink_message_rx_buffer;
static mavlink_status_t mavlink_status;

static uint8_t my_mavlink_parse_char(uint8_t c, mavlink_message_t *r_message,
                                     mavlink_status_t *r_mavlink_status) {
  uint8_t msg_received =
      mavlink_frame_char_buffer(&mavlink_message_rx_buffer, &mavlink_status, c,
                                r_message, r_mavlink_status);
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
  uint8_t buf[300];
  uint32_t len;

  if (result) {
    LOG_ERROR("Message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d",
              msg.seq, msg.msgid, msg.sysid, msg.compid);

    len = mavlink_msg_to_send_buffer(buf, &msg);
    if (ps_cdev_write(serial, buf, len, NULL, NULL) != len) {
      LOG_ERROR("Write not finished");
    }
  }
}

void pre_init() {
  ZF_LOGE("In pre_init");
  int error;
  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(TELEMETRY_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGE("Failed to initialise char device");
  } else {
    ZF_LOGI("Initialised char device");
  }
}

int run(void) {
  ZF_LOGE("In run");

  while (1) {
    ring_buffer_t *ringbuffer = (ring_buffer_t *)ring_buffer;
    uint32_t head, tail;
    head = ringbuffer->head;
    ring_buffer_acquire();
    tail = ringbuffer->tail;
    ring_buffer_acquire();
    if (head != tail) {
      handle_char(ringbuffer->buffer[head]);
      ring_buffer_acquire();
      head = (head + 1) % sizeof(ringbuffer->buffer);
      ringbuffer->head = head;
      ring_buffer_release();
    }
  }
  return 0;
}
