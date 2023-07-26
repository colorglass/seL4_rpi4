#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>
#include <stdlib.h>
#include <utils/util.h>

#include "gec.h"
#include "mavlink/v2.0/ardupilotmega/mavlink.h"
#include "mavlink/v2.0/checksum.h"
#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"
#include "mavlink/v2.0/mavlink_types.h"
#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

static mavlink_message_t mavlink_message_rx_buffer;
static mavlink_status_t mavlink_status;

static uint8_t key_material[] = {
    0xCB, 0x28, 0x4A, 0xD9, 0x1E, 0x85, 0x78, 0xB1, 0x77, 0x6E, 0x9B, 0x98,
    0x32, 0xEF, 0x11, 0xB0, 0xBC, 0xA8, 0xCF, 0xD6, 0x29, 0x98, 0xDA, 0x15,
    0x43, 0x82, 0xC5, 0xAC, 0x4C, 0xB9, 0x58, 0xC5, 0x57, 0x0A, 0x4E, 0x30,
    0xCC, 0xED, 0xFE, 0xF7, 0x76, 0xF7, 0xC7, 0x75, 0x0C, 0x53, 0xA9, 0xE5,
};
// static struct gec_sym_key symkey_chan1;
static struct gec_sym_key symkey_chan2;

static queue_t queue;

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

static int encrypt_to_frame(const mavlink_message_t *msg) {
  CipherTextFrame_t ct_frame;
  uint8_t buf[MAVLINK_MAX_FRAME_LEN];
  int len;

  len = mavlink_msg_to_send_buffer(buf, msg);

  for (int i = 0; i < len; i++) {
    enqueue(&queue, buf[i]);
  }

  uint32_t loop = queue.size / GEC_PT_LEN;

  ct_frame.magic = GEC_CIPHERTEXT_FRAME_MAGIC;
  ct_frame.tag = GEC_CIPHERTEXT_FRAME_TAG;

  // LOG_ERROR("Encrypt total blocks: %d", loop);
  for (uint32_t i = 0; i < loop; i++) {
    for (int j = 0; j < GEC_PT_LEN; j++) {
      buf[j] = queue.raw_queue[queue.head++];
      queue.head %= MAX_QUEUE_SIZE;
    }
    queue.size -= GEC_PT_LEN;

    if (gec_encrypt(&symkey_chan2, buf, ct_frame.ciphertext) != GEC_SUCCESS) {
      LOG_ERROR("Failed to encrypt block %d", i);
    } else {
      // LOG_ERROR("Encrypted block %d", i);
      if (ps_cdev_write(serial, &ct_frame, sizeof(ct_frame), NULL, NULL) !=
          sizeof(ct_frame)) {
        LOG_ERROR("Write not completed");
      }
    }
  }

  // LOG_ERROR("Queue rest size: %d", queue.size);

  return 0;
}

static inline void handle_char(uint8_t c) {
  // ZF_LOGE("%02X", c);
  mavlink_message_t msg;
  mavlink_status_t status;
  int result;

  result = my_mavlink_parse_char(c, &msg, &status);
  if (result) {
    if (result != MAVLINK_FRAMING_OK) {
      LOG_ERROR(
          "Message: [SEQ]: %03d, [MSGID]: %03d, [SYSID]: %03d, [COMPID]: %03d",
          msg.seq, msg.msgid, msg.sysid, msg.compid);
    }

    encrypt_to_frame(&msg);
  }
}

void pre_init() {
  // LOG_ERROR("In pre_init");

  int error;
  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(TELEMETRY_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  gec_init_sym_key_conf_auth(&symkey_chan2, key_material + GEC_RAW_KEY_LEN);

  queue_init(&queue);

  // LOG_ERROR("Out pre_init");
}

int run(void) {
  // LOG_ERROR("In run");

  ring_buffer_t *ringbuffer = (ring_buffer_t *)ring_buffer;
  uint32_t head, tail;

  head = ringbuffer->head;
  ring_buffer_acquire();
  while (1) {
    tail = ringbuffer->tail;
    ring_buffer_acquire();
    while (head != tail) {
      handle_char(ringbuffer->buffer[head]);
      ring_buffer_acquire();
      head = (head + 1) % sizeof(ringbuffer->buffer);
    }
  }

  // LOG_ERROR("Out run");
  return 0;
}
