#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>
#include <utils/util.h>

#include "gec.h"
#include "mavlink/v2.0/ardupilotmega/mavlink.h"
#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"
#include "mavlink/v2.0/mavlink_types.h"
#include "my_type.h"


static mavlink_message_t mavlink_message_rx_buffer;
static mavlink_status_t mavlink_status;

static deque_t deque;

static uint8_t key_material[] = {
    0xCB, 0x28, 0x4A, 0xD9, 0x1E, 0x85, 0x78, 0xB1, 0x77, 0x6E, 0x9B, 0x98,
    0x32, 0xEF, 0x11, 0xB0, 0xBC, 0xA8, 0xCF, 0xD6, 0x29, 0x98, 0xDA, 0x15,
    0x43, 0x82, 0xC5, 0xAC, 0x4C, 0xB9, 0x58, 0xC5, 0x57, 0x0A, 0x4E, 0x30,
    0xCC, 0xED, 0xFE, 0xF7, 0x76, 0xF7, 0xC7, 0x75, 0x0C, 0x53, 0xA9, 0xE5,
};
static struct gec_sym_key symkey_chan1;
// static struct gec_sym_key symkey_chan2;

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

void pre_init() {
  LOG_ERROR("In pre_init");

  // gec_key_material_to_2_channels(&symkey_chan1, &symkey_chan2, key_material);
  gec_init_sym_key_conf_auth(&symkey_chan1, key_material);

  deque_init(&deque);

  LOG_ERROR("Out pre_init");
}

static int decrypt_to_msg(const CipherTextFrame_t *ct_frame,
                          mavlink_message_t *ret_msg) {
  if (ct_frame->magic != GEC_CIPHERTEXT_FRAME_MAGIC) {
    LOG_ERROR("MAGIC not match: 0x%02X", ct_frame->magic);
    return 1;
  }
  if (ct_frame->tag != GEC_CIPHERTEXT_FRAME_TAG) {
    LOG_ERROR("TAG not match: 0x%02X", ct_frame->tag);
    return 1;
  }

  uint8_t pt[GEC_PT_LEN];
  if (gec_decrypt(&symkey_chan1, ct_frame->ciphertext, pt)) {
    LOG_ERROR("Decrypt failed");
    return 1;
  }

  mavlink_message_t msg;
  mavlink_status_t status;
  int result;

  for (int i = 0; i < GEC_PT_LEN; i++) {
    result = my_mavlink_parse_char(pt[i], &msg, &status);
    if (result) {
      LOG_ERROR("Message: [SEQ]: %03d, [MSGID]: %03d, [SYSID]: %03d, [COMPID]: %03d",
                msg.seq, msg.msgid, msg.sysid, msg.compid);
      *ret_msg = msg;
      break;
    }
  }

  return 0;
}

static inline void read_ringbuffer(void *buf, uint32_t len) {
  ring_buffer_t *ringbuffer = (ring_buffer_t *)ring_buffer;
  uint32_t head, tail;
  uint8_t *read_buf = (uint8_t *)buf;

  head = ringbuffer->head;
  ring_buffer_acquire();
  for (uint32_t i = 0; i < len;) {
    tail = ringbuffer->tail;
    ring_buffer_acquire();
    while (i < len && head != tail) {
      read_buf[i++] = ringbuffer->buffer[head];
      ring_buffer_acquire();
      // LOG_ERROR("CHAR: 0x%02X", read_buf[i-1]);
      head = (head + 1) % RING_BUFFER_SIZE;
    }
  }
  ringbuffer->head = head;
  ring_buffer_release();
}

int run(void) {
  LOG_ERROR("In run");

  CipherTextFrame_t ct_frame;
  uint8_t pt[GEC_PT_LEN];
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t buf[GEC_CT_LEN];
  uint32_t len;
  int result;
  uint8_t c;

  while (1) {
    read_ringbuffer(&ct_frame, sizeof(ct_frame));

    if (ct_frame.magic != GEC_CIPHERTEXT_FRAME_MAGIC) {
      LOG_ERROR("MAGIC not match: 0x%02X", ct_frame.magic);
      return 1;
    }
    if (ct_frame.tag != GEC_CIPHERTEXT_FRAME_TAG) {
      LOG_ERROR("TAG not match: 0x%02X", ct_frame.tag);
      return 1;
    }

    if (gec_decrypt(&symkey_chan1, ct_frame.ciphertext, pt)) {
      LOG_ERROR("Decrypt failed");
    } else {
      for (int i = 0; i < GEC_PT_LEN; i++) {
        deque_push_back(&deque, pt[i]);
      }
    }

    for (int i = 0; i < deque.size; i++) {
      deque_pop_front(&deque, &c);
      result = my_mavlink_parse_char(c, &msg, &status);
      if (result) {
        LOG_ERROR("Message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d",
                  msg.seq, msg.msgid, msg.sysid, msg.compid);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        // if (ps_cdev_write(serial, buf, len, NULL, NULL) != len) {
        //   LOG_ERROR("Write not completed");
        // }
        serial_send(buf, len);
      }
    }
  }

  LOG_ERROR("Out run");
  return 0;
}
