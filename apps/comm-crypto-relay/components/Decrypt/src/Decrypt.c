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

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

static mavlink_message_t mavlink_message_rx_buffer;
static mavlink_status_t mavlink_status;

static uint32_t head = 0;

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

static void parse_char_into_msg(uint8_t c) {
  // ZF_LOGE("%02X", c);
  mavlink_message_t msg;
  mavlink_status_t status;
  int result;
  uint8_t buf[MAVLINK_MAX_FRAME_LEN];
  uint16_t len;

  result = my_mavlink_parse_char(c, &msg, &status);
  if (result) {
    LOG_ERROR("Message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d",
              msg.seq, msg.msgid, msg.sysid, msg.compid);

    len = mavlink_msg_to_send_buffer(buf, &msg);

    if (ps_cdev_write(serial, buf, len, NULL, NULL) != len) {
      LOG_ERROR("Write not completed");
    }
  }
}

void pre_init() {
  // LOG_ERROR("In pre_init");

  int error;
  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(UART_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  // gec_key_material_to_2_channels(&symkey_chan1, &symkey_chan2, key_material);
  gec_init_sym_key_conf_auth(&symkey_chan1, key_material);

  // LOG_ERROR("Out pre_init");
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
      LOG_ERROR(
          "Message: [SEQ]: %03d, [MSGID]: %03d, [SYSID]: %03d, [COMPID]: %03d",
          msg.seq, msg.msgid, msg.sysid, msg.compid);
      *ret_msg = msg;
      break;
    }
  }

  return 0;
}

static inline uint8_t read_byte() {
  ring_buffer_t *ringbuffer = (ring_buffer_t *)ring_buffer;
  uint32_t tail;
  uint8_t c;

  do {
    tail = ringbuffer->tail;
    ring_buffer_acquire();
  } while (tail == head);

  c = ringbuffer->buffer[head++];
  ring_buffer_acquire();
  head %= RING_BUFFER_SIZE;

  return c;
}

static inline void read_buffer(void *buf, uint32_t len) {
  ring_buffer_t *ringbuffer = (ring_buffer_t *)ring_buffer;
  uint8_t *read_buf = buf;
  uint32_t tail;

  for (uint32_t i = 0; i < len;) {
    tail = ringbuffer->tail;
    ring_buffer_acquire();
    while (i < len && head != tail) {
      read_buf[i++] = ringbuffer->buffer[head++];
      ring_buffer_acquire();
      head %= RING_BUFFER_SIZE;
    }
  }
}

static inline void read_ciphertext_frame(CipherTextFrame_t *ct_frame) {
  ring_buffer_t *ringbuffer = (ring_buffer_t *)ring_buffer;
  uint32_t tail;
  uint8_t c;

  // Read header, in case of lost data
  while (1) {
    c = read_byte();
    if (c == GEC_CIPHERTEXT_FRAME_MAGIC) {
      c = read_byte();
      if (c == GEC_CIPHERTEXT_FRAME_TAG) {
        ct_frame->magic = GEC_CIPHERTEXT_FRAME_MAGIC;
        ct_frame->tag = GEC_CIPHERTEXT_FRAME_TAG;
        break;
      } else {
        continue;
      }
    }
  }

  // Read ciphertext
  read_buffer(ct_frame->ciphertext, sizeof(ct_frame->ciphertext));
}

int run(void) {
  // LOG_ERROR("In run");

  CipherTextFrame_t ct_frame;
  uint8_t pt[GEC_PT_LEN];
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t buf[GEC_CT_LEN];
  uint32_t len;
  int result;
  uint8_t c;

  while (1) {
    read_ciphertext_frame(&ct_frame);

    // if (ct_frame.magic != GEC_CIPHERTEXT_FRAME_MAGIC) {
    //   LOG_ERROR("MAGIC not match: 0x%02X", ct_frame.magic);
    //   return 1;
    // }
    // if (ct_frame.tag != GEC_CIPHERTEXT_FRAME_TAG) {
    //   LOG_ERROR("TAG not match: 0x%02X", ct_frame.tag);
    //   return 1;
    // }

    if (gec_decrypt(&symkey_chan1, ct_frame.ciphertext, pt)) {
      LOG_ERROR("Decrypt failed");
    } else {
      // for (int i = 0; i < GEC_PT_LEN; i++) {
      //   enqueue(&queue, pt[i]);
      // }
    }

    for (int i = 0; i < sizeof(pt); i++) {
      // dequeue(&queue, &c);
      c = pt[i];
      result = my_mavlink_parse_char(c, &msg, &status);
      if (result) {
        if (result != MAVLINK_FRAMING_OK) {
          LOG_ERROR(
              "Message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d",
              msg.seq, msg.msgid, msg.sysid, msg.compid);
        }
        len = mavlink_msg_to_send_buffer(buf, &msg);
        if (ps_cdev_write(serial, buf, len, NULL, NULL) != len) {
          LOG_ERROR("Write not completed");
        }
      }
    }
  }

  // LOG_ERROR("Out run");
  return 0;
}
