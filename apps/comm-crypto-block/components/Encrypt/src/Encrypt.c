#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>
#include <stdlib.h>
#include <utils/util.h>

#include "gec.h"
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

static int encrypt_to_frame(CipherTextFrame_t *ret_ct_frame,
                            const mavlink_message_t *msg) {
  uint8_t buf[GEC_PT_LEN];

  mavlink_msg_to_send_buffer(buf, msg);

  ret_ct_frame->magic = GEC_CIPHERTEXT_FRAME_MAGIC;
  ret_ct_frame->tag = GEC_CIPHERTEXT_FRAME_TAG;

  memset(ret_ct_frame->ciphertext, 0, sizeof(GEC_CT_LEN));

  if (gec_encrypt(&symkey_chan2, buf, ret_ct_frame->ciphertext)) {
    LOG_ERROR("Encrypt failed");
    return 1;
  }

  return 0;
}

static void handle_char(uint8_t c) {
  // ZF_LOGE("%02X", c);
  mavlink_message_t msg;
  mavlink_status_t status;
  int result;
  // uint8_t buf[MAVLINK_FRAME_LEN];
  // uint16_t len;
  CipherTextFrame_t ct_frame;

  result = my_mavlink_parse_char(c, &msg, &status);
  if (result) {
    LOG_ERROR("Message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d",
              msg.seq, msg.msgid, msg.sysid, msg.compid);

    // len = mavlink_msg_to_send_buffer(buf, &msg);

    // if (ps_cdev_write(serial, buf, len, NULL, NULL) != len) {
    //   LOG_ERROR("Write not completed");
    // }

    if (!encrypt_to_frame(&ct_frame, &msg)) {
      if (ps_cdev_write(serial, &ct_frame, sizeof(ct_frame), NULL, NULL) !=
          sizeof(ct_frame)) {
        LOG_ERROR("Write not completed");
      }
    }
  }
}

void pre_init() {
  LOG_ERROR("In pre_init");

  int error;
  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(TELEMETRY_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  gec_init_sym_key_conf_auth(&symkey_chan2, key_material + GEC_RAW_KEY_LEN);

  LOG_ERROR("Out pre_init");
}

int run(void) {
  LOG_ERROR("In run");

  ring_buffer_t *ringbuffer = (ring_buffer_t *)ring_buffer;
  uint32_t head, tail;

  queue_t queue;
  queue_init(&queue);

  CipherTextFrame_t ct_frame;
  mavlink_message_t msg;
  mavlink_heartbeat_t hb;
  uint8_t buf[GEC_PT_LEN];

  hb.custom_mode = 0;
  hb.type = MAV_TYPE_QUADROTOR;
  hb.system_status = 0;
  hb.mavlink_version = 3;
  ct_frame.magic = GEC_CIPHERTEXT_FRAME_MAGIC;
  ct_frame.tag = GEC_CIPHERTEXT_FRAME_TAG;

  while (1) {
    mavlink_msg_heartbeat_encode(1, 0, &msg, &hb);
    memset(buf, 0, sizeof(buf));
    mavlink_msg_to_send_buffer(buf, &msg);

    for (int i = 0; i < sizeof(buf); i++) {
      enqueue(&queue, buf[i]);
    }

    int loop = queue.size / GEC_PT_LEN;

    LOG_ERROR("Encrypt total messages: %d", loop);
    for (int i = 0; i < loop; i++) {
      for (int j = 0; j < GEC_PT_LEN; j++) {
        uint8_t c;
        dequeue(&queue, &c);
        buf[j] = c;
      }
      if (gec_encrypt(&symkey_chan2, buf, ct_frame.ciphertext) != GEC_SUCCESS) {
        LOG_ERROR("Encrypt failed");
      } else {
        // puts("Encrypt success");
        LOG_ERROR("Encrypt %d success", i + 1);
        if (ps_cdev_write(serial, &ct_frame, sizeof(ct_frame), NULL, NULL) !=
            sizeof(ct_frame)) {
          LOG_ERROR("Write not completed");
        }
      }
    }

    LOG_ERROR("Queue rest size: %d", queue.size);

    while (1) {
      int r;
      r = rand();
      if (r != 0 && r % 1145 == 0) {
        break;
      }
    }
  }

  LOG_ERROR("Out run");
  return 0;
}
