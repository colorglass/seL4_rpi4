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

static uint8_t seq_1, seq_255;

MAVLINK_HELPER uint8_t my_mavlink_frame_char_buffer(
    mavlink_message_t *rxmsg, mavlink_status_t *status, uint8_t c,
    mavlink_message_t *r_message, mavlink_status_t *r_mavlink_status) {

  status->msg_received = MAVLINK_FRAMING_INCOMPLETE;

  switch (status->parse_state) {
  case MAVLINK_PARSE_STATE_UNINIT:
  case MAVLINK_PARSE_STATE_IDLE:
    if (c == MAVLINK_STX) {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
      rxmsg->len = 0;
      rxmsg->magic = c;
      status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
      mavlink_start_checksum(rxmsg);
    } else if (c == MAVLINK_STX_MAVLINK1) {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
      rxmsg->len = 0;
      rxmsg->magic = c;
      status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
      mavlink_start_checksum(rxmsg);
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_STX:
    if (status->msg_received
/* Support shorter buffers than the
   default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
        || c > MAVLINK_MAX_PAYLOAD_LEN
#endif
    ) {
      status->buffer_overrun++;
      _mav_parse_error(status);
      status->msg_received = 0;
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
    } else {
      // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
      rxmsg->len = c;
      status->packet_idx = 0;
      mavlink_update_checksum(rxmsg, c);
      if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
        rxmsg->incompat_flags = 0;
        rxmsg->compat_flags = 0;
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
      } else {
        status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
      }
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_LENGTH:
    rxmsg->incompat_flags = c;
    if ((rxmsg->incompat_flags & ~MAVLINK_IFLAG_MASK) != 0) {
      // message includes an incompatible feature flag
      _mav_parse_error(status);
      status->msg_received = 0;
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      break;
    }
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
    break;

  case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
    rxmsg->compat_flags = c;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
    break;

  case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
    // rxmsg->seq = c;
    // mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
    break;

  case MAVLINK_PARSE_STATE_GOT_SEQ:
    rxmsg->sysid = c;

    if (c == 1) {
      rxmsg->seq = seq_1++;
    } else if (c == 255) {
      rxmsg->seq = seq_255++;
    }
    mavlink_update_checksum(rxmsg, rxmsg->seq);

    mavlink_update_checksum(rxmsg, c);

    status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
    break;

  case MAVLINK_PARSE_STATE_GOT_SYSID:
    rxmsg->compid = c;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
    break;

  case MAVLINK_PARSE_STATE_GOT_COMPID:
    rxmsg->msgid = c;
    mavlink_update_checksum(rxmsg, c);
    if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
      if (rxmsg->len > 0) {
        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
      } else {
        status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
      }
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
      if (rxmsg->len < mavlink_min_message_length(rxmsg) ||
          rxmsg->len > mavlink_max_message_length(rxmsg)) {
        _mav_parse_error(status);
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        break;
      }
#endif
    } else {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_MSGID1:
    rxmsg->msgid |= c << 8;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
    break;

  case MAVLINK_PARSE_STATE_GOT_MSGID2:
    rxmsg->msgid |= ((uint32_t)c) << 16;
    mavlink_update_checksum(rxmsg, c);
    if (rxmsg->len > 0) {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
    } else {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
    }
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
    if (rxmsg->len < mavlink_min_message_length(rxmsg) ||
        rxmsg->len > mavlink_max_message_length(rxmsg)) {
      _mav_parse_error(status);
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      break;
    }
#endif
    break;

  case MAVLINK_PARSE_STATE_GOT_MSGID3:
    _MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
    mavlink_update_checksum(rxmsg, c);
    if (status->packet_idx == rxmsg->len) {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_PAYLOAD: {
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(rxmsg->msgid);
    uint8_t crc_extra = e ? e->crc_extra : 0;
    mavlink_update_checksum(rxmsg, crc_extra);
    if (c != (rxmsg->checksum & 0xFF)) {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
    } else {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
    }
    rxmsg->ck[0] = c;

    // zero-fill the packet to cope with short incoming packets
    if (e && status->packet_idx < e->max_msg_len) {
      memset(&_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx], 0,
             e->max_msg_len - status->packet_idx);
    }
    break;
  }

  case MAVLINK_PARSE_STATE_GOT_CRC1:
  case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
    if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 ||
        c != (rxmsg->checksum >> 8)) {
      // got a bad CRC message
      status->msg_received = MAVLINK_FRAMING_BAD_CRC;
    } else {
      // Successfully got message
      status->msg_received = MAVLINK_FRAMING_OK;
    }
    rxmsg->ck[1] = c;

    if (rxmsg->incompat_flags & MAVLINK_IFLAG_SIGNED) {
      status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;
      status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;

      // If the CRC is already wrong, don't overwrite msg_received,
      // otherwise we can end up with garbage flagged as valid.
      if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) {
        status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
      }
    } else {
      if (status->signing &&
          (status->signing->accept_unsigned_callback == NULL ||
           !status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {

        // If the CRC is already wrong, don't overwrite msg_received.
        if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) {
          status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
        }
      }
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (r_message != NULL) {
        memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
      }
    }
    break;
  case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
    rxmsg->signature[MAVLINK_SIGNATURE_BLOCK_LEN - status->signature_wait] = c;
    status->signature_wait--;
    if (status->signature_wait == 0) {
      // we have the whole signature, check it is OK
#ifndef MAVLINK_NO_SIGNATURE_CHECK
      bool sig_ok = mavlink_signature_check(status->signing,
                                            status->signing_streams, rxmsg);
#else
      bool sig_ok = true;
#endif
      if (!sig_ok &&
          (status->signing->accept_unsigned_callback &&
           status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {
        // accepted via application level override
        sig_ok = true;
      }
      if (sig_ok) {
        status->msg_received = MAVLINK_FRAMING_OK;
      } else {
        status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
      }
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (r_message != NULL) {
        memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
      }
    }
    break;
  }

  // If a message has been successfully decoded, check index
  if (status->msg_received == MAVLINK_FRAMING_OK) {
    // while(status->current_seq != rxmsg->seq)
    //{
    //	status->packet_rx_drop_count++;
    //               status->current_seq++;
    //}
    status->current_rx_seq = rxmsg->seq;
    // Initial condition: If no packet has been received so far, drop count is
    // undefined
    if (status->packet_rx_success_count == 0)
      status->packet_rx_drop_count = 0;
    // Count this packet as received
    status->packet_rx_success_count++;
  }

  if (r_message != NULL) {
    r_message->len =
        rxmsg->len; // Provide visibility on how far we are into current msg
  }
  if (r_mavlink_status != NULL) {
    r_mavlink_status->parse_state = status->parse_state;
    r_mavlink_status->packet_idx = status->packet_idx;
    r_mavlink_status->current_rx_seq = status->current_rx_seq + 1;
    r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
    r_mavlink_status->packet_rx_drop_count = status->parse_error;
    r_mavlink_status->flags = status->flags;
  }
  status->parse_error = 0;

  if (status->msg_received == MAVLINK_FRAMING_BAD_CRC) {
    /*
      the CRC came out wrong. We now need to overwrite the
      msg CRC with the one on the wire so that if the
      caller decides to forward the message anyway that
      mavlink_msg_to_send_buffer() won't overwrite the
      checksum
     */
    if (r_message != NULL) {
      r_message->checksum = rxmsg->ck[0] | (rxmsg->ck[1] << 8);
    }
  }

  return status->msg_received;
}

static void recalc_checksum(mavlink_message_t *msg,
                            const mavlink_status_t *status) {
  bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
#ifndef MAVLINK_NO_SIGN_PACKET
  bool signing = (!mavlink1) && status->signing &&
                 (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);
#else
  bool signing = false;
#endif
  uint8_t signature_len = signing ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
  uint8_t header_len = mavlink1 ? 6 : MAVLINK_CORE_HEADER_LEN + 1;

  uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
  buf[0] = msg->magic;
  buf[1] = msg->len;
  if (mavlink1) {
    buf[2] = msg->seq;
    buf[3] = msg->sysid;
    buf[4] = msg->compid;
    buf[5] = msg->msgid & 0xFF;
  } else {
    buf[2] = msg->incompat_flags;
    buf[3] = msg->compat_flags;
    buf[4] = msg->seq;
    buf[5] = msg->sysid;
    buf[6] = msg->compid;
    buf[7] = msg->msgid & 0xFF;
    buf[8] = (msg->msgid >> 8) & 0xFF;
    buf[9] = (msg->msgid >> 16) & 0xFF;
  }

  uint16_t checksum = crc_calculate(&buf[1], header_len - 1);
  crc_accumulate_buffer(&checksum, _MAV_PAYLOAD(msg), msg->len);

  uint8_t crc_extra = mavlink_get_crc_extra(msg);
  crc_accumulate(crc_extra, &checksum);
  mavlink_ck_a(msg) = (uint8_t)(checksum & 0xFF);
  mavlink_ck_b(msg) = (uint8_t)(checksum >> 8);
}

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

  // recalc_checksum(r_message, r_mavlink_status);

  return msg_received;
}

// static uint8_t my_mavlink_parse_char(uint8_t c, mavlink_message_t *r_message,
//                                      mavlink_status_t *r_mavlink_status) {
//   uint8_t msg_received =
//       mavlink_frame_char_buffer(&mavlink_message_rx_buffer, &mavlink_status,
//       c,
//                                 r_message, r_mavlink_status);
//   if (msg_received == MAVLINK_FRAMING_BAD_CRC) {
//     LOG_ERROR("MAVLink message parse error: Bad CRC");
//   } else if (msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
//     LOG_ERROR("MAVLink message parse error: Bad signature");
//   }

//   return msg_received;
// }

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
      uint8_t c;
      dequeue(&queue, &c);
      buf[j] = c;
    }
    if (gec_encrypt(&symkey_chan2, buf, ct_frame.ciphertext) != GEC_SUCCESS) {
      LOG_ERROR("Failed to encrypt block %d", i);
    } else {
      LOG_ERROR("Encrypted block %d", i);
      // if (ps_cdev_write(serial, &ct_frame, sizeof(ct_frame), NULL, NULL) !=
      //     sizeof(ct_frame)) {
      //   LOG_ERROR("Write not completed");
      // }
      serial_send((uint8_t *)&ct_frame, sizeof(ct_frame));
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
    LOG_ERROR(
        "Message: [SEQ]: %03d, [MSGID]: 0x%06X, [SYSID]: %03d, [COMPID]: %03d",
        msg.seq, msg.msgid, msg.sysid, msg.compid);

    encrypt_to_frame(&msg);
  }
}

void pre_init() {
  LOG_ERROR("In pre_init");

  gec_init_sym_key_conf_auth(&symkey_chan2, key_material + GEC_RAW_KEY_LEN);

  queue_init(&queue);

  LOG_ERROR("Out pre_init");
}

int run(void) {
  LOG_ERROR("In run");

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

  LOG_ERROR("Out run");
  return 0;
}
