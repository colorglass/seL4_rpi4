#include <camkes.h>
#include <camkes/io.h>
#include <math.h>
#include <platsupport/chardev.h>
#include <stdint.h>
#include <stdlib.h>
#include <utils/util.h>

#include "gec-ke.h"
#include "gec.h"
#include "mavlink/v2.0/ardupilotmega/mavlink.h"
#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"
#include "mavlink/v2.0/mavlink_types.h"
#include "my_type.h"

#include "ed25519-donna.h"
#include "ed25519.h"

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

static void read_message(ring_buffer_t *ringbuffer, mavlink_message_t *r_msg) {
  uint32_t head, tail;
  mavlink_status_t status;
  int result;

  head = ringbuffer->head;
  ring_buffer_pixhawk_acquire();

  while (1) {
    tail = ringbuffer->tail;
    ring_buffer_pixhawk_acquire();
    while (head != tail) {
      result = my_mavlink_parse_char(ringbuffer->buffer[head], r_msg, &status);
      ring_buffer_pixhawk_acquire();
      if (result) {
        return;
      }
      head = (head + 1) % sizeof(ringbuffer->buffer);
    }
  }
}

static void read_from_telemetry(ring_buffer_t *ringbuffer, uint8_t *buf,
                                uint32_t len) {
  uint32_t head, tail;

  head = ringbuffer->head;
  ring_buffer_telemetry_acquire();

  for (int i = 0; i < len;) {
    tail = ringbuffer->tail;
    ring_buffer_telemetry_acquire();
    while (i < len && head != tail) {
      buf[i++] = ringbuffer->buffer[head];
      ring_buffer_telemetry_acquire();
      head = (head + 1) % sizeof(ringbuffer->buffer);
    }
  }
}

gec_sts_ctx_t ctx;
struct gec_privkey privkey;
struct gec_pubkey our_pubkey;
struct gec_pubkey their_pubkey;
uint8_t random_data[RANDOM_DATA_LEN];
uint8_t msg1[MSG_1_LEN];
uint8_t msg2[MSG_2_LEN];
uint8_t msg3[MSG_3_LEN];
uint8_t key_material[KEY_MATERIAL_LEN];
struct gec_sym_key symkey_chan1;
struct gec_sym_key symkey_chan2;

void pre_init() { LOG_ERROR("In pre_init"); }

#ifndef ED25519_REFHASH
#define ED25519_REFHASH
#endif

/* define ED25519_SUFFIX to have it appended to the end of each public function
 */
#if !defined(ED25519_SUFFIX)
#define ED25519_SUFFIX
#endif

#define ED25519_FN3(fn, suffix) fn##suffix
#define ED25519_FN2(fn, suffix) ED25519_FN3(fn, suffix)
#define ED25519_FN(fn) ED25519_FN2(fn, ED25519_SUFFIX)

DONNA_INLINE static void my_ed25519_extsk(hash_512bits extsk,
                                          const ed25519_secret_key sk) {
  ed25519_hash(extsk, sk, 32);
  extsk[0] &= 248;
  extsk[31] &= 127;
  extsk[31] |= 64;
}

static void ED25519_FN(my_ed25519_publickey)(const ed25519_secret_key sk,
                                             ed25519_public_key pk) {
  bignum256modm a;
  ge25519 ALIGN(16) A;
  hash_512bits extsk;

  /* A = aB */
  my_ed25519_extsk(extsk, sk);
  LOG_ERROR("my_ed25519_extsk complete");
  expand256_modm(a, extsk, 32);
  LOG_ERROR("expand256_modm complete");
  ge25519_scalarmult_base_niels(&A, ge25519_niels_base_multiples, a);
  LOG_ERROR("ge25519_scalarmult_base_niels complete");
  ge25519_pack(pk, &A);
  LOG_ERROR("ge25519_pack complete");
}

int run(void) {
  LOG_ERROR("In run");

  ring_buffer_t *ringbuffer_pixhawk = (ring_buffer_t *)ring_buffer_pixhawk;
  ring_buffer_t *ringbuffer_telemetry = (ring_buffer_t *)ring_buffer_telemetry;
  mavlink_message_t msg;
  float yaw, pitch, roll;
  uint16_t voltages[10];

  // Generate random data from vehicle status
  // for (int i = 0; i < RANDOM_DATA_LEN;) {
  //   read_message(ringbuffer_pixhawk, &msg);
  //   LOG_ERROR(
  //       "Message: [SEQ]: %03d, [MSGID]: 0x%06X, [SYSID]: %03d, [COMPID]:
  //       %03d", msg.seq, msg.msgid, msg.sysid, msg.compid);

  //   switch (msg.msgid) {
  //   case MAVLINK_MSG_ID_ATTITUDE:
  //     yaw = mavlink_msg_attitude_get_yaw(&msg);
  //     random_data[i++] = (uint8_t)(yaw * M_PI);
  //     if (i < RANDOM_DATA_LEN) {
  //       pitch = mavlink_msg_attitude_get_pitch(&msg);
  //       random_data[i++] = (uint8_t)(pitch * M_PI);
  //     }
  //     if (i < RANDOM_DATA_LEN) {
  //       roll = mavlink_msg_attitude_get_roll(&msg);
  //       random_data[i++] = (uint8_t)(roll * M_PI);
  //     }
  //     break;
  //   case MAVLINK_MSG_ID_BATTERY_STATUS:
  //     mavlink_msg_battery_status_get_voltages(&msg, voltages);
  //     random_data[i++] = (uint8_t)voltages[0];
  //     break;
  //   default:
  //     break;
  //   }
  // }
  for (int i = 0; i < RANDOM_DATA_LEN; i++) {
    random_data[i] = (uint8_t)rand();
  }

  LOG_ERROR("generate");
  // generate(&our_pubkey, &privkey, random_data);
  memcpy(privkey.priv, random_data, RANDOM_DATA_LEN);
  LOG_ERROR("memcpy complete");
  // gec_generate_sign_keypair(&privkey, &our_pubkey);
  my_ed25519_publickey(privkey.priv, our_pubkey.pub);
  LOG_ERROR("ed25519_publickey complete");
  memcpy(privkey.pub, our_pubkey.pub, GEC_PUB_KEY_LEN);
  LOG_ERROR("generate finished");

  // Send public key to GCS
  LOG_ERROR("Send public key to GCS");
  serial_send((uint8_t *)&our_pubkey, sizeof(our_pubkey));

  LOG_ERROR("Read from telemetry");
  read_from_telemetry(ringbuffer_telemetry, (uint8_t *)&their_pubkey,
                      sizeof(their_pubkey));

  LOG_ERROR("init_context");
  init_context(&ctx, &our_pubkey, &privkey, &their_pubkey);

  // Party A's step 1
  LOG_ERROR("initiate_sts");
  if (initiate_sts(msg1, &ctx, random_data)) {
    LOG_ERROR("initiate_sts failed");
  }

  // Send MSG 1
  LOG_ERROR("Send MSG 1");
  serial_send(msg1, sizeof(msg1));

  // Read MSG 2
  LOG_ERROR("read_from_telemetry");
  read_from_telemetry(ringbuffer_telemetry, msg2, sizeof(msg2));

  // Party A's step 2
  LOG_ERROR("response_ack_sts");
  if (response_ack_sts(msg2, msg3, &ctx, key_material)) {
    LOG_ERROR("response_ack_sts failed");
  }

  // Send MSG 3
  LOG_ERROR("Send MSG ");
  serial_send(msg3, sizeof(msg3));

  // Key exchange complete
  gec_key_material_to_2_channels(&symkey_chan1, &symkey_chan2, key_material);

  LOG_ERROR("Send keys");
  key_decrypt_send(symkey_chan1);
  key_encrypt_send(symkey_chan2);

  LOG_ERROR("Switch ringbuffer");
  switch_pixhawk_switch_ringbuffer();
  switch_telemetry_switch_ringbuffer();

  LOG_ERROR("Out run");
  return 0;
}
