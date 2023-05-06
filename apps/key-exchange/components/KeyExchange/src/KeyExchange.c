#include <camkes.h>
#include <camkes/io.h>
#include <math.h>
#include <platsupport/chardev.h>
#include <stdint.h>
#include <stdlib.h>
#include <utils/util.h>

#include "camkes-component-key_exchange.h"
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
  ringbuffer->head = head;
  ring_buffer_pixhawk_release();
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
  ringbuffer->head = head;
  ring_buffer_telemetry_release();
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

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

void pre_init() {
  LOG_ERROR("In pre_init");

  int error;
  ring_buffer_t *rb;

  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(TELEMETRY_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }
}

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

static void
my_barrett_reduce256_modm(bignum256modm r, const bignum256modm q1, const bignum256modm r1) {
	LOG_ERROR("In barrett_reduce256_modm");
	bignum256modm q3, r2;
	uint128_t c, mul;
	bignum256modm_element_t f, b, pb;

	/* q1 = x >> 248 = 264 bits = 5 56 bit elements
	   q2 = mu * q1
	   q3 = (q2 / 256(32+1)) = q2 / (2^8)^(32+1) = q2 >> 264 */
	mul64x64_128(c, modm_mu[0], q1[3])                 mul64x64_128(mul, modm_mu[3], q1[0]) add128(c, mul) mul64x64_128(mul, modm_mu[1], q1[2]) add128(c, mul) mul64x64_128(mul, modm_mu[2], q1[1]) add128(c, mul) shr128(f, c, 56);
	mul64x64_128(c, modm_mu[0], q1[4]) add128_64(c, f) mul64x64_128(mul, modm_mu[4], q1[0]) add128(c, mul) mul64x64_128(mul, modm_mu[3], q1[1]) add128(c, mul) mul64x64_128(mul, modm_mu[1], q1[3]) add128(c, mul) mul64x64_128(mul, modm_mu[2], q1[2]) add128(c, mul)
	f = lo128(c); q3[0] = (f >> 40) & 0xffff; shr128(f, c, 56);
	mul64x64_128(c, modm_mu[4], q1[1]) add128_64(c, f) mul64x64_128(mul, modm_mu[1], q1[4]) add128(c, mul) mul64x64_128(mul, modm_mu[2], q1[3]) add128(c, mul) mul64x64_128(mul, modm_mu[3], q1[2]) add128(c, mul)
	f = lo128(c); q3[0] |= (f << 16) & 0xffffffffffffff; q3[1] = (f >> 40) & 0xffff; shr128(f, c, 56);
	mul64x64_128(c, modm_mu[4], q1[2]) add128_64(c, f) mul64x64_128(mul, modm_mu[2], q1[4]) add128(c, mul) mul64x64_128(mul, modm_mu[3], q1[3]) add128(c, mul)
	f = lo128(c); q3[1] |= (f << 16) & 0xffffffffffffff; q3[2] = (f >> 40) & 0xffff; shr128(f, c, 56);
	mul64x64_128(c, modm_mu[4], q1[3]) add128_64(c, f) mul64x64_128(mul, modm_mu[3], q1[4]) add128(c, mul)
	f = lo128(c); q3[2] |= (f << 16) & 0xffffffffffffff; q3[3] = (f >> 40) & 0xffff; shr128(f, c, 56);
	mul64x64_128(c, modm_mu[4], q1[4]) add128_64(c, f)
	f = lo128(c); q3[3] |= (f << 16) & 0xffffffffffffff; q3[4] = (f >> 40) & 0xffff; shr128(f, c, 56);
	q3[4] |= (f << 16);

	mul64x64_128(c, modm_m[0], q3[0]) 
	r2[0] = lo128(c) & 0xffffffffffffff; shr128(f, c, 56);
	mul64x64_128(c, modm_m[0], q3[1]) add128_64(c, f) mul64x64_128(mul, modm_m[1], q3[0]) add128(c, mul)
	r2[1] = lo128(c) & 0xffffffffffffff; shr128(f, c, 56);
	mul64x64_128(c, modm_m[0], q3[2]) add128_64(c, f) mul64x64_128(mul, modm_m[2], q3[0]) add128(c, mul) mul64x64_128(mul, modm_m[1], q3[1]) add128(c, mul)
	r2[2] = lo128(c) & 0xffffffffffffff; shr128(f, c, 56);
	mul64x64_128(c, modm_m[0], q3[3]) add128_64(c, f) mul64x64_128(mul, modm_m[3], q3[0]) add128(c, mul) mul64x64_128(mul, modm_m[1], q3[2]) add128(c, mul) mul64x64_128(mul, modm_m[2], q3[1]) add128(c, mul)
	r2[3] = lo128(c) & 0xffffffffffffff; shr128(f, c, 56);
	mul64x64_128(c, modm_m[0], q3[4]) add128_64(c, f) mul64x64_128(mul, modm_m[4], q3[0]) add128(c, mul) mul64x64_128(mul, modm_m[3], q3[1]) add128(c, mul) mul64x64_128(mul, modm_m[1], q3[3]) add128(c, mul) mul64x64_128(mul, modm_m[2], q3[2]) add128(c, mul)
	r2[4] = lo128(c) & 0x0000ffffffffff;

	pb = 0;
	pb += r2[0]; b = lt_modm(r1[0], pb); r[0] = (r1[0] - pb + (b << 56)); pb = b;
	pb += r2[1]; b = lt_modm(r1[1], pb); r[1] = (r1[1] - pb + (b << 56)); pb = b;
	pb += r2[2]; b = lt_modm(r1[2], pb); r[2] = (r1[2] - pb + (b << 56)); pb = b;
	pb += r2[3]; b = lt_modm(r1[3], pb); r[3] = (r1[3] - pb + (b << 56)); pb = b;
	pb += r2[4]; b = lt_modm(r1[4], pb); r[4] = (r1[4] - pb + (b << 40)); 

	LOG_ERROR("Before reduce256_modm");
	reduce256_modm(r);
	reduce256_modm(r);
}


static void
my_expand256_modm(bignum256modm out, const unsigned char *in, size_t len) {
	unsigned char work[64] = {0};
	bignum256modm_element_t x[16];
	bignum256modm q1;

	memcpy(work, in, len);
	x[0] = U8TO64_LE(work +  0);
	x[1] = U8TO64_LE(work +  8);
	x[2] = U8TO64_LE(work + 16);
	x[3] = U8TO64_LE(work + 24);
	x[4] = U8TO64_LE(work + 32);
	x[5] = U8TO64_LE(work + 40);
	x[6] = U8TO64_LE(work + 48);
	x[7] = U8TO64_LE(work + 56);

	/* r1 = (x mod 256^(32+1)) = x mod (2^8)(31+1) = x & ((1 << 264) - 1) */
	out[0] = (                         x[0]) & 0xffffffffffffff;
	out[1] = ((x[ 0] >> 56) | (x[ 1] <<  8)) & 0xffffffffffffff;
	out[2] = ((x[ 1] >> 48) | (x[ 2] << 16)) & 0xffffffffffffff;
	out[3] = ((x[ 2] >> 40) | (x[ 3] << 24)) & 0xffffffffffffff;
	out[4] = ((x[ 3] >> 32) | (x[ 4] << 32)) & 0x0000ffffffffff;

	/* under 252 bits, no need to reduce */
	if (len < 32)
		return;

	/* q1 = x >> 248 = 264 bits */
	q1[0] = ((x[ 3] >> 56) | (x[ 4] <<  8)) & 0xffffffffffffff;
	q1[1] = ((x[ 4] >> 48) | (x[ 5] << 16)) & 0xffffffffffffff;
	q1[2] = ((x[ 5] >> 40) | (x[ 6] << 24)) & 0xffffffffffffff;
	q1[3] = ((x[ 6] >> 32) | (x[ 7] << 32)) & 0xffffffffffffff;
	q1[4] = ((x[ 7] >> 24)                );

	LOG_ERROR("Before barrett_reduce256_modm");
	my_barrett_reduce256_modm(out, q1, out);
	LOG_ERROR("After barrett_reduce256_modm");
}

static void ED25519_FN(my_ed25519_publickey)(const ed25519_secret_key sk,
                                             ed25519_public_key pk) {
  bignum256modm a;
  ge25519 ALIGN(16) A;
  hash_512bits extsk;

  /* A = aB */
  my_ed25519_extsk(extsk, sk);
  LOG_ERROR("my_ed25519_extsk complete");
  my_expand256_modm(a, extsk, 32);
  LOG_ERROR("expand256_modm complete");
  ge25519_scalarmult_base_niels(&A, ge25519_niels_base_multiples, a);
  LOG_ERROR("ge25519_scalarmult_base_niels complete");
  ge25519_pack(pk, &A);
  LOG_ERROR("ge25519_pack complete");
}

static void print_secretkey(ed25519_secret_key sk) {
  puts("ed25519_secretkey");
  for (int i = 0; i < 32; i++) {
    printf("%02X ", sk[i]);
  }
  putchar('\n');
}

static void print_publickey(ed25519_public_key pk) {
  puts("ed25519_publickey");
  for (int i = 0; i < 32; i++) {
    printf("%02X ", pk[i]);
  }
  putchar('\n');
}

static void print_privkey(struct gec_privkey *privkey) {
  puts("====================================");
  puts("gec_prikey");
  print_secretkey(privkey->priv);
  print_publickey(privkey->pub);
  puts("====================================");
}

static void print_pubkey(struct gec_pubkey *pubkey) {
  puts("====================================");
  puts("gec_pubkey");
  print_publickey(pubkey->pub);
  puts("====================================");
}

static void print_msg1(uint8_t *msg) {
  puts("====================================");
  puts("MSG 1");
  for (int i=0; i < MSG_1_LEN; i++) {
    printf("%02X ", msg[i]);
    if ((i+1) % 16 == 0 && i != MSG_1_LEN - 1) {
      putchar('\n');
    }
  }
  putchar('\n');
  puts("====================================");
}

static void print_msg2(uint8_t *msg) {
  puts("====================================");
  puts("MSG 2");
  for (int i=0; i < MSG_2_LEN; i++) {
    printf("%02X ", msg[i]);
    if ((i+1) % 16 == 0 && i != MSG_2_LEN - 1) {
      putchar('\n');
    }
  }
  putchar('\n');
  puts("====================================");
}

static void print_msg3(uint8_t *msg) {
  puts("====================================");
  puts("MSG 3");
  for (int i=0; i < MSG_3_LEN; i++) {
    printf("%02X ", msg[i]);
    if ((i+1) % 16 == 0 && i != MSG_3_LEN - 1) {
      putchar('\n');
    }
  }
  putchar('\n');
  puts("====================================");
}

void print_key_material(uint8_t *km) {
  puts("====================================");
  puts("Key material");
  for (int i = 0; i < KEY_MATERIAL_LEN; i++) {
    printf("%02X ", km[i]);
    if ((i + 1) % 16 == 0) {
      putchar('\n');
    }
  }
  puts("====================================");
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
  generate(&our_pubkey, &privkey, random_data);

  print_pubkey(&our_pubkey);

  // memcpy(privkey.priv, random_data, RANDOM_DATA_LEN);
  // LOG_ERROR("memcpy complete");
  // // gec_generate_sign_keypair(&privkey, &our_pubkey);
  // my_ed25519_publickey(privkey.priv, our_pubkey.pub);
  // LOG_ERROR("ed25519_publickey complete");
  // memcpy(privkey.pub, our_pubkey.pub, GEC_PUB_KEY_LEN);
  LOG_ERROR("generate finished");

  LOG_ERROR("Read their pubkey");
  read_from_telemetry(ringbuffer_telemetry, (uint8_t *)&their_pubkey,
                      sizeof(their_pubkey));

  print_pubkey(&their_pubkey);

  LOG_ERROR("init_context");
  init_context(&ctx, &our_pubkey, &privkey, &their_pubkey);

  // Send public key to GCS
  LOG_ERROR("Send public key to GCS");
  // serial_send((uint8_t *)&our_pubkey.pub, sizeof(our_pubkey));
  ps_cdev_write(serial, &our_pubkey, sizeof(our_pubkey), NULL, NULL);

  LOG_ERROR("Read MSG 1");
  read_from_telemetry(ringbuffer_telemetry, msg1, sizeof(msg1));

  print_msg1(msg1);

  LOG_ERROR("respond_sts");
  if (respond_sts(msg1, msg2, &ctx, random_data)) {
    LOG_ERROR("respond_sts failed");
  }

  print_msg2(msg2);

  LOG_ERROR("Send MSG 2");
  ps_cdev_write(serial, msg2, sizeof(msg2), NULL, NULL);

  LOG_ERROR("Read MSG 3");
  read_from_telemetry(ringbuffer_telemetry, msg3, sizeof(msg3));

  print_msg3(msg3);

  LOG_ERROR("finish_sts");
  if (finish_sts(msg3, &ctx, key_material)) {
    LOG_ERROR("finish_sts failed");;
  }

  /**
   * Old setup
  */

  // // Send public key to GCS
  // LOG_ERROR("Send public key to GCS");
  // // serial_send((uint8_t *)&our_pubkey.pub, sizeof(our_pubkey));
  // ps_cdev_write(serial, &our_pubkey, sizeof(our_pubkey), NULL, NULL);

  // LOG_ERROR("Read their pubkey");
  // read_from_telemetry(ringbuffer_telemetry, (uint8_t *)&their_pubkey,
  //                     sizeof(their_pubkey));

  // print_pubkey(&their_pubkey);

  // LOG_ERROR("init_context");
  // init_context(&ctx, &our_pubkey, &privkey, &their_pubkey);

  // print_privkey(&privkey);

  // // Party A's step 1
  // LOG_ERROR("initiate_sts");
  // if (initiate_sts(msg1, &ctx, random_data)) {
  //   LOG_ERROR("initiate_sts failed");
  // }

  // print_msg1(msg1);

  // // Send MSG 1
  // LOG_ERROR("Send MSG 1");
  // // serial_send(msg1, sizeof(msg1));
  // ps_cdev_write(serial, msg1, sizeof(msg1), NULL, NULL);

  // // Read MSG 2
  // LOG_ERROR("Read MSG 2");
  // read_from_telemetry(ringbuffer_telemetry, msg2, sizeof(msg2));

  // print_msg2(msg2);

  // // Party A's step 2
  // LOG_ERROR("response_ack_sts");
  // if (response_ack_sts(msg2, msg3, &ctx, key_material)) {
  //   LOG_ERROR("response_ack_sts failed");
  // }

  // print_msg3(msg3);

  // // Send MSG 3
  // LOG_ERROR("Send MSG 3");
  // // serial_send(msg3, sizeof(msg3));
  // ps_cdev_write(serial, msg3, sizeof(msg3), NULL, NULL);


  print_key_material(key_material);

  // Key exchange complete
  gec_key_material_to_2_channels(&symkey_chan1, &symkey_chan2, key_material);

  LOG_ERROR("Send keys to decrypt/encrypt");
  key_decrypt_send(&symkey_chan1);
  key_encrypt_send(&symkey_chan2);

  LOG_ERROR("Switch ringbuffer");
  switch_pixhawk_switch_ringbuffer();
  switch_telemetry_switch_ringbuffer();

  LOG_ERROR("Out run");
  return 0;
}
