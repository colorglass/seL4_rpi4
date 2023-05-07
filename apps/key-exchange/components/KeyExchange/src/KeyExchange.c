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

static gec_sts_ctx_t ctx;
static struct gec_privkey privkey;
static struct gec_pubkey our_pubkey;
static struct gec_pubkey their_pubkey;
static uint8_t random_data[RANDOM_DATA_LEN];
static uint8_t msg1[MSG_1_LEN];
static uint8_t msg2[MSG_2_LEN];
static uint8_t msg3[MSG_3_LEN];
static uint8_t key_material[KEY_MATERIAL_LEN];
static struct gec_sym_key symkey_chan1;
static struct gec_sym_key symkey_chan2;

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

static ring_buffer_t *ringbuffer_pixhawk = NULL;
static ring_buffer_t *ringbuffer_telemetry = NULL;

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
      if (result == MAVLINK_FRAMING_OK) {
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

static void print_msg(uint8_t *msg, int no) {
  puts("====================================");
  LOG_ERROR("MSG %d", no);
  int len;
  switch (no) {
  case 1:
    len = MSG_1_LEN;
    break;
  case 2:
    len = MSG_2_LEN;
    break;
  case 3:
    len = MSG_3_LEN;
    break;
  default:
    len = 0;
    break;
  }
  for (int i = 0; i < len; i++) {
    printf("%02X ", msg[i]);
    if ((i + 1) % 16 == 0 && i != len - 1) {
      putchar('\n');
    }
  }
  putchar('\n');
  puts("====================================");
}

static void print_key_material(uint8_t *km) {
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

static unsigned int seed_from_uav() {
  mavlink_message_t msg;
  float pitchspeed;
  uint16_t voltages[10];
  bool got_pitchspeed = false;
  bool got_voltage = false;
  unsigned int seed;

  // Generate random data from vehicle status
  do {
    read_message(ringbuffer_pixhawk, &msg);

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_ATTITUDE:
      if (!got_pitchspeed) {
        pitchspeed = mavlink_msg_attitude_get_pitchspeed(&msg);
        if (fabsf(pitchspeed) > 0.00000001f) {
          got_pitchspeed = true;
        }
      }
      break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      if (!got_voltage) {
        mavlink_msg_battery_status_get_voltages(&msg, voltages);
        if (voltages[0] != 0) {
          got_voltage = true;
        }
      }
      break;
    default:
      break;
    }
  } while (!got_pitchspeed && !got_voltage);

  LOG_ERROR("pitchspeed: %.8f, voltage: %u", pitchspeed, voltages[0]);

  seed = ((unsigned int)(fabs(pitchspeed) * 1e10)) * voltages[0];

  return seed;
}

void pre_init() {
  int error;
  ring_buffer_t *rb;

  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(TELEMETRY_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }
}

int run(void) {
  // LOG_ERROR("In run");

  ringbuffer_pixhawk = (ring_buffer_t *)ring_buffer_pixhawk;
  ringbuffer_telemetry = (ring_buffer_t *)ring_buffer_telemetry;

  unsigned int seed;

  seed = seed_from_uav();
  LOG_ERROR("seed: %u", seed);
  srand(seed);

  for (int i = 0; i < RANDOM_DATA_LEN; i++) {
    random_data[i] = rand();
  }

  // Generate key pair
  LOG_ERROR("generate");
  generate(&our_pubkey, &privkey, random_data);
  print_pubkey(&our_pubkey);
  LOG_ERROR("generate finished");

  // Wait for Party A's public key,
  // because Party A is the initiator.
  LOG_ERROR("Read their pubkey");
  read_from_telemetry(ringbuffer_telemetry, (uint8_t *)&their_pubkey,
                      sizeof(their_pubkey));

  print_pubkey(&their_pubkey);

  /**
   * Key Exchange start
   */

  LOG_ERROR("init_context");
  init_context(&ctx, &our_pubkey, &privkey, &their_pubkey);

  // Send public key to GCS
  LOG_ERROR("Send public key to GCS");
  ps_cdev_write(serial, &our_pubkey, sizeof(our_pubkey), NULL, NULL);

  // Party B's step 1
  LOG_ERROR("Read MSG 1");
  read_from_telemetry(ringbuffer_telemetry, msg1, sizeof(msg1));

  print_msg(msg1, 1);

  for (int i = 0; i < RANDOM_DATA_LEN; i++) {
    random_data[i] = rand();
  }

  // Party B's step 2
  LOG_ERROR("respond_sts");
  if (respond_sts(msg1, msg2, &ctx, random_data)) {
    LOG_ERROR("respond_sts failed");
  }

  print_msg(msg2, 2);

  LOG_ERROR("Send MSG 2");
  ps_cdev_write(serial, msg2, sizeof(msg2), NULL, NULL);

  LOG_ERROR("Read MSG 3");
  read_from_telemetry(ringbuffer_telemetry, msg3, sizeof(msg3));

  print_msg(msg3, 3);

  // Party B's step 3
  LOG_ERROR("finish_sts");
  if (finish_sts(msg3, &ctx, key_material)) {
    LOG_ERROR("finish_sts failed");
  }

  /**
   * Key Exchange finished
   */

  print_key_material(key_material);

  gec_key_material_to_2_channels(&symkey_chan1, &symkey_chan2, key_material);

  LOG_ERROR("Send keys to decrypt/encrypt");
  key_decrypt_send(&symkey_chan1);
  key_encrypt_send(&symkey_chan2);

  LOG_ERROR("Switch ringbuffer");
  switch_pixhawk_switch_ringbuffer();
  switch_telemetry_switch_ringbuffer();

  // This is a one-shot component.
  // So it needs to exit, instead of stalling
  // in an infinite loop.

  // LOG_ERROR("Out run");
  return 0;
}
