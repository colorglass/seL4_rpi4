#include "gec-ke.h"
#include "gec.h"
#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>

#define GEC_CT_FRAME_LEN (GEC_CT_LEN + 2)

struct Person {
  struct gec_sym_key symkey_chan1;
  struct gec_sym_key symkey_chan2;
};

struct Person P1;
int fd;

uint8_t key_material[] = {
    0xCB, 0x28, 0x4A, 0xD9, 0x1E, 0x85, 0x78, 0xB1, 0x77, 0x6E, 0x9B, 0x98,
    0x32, 0xEF, 0x11, 0xB0, 0xBC, 0xA8, 0xCF, 0xD6, 0x29, 0x98, 0xDA, 0x15,
    0x43, 0x82, 0xC5, 0xAC, 0x4C, 0xB9, 0x58, 0xC5, 0x57, 0x0A, 0x4E, 0x30,
    0xCC, 0xED, 0xFE, 0xF7, 0x76, 0xF7, 0xC7, 0x75, 0x0C, 0x53, 0xA9, 0xE5,
};

void print_ct(uint8_t ct[GEC_CT_LEN]) {
  puts("====================================");
  puts("Cyphertext");
  for (int i = 0; i < GEC_CT_LEN; i++) {
    printf("%02X ", ct[i]);
    if ((i + 1) % 12 == 0) {
      putchar('\n');
    }
  }
  putchar('\n');
  puts("====================================");
}

void *send(void *args) {
  mavlink_message_t msg;
  mavlink_heartbeat_t hb;
  hb.custom_mode = 0;
  hb.type = MAV_TYPE_QUADROTOR;
  hb.system_status = 0;
  hb.mavlink_version = 3;

  uint8_t buf[GEC_PT_LEN];

  uint8_t ct_frame[GEC_CT_FRAME_LEN];
  ct_frame[0] = 0x7e;
  ct_frame[1] = 0;

  uint8_t ct1[GEC_CT_LEN];

  const uint32_t loop_count = 16;

  for (uint32_t i = 0; i < loop_count; i++) {
    mavlink_msg_heartbeat_encode(255, 0, &msg, &hb);
    memset(buf, 0, sizeof(buf));
    mavlink_msg_to_send_buffer(buf, &msg);

    if (gec_encrypt(&P1.symkey_chan1, buf, ct1) != GEC_SUCCESS) {
      puts("Encrypt failed");
    } else {
      puts("Encrypt success");
    }

    print_ct(ct1);

    memcpy(ct_frame + 2, ct1, GEC_CT_LEN);

    uint32_t write_len = write(fd, ct_frame, GEC_CT_FRAME_LEN);
    if (write_len != GEC_CT_FRAME_LEN) {
      puts("Write not completed");
    }
    tcdrain(fd);

    usleep(1000000);
  }
  return NULL;
}

void *receive(void *args) {
  mavlink_message_t msg;
  mavlink_status_t status;
  int result;

  uint8_t ct_frame[GEC_CT_FRAME_LEN];
  int len;
  uint8_t buf[GEC_PT_LEN];

  while (1) {
    len = read(fd, ct_frame, GEC_CT_FRAME_LEN);
    if (len != GEC_CT_FRAME_LEN) {
      printf("Failed to read. Read len: %d\n", len);
    }

    if (ct_frame[0] != 0x7e) {
      printf("MAGIC error: 0x%02X\n", ct_frame[0]);
      continue;
    }

    if (ct_frame[1] != 0) {
      printf("TAG error: 0x%02X\n", ct_frame[1]);
      continue;
    }

    if (gec_decrypt(&P1.symkey_chan2, ct_frame + 2, buf)) {
      puts("Decrypt failed");
      continue;
    }

    for (int i = 0; i < GEC_PT_LEN; i++) {
      result = mavlink_parse_char(0, buf[i], &msg, &status);
      if (result) {
        printf("Message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d",
               msg.seq, msg.msgid, msg.sysid, msg.compid);
        break;
      }
    }
  }

  return NULL;
}

int main() {
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    puts("Error opening serial port");
    return 1;
  }

  struct termios config;
  if (tcgetattr(fd, &config) < 0) {
    puts("tcgetaddr error");
    return 1;
  }

  config.c_iflag &=
      ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif

  // No line processing:
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;

  // One input byte is enough to return from read()
  // Inter-character timer off
  config.c_cc[VMIN] = 1;
  config.c_cc[VTIME] = 10; // was 0

  if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0) {
    printf("\nERROR: Could not set desired baud rate of %d Baud\n", 57600);
  }
  if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
    printf("\nERROR: could not set configuration of fd %d\n", fd);
  }

  gec_key_material_to_2_channels(&P1.symkey_chan1, &P1.symkey_chan2,
                                 key_material);

  int err;
  int t1, t2;
  err = pthread_create(&t1, NULL, &send, NULL);
  if (err) {
    puts("pthread_create failed");
  }
  err = pthread_create(&t2, NULL, &send, NULL);
  if (err) {
    puts("pthread_create failed");
  }

  pthread_join(t1, NULL);
  pthread_join(t2, NULL);

  /**
   * Channel 2 Encrypt/Decrypt test
   */

  // puts("====================================");
  // puts("Channel 2 Encrypt/Decrypt test");
  // uint8_t pt2[GEC_PT_LEN];
  // uint8_t ct2[GEC_CT_LEN];
  // uint8_t pt2_new[GEC_PT_LEN];
  // for (int i=0; i<GEC_PT_LEN; i++) {
  //     pt2[i] = rand();
  // }
  // if (gec_encrypt(&P1.symkey_chan2, pt2, ct2) != GEC_SUCCESS) {
  //     puts("Encrypt failed");
  // } else {
  //     puts("Encrypt success");
  // }

  // if (gec_decrypt(&P2.symkey_chan2, ct2, pt2_new) != GEC_SUCCESS) {
  //     puts("Decrypt failed");
  // } else {
  //     puts("Decrypt success");
  // }

  // if (memcmp(pt2, pt2_new, GEC_PT_LEN)) {
  //     puts("Decrypted message wrong");
  // } else {
  //     puts("Decrypted message correct");
  // }
  // puts("====================================");

  close(fd);

  return 0;
}
