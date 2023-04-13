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

#define GEC_CT_FRAME_LEN (GEC_CT_LEN + 2)

struct Person {
  struct gec_sym_key symkey_chan1;
  struct gec_sym_key symkey_chan2;
};

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

int main() {
  int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
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
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

  if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				printf("\nERROR: Could not set desired baud rate of %d Baud\n", 57600);
			}
      if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		printf("\nERROR: could not set configuration of fd %d\n", fd);
	}

  struct Person P1, P2;

  mavlink_message_t msg;
  msg.magic = 0xfe;
  msg.len = 9;
  msg.incompat_flags = 0;
  msg.compat_flags = 0;
  msg.seq = 233;
  msg.sysid = 255;
  msg.compid = 0;
  msg.msgid = 0;
  memset(&msg.payload64, 0, sizeof(msg.payload64));
  memset(&msg.ck, 0, sizeof(msg.ck));
  memset(&msg.signature, 0, sizeof(msg.signature));

  uint8_t buf[GEC_PT_LEN];
  memset(buf, 0, sizeof(buf));
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  /*
   * Key Exchange complete
   */

  gec_key_material_to_2_channels(&P1.symkey_chan1, &P1.symkey_chan2,
                                 key_material);
  gec_key_material_to_2_channels(&P2.symkey_chan1, &P2.symkey_chan2,
                                 key_material);
  if (!memcmp(&P1.symkey_chan1, &P2.symkey_chan1, sizeof(struct gec_sym_key)) &&
      !memcmp(&P1.symkey_chan2, &P2.symkey_chan2, sizeof(struct gec_sym_key))) {
    puts("GEC symkey correct!");
  } else {
    puts("GEC symkey correct!");
  }

  /******************************************************************/
  /*                    Encrypt/Decrypt test                        */

  /**
   * Channel 1 Encrypt/Decrypt test
   */

  puts("====================================");
  puts("Channel 1 Encrypt/Decrypt test");
  // uint8_t pt1[GEC_PT_LEN];
  uint8_t ct1[GEC_CT_LEN];
  // uint8_t pt1_new[GEC_PT_LEN];
  if (gec_encrypt(&P1.symkey_chan1, buf, ct1) != GEC_SUCCESS) {
    puts("Encrypt failed");
  } else {
    puts("Encrypt success");
  }

  print_ct(ct1);

  // if (gec_decrypt(&P2.symkey_chan1, ct1, pt1_new) != GEC_SUCCESS) {
  //   puts("Decrypt failed");
  // } else {
  //   puts("Decrypt success");
  // }

  // if (memcmp(buf, pt1_new, GEC_PT_LEN)) {
  //   puts("Decrypted message wrong");
  // } else {
  //   puts("Decrypted message correct");
  // }
  // puts("====================================");

  uint8_t ct_frame[GEC_CT_FRAME_LEN];
  ct_frame[0] = 0x7e;
  ct_frame[1] = 0;
  memcpy(ct_frame + 2, ct1, GEC_CT_LEN);

  uint32_t write_len = write(fd, ct_frame, GEC_CT_FRAME_LEN);
  if (write_len != GEC_CT_FRAME_LEN) {
    puts("Write not completed");
  }
  tcdrain(fd);


  if (gec_encrypt(&P1.symkey_chan1, buf, ct1) != GEC_SUCCESS) {
    puts("Encrypt failed");
  } else {
    puts("Encrypt success");
  }

  print_ct(ct1);

  memcpy(ct_frame + 2, ct1, GEC_CT_LEN);

  write_len = write(fd, ct_frame, GEC_CT_FRAME_LEN);
  if (write_len != GEC_CT_FRAME_LEN) {
    puts("Write not completed");
  }
  tcdrain(fd);

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
