#include "gec-ke.h"
#include "gec.h"
#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define GEC_CT_FRAME_LEN (GEC_CT_LEN + 2)

struct Person {
  struct gec_sym_key symkey_chan1;
  struct gec_sym_key symkey_chan2;
};

#define MAX_QUEUE_SIZE 4096
typedef struct _queue {
    uint8_t raw_queue[MAX_QUEUE_SIZE];
    uint32_t head;
    uint32_t size;
} queue_t;

#define queue_init(q) \
    ({ \
        memset((q)->raw_queue, -1, MAX_QUEUE_SIZE); \
        (q)->head = 0; \
        (q)->size = 0; \
    })

#define queue_full(q) \
    ({ \
        (q)->size == MAX_QUEUE_SIZE; \
    })

#define queue_empty(q) \
    ({ \
        !(q)->size; \
    })

#define enqueue(q, x) \
    ({ \
        int _ret; \
        if (queue_full(q)) { \
            printf("Cannot enquque"); \
            _ret = -1; \
        } else {\
        uint32_t _index = ((q)->head + (q)->size) % MAX_QUEUE_SIZE; \
        (q)->raw_queue[_index] = x; \
        (q)->size++; \
        _ret = 0; \
        } \
        _ret; \
    })

#define dequeue(q, ret) \
    ({ \
        int _ret; \
        if (queue_empty(q)) { \
            printf("Cannot dequeue"); \
            _ret = -1; \
        } else { \
        *(ret) = (q)->raw_queue[(q)->head]; \
        (q)->head = ((q)->head + 1) % MAX_QUEUE_SIZE; \
        (q)->size--; \
        _ret = 0; \
        } \
        _ret; \
    })

#define print_queue(q) \
    ({ \
        char _str[MAX_QUEUE_SIZE * 3 + 1]; \
        char *_p = _str; \
        printf("Print queue:"); \
        uint32_t size = (q)->size; \
        for (uint32_t h = (q)->head, i = 0; i < size; i++, h = (h+1) % MAX_QUEUE_SIZE) { \
            sprintf(_p, "%02X ", (q)->raw_queue[h]); \
            _p += 3; \
        } \
        printf("%s", _str); \
    })

#define print_serial(s) { \
    for (int i=0; s[i]; i++) { \
        ps_cdev_putchar(serial, s[i]); \
    } \
}

struct Person P1;
int fd;
int time_to_exit = 0;

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

  // uint8_t ct1[GEC_CT_LEN];

  // const uint32_t loop_count = 16;

  while (!time_to_exit) {
    mavlink_msg_heartbeat_encode(255, 0, &msg, &hb);
    memset(buf, 0, sizeof(buf));
    mavlink_msg_to_send_buffer(buf, &msg);

    if (gec_encrypt(&P1.symkey_chan1, buf, ct_frame + 2) != GEC_SUCCESS) {
      puts("Encrypt failed");
    } else {
      puts("Encrypt success");
    }

    // print_ct(ct1);

    // memcpy(ct_frame + 2, ct1, GEC_CT_LEN);

    uint32_t write_len = write(fd, ct_frame, GEC_CT_FRAME_LEN);
    if (write_len != GEC_CT_FRAME_LEN) {
      puts("Write not completed");
    }
    tcdrain(fd);

    sleep(1);
  }
  return NULL;
}

void receive(void) {
  mavlink_message_t msg;
  mavlink_status_t status;
  int result;

  uint8_t ct_frame[GEC_CT_FRAME_LEN];
  uint8_t ct[GEC_CT_LEN];
  uint8_t pt[GEC_PT_LEN];
  int cnt = 0;

  queue_t queue;
  queue_init(&queue);

  int lost_packets = 0;
  int last_seq = -1;
  int all_packets = 0;

  while (!time_to_exit) {
    puts("Waiting");
    cnt = 0;
    uint8_t c = 0xff;
    while (1) {
      if (read(fd, &c, 1) && c == 0x7e) {
        if (read(fd, &c, 1) && c == 0) {
          break;
        }
      }
    }
    while (cnt < GEC_CT_LEN) {
      int len = read(fd, ct + cnt, GEC_CT_LEN - cnt);
      printf("    Read %d bytes\n", len);
      cnt += len;
    }
    puts("Got frame");
    if (gec_decrypt(&P1.symkey_chan2, ct, pt)) {
      puts("Decrypt failed");
    } else {
      for (int i=0; i<GEC_PT_LEN; i++) {
        // rb.buffer[rb.tail] = pt[i];
        // rb.tail = (rb.tail + 1) % RING_BUFFER_SIZE;
        enqueue(&queue, pt[i]);
      }
    }
    while (!queue_empty(&queue)){ 
      uint8_t c;
      dequeue(&queue, &c);
      result = mavlink_parse_char(0, c, &msg, &status);
      // result = mavlink_parse_char(0, rb.buffer[rb.head], &msg, &status);
      // rb.head = (rb.head + 1) % RING_BUFFER_SIZE;
      if (result) {
        printf("Message: [SEQ]: %03d, [SYSID]: %03d, [MSGID]: %03d\n", msg.seq, msg.sysid, msg.msgid);
        all_packets += 1;
        if (last_seq != -1 && msg.seq != (last_seq + 1) % 256) {
          lost_packets += (msg.seq - last_seq - 1 + 256) % 256;
          printf("========Lost packets: %f\n", (float)lost_packets / all_packets);
        }
        if (msg.sysid == 1) last_seq = msg.seq;
      }
    }
  }

  return NULL;
}

// void *serial_get(void *arg) {
//   puts("Thread started");
//   while (1) {
//     uint8_t buf[GEC_CT_FRAME_LEN];
//     int len = read(fd, buf, sizeof(buf));
//     // printf("    Read %d bytes\n", len);
//     for (int i=0; i<len; i++) {
//       serial_rb.buffer[serial_rb.tail] = buf[i];
//       serial_rb.tail = (serial_rb.tail + 1) % RING_BUFFER_SIZE;
//     }
//   }
// }

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
  config.c_cc[VMIN] = 64;
  // config.c_cc[VTIME] = 10; // was 0
  config.c_cc[VTIME] = 0;

  if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0) {
    printf("\nERROR: Could not set desired baud rate of %d Baud\n", 57600);
  }
  if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
    printf("\nERROR: could not set configuration of fd %d\n", fd);
  }

  gec_key_material_to_2_channels(&P1.symkey_chan1, &P1.symkey_chan2,
                                 key_material);

  // serial_rb.head = serial_rb.tail = 0;

  // int t;
  // pthread_create(&t, NULL, serial_get, NULL);

  receive();

  // pthread_join(t, NULL);

  close(fd);

  return 0;
}
