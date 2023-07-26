#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "gec.h"
#include "my_type.h"

static ps_io_ops_t io_ops1;
static ps_chardevice_t serial_device1;
static ps_chardevice_t *serial1 = NULL;

static ps_io_ops_t io_ops2;
static ps_chardevice_t serial_device2;
static ps_chardevice_t *serial2 = NULL;

uint64_t last_time = 0;

uint32_t tail1 = 0;
ring_buffer_t *rb1 = NULL;
uint32_t tail2 = 0;
ring_buffer_t *rb2 = NULL;

void pre_init() {
  // LOG_ERROR("In pre_init");
  int error;
  ring_buffer_t *rb;

  error = camkes_io_ops(&io_ops1);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  error = camkes_io_ops(&io_ops2);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial1 = ps_cdev_init(UART_PORT_NUMBER, &io_ops1, &serial_device1);
  if (serial1 == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  serial2 = ps_cdev_init(TELEMETRY_PORT_NUMBER, &io_ops2, &serial_device2);
  if (serial2 == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  rb1 = (ring_buffer_t *)ring_buffer_encrypt;
  rb1->head = 0;
  ring_buffer_encrypt_release();
  rb1->tail = 0;
  ring_buffer_encrypt_release();

  rb2 = (ring_buffer_t *)ring_buffer_decrypt;
  rb2->head = 0;
  ring_buffer_decrypt_release();
  rb2->tail = 0;
  ring_buffer_decrypt_release();

  // LOG_ERROR("Out pre_init");
}

int run(void) {
  // LOG_ERROR("In run");

  int c1, c2;

  while (1) {
    while ((c1 = ps_cdev_getchar(serial1)) == EOF && (c2 = ps_cdev_getchar(serial2)) == EOF) {
    }

    if (c1 != EOF) {
      // LOG_ERROR("c1: %02X", c1);
      rb1->buffer[tail1++] = c1;
      ring_buffer_encrypt_release();
      tail1 %= RING_BUFFER_SIZE;
      rb1->tail = tail1;
      ring_buffer_encrypt_release();
    }

    if (c2 != EOF) {
      // LOG_ERROR("c2: %02X", c2);
      rb2->buffer[tail2++] = c2;
      ring_buffer_decrypt_release();
      tail2 %= RING_BUFFER_SIZE;
      rb2->tail = tail2;
      ring_buffer_decrypt_release();
    }
  }

  // LOG_ERROR("Out run");
  return 0;
}
