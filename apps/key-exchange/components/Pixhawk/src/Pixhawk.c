#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

static uint8_t use_ringbuffer_key = 1;

void pre_init() {
  // ZF_LOGE("In pre_init");
  int error;
  ring_buffer_t *rb;

  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(UART_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  rb = (ring_buffer_t *)ring_buffer;
  rb->head = 0;
  ring_buffer_release();
  rb->tail = 0;
  ring_buffer_release();

  rb = (ring_buffer_t *)ring_buffer_key;
  rb->head = 0;
  ring_buffer_key_release();
  rb->tail = 0;
  ring_buffer_key_release();

  // LOG_ERROR("Out pre_init");
}

void switch__init() {}

void switch_switch_ringbuffer() { use_ringbuffer_key = 0; }

int run(void) {
  // ZF_LOGE("In run");

  int c;

  ring_buffer_t *rb = (ring_buffer_t *)ring_buffer_key;
  uint32_t tail;

  while (use_ringbuffer_key) {
    while (use_ringbuffer_key && (c = ps_cdev_getchar(serial)) == EOF) {
    }

    tail = rb->tail;
    ring_buffer_key_acquire();
    rb->buffer[tail] = c;
    ring_buffer_key_release();
    tail = (tail + 1) % RING_BUFFER_SIZE;
    rb->tail = tail;
    ring_buffer_key_release();
  }

  rb = (ring_buffer_t *)ring_buffer;

  LOG_ERROR("Switched ring buffer");

  while (1) {
    while ((c = ps_cdev_getchar(serial)) == EOF) {
    }

    tail = rb->tail;
    ring_buffer_acquire();
    rb->buffer[tail] = c;
    ring_buffer_release();
    tail = (tail + 1) % RING_BUFFER_SIZE;
    rb->tail = tail;
    ring_buffer_release();
  }

  // LOG_ERROR("Out run");
  return 0;
}
