#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "camkes-component-pixhawk_tx.h"
#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

void pre_init() {
  ZF_LOGE("In pre_init");
  int error;
  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(UART_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGE("Failed to initialise char device");
  } else {
    ZF_LOGI("Initialised char device");
  }
}

void write_port__init() {

}

void write_port_uart_write(const uint8_t *buf, uint32_t len) {
  if (ps_cdev_write(serial, (void*) buf, len, NULL, NULL) != len) {
    LOG_ERROR("Write not finished");
  }
}

void data_ready_callback(void *in_arg) {
  MAVLink_msg_frame_t *buf = (MAVLink_msg_frame_t *) from_decrypt;
  void *frame;
  uint32_t len;

  len = buf->len;
  from_decrypt_acquire();
  if (ps_cdev_write(serial, (void*) frame, len, NULL, NULL) != len) {
    LOG_ERROR("Write not finished");
  }
  from_decrypt_acquire();

  data_ack_emit();

  data_ready_reg_callback(data_ready_callback, NULL);
}

void data_ready__init() {
  data_ready_reg_callback(data_ready_callback, NULL);
}

int run(void) {
  ZF_LOGE("In run");

  data_ack_emit();

  while (1) {
  }
  return 0;
}
