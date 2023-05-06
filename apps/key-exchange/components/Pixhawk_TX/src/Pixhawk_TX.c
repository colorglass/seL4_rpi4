#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

void serial__init() {
  int error;
  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(BCM2xxx_UART5, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGE("Failed to initialise char device");
  } else {
    ZF_LOGI("Initialised char device");
  }
}

uint8_t serial_send(const pixhawk_buf_t *buf, uint32_t len) {
  if (ps_cdev_write(serial, (void *)buf->buf, len, NULL, NULL) != len) {
    LOG_ERROR("Write not completed");
  }
  return 0;
}
