#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>
#include <utils/util.h>

#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

void pre_init() {
  LOG_ERROR("In pre_init");

  int error;
  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(UART_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  LOG_ERROR("Out pre_init");
}

void serial__init() {

}

void serial_send(const uint8_t *buf, uint16_t len) {
    ps_cdev_write(serial, (void *)buf, len, NULL, NULL);
}

