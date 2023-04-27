#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>

#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"
#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;


void pre_init() {
  ZF_LOGE("In pre_init");
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

void serial_send__init() {

}

void serial_send(const uint8_t *buf, uint16_t len) {
  if (ps_cdev_write(serial, buf, len, NULL, NULL) != len) {
    LOG_ERROR("Write not completed");
  }
}

int run(void) {
  ZF_LOGE("In run");

  while (1) {
    
  }
  return 0;
}
