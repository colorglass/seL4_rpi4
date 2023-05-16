#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <stdint.h>
#include <utils/util.h>

#include "gec.h"
#include "my_type.h"

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

ring_buffer_t *rb;
static uint32_t tail = 0;

// void serial_irq_handle(void *data, ps_irq_acknowledge_fn_t acknowledge_fn,
//                        void *ack_data) {
//   int error;

//   LOG_ERROR("Time: %d", timer_time());

//   if (serial) {
//     int c = 0;
//     ps_cdev_handle_irq(serial, 0);
//     while (c != EOF) {
//       c = ps_cdev_getchar(serial);
//       if (c != EOF) {
//         LOG_ERROR("CHAR: %02X", c);
//       }
//     }
//   }

//   error = acknowledge_fn(ack_data);
//   ZF_LOGF_IF(error, "Failed to acknowledge IRQ");
// }

void pre_init() {
  // ZF_LOGE("In pre_init");

  int error;

  rb = (ring_buffer_t *)ring_buffer;

  error = camkes_io_ops(&io_ops);
  ZF_LOGF_IF(error, "Failed to initialise IO ops");

  serial = ps_cdev_init(TELEMETRY_PORT_NUMBER, &io_ops, &serial_device);
  if (serial == NULL) {
    ZF_LOGF("Failed to initialise char device");
  }

  // ps_irq_t irq_info = {.type = PS_INTERRUPT,
  //                      .irq = {
  //                          .number = UART3_IRQ,
  //                      }};
  // irq_id_t serial_irq_id =
  //     ps_irq_register(&io_ops.irq_ops, irq_info, serial_irq_handle, NULL);
  // ZF_LOGF_IF(serial_irq_id < 0, "Failed to register irq");

  rb->head = 0;
  ring_buffer_release();
  rb->tail = 0;
  ring_buffer_release();

  // LOG_ERROR("Out pre_init");
}

int run(void) {
  // ZF_LOGE("In run");

  int c;
  // ring_buffer_t *rb = (ring_buffer_t *)ring_buffer;

  while (1) {
    // while ((c = ps_cdev_getchar(serial)) == EOF) {
    // }

    // rb->buffer[tail] = c;
    // ring_buffer_release();
    // tail = (tail + 1) % RING_BUFFER_SIZE;
    // rb->tail = tail;
    // ring_buffer_release();
  }

  // LOG_ERROR("Out run");
  return 0;
}
