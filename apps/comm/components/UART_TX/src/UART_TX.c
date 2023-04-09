#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <platsupport/io.h>
#include <utils/util.h>

#include <top_types.h>

// #include "mavlink/v2.0/common/mavlink.h"
// #include "mavlink/v2.0/mavlink_types.h"

// static mavlink_message_t mavlink_message_rx_buffer;
// static mavlink_status_t mavlink_status;

// static uint8_t my_mavlink_parse_char(uint8_t c,
// 						   mavlink_message_t *r_message,
// 						   mavlink_status_t *r_mavlink_status)
// {
// 	uint8_t msg_received = mavlink_frame_char_buffer(&mavlink_message_rx_buffer,
// 								&mavlink_status,
// 								c,
// 								r_message,
// 								r_mavlink_status);
// 	if (msg_received == MAVLINK_FRAMING_BAD_CRC) {
// 		LOG_ERROR("MAVLink message parse error: Bad CRC");
// 	} else if (msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
// 		LOG_ERROR("MAVLink message parse error: Bad signature");
// 	}
	
// 	return msg_received;
// }

#define send_wait() \
    do { \
        if (uart_send_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define send_post() \
    do { \
        if (uart_send_post()) { \
            LOG_ERROR("[%s] failed to post\n", get_instance_name()); \
        } \
    } while (0)

#define recv_wait() \
    do { \
        if (uart_recv_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_post() \
    do { \
        if (uart_recv_post()) { \
            LOG_ERROR("[%s] failed to post\n", get_instance_name()); \
        } \
    } while (0)

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

// From UART to Encrypt,
// From outside world to UART
// static queue_t recv_queue;

// From Decrypt to UART,
// From UART to outside world
static queue_t send_queue;

// static void serial_irq_handle(void *data, ps_irq_acknowledge_fn_t acknowledge_fn, void *ack_data) {
//     int error;

//     if (serial) {
//         int c = 0;
//         ps_cdev_handle_irq(serial, 0);
//         // while (c != EOF) {
//             c = ps_cdev_getchar(serial);
//             if (c != EOF) {
//                 // LOG_ERROR("\nCHAR: %02X\n", c);
//                 mavlink_status_t status;
//                 int result;

//                 mavlink_message_t msg;
//                 result = my_mavlink_parse_char(c, &msg, &status);

//                 if (status.packet_rx_drop_count) {
//                     LOG_ERROR("ERROR: Dropped %d packets", status.packet_rx_drop_count);
//                 }
                
//                 if (result) {
//                     LOG_ERROR("UART received message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d", 
//                         msg.seq, msg.msgid, msg.sysid, msg.compid);
//                     getchar_io(msg);
//                 }
//             }
//     }

//     error = acknowledge_fn(ack_data);
//     ZF_LOGF_IF(error, "Failed to acknowledge IRQ");
// }

void pre_init() {
    LOG_ERROR("Starting UART");
    LOG_ERROR("In pre_init");

    int error;
    error = camkes_io_ops(&io_ops);
    ZF_LOGF_IF(error, "Failed to initialise IO ops");

    serial = ps_cdev_init(BCM2xxx_UART5, &io_ops, &serial_device);
    ZF_LOGF_IF(!serial, "Failed to initialise char device");

    // queue_init(&recv_queue);
    queue_init(&send_queue);

    send_post();
    // recv_post();

    // FC_Data *fc_data = (FC_Data *) send_FC_Data_UART2Encrypt;
    // fc_data->len = 0;
    // send_FC_Data_UART2Encrypt_release();
    // memset(fc_data->raw_data, -1, sizeof(fc_data->raw_data));

    LOG_ERROR("Out pre_init");
}

// Read decrypted FC data from Decrypt
// and push to send_queue
static int read_from_decrypt(void) {
    int error = 0;

    FC_Data *fc_data = (FC_Data *) recv_FC_Data_Decrypt2UART;

    send_wait();

    uint32_t data_len = fc_data->len;
    if (data_len + send_queue.size > MAX_QUEUE_SIZE) {
        LOG_ERROR("Receive queue not enough!");
    }

    for (uint32_t i = 0; i < data_len; i++) {
        send_post();
        if (enqueue(&send_queue, fc_data->raw_data[i])) {
            LOG_ERROR("Receive queue full!");
            error = -1;
            break;
        }
    }

    send_post();

    // Tell Decrypt that data has been accepted
    emit_Decrypt2UART_DataReadyAck_emit();

    return error;
}

// UART receives data from Decrypt
// when Decrypt gives UART a DataReady Event
static void consume_Decrypt2UART_DataReadyEvent_callback(void *in_arg UNUSED) {
    if (read_from_decrypt()) {
        LOG_ERROR("Error reading from decrypt");
    }

    if (consume_Decrypt2UART_DataReadyEvent_reg_callback(&consume_Decrypt2UART_DataReadyEvent_callback, NULL)) {
        ZF_LOGF("Failed to register Decrypt2UART_DataReadyEvent callback");
    }
}

void consume_Decrypt2UART_DataReadyEvent__init(void) {
    if (consume_Decrypt2UART_DataReadyEvent_reg_callback(&consume_Decrypt2UART_DataReadyEvent_callback, NULL)) {
        ZF_LOGF("Failed to register Decrypt2UART_DataReadyEvent callback");
    }
}

// Read from RX and push to recv_queue
// static int uart_rx_poll(void) {
//     int c = EOF;
//     c = ps_cdev_getchar(serial);
//     if (c == EOF) {
//         return -1;
//     }

//     mavlink_status_t status;
//     int result;

//     mavlink_message_t msg;
//     result = my_mavlink_parse_char(c, &msg, &status);

//     if (status.packet_rx_drop_count) {
//         LOG_ERROR("ERROR: Dropped %d packets", status.packet_rx_drop_count);
//     }
    
//     if (result) {
//         LOG_ERROR("UART received message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d", 
//             msg.seq, msg.msgid, msg.sysid, msg.compid);
//         getchar_io(msg);
//     }

//     return 0;
// }

// Write to TX from send_queue
static int uart_tx_poll(void) {
    int error = 0;
    send_wait();

    uint32_t size = send_queue.size;

    ps_cdev_write(serial, send_queue.raw_queue, size, NULL, NULL);

    send_queue.size = 0;

    send_post();
    return error;
}

int run(void) {
    LOG_ERROR("In run");

    emit_Decrypt2UART_DataReadyAck_emit();

    while (1) {
        uart_tx_poll();
    }
    
    return 0;
}
