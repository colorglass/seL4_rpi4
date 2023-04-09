#include <camkes.h>
#include <camkes/io.h>
#include <platsupport/chardev.h>
#include <platsupport/io.h>
#include <sys/types.h>
#include <utils/util.h>

#include <stdint.h>
#include <string.h>

#include <top_types.h>

#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_types.h"

static mavlink_message_t mavlink_message_rx_buffer;
static mavlink_status_t mavlink_status;

static uint8_t my_mavlink_parse_char(uint8_t c,
						   mavlink_message_t *r_message,
						   mavlink_status_t *r_mavlink_status)
{
	uint8_t msg_received = mavlink_frame_char_buffer(&mavlink_message_rx_buffer,
								&mavlink_status,
								c,
								r_message,
								r_mavlink_status);
	if (msg_received == MAVLINK_FRAMING_BAD_CRC) {
		LOG_ERROR("MAVLink message parse error: Bad CRC");
	} else if (msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
		LOG_ERROR("MAVLink message parse error: Bad signature");
	}
	
	return msg_received;
}

#define send_wait() \
    do { \
        if (telemetry_send_wait()) { \
            printf("[%s] failed to lock: %d\n", get_instance_name(), __LINE__); \
        } \
    } while (0)

#define send_post() \
    do { \
        if (telemetry_send_post()) { \
            printf("[%s] failed to unlock: %d\n", get_instance_name(), __LINE__); \
        } \
    } while (0)

#define recv_wait() \
    do { \
        if (telemetry_recv_wait()) { \
            printf("[%s] failed to lock: %d\n", get_instance_name(), __LINE__); \
        } \
    } while (0)

#define recv_post() \
    do { \
        if (telemetry_recv_post()) { \
            printf("[%s] failed to unlock: %d\n", get_instance_name(), __LINE__); \
        } \
    } while (0)

static ps_io_ops_t io_ops;
static ps_chardevice_t serial_device;
static ps_chardevice_t *serial = NULL;

// From Telemetry to Decrypt
// From outside world to Telemetry
// static queue_t recv_queue;

// From Telemetry to outside world
// From Encrypt to Telemetry
static queue_t send_queue;

void pre_init() {
    LOG_ERROR("Starting Telemetry");
    LOG_ERROR("In pre_init");
    int error;
    error = camkes_io_ops(&io_ops);
    ZF_LOGF_IF(error, "Failed to initialise IO ops");

    serial = ps_cdev_init(BCM2xxx_UART3, &io_ops, &serial_device);
    ZF_LOGF_IF(!serial, "Failed to initialise char device");

    // queue_init(&recv_queue);
    queue_init(&send_queue);

    // Telem_Data *telem_data = (Telem_Data *) send_Telem_Data_Telemetry2Decrypt;
    // telem_data->len = 0;
    // send_Telem_Data_Telemetry2Decrypt_release();
    // memset(telem_data->raw_data, -1, sizeof(telem_data->raw_data));

    // recv_post();
    send_post();

    LOG_ERROR("Out pre_init");
}

static int telemetry_rx_poll() {
    int c = EOF;
    c = ps_cdev_getchar(serial);
    if (c == EOF) {
        return -1;
    }

    mavlink_status_t status;
    int result;
    mavlink_message_t msg;

    result = my_mavlink_parse_char(c, &msg, &status);

    if (status.packet_rx_drop_count) {
        LOG_ERROR("ERROR: Dropped %d packets", status.packet_rx_drop_count);
    }
    
    if (result) {
        LOG_ERROR("Telemetry received message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d", 
            msg.seq, msg.msgid, msg.sysid, msg.compid);
        getchar_io(msg);
    }

    return 0;
}

static int telemetry_tx_poll() {
    int error = 0;
    uint32_t size;
    uint8_t c;

    LOG_ERROR("Telemetry TX started");

    send_wait();

    size = send_queue.size;

    // if (!queue_empty(&send_queue)) {
    //     print_queue(&send_queue);
    //     // print_queue_serial(&send_queue);
    // }

    for (uint32_t i=0; i < size; i++) {
        if (!dequeue(&send_queue, &c)) {
            ps_cdev_putchar(serial, c);
        } else {
            error = -1;
            break;
        }
    }

    // ps_cdev_write(serial, send_queue.raw_queue, size, NULL, NULL);
    // send_queue.size = 0;

    send_post();

    LOG_ERROR("Telemetry TX finished");

    return error;
}

// Read encrypted Telem data from Encrypt
// and push to send_queue
static int read_from_encrypt(void) {
    int error = 0;

    Telem_Data *telem_data = (Telem_Data *) recv_Telem_Data_Encrypt2Telemetry;
    
    send_wait();

    uint32_t data_len = telem_data->len;
    recv_Telem_Data_Encrypt2Telemetry_acquire();

    // Abandon the rest of the data in shared memory
    if (data_len + send_queue.size > MAX_QUEUE_SIZE) {
        LOG_ERROR("WARNING: Send queue not enough!");
        data_len = MAX_QUEUE_SIZE - send_queue.size;
    }

    for (uint32_t i=0; i < data_len; i++) {
        if (enqueue(&send_queue, telem_data->raw_data[i])) {
            LOG_ERROR("WARNING: Send queue full!");
            recv_Telem_Data_Encrypt2Telemetry_acquire();
            error = -1;
            break;
        }
        recv_Telem_Data_Encrypt2Telemetry_acquire();
    }

    send_post();

    // Tell Encrypt that data has been accepted
    emit_Encrypt2Telemetry_DataReadyAck_emit();

    return error;
}

// Telemetry receives data from Encrypt
// when Encrypt gives Telemetry a DataReady Event
static void consume_Encrypt2Telemetry_DataReadyEvent_callback(void *in_arg UNUSED) {
    if (read_from_encrypt()) {
        LOG_ERROR("Error reading from encrypt");
    }

    if (consume_Encrypt2Telemetry_DataReadyEvent_reg_callback(&consume_Encrypt2Telemetry_DataReadyEvent_callback, NULL)) {
        ZF_LOGF("Failed to register Encrypt2Telemetry_DataReadyEvent callback");
    }
}

void consume_Encrypt2Telemetry_DataReadyEvent__init() {
    if (consume_Encrypt2Telemetry_DataReadyEvent_reg_callback(&consume_Encrypt2Telemetry_DataReadyEvent_callback, NULL)) {
        ZF_LOGF("Failed to register Encrypt2Telemetry_DataReadyEvent callback");
    }
}

int run() {
    LOG_ERROR("In run");

    emit_Encrypt2Telemetry_DataReadyAck_emit();

    while (1) {
        telemetry_rx_poll();
        telemetry_tx_poll();
    }

    return 0;
}
