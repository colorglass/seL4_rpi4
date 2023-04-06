#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <camkes.h>
#include <utils/util.h>

#include <top_types.h>

/**
 * MAVLink parsing
 */
#include "camkes-component-encrypt.h"
#include "mavlink/v2.0/common/mavlink.h"
static mavlink_message_t g_mavlink_message_rx_buffer;
static mavlink_status_t g_mavlink_status;

static uint8_t my_mavlink_parse_char(uint8_t c,
						   mavlink_message_t *r_message,
						   mavlink_status_t *r_mavlink_status)
{
	uint8_t msg_received = mavlink_frame_char_buffer(&g_mavlink_message_rx_buffer,
								&g_mavlink_status,
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


#define send_lock() \
    do { \
        if (encrypt_send_lock()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define send_unlock() \
    do { \
        if (encrypt_send_unlock()) { \
            LOG_ERROR("[%s] failed to unlock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_lock() \
    do { \
        if (encrypt_recv_lock()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_unlock() \
    do { \
        if (encrypt_recv_unlock()) { \
            LOG_ERROR("[%s] failed to unlock\n", get_instance_name()); \
        } \
    } while (0)

// From UART to Encrypt
// static queue_t recv_queue;
static MAVLink_Message_t recv_msg;

// From Encrypt to Telemetry
// static queue_t send_queue;
static MAVLink_Message_t send_msg;

void pre_init() {
    LOG_ERROR("Starting Encrypt");
    LOG_ERROR("In pre_init");

    // queue_init(&recv_queue);
    // queue_init(&send_queue);

    recv_msg.is_msg = 0;
    send_msg.is_msg = 0;

    Telem_Data *fc_data = (Telem_Data*) send_Telem_Data_Encrypt2Telemetry;
    send_Telem_Data_Encrypt2Telemetry_release();
    fc_data->len = 0;
    memset(fc_data->raw_data, -1, sizeof(fc_data->raw_data));

    LOG_ERROR("Out pre_init");
}

// Simulate encryption.
// No encryption now, so just give FC_Data to Telem_Data.
// From recv_queue to send_queue.
// TODO: Implement actual encryption
static void Encrypt_FC_Data_to_Telem_Data() {
    if (recv_msg.is_msg) {

        // Protect both recv_queue and send_queue
        send_lock();
        recv_lock();

        // if (recv_queue.size + send_queue.size > MAX_QUEUE_SIZE) {
        //     LOG_ERROR("Send queue not enough!");
        // }

        // for (uint32_t i=0; i < recv_queue.size; i++) {
        //     uint8_t tmp;
        //     if (dequeue(&recv_queue, &tmp)) {
        //         break;
        //     }
        //     if (enqueue(&send_queue, tmp)) {
        //         break;
        //     }
        // }

        if (send_msg.is_msg) {
            LOG_ERROR("Overwrite send_msg");
        }

        send_msg = recv_msg;
        recv_msg.is_msg = 0;

        // Must in reverse order
        recv_unlock();
        send_unlock();
    }
}

// Send encrypted Telem data to Telemetry
// through shared memory
// from send_queue
static int send_to_telemetry(void) {
    int error = 0;

    uint32_t queue_size;
    uint32_t data_size;

    // Wait until send_queue is not empty,
    // because xxxAck_callback is invoked
    // once for each ACK
    while (1) {
        // queue_size = send_queue.size;
        // if (queue_size > 0) {
        //     break;
        // }
        if (send_msg.is_msg) {
            break;
        }
    }

    Telem_Data *telem_data = (Telem_Data *) send_Telem_Data_Encrypt2Telemetry;

    // Protect send_queue
    send_lock();

    // data_size = send_queue.size;
    // if (data_size > sizeof(Telem_Data_raw)) {
    //     data_size = sizeof(Telem_Data_raw);
    // }

    // uint8_t tmp;
    // for (uint32_t i = 0; i < data_size; i++) {
    //     if (!dequeue(&send_queue, &tmp)) {
    //         telem_data->raw_data[i] = tmp;
    //         send_Telem_Data_Encrypt2Telemetry_release();
    //     } else {
    //         LOG_ERROR("Should not get here");
    //         error = -1;
    //         data_size = 0;
    //         break;
    //     }
    // }
    // telem_data->len = data_size;

    uint32_t len = mavlink_msg_to_send_buffer(telem_data->raw_data, &send_msg.msg);
    send_Telem_Data_Encrypt2Telemetry_release();
    telem_data->len = len;

    send_msg.is_msg = 0;

    send_unlock();

    // LOG_ERROR("To telemetry");
    // Encrypt => Telemetry
    emit_Encrypt2Telemetry_DataReadyEvent_emit();

    return error;
}

// Read FC data from UART
// and push to recv_queue
static int read_from_uart(void) {
    int error = 0;

    // Protect recv_queue
    recv_lock();

    FC_Data *fc_data = (FC_Data *) recv_FC_Data_UART2Encrypt;
    uint32_t size = fc_data->len;
    mavlink_status_t status;
    int result;

    for (uint32_t i = 0; i < size; i++) {
        recv_FC_Data_UART2Encrypt_acquire();

        result = my_mavlink_parse_char(fc_data->raw_data[i], &recv_msg.msg, &status);

        if (status.packet_rx_drop_count) {
            LOG_ERROR("ERROR: Dropped %d packets", status.packet_rx_drop_count);
        }
        
        if (!result) {
            LOG_ERROR("Encrypt received message: [SEQ]: %d, [MSGID]: %d, [SYSID]: %d, [COMPID]: %d", 
                recv_msg.msg.seq, recv_msg.msg.msgid, recv_msg.msg.sysid, recv_msg.msg.compid);
            recv_msg.is_msg = 1;
        }
    }

    recv_unlock();

    // Tell UART that data has been accepted
    emit_UART2Encrypt_DataReadyAck_emit();

    return error;
}

// Encrypt sends encrypted Telem data to Telemetry
// when Telemetry gives Encrypt an ACK
static void consume_Encrypt2Telemetry_DataReadyAck_callback(void *in_arg) {
    if (send_to_telemetry()) {
        LOG_ERROR("Error sending to telemetry");
    }

    if (consume_Encrypt2Telemetry_DataReadyAck_reg_callback(&consume_Encrypt2Telemetry_DataReadyAck_callback, NULL)) {
        ZF_LOGF("Failed to register Encrypt2Telemetry_DataReadyAck callback");
    }
}

// Encrypt reads FC data from UART
// when UART gives Encrypt a DataReady Event
static void consume_UART2Encrypt_DataReadyEvent_callback(void *in_arg) {
    if (read_from_uart()) {
        LOG_ERROR("Error reading from uart");
    }
    
    if (consume_UART2Encrypt_DataReadyEvent_reg_callback(&consume_UART2Encrypt_DataReadyEvent_callback, NULL)) {
        ZF_LOGF("Failed to register UART2Encrypt_DataReadyEvent callback");
    }
}

void consume_Encrypt2Telemetry_DataReadyAck__init(void) {
    if (consume_Encrypt2Telemetry_DataReadyAck_reg_callback(&consume_Encrypt2Telemetry_DataReadyAck_callback, NULL)) {
        ZF_LOGF("Failed to register Encrypt2Telemetry_DataReadyAck callback");
    }
}

void consume_UART2Encrypt_DataReadyEvent__init(void) {
    if (consume_UART2Encrypt_DataReadyEvent_reg_callback(&consume_UART2Encrypt_DataReadyEvent_callback, NULL)) {
        ZF_LOGF("Failed to register UART2Encrypt_DataReadyEvent callback");
    }
}

int run() {
    LOG_ERROR("In run");

    emit_UART2Encrypt_DataReadyAck_emit();

    while (1) {
        Encrypt_FC_Data_to_Telem_Data();
    }

    return 0;
}
