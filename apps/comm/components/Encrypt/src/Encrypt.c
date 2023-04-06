#include "mavlink/v2.0/mavlink_types.h"
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
#include "mavlink/v2.0/common/mavlink.h"

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
        if (encrypt_send_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define send_post() \
    do { \
        if (encrypt_send_post()) { \
            LOG_ERROR("[%s] failed to unlock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_wait() \
    do { \
        if (encrypt_recv_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_post() \
    do { \
        if (encrypt_recv_post()) { \
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

    // memset(&mavlink_message_rx_buffer, 0, sizeof(mavlink_message_rx_buffer));
    // memset(&mavlink_status, 0, sizeof(mavlink_status));

    Telem_Data *fc_data = (Telem_Data*) send_Telem_Data_Encrypt2Telemetry;
    send_Telem_Data_Encrypt2Telemetry_release();
    fc_data->len = 0;
    memset(fc_data->raw_data, -1, sizeof(fc_data->raw_data));

    recv_post();
    send_post();

    LOG_ERROR("Out pre_init");
}

// Simulate encryption.
// No encryption now, so just give FC_Data to Telem_Data.
// From recv_queue to send_queue.
// TODO: Implement actual encryption
static void Encrypt_FC_Data_to_Telem_Data() {
    recv_wait();
    if (recv_msg.is_msg) {

        // Protect both recv_queue and send_queue
        send_wait();

        if (send_msg.is_msg) {
            LOG_ERROR("Overwrite send_msg");
        }

        send_msg = recv_msg;
        recv_msg.is_msg = 0;

        // Must in reverse order
        send_post();
    }
    recv_post();
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
        send_wait();
        if (send_msg.is_msg) {
            break;
        }
        send_post();
    }

    Telem_Data *telem_data = (Telem_Data *) send_Telem_Data_Encrypt2Telemetry;

    // Protect send_queue
    // send_wait();

    uint32_t len = mavlink_msg_to_send_buffer(telem_data->raw_data, &send_msg.msg);
    send_Telem_Data_Encrypt2Telemetry_release();
    telem_data->len = len;

    send_msg.is_msg = 0;

    LOG_ERROR("Encrypt sent data: [SEQ]: %d, [MSGID]: %d, [COMPID]: %d, [SYSID]: %d", send_msg.msg.seq, send_msg.msg.msgid, send_msg.msg.compid, send_msg.msg.sysid);

    send_post();

    // Encrypt => Telemetry
    emit_Encrypt2Telemetry_DataReadyEvent_emit();

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
// static void consume_UART2Encrypt_DataReadyEvent_callback(void *in_arg) {
//     if (read_from_uart()) {
//         LOG_ERROR("Error reading from uart");
//     }
    
//     if (consume_UART2Encrypt_DataReadyEvent_reg_callback(&consume_UART2Encrypt_DataReadyEvent_callback, NULL)) {
//         ZF_LOGF("Failed to register UART2Encrypt_DataReadyEvent callback");
//     }
// }

void consume_Encrypt2Telemetry_DataReadyAck__init(void) {
    if (consume_Encrypt2Telemetry_DataReadyAck_reg_callback(&consume_Encrypt2Telemetry_DataReadyAck_callback, NULL)) {
        ZF_LOGF("Failed to register Encrypt2Telemetry_DataReadyAck callback");
    }
}

// void consume_UART2Encrypt_DataReadyEvent__init(void) {
//     if (consume_UART2Encrypt_DataReadyEvent_reg_callback(&consume_UART2Encrypt_DataReadyEvent_callback, NULL)) {
//         ZF_LOGF("Failed to register UART2Encrypt_DataReadyEvent callback");
//     }
// }

void getchar__init() {

}

void getchar_io(mavlink_message_t m) {
    recv_wait();

    recv_msg.msg = m;
    recv_msg.is_msg = 1;
    
    recv_post();

    Encrypt_FC_Data_to_Telem_Data();
}

int run() {
    LOG_ERROR("In run");

    // emit_UART2Encrypt_DataReadyAck_emit();

    while (1) {
        // Encrypt_FC_Data_to_Telem_Data();
    }

    return 0;
}
