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
        if (decrypt_send_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define send_post() \
    do { \
        if (decrypt_send_post()) { \
            LOG_ERROR("[%s] failed to unlock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_wait() \
    do { \
        if (decrypt_recv_wait()) { \
            LOG_ERROR("[%s] failed to lock\n", get_instance_name()); \
        } \
    } while (0)

#define recv_post() \
    do { \
        if (decrypt_recv_post()) { \
            LOG_ERROR("[%s] failed to unlock\n", get_instance_name()); \
        } \
    } while (0)

// From Telemetry to Decrypt
// static queue_t recv_queue;
static MAVLink_Message_t recv_msg;

// From Decrypt to UART
// static queue_t send_queue;
static MAVLink_Message_t send_msg;

void pre_init() {
    LOG_ERROR("Starting Decrypt");
    LOG_ERROR("In pre_init");

    // queue_init(&recv_queue);
    // queue_init(&send_queue);

    recv_msg.is_msg = 0;
    send_msg.is_msg = 0;

    // memset(&mavlink_message_rx_buffer, 0, sizeof(mavlink_message_rx_buffer));
    // memset(&mavlink_status, 0, sizeof(mavlink_status));

    FC_Data *fc_data = (FC_Data *) send_FC_Data_Decrypt2UART;
    fc_data->len = 0;
    send_FC_Data_Decrypt2UART_release();
    memset(fc_data->raw_data, -1, sizeof(fc_data->raw_data));

    recv_post();
    send_post();

    LOG_ERROR("Out pre_init");
}

// Simulate decryption.
// No encryption now, so just give Telem_Data to FC_Data.
// From recv_queue to send_queue
// TODO: Implement actual decryption
static int Decrypt_Telem_Data_to_FC_Data() {
    int error = 0;

    recv_wait();
    if (recv_msg.is_msg) {
        send_wait();

        if (send_msg.is_msg) {
            LOG_ERROR("Overwrite send_msg");
        }

        send_msg = recv_msg;
        recv_msg.is_msg = 0;

        send_post();
    }
    recv_post();

    return error;
}

// Send decrypted FC data to UART
// from send_queue
static int send_to_uart(void) {
    int error = 0;

    uint32_t queue_size;
    while (1) {
        send_wait();
        if (send_msg.is_msg) {
            break;
        }
        send_post();
    }

    FC_Data *fc_data = (FC_Data *) send_FC_Data_Decrypt2UART;

    // Protect send_queue
    // send_wait();

    uint32_t len = mavlink_msg_to_send_buffer(fc_data->raw_data, &send_msg.msg);
    send_FC_Data_Decrypt2UART_release();
    fc_data->len = len;

    send_msg.is_msg = 0;

    LOG_ERROR("Decrypt sent data: [SEQ]: %d, [MSGID]: %d, [COMPID]: %d, [SYSID]: %d", send_msg.msg.seq, send_msg.msg.msgid, send_msg.msg.compid, send_msg.msg.sysid);

    send_post();

    // Decrypt => UART
    emit_Decrypt2UART_DataReadyEvent_emit();

    return error;
}

// Decrypt sends decrypted FC data to UART
// when UART gives Decrypt an ACK
static void consume_Decrypt2UART_DataReadyAck_callback(void *in_arg UNUSED) {
    if (send_to_uart()) {
        LOG_ERROR("Error sending to uart");
    }

    if (consume_Decrypt2UART_DataReadyAck_reg_callback(&consume_Decrypt2UART_DataReadyAck_callback, NULL)) {
        ZF_LOGF("Failed to register Decrypt2UART_DataReadyAck callback");
    }
}

// void consume_Telemetry2Decrypt_DataReadyEvent__init(void) {
//     if (consume_Telemetry2Decrypt_DataReadyEvent_reg_callback(&consume_Telemetry2Decrypt_DataReadyEvent_callback, NULL)) {
//         ZF_LOGF("Failed to register Telemetry2Decrypt_DataReadyEvent callback");
//     }
// }

void consume_Decrypt2UART_DataReadyAck__init(void) {
    if (consume_Decrypt2UART_DataReadyAck_reg_callback(&consume_Decrypt2UART_DataReadyAck_callback, NULL)) {
        ZF_LOGF("Failed to register Decrypt2UART_DataReadyAck callback");
    }
}

void getchar__init(void) {

}

// Called by Telemetry.
void getchar_io(mavlink_message_t m) {
    recv_wait();
    
    recv_msg.msg = m;
    recv_msg.is_msg = 1;

    recv_post();

    Decrypt_Telem_Data_to_FC_Data();
}

int run(void) {
    LOG_ERROR("In run");

    // emit_Telemetry2Decrypt_DataReadyAck_emit();

    while (1) {
        // Decrypt_Telem_Data_to_FC_Data();
    }

    return 0;
}
