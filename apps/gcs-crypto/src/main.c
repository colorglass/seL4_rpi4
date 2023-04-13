#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "gec.h"
#include "gec-ke.h"
#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/mavlink_helpers.h"

struct Person {
    struct gec_sym_key symkey_chan1;
    struct gec_sym_key symkey_chan2;
};

uint8_t key_material[] = {
    0xCB, 0x28, 0x4A, 0xD9, 0x1E, 0x85, 0x78, 0xB1 ,
    0x77, 0x6E, 0x9B, 0x98, 0x32, 0xEF, 0x11, 0xB0 ,
    0xBC, 0xA8, 0xCF, 0xD6, 0x29, 0x98, 0xDA, 0x15 ,
    0x43, 0x82, 0xC5, 0xAC, 0x4C, 0xB9, 0x58, 0xC5 ,
    0x57, 0x0A, 0x4E, 0x30, 0xCC, 0xED, 0xFE, 0xF7 ,
    0x76, 0xF7, 0xC7, 0x75, 0x0C, 0x53, 0xA9, 0xE5,
};

int main() {
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        puts("Error opening serial port");
        return 1;
    }

    struct Person P1, P2;

    mavlink_message_t msg;
    msg.magic = 0xfe;
    msg.len = 9;
    msg.incompat_flags = 0;
    msg.compat_flags = 0;
    msg.seq = 233;
    msg.sysid = 255;
    msg.compid = 0;
    msg.msgid = 0;
    memset(&msg.payload64, 0, sizeof(msg.payload64));
    memset(&msg.ck, 0, sizeof(msg.ck));
    memset(&msg.signature, 0, sizeof(msg.signature));

    uint8_t buf[GEC_PT_LEN];
    memset(buf, 0, sizeof(buf));
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    /*
     * Key Exchange complete
    */

    gec_key_material_to_2_channels(&P1.symkey_chan1, &P1.symkey_chan2, key_material);
    gec_key_material_to_2_channels(&P2.symkey_chan1, &P2.symkey_chan2, key_material);
    if (!memcmp(&P1.symkey_chan1, &P2.symkey_chan1, sizeof(struct gec_sym_key)) && 
        !memcmp(&P1.symkey_chan2, &P2.symkey_chan2, sizeof(struct gec_sym_key)))
    {
        puts("GEC symkey correct!");
    } else {
        puts("GEC symkey correct!");
    }

    /******************************************************************/
    /*                    Encrypt/Decrypt test                        */

    /**
     * Channel 1 Encrypt/Decrypt test
     */
    
    puts("====================================");
    puts("Channel 1 Encrypt/Decrypt test");
    // uint8_t pt1[GEC_PT_LEN];
    uint8_t ct1[GEC_CT_LEN];
    uint8_t pt1_new[GEC_PT_LEN];
    if (gec_encrypt(&P1.symkey_chan1, buf, ct1) != GEC_SUCCESS) {
        puts("Encrypt failed");
    } else {
        puts("Encrypt success");
    }

    if (gec_decrypt(&P2.symkey_chan1, ct1, pt1_new) != GEC_SUCCESS) {
        puts("Decrypt failed");
    } else {
        puts("Decrypt success");
    }

    if (memcmp(buf, pt1_new, GEC_PT_LEN)) {
        puts("Decrypted message wrong");
    } else {
        puts("Decrypted message correct");
    }
    puts("====================================");

    uint32_t write_len = write(fd, buf, GEC_CT_LEN);
    if (write_len != GEC_CT_LEN) {
        puts("Write not completed");
    }
    tcdrain(fd);

    /**
     * Channel 2 Encrypt/Decrypt test
     */
    
    // puts("====================================");
    // puts("Channel 2 Encrypt/Decrypt test");
    // uint8_t pt2[GEC_PT_LEN];
    // uint8_t ct2[GEC_CT_LEN];
    // uint8_t pt2_new[GEC_PT_LEN];
    // for (int i=0; i<GEC_PT_LEN; i++) {
    //     pt2[i] = rand();
    // }
    // if (gec_encrypt(&P1.symkey_chan2, pt2, ct2) != GEC_SUCCESS) {
    //     puts("Encrypt failed");
    // } else {
    //     puts("Encrypt success");
    // }

    // if (gec_decrypt(&P2.symkey_chan2, ct2, pt2_new) != GEC_SUCCESS) {
    //     puts("Decrypt failed");
    // } else {
    //     puts("Decrypt success");
    // }

    // if (memcmp(pt2, pt2_new, GEC_PT_LEN)) {
    //     puts("Decrypted message wrong");
    // } else {
    //     puts("Decrypted message correct");
    // }
    // puts("====================================");

    close(fd);

    return 0;
}
