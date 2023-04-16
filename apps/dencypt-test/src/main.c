#include "gec-ke.h"
#include "gec.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct Person {
  struct gec_privkey privkey;
  struct gec_pubkey pubkey;
  struct gec_sym_key symkey_chan1;
  struct gec_sym_key symkey_chan2;
};

void print_secretkey(ed25519_secret_key sk) {
  puts("ed25519_secretkey");
  for (int i = 0; i < 32; i++) {
    printf("%02X ", sk[i]);
  }
  putchar('\n');
}

void print_publickey(ed25519_public_key pk) {
  puts("ed25519_publickey");
  for (int i = 0; i < 32; i++) {
    printf("%02X ", pk[i]);
  }
  putchar('\n');
}

void print_privkey(struct gec_privkey *privkey) {
  puts("====================================");
  puts("gec_prikey");
  print_secretkey(privkey->priv);
  print_publickey(privkey->pub);
  puts("====================================");
}

void print_pubkey(struct gec_pubkey *pubkey) {
  puts("====================================");
  puts("gec_pubkey");
  print_publickey(pubkey->pub);
  puts("====================================");
}

void print_pt(uint8_t pt[GEC_PT_LEN]) {
  puts("====================================");
  puts("Plaintext");
  for (int i = 0; i < GEC_PT_LEN; i++) {
    printf("%02X ", pt[i]);
    if ((i + 1) % 12 == 0) {
      putchar('\n');
    }
  }
  putchar('\n');
  puts("====================================");
}

void print_ct(uint8_t ct[GEC_CT_LEN]) {
  puts("====================================");
  puts("Cyphertext");
  for (int i = 0; i < GEC_CT_LEN; i++) {
    printf("%02X ", ct[i]);
    if ((i + 1) % 12 == 0) {
      putchar('\n');
    }
  }
  putchar('\n');
  puts("====================================");
}

void print_key_material(uint8_t *km) {
  puts("====================================");
  puts("Key material");
  for (int i = 0; i < KEY_MATERIAL_LEN; i++) {
    printf("%02X ", km[i]);
    if ((i + 1) % 8 == 0) {
      putchar('\n');
    }
  }
  puts("====================================");
}

int main() {
  struct Person P1;
  uint8_t random_data1[RANDOM_DATA_LEN];
  for (int i = 0; i < RANDOM_DATA_LEN; i++) {
    random_data1[i] = rand() & 0xff;
  }
  generate(&P1.pubkey, &P1.privkey, random_data1);

  puts("Party A");
  print_privkey(&P1.privkey);
  print_pubkey(&P1.pubkey);

  struct Person P2;
  uint8_t random_data2[RANDOM_DATA_LEN];
  for (int i = 0; i < RANDOM_DATA_LEN; i++) {
    random_data1[i] = rand() & 0xff;
  }
  generate(&P2.pubkey, &P2.privkey, random_data2);

  puts("Party B");
  print_privkey(&P1.privkey);
  print_pubkey(&P1.pubkey);

  gec_sts_ctx_t ctx1;
  // Give B's public key to A
  init_context(&ctx1, &P1.pubkey, &P1.privkey, &P2.pubkey);

  gec_sts_ctx_t ctx2;
  // Give A's public key to B
  init_context(&ctx2, &P2.pubkey, &P2.privkey, &P1.pubkey);

  // Party A's step 1
  uint8_t msg1[MSG_1_LEN];
  if (initiate_sts(msg1, &ctx1, random_data1)) {
    puts("initiate_sta failed");
  }

  // Party B's step 1
  uint8_t msg2[MSG_2_LEN];
  if (respond_sts(msg1, msg2, &ctx2, random_data2)) {
    puts("respond_sts failed");
  }

  // Party A's step 2
  uint8_t key_material1[KEY_MATERIAL_LEN];
  uint8_t msg3[MSG_3_LEN];
  if (response_ack_sts(msg2, msg3, &ctx1, key_material1)) {
    puts("response_ack_sts failed");
  }

  puts("Party A");
  print_key_material(key_material1);

  // Party B's step 2
  uint8_t key_material2[KEY_MATERIAL_LEN];
  if (finish_sts(msg3, &ctx2, key_material2)) {
    puts("finish_sts failed");
  }

  puts("Party B");
  print_key_material(key_material2);

  /*
   * Key Exchange complete
   */

  gec_key_material_to_2_channels(&P1.symkey_chan1, &P1.symkey_chan2,
                                 key_material1);
  gec_key_material_to_2_channels(&P2.symkey_chan1, &P2.symkey_chan2,
                                 key_material2);
  if (!memcmp(&P1.symkey_chan1, &P2.symkey_chan1, sizeof(struct gec_sym_key)) &&
      !memcmp(&P1.symkey_chan2, &P2.symkey_chan2, sizeof(struct gec_sym_key))) {
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
  uint8_t pt1[GEC_PT_LEN];
  uint8_t ct1[GEC_CT_LEN];
  uint8_t pt1_new[GEC_PT_LEN];
  for (int i = 0; i < GEC_PT_LEN; i++) {
    pt1[i] = rand();
  }
  for (int i = 0; i < 256; i++) {
    printf("%d\n", i);
    if (gec_encrypt(&P1.symkey_chan1, pt1, ct1) != GEC_SUCCESS) {
      puts("Encrypt failed");
    } else {
      puts("Encrypt success");
    }

    if (gec_decrypt(&P2.symkey_chan1, ct1, pt1_new) != GEC_SUCCESS) {
      puts("Decrypt failed");
    } else {
      puts("Decrypt success");
    }

    if (memcmp(pt1, pt1_new, GEC_PT_LEN)) {
      puts("Decrypted message wrong");
    } else {
      puts("Decrypted message correct");
    }
    puts("====================================");
  }

  /**
   * Channel 2 Encrypt/Decrypt test
   */

  puts("====================================");
  puts("Channel 2 Encrypt/Decrypt test");
  uint8_t pt2[GEC_PT_LEN];
  uint8_t ct2[GEC_CT_LEN];
  uint8_t pt2_new[GEC_PT_LEN];
  for (int i = 0; i < GEC_PT_LEN; i++) {
    pt2[i] = rand();
  }
  if (gec_encrypt(&P1.symkey_chan2, pt2, ct2) != GEC_SUCCESS) {
    puts("Encrypt failed");
  } else {
    puts("Encrypt success");
  }

  if (gec_decrypt(&P2.symkey_chan2, ct2, pt2_new) != GEC_SUCCESS) {
    puts("Decrypt failed");
  } else {
    puts("Decrypt success");
  }

  if (memcmp(pt2, pt2_new, GEC_PT_LEN)) {
    puts("Decrypted message wrong");
  } else {
    puts("Decrypted message correct");
  }
  puts("====================================");

  return 0;
}
