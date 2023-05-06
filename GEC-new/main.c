#include "include/gec-ke.h"
#include "include/gec.h"
#include "ed25519/src/ed25519.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

void print_privkey(ed25519_secret_key privkey) {\
  puts("===========================");
  puts("Priv key");
  uint8_t *p = privkey;
  for (int i = 0; i < sizeof(ed25519_secret_key); i++) {
    printf("%02X ", p[i]);
    if ((i + 1) % 8 == 0) {
      putchar('\n');
    }
  }
  puts("===========================");
}

void print_pubkey(ed25519_public_key pubkey) {\
  puts("===========================");
  puts("Pub key");
  uint8_t *p = pubkey;
  for (int i = 0; i < sizeof(ed25519_public_key); i++) {
    printf("%02X ", p[i]);
    if ((i + 1) % 8 == 0) {
      putchar('\n');
    }
  }
  puts("===========================");
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

struct Person {
  uint8_t random_data[RANDOM_DATA_LEN];
  gec_sts_ctx_t ctx;
  struct gec_privkey privkey;
  struct gec_pubkey our_pubkey, their_pubkey;
  uint8_t key_material[KEY_MATERIAL_LEN];
  struct gec_sym_key symkey1, symkey2;
};

int main() {
  struct Person P1, P2;

  memset(&P1, 0, sizeof(P1));
  memset(&P2, 0, sizeof(P2));

  for (int i = 0; i < RANDOM_DATA_LEN; i++) {
    P1.random_data[i] = rand();
    P2.random_data[i] = rand();
  }

  uint8_t msg1[MSG_1_LEN], msg2[MSG_2_LEN], msg3[MSG_3_LEN];

  // ed25519_create_keypair(P1.our_pubkey.pub, P1.privkey.priv, P1.random_data);
  // ed25519_create_keypair(P2.our_pubkey.pub, P2.privkey.priv, P2.random_data);

  generate(&P1.our_pubkey, &P1.privkey, P1.random_data);
  generate(&P2.our_pubkey, &P2.privkey, P2.random_data);

  memcpy(P1.privkey.pub, P1.our_pubkey.pub, sizeof(P1.our_pubkey.pub));
  memcpy(P2.privkey.pub, P2.our_pubkey.pub, sizeof(P2.our_pubkey.pub));


  print_privkey(P1.privkey.priv);
  print_privkey(P2.privkey.priv);

  print_pubkey(P1.our_pubkey.pub);
  print_pubkey(P2.our_pubkey.pub);

  print_pubkey(P1.privkey.pub);
  print_pubkey(P2.privkey.pub);

  memcpy(P1.their_pubkey.pub, P2.our_pubkey.pub, sizeof(P2.our_pubkey.pub));
  memcpy(P2.their_pubkey.pub, P1.our_pubkey.pub, sizeof(P1.our_pubkey.pub));


  init_context(&P1.ctx, &P1.our_pubkey, &P1.privkey, &P1.their_pubkey);
  init_context(&P2.ctx, &P2.our_pubkey, &P2.privkey, &P2.their_pubkey);

  if (initiate_sts(msg1, &P1.ctx, P1.random_data)) {
    puts("initiate_sts failed");
  }
  if (respond_sts(msg1, msg2, &P2.ctx, P2.random_data)) {
    puts("respond_sts failed");
  }
  if (response_ack_sts(msg2, msg3, &P1.ctx, P1.key_material)) {
    puts("response_ack_sts failed");
  }
  if (finish_sts(msg3, &P2.ctx, P2.key_material)) {
    puts("finish_sts failed");
  }

  print_key_material(P1.key_material);
  print_key_material(P2.key_material);


  return 0;
}
