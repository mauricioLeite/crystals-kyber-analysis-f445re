#ifndef PQCLEAN_KYBER1024_CLEAN_API_H
#define PQCLEAN_KYBER1024_CLEAN_API_H

#include <stdint.h>
#include "kem.h"

#define CRYPTO_SECRETKEYBYTES  3168
#define CRYPTO_PUBLICKEYBYTES  1568
#define CRYPTO_CIPHERTEXTBYTES 1568
#define CRYPTO_BYTES           32

int crypto_kem_keypair(uint8_t *pk, uint8_t *sk);

int crypto_kem_enc(uint8_t *ct, uint8_t *ss, const uint8_t *pk);

int crypto_kem_dec(uint8_t *ss, const uint8_t *ct, const uint8_t *sk);


#endif
