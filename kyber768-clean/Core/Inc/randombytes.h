//
//  randombytes.h
//
//  Created by Bassham, Lawrence E (Fed) on 8/29/17.
//  Copyright © 2017 Bassham, Lawrence E (Fed). All rights reserved.
//

#ifndef randombytes_h
#define randombytes_h

#include <stdio.h>
#include <stdint.h>

typedef uint32_t uint32;

int
randombytes(uint8_t *x, size_t xlen);

#endif /* randombytes_h */
