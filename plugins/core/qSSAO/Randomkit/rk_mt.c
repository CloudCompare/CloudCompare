/* Random kit 1.6 */

/*
 * Copyright (c) 2003-2006, Jean-Sebastien Roy (js@jeannot.org)
 *
 * The rk_random and rk_seed functions algorithms and the original design of 
 * the Mersenne Twister RNG:
 *
 * Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
 * All rights reserved.
 *
 * Original algorithm for the implementation of rk_interval function from
 * Richard J. Wagner's implementation of the Mersenne Twister RNG, optimised by
 * Magnus Jonsson.
 *
 * Constants used in the rk_double implementation by Isaku Wada.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

static char const rcsid[] =
  "@(#) $Jeannot: rk_mt.c,v 1.6 2006/02/19 13:48:34 js Exp $";

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#ifdef _WIN32
/* Windows */
#include <sys/timeb.h>
#ifndef RK_NO_WINCRYPT
/* Windows crypto */
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0400
#endif
#include <windows.h>
#include <wincrypt.h>
#endif
#else
/* Unix */
#include <sys/time.h>
#include <unistd.h>
#endif

#include "randomkit.h"

#ifndef RK_DEV_URANDOM
#define RK_DEV_URANDOM "/dev/urandom"
#endif

#ifndef RK_DEV_RANDOM
#define RK_DEV_RANDOM "/dev/random"
#endif

char *rk_strerror[RK_ERR_MAX] =
{
  "no error",
  "random device unvavailable"
};

/* static functions */
static unsigned long rk_hash(unsigned long key);

void rk_knuth_fill(unsigned long seed, unsigned long *key, unsigned long len)
{
  unsigned long pos;
  seed &= 0xffffffffUL;

  /* Knuth's PRNG as used in the Mersenne Twister reference implementation */
  for (pos=0; pos<len; pos++)
  {
    key[pos] = seed;
    seed = (1812433253UL * (seed ^ (seed >> 30)) + pos + 1) & 0xffffffffUL;
  }
}

void rk_seed(unsigned long seed, rk_state *state)
{
  rk_knuth_fill(seed, state->key, RK_STATE_LEN);
  state->pos = RK_STATE_LEN;
  state->has_gauss = 0;
}

/* Thomas Wang 32 bits integer hash function */
unsigned long rk_hash(unsigned long key)
{
  key += ~(key << 15);
  key ^=  (key >> 10);
  key +=  (key << 3);
  key ^=  (key >> 6);
  key += ~(key << 11);
  key ^=  (key >> 16);
  return key;
}

unsigned long rk_seedfromsystem()
{
#ifndef _WIN32
  struct timeval tv;
#else
  struct _timeb  tv;
#endif

#ifndef _WIN32
  gettimeofday(&tv, NULL);
  return rk_hash(getpid()) ^ rk_hash(tv.tv_sec) ^ rk_hash(tv.tv_usec)
    ^ rk_hash(clock());
#else
  _ftime(&tv);
  return rk_hash(tv.time) ^ rk_hash(tv.millitm) ^ rk_hash(clock());
#endif
}

rk_error rk_randomseed(rk_state *state)
{
  if(rk_devfill(state->key, sizeof(state->key), 0) == RK_NOERR)
  {
    state->key[0] |= 0x80000000UL; /* ensures non-zero key */
    state->pos = RK_STATE_LEN;
    state->has_gauss = 0;
    return RK_NOERR;
  }

  rk_seed(rk_seedfromsystem(), state);

  return RK_ENODEV;
}

/* Magic Mersenne Twister constants */
#define N 624
#define M 397
#define MATRIX_A 0x9908b0dfUL
#define UPPER_MASK 0x80000000UL
#define LOWER_MASK 0x7fffffffUL

/* Slightly optimised reference implementation of the Mersenne Twister */
unsigned long rk_random(rk_state *state)
{
  unsigned long y;

  if (state->pos == RK_STATE_LEN)
  {
    int i;

    for (i=0;i<N-M;i++)
    {
      y = (state->key[i] & UPPER_MASK) | (state->key[i+1] & LOWER_MASK);
      state->key[i] = state->key[i+M] ^ (y>>1) ^ (-(y & 1) & MATRIX_A);
    }
    for (;i<N-1;i++)
    {
      y = (state->key[i] & UPPER_MASK) | (state->key[i+1] & LOWER_MASK);
      state->key[i] = state->key[i+(M-N)] ^ (y>>1) ^ (-(y & 1) & MATRIX_A);
    }
    y = (state->key[N-1] & UPPER_MASK) | (state->key[0] & LOWER_MASK);
    state->key[N-1] = state->key[M-1] ^ (y>>1) ^ (-(y & 1) & MATRIX_A);

    state->pos = 0;
  }
  
  y = state->key[state->pos++];

  /* Tempering */
  y ^= (y >> 11);
  y ^= (y << 7) & 0x9d2c5680UL;
  y ^= (y << 15) & 0xefc60000UL;
  y ^= (y >> 18);

  return y;
}

long rk_long(rk_state *state)
{
  return rk_ulong(state) >> 1;
}

unsigned long rk_ulong(rk_state *state)
{
#if ULONG_MAX <= 0xffffffffUL
  return rk_random(state);
#else
  /* Assumes 64 bits */
  return (rk_random(state) << 32) | (rk_random(state));
#endif
}

unsigned long rk_interval(unsigned long max, rk_state *state)
{
  unsigned long mask = max, value;

  if (max == 0) return 0;

  /* Smallest bit mask >= max */
  mask |= mask >> 1;
  mask |= mask >> 2;
  mask |= mask >> 4;
  mask |= mask >> 8;
  mask |= mask >> 16;
#if ULONG_MAX > 0xffffffffUL
  mask |= mask >> 32;
#endif

  /* Search a random value in [0..mask] <= max */
  while ((value = (rk_ulong(state) & mask)) > max);

  return value;
}

double rk_double(rk_state *state)
{
  /* shifts : 67108864 = 0x4000000, 9007199254740992 = 0x20000000000000 */
  long a = rk_random(state) >> 5, b = rk_random(state) >> 6;
  return (a * 67108864.0 + b) / 9007199254740992.0;
}

void rk_copy(rk_state *copy, rk_state *orig)
{
  memcpy(copy, orig, sizeof(rk_state));
}

void rk_fill(void *buffer, size_t size, rk_state *state)
{
  unsigned long r;
  unsigned char *buf = buffer;
  rk_state tempstate;
  
  if (size > 0 && state == NULL)
  {
    rk_randomseed(&tempstate);
    state = &tempstate;
  }
  
  for (; size >= 4; size -= 4)
  {
    r = rk_random(state);
    *(buf++) = r & 0xFF;
    *(buf++) = (r >> 8) & 0xFF;
    *(buf++) = (r >> 16) & 0xFF;
    *(buf++) = (r >> 24) & 0xFF;
  }
  
  if (!size) return;

  r = rk_random(state);

  for (; size; r >>= 8, size --)
    *(buf++) = (unsigned char)(r & 0xFF);
}

rk_error rk_devfill(void *buffer, size_t size, int strong)
{
#ifndef _WIN32
  FILE *rfile;
  int done;

  if (strong)
    rfile = fopen(RK_DEV_RANDOM, "rb");
  else
    rfile = fopen(RK_DEV_URANDOM, "rb");
  if (rfile == NULL)
    return RK_ENODEV;
  done = fread(buffer, size, 1, rfile);
  fclose(rfile);
  if (done)
    return RK_NOERR;
#else

#ifndef RK_NO_WINCRYPT
  HCRYPTPROV hCryptProv;
  BOOL done;

  if (!CryptAcquireContext(&hCryptProv, NULL, NULL, PROV_RSA_FULL,
    CRYPT_VERIFYCONTEXT) || !hCryptProv)
    return RK_ENODEV;
  done = CryptGenRandom(hCryptProv, (DWORD)size, (unsigned char *)buffer);
  CryptReleaseContext(hCryptProv, 0);
  if (done)
    return RK_NOERR;
#endif

#endif

  return RK_ENODEV;
}

rk_error rk_altfill(void *buffer, size_t size, int strong, rk_state *state)
{
  rk_error err;

  err = rk_devfill(buffer, size, strong);
  if (err)
    rk_fill(buffer, size, state);

  return err;
}

double rk_gauss(rk_state *state)
{
  if (state->has_gauss)
  {
    state->has_gauss = 0;
    return state->gauss;
  }
  else
  {
    double f, x1, x2, r2;
    do
    {
      x1 = 2.0*rk_double(state) - 1.0;
      x2 = 2.0*rk_double(state) - 1.0;
      r2 = x1*x1 + x2*x2;
    }
    while (r2 >= 1.0 || r2 == 0.0);
    
    f = sqrt(-2.0*log(r2)/r2); /* Box-Muller transform */
    state->has_gauss = 1;
    state->gauss = f*x1; /* Keep for next call */
    return f*x2;
  }
}
