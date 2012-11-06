/* Random kit 1.6 */

/*
 * Anyone who attempts to generate random numbers by deterministic means is, 
 * of course, living in a state of sin.
 * -- John von Neumann 
 */

/*
 * Copyright (c) 2003-2006, Jean-Sebastien Roy (js@jeannot.org)
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

/* @(#) $Jeannot: rk_mt.h,v 1.6 2006/02/20 19:13:37 js Exp $ */

/*
 * Typical use:
 *
 * {
 *   rk_state state;
 *   unsigned long seed = 1, random_value;
 *   
 *   rk_seed(seed, &state); // Initialize the RNG
 *   ...
 *   random_value = rk_random(&state); // a random value in [0..RK_MAX]
 * }
 * 
 * Instead of rk_seed, you can use rk_randomseed which will get a random seed
 * from /dev/urandom (or the clock, if /dev/urandom is unavailable):
 *
 * {
 *   rk_state state;
 *   unsigned long random_value;
 *   
 *   rk_randomseed(&state); // Initialize the RNG with a random seed
 *   ...
 *   random_value = rk_random(&state); // a random value in [0..RK_MAX]
 * }
 */
 
/*
 * Useful macro:
 *   RK_DEV_RANDOM: the device used for random seeding.
 *                 defaults to "/dev/urandom"
 */

#include <stddef.h>

#ifndef _RK_MT_
#define _RK_MT_

#define  RK_STATE_LEN 624

typedef struct rk_state_
{
  unsigned long key[RK_STATE_LEN];
  int pos;
  int has_gauss; /* !=0: gauss contains a gaussian deviate */
  double gauss;
}
rk_state;

typedef enum {
  RK_NOERR = 0, /* no error */
  RK_ENODEV = 1, /* no RK_DEV_RANDOM device */
  RK_ERR_MAX = 2
} rk_error;

/* error strings */
extern char *rk_strerror[RK_ERR_MAX];

/* Maximum generated random value */
#define RK_MAX 0xFFFFFFFFUL

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize the RNG state using the given seed.
 */
extern void rk_seed(unsigned long seed, rk_state *state);

/*
 * Initialize the RNG state using a random seed.
 * Uses /dev/urandom or, when unavailable, the clock (see rk_mt.c).
 * Returns RK_NOERR when no errors occurs.
 * Returns RK_ENODEV when the use of RK_DEV_RANDOM failed (for example because
 * there is no such device). In this case, the RNG was initialized using the
 * clock.
 */
extern rk_error rk_randomseed(rk_state *state);

/*
 * Returns a random unsigned long between 0 and RK_MAX inclusive
 */
extern unsigned long rk_random(rk_state *state);

/*
 * Returns a random long between 0 and LONG_MAX inclusive
 */
extern long rk_long(rk_state *state);

/*
 * Returns a random unsigned long between 0 and ULONG_MAX inclusive
 */
extern unsigned long rk_ulong(rk_state *state);

/*
 * Returns a random unsigned long between 0 and max inclusive.
 */
extern unsigned long rk_interval(unsigned long max, rk_state *state);

/*
 * Returns a random double between 0.0 and 1.0, 1.0 excluded.
 */
extern double rk_double(rk_state *state);

/*
 * Copy a random generator state.
 */
extern void rk_copy(rk_state *copy, rk_state *orig);

/*
 * fill the buffer with size random bytes.
 * If state == NULL, the random generator is inilialized using rk_randomseed.
 * Calling multiple times rk_randomseed should be avoided therefore calling
 * multiple times rk_fill with state == NULL should be avoided.
 */
extern void rk_fill(void *buffer, size_t size, rk_state *state);

/*
 * fill the buffer with randombytes from the random device
 * Returns RK_ENODEV if the device is unavailable, or RK_NOERR if it is
 * On Unix, if strong is defined, RK_DEV_RANDOM is used. If not, RK_DEV_URANDOM
 * is used instead. This parameter has no effect on Windows.
 * Warning: on most unixes RK_DEV_RANDOM will wait for enough entropy to answer
 * which can take a very long time on quiet systems.
 */
extern rk_error rk_devfill(void *buffer, size_t size, int strong);

/*
 * fill the buffer using rk_devfill if the random device is available and using
 * rk_fill if is is not
 * parameters have the same meaning as rk_fill and rk_devfill
 * Returns RK_ENODEV if the device is unavailable, or RK_NOERR if it is
 */
extern rk_error rk_altfill(void *buffer, size_t size, int strong,
  rk_state *state);

/*
 * return a random gaussian deviate with variance unity and zero mean.
 */
extern double rk_gauss(rk_state *state);

/* Utility functions */

/*
 * fill the key vector using Knuth RNG as used in MT reference implementation
 * using the provided seed. The key vector length is len.
 */
extern void rk_knuth_fill(unsigned long seed, unsigned long *key,
  unsigned long len);

/*
 * return a random unsigned long based upon the system state (clock, pid)
 * used as seed when there is no random device.
 */
extern unsigned long rk_seedfromsystem(void);


#ifdef __cplusplus
}
#endif

#endif /* _RK_MT_ */
