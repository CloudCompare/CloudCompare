/* Random kit 1.6 */

/*
 * Copyright (c) 2004-2006, Jean-Sebastien Roy (js@jeannot.org)
 *
 * ISAAC RNG by By Bob Jenkins. Based on Bob Jenkins public domain code.
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

/* @(#) $Jeannot: rk_isaac.h,v 1.2 2006/02/19 14:40:26 js Exp $ */

/*
 * Typical use:
 *
 * {
 *   rk_isaac_state state;
 *   unsigned long seed = 1, random_value;
 *   
 *   rk_isaac_seed(seed, &state); // Initialize the RNG
 *   ...
 *   random_value = rk_isaac_random(&state); // a random value in [0..RK_MAX]
 * }
 * 
 * Instead of rk_isaac_seed, you can use rk_isaac_randomseed which will get a
 * random seed from /dev/random (or the clock, if /dev/random is unavailable):
 *
 * {
 *   rk_isaac_state state;
 *   unsigned long random_value;
 *   
 *   rk_isaac_randomseed(&state); // Initialize the RNG with a random seed
 *   ...
 *   random_value = rk_isaac_random(&state); // a random value in [0..RK_MAX]
 * }
 */

#ifndef _RK_ISAAC_
#define _RK_ISAAC_

#include "rk_mt.h"

#define RK_ISAAC_STATE_POW 8
#define RK_ISAAC_STATE_LEN (1<<RK_ISAAC_STATE_POW)

/* context of random number generator */
typedef struct rk_isaac_state_
{
  unsigned long randcnt;
  unsigned long randrsl[RK_ISAAC_STATE_LEN];
  unsigned long randmem[RK_ISAAC_STATE_LEN];
  unsigned long randa;
  unsigned long randb;
  unsigned long randc;
  int has_gauss; /* !=0: gauss contains a gaussian deviate */
  double gauss;
} rk_isaac_state;

/*
 * Initialize the RNG state using the given seed.
 */
extern void rk_isaac_seed(unsigned long seed, rk_isaac_state *state);

/*
 * Initialize the RNG state using a random seed.
 * Uses /dev/random or, when unavailable, the clock (see rk_mt.c).
 * Returns RK_NOERR when no errors occurs.
 * Returns RK_ENODEV when the use of RK_DEV_RANDOM failed (for example because
 * there is no such device). In this case, the RNG was initialized using the
 * clock.
 */
extern rk_error rk_isaac_randomseed(rk_isaac_state *state);

/*
 * Returns a random unsigned long between 0 and RK_MAX inclusive
 */
extern unsigned long rk_isaac_random(rk_isaac_state *state);

/*
 * Returns a random long between 0 and LONG_MAX inclusive
 */
extern long rk_isaac_long(rk_isaac_state *state);

/*
 * Returns a random unsigned long between 0 and ULONG_MAX inclusive
 */
extern unsigned long rk_isaac_ulong(rk_isaac_state *state);

/*
 * Returns a random unsigned long between 0 and max inclusive.
 */
extern unsigned long rk_isaac_interval(unsigned long max,
  rk_isaac_state *state);

/*
 * Returns a random double between 0.0 and 1.0, 1.0 excluded.
 */
extern double rk_isaac_double(rk_isaac_state *state);

/*
 * return a random gaussian deviate with variance unity and zero mean.
 */
extern double rk_isaac_gauss(rk_isaac_state *state);

/*
 * Copy a random generator state.
 */
extern void rk_isaac_copy(rk_isaac_state *copy, rk_isaac_state *orig);

/*
 * fill the buffer with size random bytes.
 * If state == NULL, the random generator is inilialized using
 * rk_isaac_randomseed.
 * Calling multiple times rk_isaac_randomseed should be avoided therefore
 * calling multiple times rk_isaac_fill with state == NULL should be avoided.
 */
extern void rk_isaac_fill(void *buffer, size_t size, rk_isaac_state *state);

/*
 * seed the Mersenne Twister PRNG using an ISAAC RNG. if i_state == NULL, the
 * ISAAC random generator is inilialized using rk_isaac_randomseed.
 */
extern void rk_seed_isaac(rk_isaac_state *i_state, rk_state *state);

#endif /* _RK_ISAAC_ */
