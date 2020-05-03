/* Random kit 1.6 */

/*
 * Copyright (c) 2004-2006, Jean-Sebastien Roy (js@jeannot.org)
 *
 * ISAAC RNG by By Bob Jenkins. Based on Bob Jenkins public domain code.
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
  "@(#) $Jeannot: rk_isaac.c,v 1.4 2006/02/20 19:13:37 js Exp $";

#include "rk_isaac.h"
#include <limits.h>
#include <math.h>
#include <string.h>

/* use the contents of randrsl[0..RK_ISAAC_STATE_LEN-1] as the seed. */
static void isaac_init(rk_isaac_state *rg);

void rk_isaac_seed(unsigned long seed, rk_isaac_state *state)
{
  rk_knuth_fill(seed, state->randrsl, RK_ISAAC_STATE_LEN);
  isaac_init(state);
}

rk_error rk_isaac_randomseed(rk_isaac_state *state)
{
  if(rk_devfill(state->randrsl, sizeof(state->randrsl), 1) == RK_NOERR)
  {
    isaac_init(state);
    return RK_NOERR;
  }

  rk_isaac_seed(rk_seedfromsystem(), state);

  return RK_ENODEV;
}

#define ind(mm,x)  ((mm)[(x>>2)&(RK_ISAAC_STATE_LEN-1)])
#define rngstep(mix,a,b,mm,m,m2,r,x) \
{ \
  x = *m;  \
  a = ((a^(mix)) + *(m2++)) & 0xffffffff; \
  *(m++) = y = (ind(mm,x) + a + b) & 0xffffffff; \
  *(r++) = b = (ind(mm,y>>RK_ISAAC_STATE_POW) + x) & 0xffffffff; \
}

/* Call rand(rk_isaac_state *r) to retrieve a single 32-bit random value */
unsigned long rk_isaac_random(rk_isaac_state *state)
{
  if (!state->randcnt--)
  {
    register unsigned long a,b,x,y,*m,*mm,*m2,*r,*mend;
    mm=state->randmem; r=state->randrsl;
    a = state->randa; b = (state->randb + (++state->randc)) & 0xffffffff;
    for (m = mm, mend = m2 = m+(RK_ISAAC_STATE_LEN/2); m<mend; )
    {
      rngstep( a<<13, a, b, mm, m, m2, r, x);
      rngstep( a>>6 , a, b, mm, m, m2, r, x);
      rngstep( a<<2 , a, b, mm, m, m2, r, x);
      rngstep( a>>16, a, b, mm, m, m2, r, x);
    }
    for (m2 = mm; m2<mend; )
    {
      rngstep( a<<13, a, b, mm, m, m2, r, x);
      rngstep( a>>6 , a, b, mm, m, m2, r, x);
      rngstep( a<<2 , a, b, mm, m, m2, r, x);
      rngstep( a>>16, a, b, mm, m, m2, r, x);
    }
    state->randb = b; state->randa = a;
    state->randcnt=RK_ISAAC_STATE_LEN-1;
  }
  return state->randrsl[state->randcnt] & 0xFFFFFFFF;
}

#define mix(a,b,c,d,e,f,g,h) \
{ \
  a^=b<<11; d+=a; b+=c; \
  b^=(c & 0xFFFFFFFF)>>2;  e+=b; c+=d; \
  c^=d<<8;  f+=c; d+=e; \
  d^=(e & 0xFFFFFFFF)>>16; g+=d; e+=f; \
  e^=f<<10; h+=e; f+=g; \
  f^=(g & 0xFFFFFFFF)>>4;  a+=f; g+=h; \
  g^=h<<8;  b+=g; h+=a; \
  h^=(a & 0xFFFFFFFF)>>9;  c+=h; a+=b; \
}

/* if (flag==1), then use the contents of randrsl[] to initialize mm[]. */
void isaac_init(rk_isaac_state *rk_isaac_state)
{
  int i;
  unsigned long a,b,c,d,e,f,g,h;
  unsigned long *m,*r;
  rk_isaac_state->randa = rk_isaac_state->randb = rk_isaac_state->randc = 0;
  m=rk_isaac_state->randmem;
  r=rk_isaac_state->randrsl;
  a=b=c=d=e=f=g=h=0x9e3779b9;  /* the golden ratio */

  for (i=0; i<4; ++i)          /* scramble it */
    mix(a,b,c,d,e,f,g,h);

  /* initialize using the contents of r[] as the seed */
  for (i=0; i<RK_ISAAC_STATE_LEN; i+=8)
  {
    a+=r[i  ]; b+=r[i+1]; c+=r[i+2]; d+=r[i+3];
    e+=r[i+4]; f+=r[i+5]; g+=r[i+6]; h+=r[i+7];
    mix(a,b,c,d,e,f,g,h);
    m[i  ]=a; m[i+1]=b; m[i+2]=c; m[i+3]=d;
    m[i+4]=e; m[i+5]=f; m[i+6]=g; m[i+7]=h;
  }
  /* do a second pass to make all of the seed affect all of m */
  for (i=0; i<RK_ISAAC_STATE_LEN; i+=8)
  {
    a+=m[i  ]; b+=m[i+1]; c+=m[i+2]; d+=m[i+3];
    e+=m[i+4]; f+=m[i+5]; g+=m[i+6]; h+=m[i+7];
    mix(a,b,c,d,e,f,g,h);
    m[i  ]=a; m[i+1]=b; m[i+2]=c; m[i+3]=d;
    m[i+4]=e; m[i+5]=f; m[i+6]=g; m[i+7]=h;
  }

  rk_isaac_state->randcnt=0;
  rk_isaac_state->has_gauss=0;
}

long rk_isaac_long(rk_isaac_state *state)
{
  return rk_isaac_ulong(state) >> 1;
}

unsigned long rk_isaac_ulong(rk_isaac_state *state)
{
#if ULONG_MAX <= 0xffffffffUL
  return rk_isaac_random(state);
#else
  /* Assumes 64 bits */
  return (rk_isaac_random(state) << 32) | (rk_isaac_random(state));
#endif
}

unsigned long rk_isaac_interval(unsigned long max, rk_isaac_state *state)
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
  while ((value = (rk_isaac_ulong(state) & mask)) > max);

  return value;
}

double rk_isaac_double(rk_isaac_state *state)
{
  /* shifts : 67108864 = 0x4000000, 9007199254740992 = 0x20000000000000 */
  long a = rk_isaac_random(state) >> 5, b = rk_isaac_random(state) >> 6;
  return (a * 67108864.0 + b) / 9007199254740992.0;
}

void rk_isaac_copy(rk_isaac_state *copy, rk_isaac_state *orig)
{
  memcpy(copy, orig, sizeof(rk_isaac_state));
}

double rk_isaac_gauss(rk_isaac_state *state)
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
      x1 = 2.0*rk_isaac_double(state) - 1.0;
      x2 = 2.0*rk_isaac_double(state) - 1.0;
      r2 = x1*x1 + x2*x2;
    }
    while (r2 >= 1.0 || r2 == 0.0);
    
    f = sqrt(-2.0*log(r2)/r2); /* Box-Muller transform */
    state->has_gauss = 1;
    state->gauss = f*x1; /* Keep for next call */
    return f*x2;
  }
}

void rk_isaac_fill(void *buffer, size_t size, rk_isaac_state *state)
{
  unsigned long r;
  unsigned char *buf = buffer;
  rk_isaac_state tempstate;
  
  if (size > 0 && state == NULL)
  {
    rk_isaac_randomseed(&tempstate);
    state = &tempstate;
  }
  
  for (; size >= 4; size -= 4)
  {
    r = rk_isaac_random(state);
    *(buf++) = r & 0xFF;
    *(buf++) = (r >> 8) & 0xFF;
    *(buf++) = (r >> 16) & 0xFF;
    *(buf++) = (r >> 24) & 0xFF;
  }
  
  if (!size) return;

  r = rk_isaac_random(state);

  for (; size; r >>= 8, size --)
    *(buf++) = (unsigned char)(r & 0xFF);
}

void rk_seed_isaac(rk_isaac_state *i_state, rk_state *state)
{
  rk_isaac_fill(state->key, sizeof(state->key), i_state);
  state->pos = RK_STATE_LEN;
  state->has_gauss = 0;
}
