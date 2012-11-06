/* Random kit 1.6 */

/*
 * Copyright (c) 2004-2006, Jean-Sebastien Roy (js@jeannot.org)
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

/* @(#) $Jeannot: rk_sobol.h,v 1.7 2006/02/19 13:48:34 js Exp $ */

/*
 * Typical use:
 * 
 * int dimension = 2;
 * rk_sobol_state s;
 * rk_sobol_error rc;
 * double x[dimension], y[dimension];
 * 
 * // Init
 * if (rc = rk_sobol_init(dimension, &s, NULL, NULL, NULL))
 * {
 *   fprintf(stderr, "%s\n", rk_sobol_strerror[rc]);
 *   abort();
 * }
 *   
 * // Draw uniform quasirandom doubles
 * if (rc = rk_sobol_double(&s, x))
 * {
 *   fprintf(stderr, "%s\n", rk_sobol_strerror[rc]);
 *   abort();
 * }
 *   
 * // Draw gaussian quasirandom doubles
 * if (rc = rk_sobol_gauss(&s, y))
 * {
 *   fprintf(stderr, "%s\n", rk_sobol_strerror[rc]);
 *   abort();
 * }
 * 
 * // Free allocated memory
 * rk_sobol_free(&s);
 */


#ifndef _RK_SOBOL_
#define _RK_SOBOL_

#include "rk_mt.h"

typedef enum {
  RK_SOBOL_OK      = 0, /* No error */
  RK_SOBOL_EINVAL  = 1, /* Invalid dimension (<= 0 or too large) */
  RK_SOBOL_EXHAUST = 2, /* Too many number generated */
  RK_SOBOL_ENOMEM  = 3, /* Not enough memory */
  RK_SOBOL_ERR_MAX = 4
} rk_sobol_error;

/* error strings */
extern char *rk_sobol_strerror[];

typedef struct
{
  size_t dimension;
  unsigned long *direction;
  unsigned long *numerator;
  unsigned long count;
  unsigned long gcount;
} rk_sobol_state;

#ifdef __cplusplus
extern "C" {
#endif

/* Sobol directions initializations (zero terminated lists) */

/*
 * Sobol/Levitan coefficients of the free direction integers as given
 * by Bratley, P., Fox, B.L. (1988)
 * Defined up to dimension 40.
 */
extern const unsigned long rk_sobol_SLdirections[];

/*
 * Lemieux coefficients of the free direction integers as given
 * in QuantLib by Christiane Lemieux, private communication, September 2004
 * Defined up to dimension 360.
 */
extern const unsigned long rk_sobol_Ldirections[];

/*
 * Peter Jäckel coefficients of the free direction integers as given
 * in "Monte Carlo Methods in Finance", by Peter Jäckel, section 8.3
 * Defined up to dimension 32.
 */
extern const unsigned long rk_sobol_Jdirections[];

/*
 * Initialize a sobol quasirandom number generator. 
 * 1 <= dimension <= the number of primitive polylonimals of degree < LONG_BIT
 * If directions == NULL (or more directions than provided are required),
 * the directions are picked at random using rs_dir.
 * If rs_dir == NULL, it is initialized using rk_randomseed.
 * polynomials is a zero terminated list of primitive polynomials to use if
 * it is != NULL to speed up initialization for dimension > 1024.
 */
extern rk_sobol_error rk_sobol_init(size_t dimension, rk_sobol_state *s, 
  rk_state *rs_dir, const unsigned long *directions,
  const unsigned long *polynomials);

/*
 * Reinitialize the random generator with same directions.
 */
extern void rk_sobol_reinit(rk_sobol_state *s);

/*
 * You can change the starting rank in the sequence by changing s->count.
 */
extern void rk_sobol_setcount(rk_sobol_state *s, unsigned long count);

/*
 * XOR the numerators at random using rs_num.
 * To be used once, after (re-)initialization.
 * Useful for randomized quasi monte carlo.
 * If rs_num == NULL, it is initialized using rk_randomseed.
 */
extern void rk_sobol_randomshift(rk_sobol_state *s, rk_state *rs_num);

/*
 * Copy a sobol generator.
 * Can be used to avoid the time consuming initialization.
 */
extern rk_sobol_error rk_sobol_copy(rk_sobol_state *copy, rk_sobol_state *orig);

/*
 * Free the memory allocated by rk_sobol_init
 */
extern void rk_sobol_free(rk_sobol_state *s);

/*
 * return a vector of dimension quasirandom uniform deviates between 0 and 1
 */
extern rk_sobol_error rk_sobol_double(rk_sobol_state *s, double *x);

/*
 * return a vector of dimension quasirandom gaussian deviates
 * with variance unity and zero mean.
 * On Windows, the standard function erfc is missing, which results in
 * lower precision (9 digits instead of full precision).
 */
extern rk_sobol_error rk_sobol_gauss(rk_sobol_state *s, double *x);

#ifdef __cplusplus
}
#endif

#endif /* _RK_SOBOL_ */
