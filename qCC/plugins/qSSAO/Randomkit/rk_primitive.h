/* Random kit 1.6 */
/* Primitivity test for binary polynomials of low degree */

/*
 * Copyright (c) 2005-2006, Jean-Sebastien Roy (js@jeannot.org)
 * 
 * Methodology inspired by scott duplichan's ppsearch code.
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

/* @(#) $Jeannot: rk_primitive.h,v 1.6 2006/02/19 13:48:34 js Exp $ */

#ifndef _RK_PRIMITIVE_
#define _RK_PRIMITIVE_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Return 1 if the binary polynomial is primitive.
 *
 * Note that if p is primitive, the the polynomial obtained by reversing the
 * bits of p is also primitive. (see list_primitive.c for an example)
 *
 * Typical use:
 *   int test;
 *   test = rk_isprimitive(3, &divisors);
 */
extern int rk_isprimitive(unsigned long polynomial);

#ifdef __cplusplus
}
#endif

#endif /* _RK_PRIMITIVE_ */
