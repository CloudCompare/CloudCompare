/* Random kit 1.6 */

/*
 * Copyright (c) 2004-2006, Jean-Sebastien Roy (js@jeannot.org)
 *
 * Original algorithm from Numerical Recipes, 2nd edition, by Press et al.
 * The inverse normal cdf formulas are from Peter J. Acklam.
 * The initialization directions were found in Ferdinando Ametrano's
 * implementation in QuantLib.
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
  "@(#) $Jeannot: rk_sobol.c,v 1.9 2006/02/19 13:48:34 js Exp $";

#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include "rk_sobol.h"
#include "rk_mt.h"
#include "rk_primitive.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_SQRT1_2
#define  M_SQRT1_2  0.70710678118654752440  /* 1/sqrt(2) */
#endif
#define RK_SOBOL_M_SQRT2PI 2.506628274631000502415 /* sqrt(2*pi) */

#ifndef LONG_BIT
#if ULONG_MAX <= 0xffffffffUL
#define LONG_BIT 32
#else
#define LONG_BIT 64
#endif
#endif

#ifdef _WIN32
#include "erfc.h"
#endif


char *rk_sobol_strerror[] =
{
  "no error",
  "invalid dimension",
  "too many numbers generated",
  "not enough memory"
};

static double inverse_normal(double p);

/*
 * Sobol/Levitan coefficients of the free direction integers as given
 * by Bratley, P., Fox, B.L. (1988)
 */

const unsigned long rk_sobol_SLdirections[] = {
  1,
  1, 1,
  1, 3, 7,
  1, 1, 5,
  1, 3, 1, 1,
  1, 1, 3, 7,
  1, 3, 3, 9, 9,
  1, 3, 7, 13, 3,
  1, 1, 5, 11, 27,
  1, 3, 5, 1, 15,
  1, 1, 7, 3, 29,
  1, 3, 7, 7, 21,
  1, 1, 1, 9, 23, 37,
  1, 3, 3, 5, 19, 33,
  1, 1, 3, 13, 11, 7,
  1, 1, 7, 13, 25, 5,
  1, 3, 5, 11, 7, 11,
  1, 1, 1, 3, 13, 39,
  1, 3, 1, 15, 17, 63, 13,
  1, 1, 5, 5, 1, 27, 33,
  1, 3, 3, 3, 25, 17, 115,
  1, 1, 3, 15, 29, 15, 41,
  1, 3, 1, 7, 3, 23, 79,
  1, 3, 7, 9, 31, 29, 17,
  1, 1, 5, 13, 11, 3, 29,
  1, 3, 1, 9, 5, 21, 119,
  1, 1, 3, 1, 23, 13, 75,
  1, 3, 3, 11, 27, 31, 73,
  1, 1, 7, 7, 19, 25, 105,
  1, 3, 5, 5, 21, 9, 7,
  1, 1, 1, 15, 5, 49, 59,
  1, 1, 1, 1, 1, 33, 65,
  1, 3, 5, 15, 17, 19, 21,
  1, 1, 7, 11, 13, 29, 3,
  1, 3, 7, 5, 7, 11, 113,
  1, 1, 5, 3, 15, 19, 61,
  1, 3, 1, 1, 9, 27, 89, 7,
  1, 1, 3, 7, 31, 15, 45, 23,
  1, 3, 3, 9, 9, 25, 107, 39,
  0
};

/*
 * Lemieux coefficients of the free direction integers as given
 * in QuantLib by Christiane Lemieux, private communication, September 2004
 */

const unsigned long rk_sobol_Ldirections[] = {
  1,
  1, 1,
  1, 3, 7,
  1, 1, 5,
  1, 3, 1, 1,
  1, 1, 3, 7,
  1, 3, 3, 9, 9,
  1, 3, 7, 13, 3,
  1, 1, 5, 11, 27,
  1, 3, 5, 1, 15,
  1, 1, 7, 3, 29,
  1, 3, 7, 7, 21,
  1, 1, 1, 9, 23, 37,
  1, 3, 3, 5, 19, 33,
  1, 1, 3, 13, 11, 7,
  1, 1, 7, 13, 25, 5,
  1, 3, 5, 11, 7, 11,
  1, 1, 1, 3, 13, 39,
  1, 3, 1, 15, 17, 63, 13,
  1, 1, 5, 5, 1, 27, 33,
  1, 3, 3, 3, 25, 17, 115,
  1, 1, 3, 15, 29, 15, 41,
  1, 3, 1, 7, 3, 23, 79,
  1, 3, 7, 9, 31, 29, 17,
  1, 1, 5, 13, 11, 3, 29,
  1, 3, 1, 9, 5, 21, 119,
  1, 1, 3, 1, 23, 13, 75,
  1, 3, 3, 11, 27, 31, 73,
  1, 1, 7, 7, 19, 25, 105,
  1, 3, 5, 5, 21, 9, 7,
  1, 1, 1, 15, 5, 49, 59,
  1, 1, 1, 1, 1, 33, 65,
  1, 3, 5, 15, 17, 19, 21,
  1, 1, 7, 11, 13, 29, 3,
  1, 3, 7, 5, 7, 11, 113,
  1, 1, 5, 3, 15, 19, 61,
  1, 3, 1, 1, 9, 27, 89, 7,
  1, 1, 3, 7, 31, 15, 45, 23,
  1, 3, 3, 9, 9, 25, 107, 39,
  1, 1, 3, 13, 7, 35, 61, 91,
  1, 1, 7, 11, 5, 35, 55, 75,
  1, 3, 5, 5, 11, 23, 29, 139,
  1, 1, 1, 7, 11, 15, 17, 81,
  1, 1, 7, 9, 5, 57, 79, 103,
  1, 1, 7, 13, 19, 5, 5, 185,
  1, 3, 1, 3, 13, 57, 97, 131,
  1, 1, 5, 5, 21, 25, 125, 197,
  1, 3, 3, 9, 31, 11, 103, 201,
  1, 1, 5, 3, 7, 25, 51, 121,
  1, 3, 7, 15, 19, 53, 73, 189,
  1, 1, 1, 15, 19, 55, 27, 183,
  1, 1, 7, 13, 3, 29, 109, 69,
  1, 1, 5, 15, 15, 23, 15, 1, 57,
  1, 3, 1, 3, 23, 55, 43, 143, 397,
  1, 1, 3, 11, 29, 9, 35, 131, 411,
  1, 3, 1, 7, 27, 39, 103, 199, 277,
  1, 3, 7, 3, 19, 55, 127, 67, 449,
  1, 3, 7, 3, 5, 29, 45, 85, 3,
  1, 3, 5, 5, 13, 23, 75, 245, 453,
  1, 3, 1, 15, 21, 47, 3, 77, 165,
  1, 1, 7, 9, 15, 5, 117, 73, 473,
  1, 3, 1, 9, 1, 21, 13, 173, 313,
  1, 1, 7, 3, 11, 45, 63, 77, 49,
  1, 1, 1, 1, 1, 25, 123, 39, 259,
  1, 1, 1, 5, 23, 11, 59, 11, 203,
  1, 3, 3, 15, 21, 1, 73, 71, 421,
  1, 1, 5, 11, 15, 31, 115, 95, 217,
  1, 1, 3, 3, 7, 53, 37, 43, 439,
  1, 1, 1, 1, 27, 53, 69, 159, 321,
  1, 1, 5, 15, 29, 17, 19, 43, 449,
  1, 1, 3, 9, 1, 55, 121, 205, 255,
  1, 1, 3, 11, 9, 47, 107, 11, 417,
  1, 1, 1, 5, 17, 25, 21, 83, 95,
  1, 3, 5, 13, 31, 25, 61, 157, 407,
  1, 1, 7, 9, 25, 33, 41, 35, 17,
  1, 3, 7, 15, 13, 39, 61, 187, 461,
  1, 3, 7, 13, 5, 57, 23, 177, 435,
  1, 1, 3, 15, 11, 27, 115, 5, 337,
  1, 3, 7, 3, 15, 63, 61, 171, 339,
  1, 3, 3, 13, 15, 61, 59, 47, 1,
  1, 1, 5, 15, 13, 5, 39, 83, 329,
  1, 1, 5, 5, 5, 27, 25, 39, 301,
  1, 1, 5, 11, 31, 41, 35, 233, 27,
  1, 3, 5, 15, 7, 37, 119, 171, 419,
  1, 3, 5, 5, 3, 29, 21, 189, 417,
  1, 1, 1, 1, 21, 41, 117, 119, 351,
  1, 1, 3, 1, 7, 27, 87, 19, 213,
  1, 1, 1, 1, 17, 7, 97, 217, 477,
  1, 1, 7, 1, 29, 61, 103, 231, 269,
  1, 1, 7, 13, 9, 27, 107, 207, 311,
  1, 1, 7, 5, 25, 21, 107, 179, 423,
  1, 3, 5, 11, 7, 1, 17, 245, 281,
  1, 3, 5, 9, 1, 5, 53, 59, 125,
  1, 1, 7, 1, 31, 57, 71, 245, 125,
  1, 1, 7, 5, 5, 57, 53, 253, 441,
  1, 3, 1, 13, 19, 35, 119, 235, 381,
  1, 3, 1, 7, 19, 59, 115, 33, 361,
  1, 1, 3, 5, 13, 1, 49, 143, 501,
  1, 1, 3, 5, 1, 63, 101, 85, 189,
  1, 1, 5, 11, 27, 63, 13, 131, 5,
  1, 1, 5, 7, 15, 45, 75, 59, 455, 585,
  1, 3, 1, 3, 7, 7, 111, 23, 119, 959,
  1, 3, 3, 9, 11, 41, 109, 163, 161, 879,
  1, 3, 5, 1, 21, 41, 121, 183, 315, 219,
  1, 1, 3, 9, 15, 3, 9, 223, 441, 929,
  1, 1, 7, 9, 3, 5, 93, 57, 253, 457,
  1, 1, 7, 13, 15, 29, 83, 21, 35, 45,
  1, 1, 3, 7, 13, 61, 119, 219, 85, 505,
  1, 1, 3, 3, 17, 13, 35, 197, 291, 109,
  1, 1, 3, 3, 5, 1, 113, 103, 217, 253,
  1, 1, 7, 1, 15, 39, 63, 223, 17, 9,
  1, 3, 7, 1, 17, 29, 67, 103, 495, 383,
  1, 3, 3, 15, 31, 59, 75, 165, 51, 913,
  1, 3, 7, 9, 5, 27, 79, 219, 233, 37,
  1, 3, 5, 15, 1, 11, 15, 211, 417, 811,
  1, 3, 5, 3, 29, 27, 39, 137, 407, 231,
  1, 1, 3, 5, 29, 43, 125, 135, 109, 67,
  1, 1, 1, 5, 11, 39, 107, 159, 323, 381,
  1, 1, 1, 1, 9, 11, 33, 55, 169, 253,
  1, 3, 5, 5, 11, 53, 63, 101, 251, 897,
  1, 3, 7, 1, 25, 15, 83, 119, 53, 157,
  1, 3, 5, 13, 5, 5, 3, 195, 111, 451,
  1, 3, 1, 15, 11, 1, 19, 11, 307, 777,
  1, 3, 7, 11, 5, 5, 17, 231, 345, 981,
  1, 1, 3, 3, 1, 33, 83, 201, 57, 475,
  1, 3, 7, 7, 17, 13, 35, 175, 499, 809,
  1, 1, 5, 3, 3, 17, 103, 119, 499, 865,
  1, 1, 1, 11, 27, 25, 37, 121, 401, 11,
  1, 1, 1, 11, 9, 25, 25, 241, 403, 3,
  1, 1, 1, 1, 11, 1, 39, 163, 231, 573,
  1, 1, 1, 13, 13, 21, 75, 185, 99, 545,
  1, 1, 1, 15, 3, 63, 69, 11, 173, 315,
  1, 3, 5, 15, 11, 3, 95, 49, 123, 765,
  1, 1, 1, 15, 3, 63, 77, 31, 425, 711,
  1, 1, 7, 15, 1, 37, 119, 145, 489, 583,
  1, 3, 5, 15, 3, 49, 117, 211, 165, 323,
  1, 3, 7, 1, 27, 63, 77, 201, 225, 803,
  1, 1, 1, 11, 23, 35, 67, 21, 469, 357,
  1, 1, 7, 7, 9, 7, 25, 237, 237, 571,
  1, 1, 3, 15, 29, 5, 107, 109, 241, 47,
  1, 3, 5, 11, 27, 63, 29, 13, 203, 675,
  1, 1, 3, 9, 9, 11, 103, 179, 449, 263,
  1, 3, 5, 11, 29, 63, 53, 151, 259, 223,
  1, 1, 3, 7, 9, 25, 5, 197, 237, 163,
  1, 3, 7, 13, 5, 57, 67, 193, 147, 241,
  1, 1, 5, 15, 15, 33, 17, 67, 161, 341,
  1, 1, 3, 13, 17, 43, 21, 197, 441, 985,
  1, 3, 1, 5, 15, 33, 33, 193, 305, 829,
  1, 1, 1, 13, 19, 27, 71, 187, 477, 239,
  1, 1, 1, 9, 9, 17, 41, 177, 229, 983,
  1, 3, 5, 9, 15, 45, 97, 205, 43, 767,
  1, 1, 1, 9, 31, 31, 77, 159, 395, 809,
  1, 3, 3, 3, 29, 19, 73, 123, 165, 307,
  1, 3, 1, 7, 5, 11, 77, 227, 355, 403,
  1, 3, 5, 5, 25, 31, 1, 215, 451, 195,
  1, 3, 7, 15, 29, 37, 101, 241, 17, 633,
  1, 1, 5, 1, 11, 3, 107, 137, 489, 5,
  1, 1, 1, 7, 19, 19, 75, 85, 471, 355,
  1, 1, 3, 3, 9, 13, 113, 167, 13, 27,
  1, 3, 5, 11, 21, 3, 89, 205, 377, 307,
  1, 1, 1, 9, 31, 61, 65, 9, 391, 141, 867,
  1, 1, 1, 9, 19, 19, 61, 227, 241, 55, 161,
  1, 1, 1, 11, 1, 19, 7, 233, 463, 171, 1941,
  1, 1, 5, 7, 25, 13, 103, 75, 19, 1021, 1063,
  1, 1, 1, 15, 17, 17, 79, 63, 391, 403, 1221,
  1, 3, 3, 11, 29, 25, 29, 107, 335, 475, 963,
  1, 3, 5, 1, 31, 33, 49, 43, 155, 9, 1285,
  1, 1, 5, 5, 15, 47, 39, 161, 357, 863, 1039,
  1, 3, 7, 15, 1, 39, 47, 109, 427, 393, 1103,
  1, 1, 1, 9, 9, 29, 121, 233, 157, 99, 701,
  1, 1, 1, 7, 1, 29, 75, 121, 439, 109, 993,
  1, 1, 1, 9, 5, 1, 39, 59, 89, 157, 1865,
  1, 1, 5, 1, 3, 37, 89, 93, 143, 533, 175,
  1, 1, 3, 5, 7, 33, 35, 173, 159, 135, 241,
  1, 1, 1, 15, 17, 37, 79, 131, 43, 891, 229,
  1, 1, 1, 1, 1, 35, 121, 177, 397, 1017, 583,
  1, 1, 3, 15, 31, 21, 43, 67, 467, 923, 1473,
  1, 1, 1, 7, 1, 33, 77, 111, 125, 771, 1975,
  1, 3, 7, 13, 1, 51, 113, 139, 245, 573, 503,
  1, 3, 1, 9, 21, 49, 15, 157, 49, 483, 291,
  1, 1, 1, 1, 29, 35, 17, 65, 403, 485, 1603,
  1, 1, 1, 7, 19, 1, 37, 129, 203, 321, 1809,
  1, 3, 7, 15, 15, 9, 5, 77, 29, 485, 581,
  1, 1, 3, 5, 15, 49, 97, 105, 309, 875, 1581,
  1, 3, 5, 1, 5, 19, 63, 35, 165, 399, 1489,
  1, 3, 5, 3, 23, 5, 79, 137, 115, 599, 1127,
  1, 1, 7, 5, 3, 61, 27, 177, 257, 91, 841,
  1, 1, 3, 5, 9, 31, 91, 209, 409, 661, 159,
  1, 3, 1, 15, 23, 39, 23, 195, 245, 203, 947,
  1, 1, 3, 1, 15, 59, 67, 95, 155, 461, 147,
  1, 3, 7, 5, 23, 25, 87, 11, 51, 449, 1631,
  1, 1, 1, 1, 17, 57, 7, 197, 409, 609, 135,
  1, 1, 1, 9, 1, 61, 115, 113, 495, 895, 1595,
  1, 3, 7, 15, 9, 47, 121, 211, 379, 985, 1755,
  1, 3, 1, 3, 7, 57, 27, 231, 339, 325, 1023,
  1, 1, 1, 1, 19, 63, 63, 239, 31, 643, 373,
  1, 3, 1, 11, 19, 9, 7, 171, 21, 691, 215,
  1, 1, 5, 13, 11, 57, 39, 211, 241, 893, 555,
  1, 1, 7, 5, 29, 21, 45, 59, 509, 223, 491,
  1, 1, 7, 9, 15, 61, 97, 75, 127, 779, 839,
  1, 1, 7, 15, 17, 33, 75, 237, 191, 925, 681,
  1, 3, 5, 7, 27, 57, 123, 111, 101, 371, 1129,
  1, 3, 5, 5, 29, 45, 59, 127, 229, 967, 2027,
  1, 1, 1, 1, 17, 7, 23, 199, 241, 455, 135,
  1, 1, 7, 15, 27, 29, 105, 171, 337, 503, 1817,
  1, 1, 3, 7, 21, 35, 61, 71, 405, 647, 2045,
  1, 1, 1, 1, 1, 15, 65, 167, 501, 79, 737,
  1, 1, 5, 1, 3, 49, 27, 189, 341, 615, 1287,
  1, 1, 1, 9, 1, 7, 31, 159, 503, 327, 1613,
  1, 3, 3, 3, 3, 23, 99, 115, 323, 997, 987,
  1, 1, 1, 9, 19, 33, 93, 247, 509, 453, 891,
  1, 1, 3, 1, 13, 19, 35, 153, 161, 633, 445,
  1, 3, 5, 15, 31, 5, 87, 197, 183, 783, 1823,
  1, 1, 7, 5, 19, 63, 69, 221, 129, 231, 1195,
  1, 1, 5, 5, 13, 23, 19, 231, 245, 917, 379,
  1, 3, 1, 15, 19, 43, 27, 223, 171, 413, 125,
  1, 1, 1, 9, 1, 59, 21, 15, 509, 207, 589,
  1, 3, 5, 3, 19, 31, 113, 19, 23, 733, 499,
  1, 1, 7, 1, 19, 51, 101, 165, 47, 925, 1093,
  1, 3, 3, 9, 15, 21, 43, 243, 237, 461, 1361,
  1, 1, 1, 9, 17, 15, 75, 75, 113, 715, 1419,
  1, 1, 7, 13, 17, 1, 99, 15, 347, 721, 1405,
  1, 1, 7, 15, 7, 27, 23, 183, 39, 59, 571,
  1, 3, 5, 9, 7, 43, 35, 165, 463, 567, 859,
  1, 3, 3, 11, 15, 19, 17, 129, 311, 343, 15,
  1, 1, 1, 15, 31, 59, 63, 39, 347, 359, 105,
  1, 1, 1, 15, 5, 43, 87, 241, 109, 61, 685,
  1, 1, 7, 7, 9, 39, 121, 127, 369, 579, 853,
  1, 1, 1, 1, 17, 15, 15, 95, 325, 627, 299,
  1, 1, 3, 13, 31, 53, 85, 111, 289, 811, 1635,
  1, 3, 7, 1, 19, 29, 75, 185, 153, 573, 653,
  1, 3, 7, 1, 29, 31, 55, 91, 249, 247, 1015,
  1, 3, 5, 7, 1, 49, 113, 139, 257, 127, 307,
  1, 3, 5, 9, 15, 15, 123, 105, 105, 225, 1893,
  1, 3, 3, 1, 15, 5, 105, 249, 73, 709, 1557,
  1, 1, 1, 9, 17, 31, 113, 73, 65, 701, 1439,
  1, 3, 5, 15, 13, 21, 117, 131, 243, 859, 323,
  1, 1, 1, 9, 19, 15, 69, 149, 89, 681, 515,
  1, 1, 1, 5, 29, 13, 21, 97, 301, 27, 967,
  1, 1, 3, 3, 15, 45, 107, 227, 495, 769, 1935,
  1, 1, 1, 11, 5, 27, 41, 173, 261, 703, 1349,
  1, 3, 3, 3, 11, 35, 97, 43, 501, 563, 1331,
  1, 1, 1, 7, 1, 17, 87, 17, 429, 245, 1941,
  1, 1, 7, 15, 29, 13, 1, 175, 425, 233, 797,
  1, 1, 3, 11, 21, 57, 49, 49, 163, 685, 701,
  1, 3, 3, 7, 11, 45, 107, 111, 379, 703, 1403,
  1, 1, 7, 3, 21, 7, 117, 49, 469, 37, 775,
  1, 1, 5, 15, 31, 63, 101, 77, 507, 489, 1955,
  1, 3, 3, 11, 19, 21, 101, 255, 203, 673, 665,
  1, 3, 3, 15, 17, 47, 125, 187, 271, 899, 2003,
  1, 1, 7, 7, 1, 35, 13, 235, 5, 337, 905,
  1, 3, 1, 15, 1, 43, 1, 27, 37, 695, 1429,
  1, 3, 1, 11, 21, 27, 93, 161, 299, 665, 495,
  1, 3, 3, 15, 3, 1, 81, 111, 105, 547, 897,
  1, 3, 5, 1, 3, 53, 97, 253, 401, 827, 1467,
  1, 1, 1, 5, 19, 59, 105, 125, 271, 351, 719,
  1, 3, 5, 13, 7, 11, 91, 41, 441, 759, 1827,
  1, 3, 7, 11, 29, 61, 61, 23, 307, 863, 363,
  1, 1, 7, 1, 15, 35, 29, 133, 415, 473, 1737,
  1, 1, 1, 13, 7, 33, 35, 225, 117, 681, 1545,
  1, 1, 1, 3, 5, 41, 83, 247, 13, 373, 1091,
  1, 3, 1, 13, 25, 61, 71, 217, 233, 313, 547,
  1, 3, 1, 7, 3, 29, 3, 49, 93, 465, 15,
  1, 1, 1, 9, 17, 61, 99, 163, 129, 485, 1087,
  1, 1, 1, 9, 9, 33, 31, 163, 145, 649, 253,
  1, 1, 1, 1, 17, 63, 43, 235, 287, 111, 567,
  1, 3, 5, 13, 29, 7, 11, 69, 153, 127, 449,
  1, 1, 5, 9, 11, 21, 15, 189, 431, 493, 1219,
  1, 1, 1, 15, 19, 5, 47, 91, 399, 293, 1743,
  1, 3, 3, 11, 29, 53, 53, 225, 409, 303, 333,
  1, 1, 1, 15, 31, 31, 21, 81, 147, 287, 1753,
  1, 3, 5, 5, 5, 63, 35, 125, 41, 687, 1793,
  1, 1, 1, 9, 19, 59, 107, 219, 455, 971, 297,
  1, 1, 3, 5, 3, 51, 121, 31, 245, 105, 1311,
  1, 3, 1, 5, 5, 57, 75, 107, 161, 431, 1693,
  1, 3, 1, 3, 19, 53, 27, 31, 191, 565, 1015,
  1, 3, 5, 13, 9, 41, 35, 249, 287, 49, 123,
  1, 1, 5, 7, 27, 17, 21, 3, 151, 885, 1165,
  1, 1, 7, 1, 15, 17, 65, 139, 427, 339, 1171,
  1, 1, 1, 5, 23, 5, 9, 89, 321, 907, 391,
  1, 1, 7, 9, 15, 1, 77, 71, 87, 701, 917,
  1, 1, 7, 1, 17, 37, 115, 127, 469, 779, 1543,
  1, 3, 7, 3, 5, 61, 15, 37, 301, 951, 1437,
  1, 1, 1, 13, 9, 51, 127, 145, 229, 55, 1567,
  1, 3, 7, 15, 19, 47, 53, 153, 295, 47, 1337,
  1, 3, 3, 5, 11, 31, 29, 133, 327, 287, 507,
  1, 1, 7, 7, 25, 31, 37, 199, 25, 927, 1317,
  1, 1, 7, 9, 3, 39, 127, 167, 345, 467, 759,
  1, 1, 1, 1, 31, 21, 15, 101, 293, 787, 1025,
  1, 1, 5, 3, 11, 41, 105, 109, 149, 837, 1813,
  1, 1, 3, 5, 29, 13, 19, 97, 309, 901, 753,
  1, 1, 7, 1, 19, 17, 31, 39, 173, 361, 1177,
  1, 3, 3, 3, 3, 41, 81, 7, 341, 491, 43,
  1, 1, 7, 7, 31, 35, 29, 77, 11, 335, 1275,
  1, 3, 3, 15, 17, 45, 19, 63, 151, 849, 129,
  1, 1, 7, 5, 7, 13, 47, 73, 79, 31, 499,
  1, 3, 1, 11, 1, 41, 59, 151, 247, 115, 1295,
  1, 1, 1, 9, 31, 37, 73, 23, 295, 483, 179,
  1, 3, 1, 15, 13, 63, 81, 27, 169, 825, 2037,
  1, 3, 5, 15, 7, 11, 73, 1, 451, 101, 2039,
  1, 3, 5, 3, 13, 53, 31, 137, 173, 319, 1521,
  1, 3, 1, 3, 29, 1, 73, 227, 377, 337, 1189,
  1, 3, 3, 13, 27, 9, 31, 101, 229, 165, 1983,
  1, 3, 1, 13, 13, 19, 19, 111, 319, 421, 223,
  1, 1, 7, 15, 25, 37, 61, 55, 359, 255, 1955,
  1, 1, 5, 13, 17, 43, 49, 215, 383, 915, 51,
  1, 1, 3, 1, 3, 7, 13, 119, 155, 585, 967,
  1, 3, 1, 13, 1, 63, 125, 21, 103, 287, 457,
  1, 1, 7, 1, 31, 17, 125, 137, 345, 379, 1925,
  1, 1, 3, 5, 5, 25, 119, 153, 455, 271, 2023,
  1, 1, 7, 9, 9, 37, 115, 47, 5, 255, 917,
  1, 3, 5, 3, 31, 21, 75, 203, 489, 593, 1,
  1, 3, 7, 15, 19, 63, 123, 153, 135, 977, 1875,
  1, 1, 1, 1, 5, 59, 31, 25, 127, 209, 745,
  1, 1, 1, 1, 19, 45, 67, 159, 301, 199, 535,
  1, 1, 7, 1, 31, 17, 19, 225, 369, 125, 421,
  1, 3, 3, 11, 7, 59, 115, 197, 459, 469, 1055,
  1, 3, 1, 3, 27, 45, 35, 131, 349, 101, 411,
  1, 3, 7, 11, 9, 3, 67, 145, 299, 253, 1339,
  1, 3, 3, 11, 9, 37, 123, 229, 273, 269, 515,
  1, 3, 7, 15, 11, 25, 75, 5, 367, 217, 951,
  1, 1, 3, 7, 9, 23, 63, 237, 385, 159, 1273,
  1, 1, 5, 11, 23, 5, 55, 193, 109, 865, 663,
  1, 1, 7, 15, 1, 57, 17, 141, 51, 217, 1259,
  1, 1, 3, 3, 15, 7, 89, 233, 71, 329, 203,
  1, 3, 7, 11, 11, 1, 19, 155, 89, 437, 573,
  1, 3, 1, 9, 27, 61, 47, 109, 161, 913, 1681,
  1, 1, 7, 15, 1, 33, 19, 15, 23, 913, 989,
  1, 3, 1, 1, 25, 39, 119, 193, 13, 571, 157,
  1, 1, 7, 13, 9, 55, 59, 147, 361, 935, 515,
  1, 1, 1, 9, 7, 59, 67, 117, 71, 855, 1493,
  1, 3, 1, 3, 13, 19, 57, 141, 305, 275, 1079,
  1, 1, 1, 9, 17, 61, 33, 7, 43, 931, 781,
  1, 1, 3, 1, 11, 17, 21, 97, 295, 277, 1721,
  1, 3, 1, 13, 15, 43, 11, 241, 147, 391, 1641,
  1, 1, 1, 1, 1, 19, 37, 21, 255, 263, 1571,
  1, 1, 3, 3, 23, 59, 89, 17, 475, 303, 757, 543,
  1, 3, 3, 9, 11, 55, 35, 159, 139, 203, 1531, 1825,
  1, 1, 5, 3, 17, 53, 51, 241, 269, 949, 1373, 325,
  1, 3, 7, 7, 5, 29, 91, 149, 239, 193, 1951, 2675,
  1, 3, 5, 1, 27, 33, 69, 11, 51, 371, 833, 2685,
  1, 1, 1, 15, 1, 17, 35, 57, 171, 1007, 449, 367,
  1, 1, 1, 7, 25, 61, 73, 219, 379, 53, 589, 4065,
  1, 3, 5, 13, 21, 29, 45, 19, 163, 169, 147, 597,
  1, 1, 5, 11, 21, 27, 7, 17, 237, 591, 255, 1235,
  1, 1, 7, 7, 17, 41, 69, 237, 397, 173, 1229, 2341,
  1, 1, 3, 1, 1, 33, 125, 47, 11, 783, 1323, 2469,
  1, 3, 1, 11, 3, 39, 35, 133, 153, 55, 1171, 3165,
  1, 1, 5, 11, 27, 23, 103, 245, 375, 753, 477, 2165,
  1, 3, 1, 15, 15, 49, 127, 223, 387, 771, 1719, 1465,
  1, 1, 1, 9, 11, 9, 17, 185, 239, 899, 1273, 3961,
  1, 1, 3, 13, 11, 51, 73, 81, 389, 647, 1767, 1215,
  1, 3, 5, 15, 19, 9, 69, 35, 349, 977, 1603, 1435,
  1, 1, 1, 1, 19, 59, 123, 37, 41, 961, 181, 1275,
  1, 1, 1, 1, 31, 29, 37, 71, 205, 947, 115, 3017,
  1, 1, 7, 15, 5, 37, 101, 169, 221, 245, 687, 195,
  1, 1, 1, 1, 19, 9, 125, 157, 119, 283, 1721, 743,
  1, 1, 7, 3, 1, 7, 61, 71, 119, 257, 1227, 2893,
  1, 3, 3, 3, 25, 41, 25, 225, 31, 57, 925, 2139,
  0
};


/*
 * coefficients of the free direction integers as given in
 * "Monte Carlo Methods in Finance", by Peter Jäckel, section 8.3
 */

const unsigned long rk_sobol_Jdirections[] = {
  1,
  1, 1,
  1, 3, 7,
  1, 1, 5,
  1, 3, 1, 1,
  1, 1, 3, 7,
  1, 3, 3, 9, 9,
  1, 3, 7, 7, 21,
  1, 1, 5, 11, 27,
  1, 1, 7, 3, 29,
  1, 3, 7, 13, 3,
  1, 3, 5, 1, 15,
  1, 1, 1, 9, 23, 37,
  1, 1, 3, 13, 11, 7,
  1, 3, 3, 5, 19, 33,
  1, 1, 7, 13, 25, 5,
  1, 1, 1, 3, 13, 39,
  1, 3, 5, 11, 7, 11,
  1, 3, 1, 7, 3, 23, 79,
  1, 3, 1, 15, 17, 63, 13,
  1, 3, 3, 3, 25, 17, 115,
  1, 3, 7, 9, 31, 29, 17,
  1, 1, 3, 15, 29, 15, 41,
  1, 3, 1, 9, 5, 21, 119,
  1, 1, 5, 5, 1, 27, 33,
  1, 1, 3, 1, 23, 13, 75,
  1, 1, 7, 7, 19, 25, 105,
  1, 3, 5, 5, 21, 9, 7,
  1, 1, 1, 15, 5, 49, 59,
  1, 3, 5, 15, 17, 19, 21,
  1, 1, 7, 11, 13, 29, 3,
  0
};

/*
 * 0 terminated list of primitive polynomials to speed up initialization
 * All polynomials up to degree 13 (ie. 1111 polynomials)
 */
static const unsigned long rk_sobol_primitive_polynomials[] = {
  0x1UL, 0x3UL, 0x7UL, 0xBUL, 0xDUL, 0x13UL, 0x19UL, 0x25UL, 0x29UL,
  0x2FUL, 0x37UL, 0x3BUL, 0x3DUL, 0x43UL, 0x5BUL, 0x61UL, 0x67UL, 0x6DUL,
  0x73UL, 0x83UL, 0x89UL, 0x8FUL, 0x91UL, 0x9DUL, 0xA7UL, 0xABUL, 0xB9UL,
  0xBFUL, 0xC1UL, 0xCBUL, 0xD3UL, 0xD5UL, 0xE5UL, 0xEFUL, 0xF1UL, 0xF7UL,
  0xFDUL, 0x11DUL, 0x12BUL, 0x12DUL, 0x14DUL, 0x15FUL, 0x163UL, 0x165UL,
  0x169UL, 0x171UL, 0x187UL, 0x18DUL, 0x1A9UL, 0x1C3UL, 0x1CFUL, 0x1E7UL,
  0x1F5UL, 0x211UL, 0x21BUL, 0x221UL, 0x22DUL, 0x233UL, 0x259UL, 0x25FUL,
  0x269UL, 0x26FUL, 0x277UL, 0x27DUL, 0x287UL, 0x295UL, 0x2A3UL, 0x2A5UL,
  0x2AFUL, 0x2B7UL, 0x2BDUL, 0x2CFUL, 0x2D1UL, 0x2DBUL, 0x2F5UL, 0x2F9UL,
  0x313UL, 0x315UL, 0x31FUL, 0x323UL, 0x331UL, 0x33BUL, 0x34FUL, 0x35BUL,
  0x361UL, 0x36BUL, 0x36DUL, 0x373UL, 0x37FUL, 0x385UL, 0x38FUL, 0x3B5UL,
  0x3B9UL, 0x3C7UL, 0x3CBUL, 0x3CDUL, 0x3D5UL, 0x3D9UL, 0x3E3UL, 0x3E9UL,
  0x3FBUL, 0x409UL, 0x41BUL, 0x427UL, 0x42DUL, 0x465UL, 0x46FUL, 0x481UL,
  0x48BUL, 0x4C5UL, 0x4D7UL, 0x4E7UL, 0x4F3UL, 0x4FFUL, 0x50DUL, 0x519UL,
  0x523UL, 0x531UL, 0x53DUL, 0x543UL, 0x557UL, 0x56BUL, 0x585UL, 0x58FUL,
  0x597UL, 0x5A1UL, 0x5C7UL, 0x5E5UL, 0x5F7UL, 0x5FBUL, 0x613UL, 0x615UL,
  0x625UL, 0x637UL, 0x643UL, 0x64FUL, 0x65BUL, 0x679UL, 0x67FUL, 0x689UL,
  0x6B5UL, 0x6C1UL, 0x6D3UL, 0x6DFUL, 0x6FDUL, 0x717UL, 0x71DUL, 0x721UL,
  0x739UL, 0x747UL, 0x74DUL, 0x755UL, 0x759UL, 0x763UL, 0x77DUL, 0x78DUL,
  0x793UL, 0x7B1UL, 0x7DBUL, 0x7F3UL, 0x7F9UL, 0x805UL, 0x817UL, 0x82BUL,
  0x82DUL, 0x847UL, 0x863UL, 0x865UL, 0x871UL, 0x87BUL, 0x88DUL, 0x895UL,
  0x89FUL, 0x8A9UL, 0x8B1UL, 0x8CFUL, 0x8D1UL, 0x8E1UL, 0x8E7UL, 0x8EBUL,
  0x8F5UL, 0x90DUL, 0x913UL, 0x925UL, 0x929UL, 0x93BUL, 0x93DUL, 0x945UL,
  0x949UL, 0x951UL, 0x95BUL, 0x973UL, 0x975UL, 0x97FUL, 0x983UL, 0x98FUL,
  0x9ABUL, 0x9ADUL, 0x9B9UL, 0x9C7UL, 0x9D9UL, 0x9E5UL, 0x9F7UL, 0xA01UL,
  0xA07UL, 0xA13UL, 0xA15UL, 0xA29UL, 0xA49UL, 0xA61UL, 0xA6DUL, 0xA79UL,
  0xA7FUL, 0xA85UL, 0xA91UL, 0xA9DUL, 0xAA7UL, 0xAABUL, 0xAB3UL, 0xAB5UL,
  0xAD5UL, 0xADFUL, 0xAE9UL, 0xAEFUL, 0xAF1UL, 0xAFBUL, 0xB03UL, 0xB09UL,
  0xB11UL, 0xB33UL, 0xB3FUL, 0xB41UL, 0xB4BUL, 0xB59UL, 0xB5FUL, 0xB65UL,
  0xB6FUL, 0xB7DUL, 0xB87UL, 0xB8BUL, 0xB93UL, 0xB95UL, 0xBAFUL, 0xBB7UL,
  0xBBDUL, 0xBC9UL, 0xBDBUL, 0xBDDUL, 0xBE7UL, 0xBEDUL, 0xC0BUL, 0xC0DUL,
  0xC19UL, 0xC1FUL, 0xC57UL, 0xC61UL, 0xC6BUL, 0xC73UL, 0xC85UL, 0xC89UL,
  0xC97UL, 0xC9BUL, 0xC9DUL, 0xCB3UL, 0xCBFUL, 0xCC7UL, 0xCCDUL, 0xCD3UL,
  0xCD5UL, 0xCE3UL, 0xCE9UL, 0xCF7UL, 0xD03UL, 0xD0FUL, 0xD1DUL, 0xD27UL,
  0xD2DUL, 0xD41UL, 0xD47UL, 0xD55UL, 0xD59UL, 0xD63UL, 0xD6FUL, 0xD71UL,
  0xD93UL, 0xD9FUL, 0xDA9UL, 0xDBBUL, 0xDBDUL, 0xDC9UL, 0xDD7UL, 0xDDBUL,
  0xDE1UL, 0xDE7UL, 0xDF5UL, 0xE05UL, 0xE1DUL, 0xE21UL, 0xE27UL, 0xE2BUL,
  0xE33UL, 0xE39UL, 0xE47UL, 0xE4BUL, 0xE55UL, 0xE5FUL, 0xE71UL, 0xE7BUL,
  0xE7DUL, 0xE81UL, 0xE93UL, 0xE9FUL, 0xEA3UL, 0xEBBUL, 0xECFUL, 0xEDDUL,
  0xEF3UL, 0xEF9UL, 0xF0BUL, 0xF19UL, 0xF31UL, 0xF37UL, 0xF5DUL, 0xF6BUL,
  0xF6DUL, 0xF75UL, 0xF83UL, 0xF91UL, 0xF97UL, 0xF9BUL, 0xFA7UL, 0xFADUL,
  0xFB5UL, 0xFCDUL, 0xFD3UL, 0xFE5UL, 0xFE9UL, 0x1053UL, 0x1069UL,
  0x107BUL, 0x107DUL, 0x1099UL, 0x10D1UL, 0x10EBUL, 0x1107UL, 0x111FUL,
  0x1123UL, 0x113BUL, 0x114FUL, 0x1157UL, 0x1161UL, 0x116BUL, 0x1185UL,
  0x11B3UL, 0x11D9UL, 0x11DFUL, 0x120DUL, 0x1237UL, 0x123DUL, 0x1267UL,
  0x1273UL, 0x127FUL, 0x12B9UL, 0x12C1UL, 0x12CBUL, 0x130FUL, 0x131DUL,
  0x1321UL, 0x1339UL, 0x133FUL, 0x134DUL, 0x1371UL, 0x1399UL, 0x13A3UL,
  0x13A9UL, 0x1407UL, 0x1431UL, 0x1437UL, 0x144FUL, 0x145DUL, 0x1467UL,
  0x1475UL, 0x14A7UL, 0x14ADUL, 0x14D3UL, 0x150FUL, 0x151DUL, 0x154DUL,
  0x1593UL, 0x15C5UL, 0x15D7UL, 0x15DDUL, 0x15EBUL, 0x1609UL, 0x1647UL,
  0x1655UL, 0x1659UL, 0x16A5UL, 0x16BDUL, 0x1715UL, 0x1719UL, 0x1743UL,
  0x1745UL, 0x1775UL, 0x1789UL, 0x17ADUL, 0x17B3UL, 0x17BFUL, 0x17C1UL,
  0x1857UL, 0x185DUL, 0x1891UL, 0x1897UL, 0x18B9UL, 0x18EFUL, 0x191BUL,
  0x1935UL, 0x1941UL, 0x1965UL, 0x197BUL, 0x198BUL, 0x19B1UL, 0x19BDUL,
  0x19C9UL, 0x19CFUL, 0x19E7UL, 0x1A1BUL, 0x1A2BUL, 0x1A33UL, 0x1A69UL,
  0x1A8BUL, 0x1AD1UL, 0x1AE1UL, 0x1AF5UL, 0x1B0BUL, 0x1B13UL, 0x1B1FUL,
  0x1B57UL, 0x1B91UL, 0x1BA7UL, 0x1BBFUL, 0x1BC1UL, 0x1BD3UL, 0x1C05UL,
  0x1C11UL, 0x1C17UL, 0x1C27UL, 0x1C4DUL, 0x1C87UL, 0x1C9FUL, 0x1CA5UL,
  0x1CBBUL, 0x1CC5UL, 0x1CC9UL, 0x1CCFUL, 0x1CF3UL, 0x1D07UL, 0x1D23UL,
  0x1D43UL, 0x1D51UL, 0x1D5BUL, 0x1D75UL, 0x1D85UL, 0x1D89UL, 0x1E15UL,
  0x1E19UL, 0x1E2FUL, 0x1E45UL, 0x1E51UL, 0x1E67UL, 0x1E73UL, 0x1E8FUL,
  0x1EE3UL, 0x1F11UL, 0x1F1BUL, 0x1F27UL, 0x1F71UL, 0x1F99UL, 0x1FBBUL,
  0x1FBDUL, 0x1FC9UL, 0x201BUL, 0x2027UL, 0x2035UL, 0x2053UL, 0x2065UL,
  0x206FUL, 0x208BUL, 0x208DUL, 0x209FUL, 0x20A5UL, 0x20AFUL, 0x20BBUL,
  0x20BDUL, 0x20C3UL, 0x20C9UL, 0x20E1UL, 0x20F3UL, 0x210DUL, 0x2115UL,
  0x2129UL, 0x212FUL, 0x213BUL, 0x2143UL, 0x2167UL, 0x216BUL, 0x2179UL,
  0x2189UL, 0x2197UL, 0x219DUL, 0x21BFUL, 0x21C1UL, 0x21C7UL, 0x21CDUL,
  0x21DFUL, 0x21E3UL, 0x21F1UL, 0x21FBUL, 0x2219UL, 0x2225UL, 0x2237UL,
  0x223DUL, 0x2243UL, 0x225BUL, 0x225DUL, 0x2279UL, 0x227FUL, 0x2289UL,
  0x2297UL, 0x229BUL, 0x22B3UL, 0x22BFUL, 0x22CDUL, 0x22EFUL, 0x22F7UL,
  0x22FBUL, 0x2305UL, 0x2327UL, 0x232BUL, 0x2347UL, 0x2355UL, 0x2359UL,
  0x236FUL, 0x2371UL, 0x237DUL, 0x2387UL, 0x238DUL, 0x2395UL, 0x23A3UL,
  0x23A9UL, 0x23B1UL, 0x23B7UL, 0x23BBUL, 0x23E1UL, 0x23EDUL, 0x23F9UL,
  0x240BUL, 0x2413UL, 0x241FUL, 0x2425UL, 0x2429UL, 0x243DUL, 0x2451UL,
  0x2457UL, 0x2461UL, 0x246DUL, 0x247FUL, 0x2483UL, 0x249BUL, 0x249DUL,
  0x24B5UL, 0x24BFUL, 0x24C1UL, 0x24C7UL, 0x24CBUL, 0x24E3UL, 0x2509UL,
  0x2517UL, 0x251DUL, 0x2521UL, 0x252DUL, 0x2539UL, 0x2553UL, 0x2555UL,
  0x2563UL, 0x2571UL, 0x2577UL, 0x2587UL, 0x258BUL, 0x2595UL, 0x2599UL,
  0x259FUL, 0x25AFUL, 0x25BDUL, 0x25C5UL, 0x25CFUL, 0x25D7UL, 0x25EBUL,
  0x2603UL, 0x2605UL, 0x2611UL, 0x262DUL, 0x263FUL, 0x264BUL, 0x2653UL,
  0x2659UL, 0x2669UL, 0x2677UL, 0x267BUL, 0x2687UL, 0x2693UL, 0x2699UL,
  0x26B1UL, 0x26B7UL, 0x26BDUL, 0x26C3UL, 0x26EBUL, 0x26F5UL, 0x2713UL,
  0x2729UL, 0x273BUL, 0x274FUL, 0x2757UL, 0x275DUL, 0x276BUL, 0x2773UL,
  0x2779UL, 0x2783UL, 0x2791UL, 0x27A1UL, 0x27B9UL, 0x27C7UL, 0x27CBUL,
  0x27DFUL, 0x27EFUL, 0x27F1UL, 0x2807UL, 0x2819UL, 0x281FUL, 0x2823UL,
  0x2831UL, 0x283BUL, 0x283DUL, 0x2845UL, 0x2867UL, 0x2875UL, 0x2885UL,
  0x28ABUL, 0x28ADUL, 0x28BFUL, 0x28CDUL, 0x28D5UL, 0x28DFUL, 0x28E3UL,
  0x28E9UL, 0x28FBUL, 0x2909UL, 0x290FUL, 0x2911UL, 0x291BUL, 0x292BUL,
  0x2935UL, 0x293FUL, 0x2941UL, 0x294BUL, 0x2955UL, 0x2977UL, 0x297DUL,
  0x2981UL, 0x2993UL, 0x299FUL, 0x29AFUL, 0x29B7UL, 0x29BDUL, 0x29C3UL,
  0x29D7UL, 0x29F3UL, 0x29F5UL, 0x2A03UL, 0x2A0FUL, 0x2A1DUL, 0x2A21UL,
  0x2A33UL, 0x2A35UL, 0x2A4DUL, 0x2A69UL, 0x2A6FUL, 0x2A71UL, 0x2A7BUL,
  0x2A7DUL, 0x2AA5UL, 0x2AA9UL, 0x2AB1UL, 0x2AC5UL, 0x2AD7UL, 0x2ADBUL,
  0x2AEBUL, 0x2AF3UL, 0x2B01UL, 0x2B15UL, 0x2B23UL, 0x2B25UL, 0x2B2FUL,
  0x2B37UL, 0x2B43UL, 0x2B49UL, 0x2B6DUL, 0x2B7FUL, 0x2B85UL, 0x2B97UL,
  0x2B9BUL, 0x2BADUL, 0x2BB3UL, 0x2BD9UL, 0x2BE5UL, 0x2BFDUL, 0x2C0FUL,
  0x2C21UL, 0x2C2BUL, 0x2C2DUL, 0x2C3FUL, 0x2C41UL, 0x2C4DUL, 0x2C71UL,
  0x2C8BUL, 0x2C8DUL, 0x2C95UL, 0x2CA3UL, 0x2CAFUL, 0x2CBDUL, 0x2CC5UL,
  0x2CD1UL, 0x2CD7UL, 0x2CE1UL, 0x2CE7UL, 0x2CEBUL, 0x2D0DUL, 0x2D19UL,
  0x2D29UL, 0x2D2FUL, 0x2D37UL, 0x2D3BUL, 0x2D45UL, 0x2D5BUL, 0x2D67UL,
  0x2D75UL, 0x2D89UL, 0x2D8FUL, 0x2DA7UL, 0x2DABUL, 0x2DB5UL, 0x2DE3UL,
  0x2DF1UL, 0x2DFDUL, 0x2E07UL, 0x2E13UL, 0x2E15UL, 0x2E29UL, 0x2E49UL,
  0x2E4FUL, 0x2E5BUL, 0x2E5DUL, 0x2E61UL, 0x2E6BUL, 0x2E8FUL, 0x2E91UL,
  0x2E97UL, 0x2E9DUL, 0x2EABUL, 0x2EB3UL, 0x2EB9UL, 0x2EDFUL, 0x2EFBUL,
  0x2EFDUL, 0x2F05UL, 0x2F09UL, 0x2F11UL, 0x2F17UL, 0x2F3FUL, 0x2F41UL,
  0x2F4BUL, 0x2F4DUL, 0x2F59UL, 0x2F5FUL, 0x2F65UL, 0x2F69UL, 0x2F95UL,
  0x2FA5UL, 0x2FAFUL, 0x2FB1UL, 0x2FCFUL, 0x2FDDUL, 0x2FE7UL, 0x2FEDUL,
  0x2FF5UL, 0x2FFFUL, 0x3007UL, 0x3015UL, 0x3019UL, 0x302FUL, 0x3049UL,
  0x304FUL, 0x3067UL, 0x3079UL, 0x307FUL, 0x3091UL, 0x30A1UL, 0x30B5UL,
  0x30BFUL, 0x30C1UL, 0x30D3UL, 0x30D9UL, 0x30E5UL, 0x30EFUL, 0x3105UL,
  0x310FUL, 0x3135UL, 0x3147UL, 0x314DUL, 0x315FUL, 0x3163UL, 0x3171UL,
  0x317BUL, 0x31A3UL, 0x31A9UL, 0x31B7UL, 0x31C5UL, 0x31C9UL, 0x31DBUL,
  0x31E1UL, 0x31EBUL, 0x31EDUL, 0x31F3UL, 0x31FFUL, 0x3209UL, 0x320FUL,
  0x321DUL, 0x3227UL, 0x3239UL, 0x324BUL, 0x3253UL, 0x3259UL, 0x3265UL,
  0x3281UL, 0x3293UL, 0x3299UL, 0x329FUL, 0x32A9UL, 0x32B7UL, 0x32BBUL,
  0x32C3UL, 0x32D7UL, 0x32DBUL, 0x32E7UL, 0x3307UL, 0x3315UL, 0x332FUL,
  0x3351UL, 0x335DUL, 0x3375UL, 0x3397UL, 0x339BUL, 0x33ABUL, 0x33B9UL,
  0x33C1UL, 0x33C7UL, 0x33D5UL, 0x33E3UL, 0x33E5UL, 0x33F7UL, 0x33FBUL,
  0x3409UL, 0x341BUL, 0x3427UL, 0x3441UL, 0x344DUL, 0x345FUL, 0x3469UL,
  0x3477UL, 0x347BUL, 0x3487UL, 0x3493UL, 0x3499UL, 0x34A5UL, 0x34BDUL,
  0x34C9UL, 0x34DBUL, 0x34E7UL, 0x34F9UL, 0x350DUL, 0x351FUL, 0x3525UL,
  0x3531UL, 0x3537UL, 0x3545UL, 0x354FUL, 0x355DUL, 0x356DUL, 0x3573UL,
  0x357FUL, 0x359DUL, 0x35A1UL, 0x35B9UL, 0x35CDUL, 0x35D5UL, 0x35D9UL,
  0x35E3UL, 0x35E9UL, 0x35EFUL, 0x3601UL, 0x360BUL, 0x361FUL, 0x3625UL,
  0x362FUL, 0x363BUL, 0x3649UL, 0x3651UL, 0x365BUL, 0x3673UL, 0x3675UL,
  0x3691UL, 0x369BUL, 0x369DUL, 0x36ADUL, 0x36CBUL, 0x36D3UL, 0x36D5UL,
  0x36E3UL, 0x36EFUL, 0x3705UL, 0x370FUL, 0x371BUL, 0x3721UL, 0x372DUL,
  0x3739UL, 0x3741UL, 0x3747UL, 0x3753UL, 0x3771UL, 0x3777UL, 0x378BUL,
  0x3795UL, 0x3799UL, 0x37A3UL, 0x37C5UL, 0x37CFUL, 0x37D1UL, 0x37D7UL,
  0x37DDUL, 0x37E1UL, 0x37F3UL, 0x3803UL, 0x3805UL, 0x3817UL, 0x381DUL,
  0x3827UL, 0x3833UL, 0x384BUL, 0x3859UL, 0x3869UL, 0x3871UL, 0x38A3UL,
  0x38B1UL, 0x38BBUL, 0x38C9UL, 0x38CFUL, 0x38E1UL, 0x38F3UL, 0x38F9UL,
  0x3901UL, 0x3907UL, 0x390BUL, 0x3913UL, 0x3931UL, 0x394FUL, 0x3967UL,
  0x396DUL, 0x3983UL, 0x3985UL, 0x3997UL, 0x39A1UL, 0x39A7UL, 0x39ADUL,
  0x39CBUL, 0x39CDUL, 0x39D3UL, 0x39EFUL, 0x39F7UL, 0x39FDUL, 0x3A07UL,
  0x3A29UL, 0x3A2FUL, 0x3A3DUL, 0x3A51UL, 0x3A5DUL, 0x3A61UL, 0x3A67UL,
  0x3A73UL, 0x3A75UL, 0x3A89UL, 0x3AB9UL, 0x3ABFUL, 0x3ACDUL, 0x3AD3UL,
  0x3AD5UL, 0x3ADFUL, 0x3AE5UL, 0x3AE9UL, 0x3AFBUL, 0x3B11UL, 0x3B2BUL,
  0x3B2DUL, 0x3B35UL, 0x3B3FUL, 0x3B53UL, 0x3B59UL, 0x3B63UL, 0x3B65UL,
  0x3B6FUL, 0x3B71UL, 0x3B77UL, 0x3B8BUL, 0x3B99UL, 0x3BA5UL, 0x3BA9UL,
  0x3BB7UL, 0x3BBBUL, 0x3BD1UL, 0x3BE7UL, 0x3BF3UL, 0x3BFFUL, 0x3C0DUL,
  0x3C13UL, 0x3C15UL, 0x3C1FUL, 0x3C23UL, 0x3C25UL, 0x3C3BUL, 0x3C4FUL,
  0x3C5DUL, 0x3C6DUL, 0x3C83UL, 0x3C8FUL, 0x3C9DUL, 0x3CA7UL, 0x3CABUL,
  0x3CB9UL, 0x3CC7UL, 0x3CE9UL, 0x3CFBUL, 0x3CFDUL, 0x3D03UL, 0x3D17UL,
  0x3D1BUL, 0x3D21UL, 0x3D2DUL, 0x3D33UL, 0x3D35UL, 0x3D41UL, 0x3D4DUL,
  0x3D65UL, 0x3D69UL, 0x3D7DUL, 0x3D81UL, 0x3D95UL, 0x3DB1UL, 0x3DB7UL,
  0x3DC3UL, 0x3DD1UL, 0x3DDBUL, 0x3DE7UL, 0x3DEBUL, 0x3DF9UL, 0x3E05UL,
  0x3E09UL, 0x3E0FUL, 0x3E1BUL, 0x3E2BUL, 0x3E3FUL, 0x3E41UL, 0x3E53UL,
  0x3E65UL, 0x3E69UL, 0x3E8BUL, 0x3EA3UL, 0x3EBDUL, 0x3EC5UL, 0x3ED7UL,
  0x3EDDUL, 0x3EE1UL, 0x3EF9UL, 0x3F0DUL, 0x3F19UL, 0x3F1FUL, 0x3F25UL,
  0x3F37UL, 0x3F3DUL, 0x3F43UL, 0x3F45UL, 0x3F49UL, 0x3F51UL, 0x3F57UL,
  0x3F61UL, 0x3F83UL, 0x3F89UL, 0x3F91UL, 0x3FABUL, 0x3FB5UL, 0x3FE3UL,
  0x3FF7UL, 0x3FFDUL,
  0UL
};

rk_sobol_error rk_sobol_init(size_t dimension, rk_sobol_state *s, 
  rk_state *rs_dir, const unsigned long *directions,
  const unsigned long *polynomials)
{
  rk_state rs_dir_temp;
  int j, l, degree = 0, last_degree = 0, ooord = 0;
  size_t k, cdir = 0, cpol = 0;
  unsigned long polynomial = 1, rev = 0, last = 0;

  if (dimension == 0)
    return RK_SOBOL_EINVAL;

  if (polynomials == NULL)
    polynomials = rk_sobol_primitive_polynomials;

  /* Allocate the structure */
  s->direction = NULL; s->numerator = NULL;
  s->direction = malloc(sizeof(*(s->direction))*dimension*LONG_BIT);
  s->numerator = malloc(sizeof(*(s->numerator))*dimension);
  if (!s->direction || !s->numerator)
  {
    if (!s->direction) free(s->direction);
    if (!s->numerator) free(s->numerator);
    return RK_SOBOL_ENOMEM;
  }

  /* Initialize directions */
  /* Degree 0 */
  for (j = degree; j < LONG_BIT; j++)
    s->direction[j*dimension] = 1UL << (LONG_BIT-j-1);

  /* Skip unused first polynomial */
  if (polynomials[cpol])
    cpol++;

  /* Degree >0 */
  for (k = 1; k < dimension; k++)
  {
    unsigned long temp;

    /* Find a new primitive polynomial */
    if (polynomials[cpol])
      polynomial = polynomials[cpol++];
    else if (rev)
    {
      /* We are generating polynomials out of order: 
         use the reverse of the previous polynomial */
      last = polynomial;
      polynomial = rev;
      rev = 0;
    }
    else
    {
      if (last)
      {
        polynomial = last;
        last = 0;
      }

      /* Find a new primitive polynomial */
      while(1)
      {
        if (polynomial == ULONG_MAX)
        {
          /* Not enough polynomials */
          free(s->direction);
          free(s->numerator);
          return RK_SOBOL_EINVAL;
        }

        polynomial += 2;

        if (ooord)
        {
          unsigned long copy = polynomial;
          /* We are generating polynomials out of order: 
             check if the reverse was already checked */
          for (rev = 0; copy; copy >>= 1)
            rev = (rev << 1) | (copy & 1);
          if (ooord && rev < polynomial)
            continue;
        }

        if (rk_isprimitive(polynomial))
          break;
      }

      if (rev == polynomial)
        /* We are generating polynomials out of order: 
           the reverse is not different, discard it */
        rev = 0;
    }

    /* Compute the degree */
    for (temp = polynomial >> 1, degree = 0; temp; degree++, temp >>= 1);

    for (j=0; j<degree; j++) 
    {
      unsigned long m;
      if (directions == NULL || directions[cdir] == 0)
      {
        /* If necessary initialize a pseudo random generator */
        if (rs_dir == NULL)
        {
          rs_dir = &rs_dir_temp;
          rk_randomseed(rs_dir);
        }
        /* Draw a direction at random (the highest bits will be discarded) */
        m = rk_ulong(rs_dir) | 1;
      }
      else
        m = directions[cdir++];
      /* Scale direction */
      s->direction[j*dimension+k] = m << (LONG_BIT-j-1);
    }

    /* Scaled recursion for directions */
    for (j = degree; j < LONG_BIT; j++)
    {
      unsigned long effdir = s->direction[(j-degree)*dimension+k],
        ptemp = polynomial >> 1;
      effdir ^= (effdir >> degree);
      for (l = degree-1; l >= 1; l--, ptemp >>= 1)
        if (ptemp & 1)
          effdir ^= s->direction[(j-l)*dimension+k];
      s->direction[j*dimension+k] = effdir;
    }
    
    /* Can we generate polynomials out of order ? */
    if (!ooord && polynomials[cpol] == 0 && degree > last_degree
        && (directions == NULL || directions[cdir] == 0))
      ooord = 0;
    else
      last_degree = degree;
  }

  /* Initialize numerator */
  for (k=0; k<dimension; k++)
    s->numerator[k] = 0;

  s->dimension = dimension;
  s->gcount = 0;
  s->count = 0;
  return RK_SOBOL_OK;
}

void rk_sobol_reinit(rk_sobol_state *s)
{
  size_t k;

  /* Initialize numerator */
  for (k=0; k<s->dimension; k++)
    s->numerator[k] = 0;

  s->count = 0;
  s->gcount = 0;
}

void rk_sobol_randomshift(rk_sobol_state *s, rk_state *rs_num)
{
  rk_state rs_num_temp;
  size_t k;

  if (rs_num == NULL)
  {
    rs_num = &rs_num_temp;
    rk_randomseed(rs_num);
  }

  /* Initialize numerator */
  for (k=0; k<s->dimension; k++)
    s->numerator[k] = rk_ulong(rs_num);
}

rk_sobol_error rk_sobol_copy(rk_sobol_state *copy, rk_sobol_state *orig)
{
  size_t k;

  /* Allocate the structure */
  copy->direction = NULL; copy->numerator = NULL;
  copy->direction = malloc(sizeof(*(copy->direction))*orig->dimension*LONG_BIT);
  copy->numerator = malloc(sizeof(*(copy->numerator))*orig->dimension);
  if (!copy->direction || !copy->numerator)
  {
    if (!copy->direction) free(copy->direction);
    if (!copy->numerator) free(copy->numerator);
    return RK_SOBOL_ENOMEM;
  }

  /* Initialize numerator */
  for (k=0; k<orig->dimension; k++)
    copy->numerator[k] = orig->numerator[k];
  for (k=0; k<(orig->dimension*LONG_BIT); k++)
    copy->direction[k] = orig->direction[k];

  copy->count = orig->count;
  copy->gcount = orig->gcount;
  copy->dimension = orig->dimension;

  return RK_SOBOL_OK;
}

rk_sobol_error rk_sobol_double(rk_sobol_state *s, double *x)
{
  int j;
  size_t k;
  unsigned long im;
  const double inverse_denominator=1.0/(ULONG_MAX+1.0);

  if (s->count == ULONG_MAX) 
    j = 0;
  else
    for (im = s->count, j=0; im & 1; j++, im >>= 1);
  s->count++;

  for (k=0; k<s->dimension; k++)
  {
    s->numerator[k] ^= s->direction[j*s->dimension+k];
    x[k] = s->numerator[k]*inverse_denominator;
  }

  if ((s->gcount++) == ULONG_MAX) return RK_SOBOL_EXHAUST;
  return RK_SOBOL_OK;
}

void rk_sobol_setcount(rk_sobol_state *s, unsigned long count)
{
  s->count = count;
}

void rk_sobol_free(rk_sobol_state *s)
{
  free(s->direction);
  free(s->numerator);
}

double inverse_normal(double p)
{
  double q, t, x;
  
  /* Peter J. Acklam constants for the rational approximation */
  const double a[6] =
  {
    -3.969683028665376e+01,  2.209460984245205e+02,
    -2.759285104469687e+02,  1.383577518672690e+02,
    -3.066479806614716e+01,  2.506628277459239e+00
  };
  const double b[5] = 
  {
    -5.447609879822406e+01,  1.615858368580409e+02,
    -1.556989798598866e+02,  6.680131188771972e+01,
    -1.328068155288572e+01
  };
  const double c[6] = 
  {
    -7.784894002430293e-03, -3.223964580411365e-01,
    -2.400758277161838e+00, -2.549732539343734e+00,
     4.374664141464968e+00,  2.938163982698783e+00
  };
  const double d[4] = 
  {
     7.784695709041462e-03,  3.224671290700398e-01,
     2.445134137142996e+00,  3.754408661907416e+00
  };

  if (p <= 0)
    return -HUGE_VAL;
  else if (p >= 1)
    return HUGE_VAL;

  q = p<0.5 ? p : 1-p;
  if (q > 0.02425)
  {
    /* Rational approximation for central region */
    x = q-0.5;
    t = x*x;
    x = x*(((((a[0]*t+a[1])*t+a[2])*t+a[3])*t+a[4])*t+a[5])
        /(((((b[0]*t+b[1])*t+b[2])*t+b[3])*t+b[4])*t+1);
  }
  else
  {
    /* Rational approximation for tail region */
    t = sqrt(-2*log(q));
    x = (((((c[0]*t+c[1])*t+c[2])*t+c[3])*t+c[4])*t+c[5])
        /((((d[0]*t+d[1])*t+d[2])*t+d[3])*t+1);
  }

  //#ifndef _WIN32
  /* If we have erfc, improve the precision */
  /* Halley's rational method */
  t = (erfc(-x*M_SQRT1_2)/2 - q) * RK_SOBOL_M_SQRT2PI * exp(x*x/2);
  x -= t/(1 + x*t/2);
  //#endif

  return p>0.5 ? -x : x;
}

rk_sobol_error rk_sobol_gauss(rk_sobol_state *s, double *x)
{
  size_t k;
  rk_sobol_error rc = rk_sobol_double(s, x);
 
  for (k=0; k<s->dimension; k++)
    x[k] = inverse_normal(x[k]);

  return rc;
}
