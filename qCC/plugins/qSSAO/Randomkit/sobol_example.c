/* Random kit 1.6 */

/*
 * Copyright (c) 2004-2006, Jean-Sebastien Roy (js@jeannot.org)
 *
 * Original algorithm from Numerical Recipes, 2nd edition, by Press et al.
 * The inverse normal cdf formulas are from Peter J. Acklam.
 * The initialization directions are reproduced from Ferdinando Ametrano's
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
  "@(#) $Jeannot: sobol_example.c,v 1.7 2006/02/19 13:48:34 js Exp $";

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "randomkit.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Gamma(n/2) for n a (small) positive integer */
static double gamma_half(int n);

double gamma_half(int n)
{
  double x = 1.0;
  if (n & 1)
    x = sqrt(M_PI)*0.5;
  for (n-=2; n>1; n-=2)
    x *= n*0.5;
  return x;
}

int main(int argc, char **argv)
{
  rk_sobol_state s;
  rk_sobol_error rc;
  size_t dimension = 3, d, runs = 5, draws = 100000, run, draw;
  unsigned long drawn_in = 0; /* number of draws in ball */
  double *x, estimated_volume, correct_volume;
  
  x = malloc(sizeof(*x)*dimension);
  if (!x)
  {
    fprintf(stderr, "not enough memory\n");
    exit(EXIT_FAILURE);
  }
  
  /* Initialize the sobol QRNG */
  if ((rc = rk_sobol_init(dimension, &s, NULL, rk_sobol_Ldirections, NULL)))
  {
    fprintf(stderr, "sobol_init error: %s\n", rk_sobol_strerror[rc]);
    exit(EXIT_FAILURE);
  }

  printf("RQMC estimation of the volume of a %d-dimensional ball.\n",
    dimension);
  printf("%d runs with %d draws per run.\n", runs, draws);
  
  for (run = 0; run < runs; run++)
  {
    if (run > 0)
      /* Re-initilize the sobol QRNG */
      rk_sobol_reinit(&s);

    /* Randomly shift the sobol QRNG for Randomized Quasi Monte-Carlo */
    rk_sobol_randomshift(&s, NULL);

    for (draw = 0; draw < draws; draw++)
    {
      double length = 0;
      rk_sobol_double(&s, x);
      for (d = 0; d < dimension; d++)
        length += x[d]*x[d];
      if (length <= 1.0)
        drawn_in ++;
    }
  }
  
  /* Free allocated memory */
  rk_sobol_free(&s);
  free(x);

  estimated_volume = pow(2.0, dimension)*drawn_in/(((double)runs)*draws);
  correct_volume = pow(M_PI,dimension*0.5)/gamma_half(2+dimension);

  printf("Estimated volume: %g\n", estimated_volume);
  printf("Correct volume: %g\n", correct_volume);
  printf("Error: %g\n", estimated_volume-correct_volume);
  
  return 0;
}
