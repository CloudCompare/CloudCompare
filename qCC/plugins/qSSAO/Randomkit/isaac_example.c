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
  "@(#) $Jeannot: isaac_example.c,v 1.1 2006/02/19 13:48:34 js Exp $";

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
  rk_isaac_state rk;
  size_t dimension = 3, d, draws = 500000, draw;
  unsigned long drawn_in = 0; /* number of draws in ball */
  double estimated_volume, correct_volume;
  
  /* Initialize the ISAAC RNG */
  rk_isaac_randomseed(&rk);

  printf("MC estimation of the volume of a %d-dimensional ball (ISAAC).\n",
    dimension);
  printf("%d draws\n", draws);
  
  for (draw = 0; draw < draws; draw++)
  {
    double length = 0;
    for (d = 0; d < dimension; d++)
    {
      double x = rk_isaac_double(&rk);
      length += x*x;
    }
    if (length <= 1.0)
      drawn_in ++;
  }
  
  estimated_volume = pow(2.0, dimension)*drawn_in/((double)draws);
  correct_volume = pow(M_PI,dimension*0.5)/gamma_half(2+dimension);

  printf("Estimated volume: %g\n", estimated_volume);
  printf("Correct volume: %g\n", correct_volume);
  printf("Error: %g\n", estimated_volume-correct_volume);
  
  return 0;
}
