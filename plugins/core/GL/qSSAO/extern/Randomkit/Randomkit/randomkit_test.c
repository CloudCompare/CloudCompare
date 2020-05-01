/* Random kit 1.6 */
/* Copyright (c) 2003-2006, Jean-Sebastien Roy (js@jeannot.org) */

static char const rcsid[] =
  "@(#) $Jeannot: randomkit_test.c,v 1.18 2006/02/19 14:44:14 js Exp $";

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include "randomkit.h"

int main(int argv, char **argc)
{
  /* For mt tests */
  unsigned long l;
  int i;
  rk_state state;
  rk_sobol_error rc;

  /* For primitive tests */
  unsigned long polynomial = 0, count = 1000, crc = 0;

  /* For sobol tests */
  rk_sobol_state s;
  double x[5], 
    x_ok[5] = {0.01171875, 0.33203125, 0.78515625, 0.94140625, 0.46484375};
  
	/* For ISAAC tests */
	rk_isaac_state is;

  fprintf(stderr, "Checking urandom device: ");
  if (rk_randomseed(&state) == RK_NOERR)
    fprintf(stderr, "found.\n");
  else
    fprintf(stderr, "not found.\n");


  fprintf(stderr, "PRNG coherency: ");
  rk_seed(1, &state);

  for (i=0;i<100000;i++)
    l = rk_random(&state);

  if (l != 2160364578UL || rk_double(&state) != 0.44476027670606876896)
  {
    fprintf(stderr, "test failed !\n");
    exit(EXIT_FAILURE);
  }
  else
    fprintf(stderr, "test successful.\n");


  fprintf(stderr, "Primitive polynomials coherency: ");

  for (polynomial = 1UL; count && polynomial < ULONG_MAX; polynomial += 2)
    if (rk_isprimitive(polynomial))
    {
      count--;
      crc ^= polynomial; /* very basic checksum */
    }

  if (crc != 11874 || polynomial != 14987)
  {
    fprintf(stderr, "test failed !\n");
    exit(EXIT_FAILURE);
  }
  else
    fprintf(stderr, "test successful.\n");


  fprintf(stderr, "Sobol QRNG coherency: ");

  if ((rc = rk_sobol_init(5, &s, NULL, rk_sobol_Ldirections, NULL)))
  {
    fprintf(stderr, "sobol_init error: %s\n", rk_sobol_strerror[rc]);
    exit(EXIT_FAILURE);
  }
  
  for (i=0; i<128; i++)
    rk_sobol_double(&s, x);

  rk_sobol_free(&s);

  for (i=0; i<5; i++)
    if (x[i] != x_ok[i])
    {
      fprintf(stderr, "test failed !\n");
      exit(EXIT_FAILURE);
    }

  fprintf(stderr, "test successful.\n");

  fprintf(stderr, "Checking random device: ");
  if (rk_isaac_randomseed(&is) == RK_NOERR)
    fprintf(stderr, "found.\n");
  else
    fprintf(stderr, "not found.\n");

  fprintf(stderr, "ISAAC RNG coherency: ");

	rk_isaac_seed(12345, &is);
  for (i=0; i<20000; ++i)
		rk_isaac_random(&is);
	
	if (rk_isaac_random(&is) != 416328131)
  {
    fprintf(stderr, "test failed !\n");
    exit(EXIT_FAILURE);
  }
	
	fprintf(stderr, "test successful.\n");

  return 0;
}
