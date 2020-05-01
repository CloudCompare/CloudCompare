/* Random kit 1.6 */

/*
 * Generate a list of binary primitive polynomials.
 */

/*
 * Copyright (c) 2005-2006, Jean-Sebastien Roy (js@jeannot.org)
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
  "@(#) $Jeannot: list_primitive.c,v 1.7 2006/02/19 13:48:34 js Exp $";

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#ifdef WIN32
/* On Windows, you must compile and link with getopt.c */
#include "getopt.h"
#else
#include <unistd.h>
#endif

#include "randomkit.h"

static void usage()
{
  fprintf(stderr, 
    "Usage: list_primitive [-c] [-o] [-d D] N\n"
    "Generate a list of binary primitive polynomials.\n"
    " -c: output a zero terminated list as a C source\n"
    " -o: generates polynonials out of order (~2x faster)\n"
    "     WARNING: if used with sobol_init, will not match the direction\n"
    "              numbers provided by sobol_*directions.\n"
    " -d: set the maximum degree of the polynomials to D.\n"
    "  N: maximum number of polynomials to find.\n");
  exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
  unsigned long polynomial = 0, ooorder = 0, count;
  int csource = 0, ch, line = -1, maxdegree = -1;

  while ((ch = getopt(argc, argv, "cod:")) != -1)
  {
    switch (ch) 
    {
      case 'c':
         csource = 1;
         break;
      case 'o':
         ooorder = 1;
         break;
      case 'd':
         maxdegree = strtoul(optarg, NULL, 10);
         break;
      default:
        usage();
    }
  }
  argc -= optind;
  argv += optind;

  if (argc != 1)
    usage();
  count = strtoul(argv[0], NULL, 10);

  if (csource)
  {
    if (maxdegree >=0)
      printf("const unsigned long primitive_polynomials_%lu_%d[] = {\n", count,
        maxdegree);
    else
      printf("const unsigned long primitive_polynomials_%lu[] = {\n", count);
  }

  for (polynomial = 1UL; count && polynomial < ULONG_MAX; polynomial += 2)
  {
    unsigned long rev = 0;

    if (maxdegree >=0)
    {
      unsigned long temp = polynomial >> 1;
      int degree = 0;
      /* Compute the degree */
      for (; temp; degree++, temp >>= 1);
      if (degree > maxdegree)
        break;
    }

    if (ooorder)
    {
      /* Reversing the bits of a primitive polynomial give another p.p. */
      unsigned long copy = polynomial;
      for (; copy; copy >>= 1)
        rev = (rev << 1) | (copy & 1);
      if (rev < polynomial)
        continue;
    }

    if (rk_isprimitive(polynomial))
    {
      count--;
      if (csource)
      {
        if (line > -1)
        {
          printf(",");
          if (line == 0)
            printf("\n ");
        }
        else
          line = 0;
        line += printf(" 0x%lXUL", polynomial);
        if (line >= (78-21))
          line = 0;
      }
      else
        printf("%lu\n", polynomial);
      if (count && ooorder && rev != polynomial)
      {
        count--;
        if (csource)
        {
          if (line > -1)
          {
            printf(",");
            if (line == 0)
              printf("\n ");
          }
          else
            line = 0;
          line += printf(" 0x%lXUL", rev);
          if (line >= (78-21))
            line = 0;
        }
        else
          printf("%lu\n", rev);
      }
    }
  }

  if (csource)
  {
    if (line > -1)
      printf(",");
    printf("\n  0UL\n};\n");
  }

  return 0;
}
