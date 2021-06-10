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

static char const rcsid[] =
  "@(#) $Jeannot: rk_primitive.c,v 1.7 2006/02/19 13:48:34 js Exp $";

#include <math.h>
#include <limits.h>
#include "rk_primitive.h"

#ifndef LONG_BIT
#if ULONG_MAX <= 0xffffffffUL
#define LONG_BIT 32
#else
#define LONG_BIT 64
#endif
#endif

static unsigned long modmul(unsigned long poly1, unsigned long poly2,
  unsigned long modulo, unsigned long mask);
static unsigned long modpow(unsigned long polynomial, unsigned long power,
  unsigned long modulo, int degree);

/*
 * For all powers i of two up to 64, list all the number of the form
 * (2^i-1)/p for p a prime factor of 2^i-1.
 */
static const unsigned long divisors[][12]={
/* 2^0-1 */
  {1UL,
  0UL},
/* 2^1-1 */
  {1UL,
  0UL},
/* 2^2-1 */
  {1UL,
  0UL},
/* 2^3-1 */
  {1UL,
  0UL},
/* 2^4-1 */
  {5UL,
  3UL,
  0UL},
/* 2^5-1 */
  {1UL,
  0UL},
/* 2^6-1 */
  {21UL,
  9UL,
  0UL},
/* 2^7-1 */
  {1UL,
  0UL},
/* 2^8-1 */
  {85UL,
  51UL,
  15UL,
  0UL},
/* 2^9-1 */
  {73UL,
  7UL,
  0UL},
/* 2^10-1 */
  {341UL,
  93UL,
  33UL,
  0UL},
/* 2^11-1 */
  {89UL,
  23UL,
  0UL},
/* 2^12-1 */
  {1365UL,
  819UL,
  585UL,
  315UL,
  0UL},
/* 2^13-1 */
  {1UL,
  0UL},
/* 2^14-1 */
  {5461UL,
  381UL,
  129UL,
  0UL},
/* 2^15-1 */
  {4681UL,
  1057UL,
  217UL,
  0UL},
/* 2^16-1 */
  {21845UL,
  13107UL,
  3855UL,
  255UL,
  0UL},
/* 2^17-1 */
  {1UL,
  0UL},
/* 2^18-1 */
  {87381UL,
  37449UL,
  13797UL,
  3591UL,
  0UL},
/* 2^19-1 */
  {1UL,
  0UL},
/* 2^20-1 */
  {349525UL,
  209715UL,
  95325UL,
  33825UL,
  25575UL,
  0UL},
/* 2^21-1 */
  {299593UL,
  16513UL,
  6223UL,
  0UL},
/* 2^22-1 */
  {1398101UL,
  182361UL,
  47127UL,
  6141UL,
  0UL},
/* 2^23-1 */
  {178481UL,
  47UL,
  0UL},
/* 2^24-1 */
  {5592405UL,
  3355443UL,
  2396745UL,
  1290555UL,
  986895UL,
  69615UL,
  0UL},
/* 2^25-1 */
  {1082401UL,
  55831UL,
  18631UL,
  0UL},
/* 2^26-1 */
  {22369621UL,
  24573UL,
  8193UL,
  0UL},
/* 2^27-1 */
  {19173961UL,
  1838599UL,
  511UL,
  0UL},
/* 2^28-1 */
  {89478485UL,
  53687091UL,
  9256395UL,
  6242685UL,
  2375535UL,
  2113665UL,
  0UL},
/* 2^29-1 */
  {2304167UL,
  486737UL,
  256999UL,
  0UL},
/* 2^30-1 */
  {357913941UL,
  153391689UL,
  97612893UL,
  34636833UL,
  7110873UL,
  3243933UL,
  0UL},
/* 2^31-1 */
  {1UL,
  0UL},
/* 2^32-1 */
  {1431655765UL,
  858993459UL,
  252645135UL,
  16711935UL,
  65535UL,
  0UL},
#if LONG_BIT > 32
/* 2^33-1 */
  {1227133513UL,
  373475417UL,
  96516119UL,
  14329UL,
  0UL},
/* 2^34-1 */
  {5726623061UL,
  393213UL,
  131073UL,
  0UL},
/* 2^35-1 */
  {1108378657UL,
  483939977UL,
  270549121UL,
  279527UL,
  0UL},
/* 2^36-1 */
  {22906492245UL,
  13743895347UL,
  9817068105UL,
  5286113595UL,
  3616814565UL,
  1857283155UL,
  941362695UL,
  630453915UL,
  0UL},
/* 2^37-1 */
  {616318177UL,
  223UL,
  0UL},
/* 2^38-1 */
  {91625968981UL,
  1572861UL,
  524289UL,
  0UL},
/* 2^39-1 */
  {78536544841UL,
  6958934353UL,
  67117057UL,
  4529623UL,
  0UL},
/* 2^40-1 */
  {366503875925UL,
  219902325555UL,
  99955602525UL,
  64677154575UL,
  35468117025UL,
  26817356775UL,
  17825775UL,
  0UL},
/* 2^41-1 */
  {164511353UL,
  13367UL,
  0UL},
/* 2^42-1 */
  {1466015503701UL,
  628292358729UL,
  102280151421UL,
  34630287489UL,
  13050583119UL,
  811597437UL,
  0UL},
/* 2^43-1 */
  {20408568497UL,
  905040953UL,
  4188889UL,
  0UL},
/* 2^44-1 */
  {5864062014805UL,
  3518437208883UL,
  764877654105UL,
  197665011735UL,
  44312811195UL,
  25757227005UL,
  8325691455UL,
  0UL},
/* 2^45-1 */
  {5026338869833UL,
  1134979744801UL,
  481977699847UL,
  233009086681UL,
  55759702201UL,
  1509346321UL,
  0UL},
/* 2^46-1 */
  {23456248059221UL,
  1497207322929UL,
  394264623UL,
  25165821UL,
  0UL},
/* 2^47-1 */
  {59862819377UL,
  31184907679UL,
  10610063UL,
  0UL},
/* 2^48-1 */
  {93824992236885UL,
  56294995342131UL,
  40210710958665UL,
  21651921285435UL,
  16557351571215UL,
  2901803883615UL,
  1167945961455UL,
  1095233372415UL,
  418239192735UL,
  0UL},
/* 2^49-1 */
  {4432676798593UL,
  127UL,
  0UL},
/* 2^50-1 */
  {375299968947541UL,
  102354536985693UL,
  36319351833633UL,
  4485656999373UL,
  1873377548823UL,
  625152641223UL,
  277931351973UL,
  0UL},
/* 2^51-1 */
  {321685687669321UL,
  21862134113449UL,
  1050769861729UL,
  202518195313UL,
  17180000257UL,
  0UL},
/* 2^52-1 */
  {1501199875790165UL,
  900719925474099UL,
  84973577874915UL,
  28685347945035UL,
  2792064245115UL,
  1649066139645UL,
  549822930945UL,
  0UL},
/* 2^53-1 */
  {1416003655831UL,
  129728784761UL,
  441650591UL,
  0UL},
/* 2^54-1 */
  {6004799503160661UL,
  2573485501354569UL,
  948126237341157UL,
  246772582321671UL,
  206561081853UL,
  68585259519UL,
  0UL},
/* 2^55-1 */
  {1566469435607129UL,
  1162219258676257UL,
  404817944033303UL,
  40895342813807UL,
  11290754314937UL,
  178394823847UL,
  0UL},
/* 2^56-1 */
  {24019198012642645UL,
  14411518807585587UL,
  4238682002231055UL,
  2484744621997515UL,
  1675758000882045UL,
  637677823344495UL,
  567382630219905UL,
  4563402735UL,
  0UL},
/* 2^57-1 */
  {20587884010836553UL,
  4451159405623UL,
  274878431233UL,
  118823881393UL,
  0UL},
/* 2^58-1 */
  {96076792050570581UL,
  4885260612740877UL,
  1237040240994471UL,
  261314937580881UL,
  137975287770087UL,
  95026151247UL,
  0UL},
/* 2^59-1 */
  {3203431780337UL,
  179951UL,
  0UL},
/* 2^60-1 */
  {384307168202282325UL,
  230584300921369395UL,
  164703072086692425UL,
  104811045873349725UL,
  88686269585142075UL,
  37191016277640225UL,
  28120036697727975UL,
  18900352534538475UL,
  7635241752363225UL,
  3483146539597725UL,
  872764197279975UL,
  0UL},
/* 2^61-1 */
  {1UL,
  0UL},
/* 2^62-1 */
  {1537228672809129301UL,
  6442450941UL,
  2147483649UL,
  0UL},
/* 2^63-1 */
  {1317624576693539401UL,
  126347562148695559UL,
  72624976668147841UL,
  27369056489183311UL,
  99457304386111UL,
  14197294936951UL,
  0UL},
/* 2^64-1 */
  {6148914691236517205UL,
  3689348814741910323UL,
  1085102592571150095UL,
  71777214294589695UL,
  28778071877862015UL,
  281470681808895UL,
  2753074036095UL,
  0UL},
#if LONG_BIT > 64
#error Factorization of numbers up to 2^LONG_BIT required
#endif
#endif
};

/*
 * Modular multiply for two binary polynomial
 * mask is 1UL << the degree of the modulus.
 */
unsigned long modmul(unsigned long poly1, unsigned long poly2,
  unsigned long modulo, unsigned long mask)
{
  unsigned long result = 0;

  for (; poly1; poly1 >>= 1)
  {
    if (poly1 & 1)
      result ^= poly2;

    poly2 <<= 1;
    if (poly2 & mask)
      poly2 ^= modulo;
  }
  return result;
}

/*
 * Modular exponentiation for a binary polynomial
 * degree is the degree of the modulus.
 */
unsigned long modpow(unsigned long polynomial, unsigned long power,
  unsigned long modulo, int degree)
{
  unsigned long result = 1, mask = 1UL << degree;

  for (; power; power >>= 1)
  {
    if (power & 1)
      result = modmul(result, polynomial, modulo, mask);
    polynomial = modmul(polynomial, polynomial, modulo, mask);
  }
  return result;
}

/*
 * Test the primitivity of a polynomial
 */
int rk_isprimitive(unsigned long polynomial)
{
  unsigned long pelement = 2, temp = polynomial >> 1;
  int k, degree = 0, weight = 1;

  /* Special case for polynomials of degree < 2 */
  if (polynomial < 4)
    return (polynomial == 3) || (polynomial == 1);

  /* A binary primitive polynomial has a constant term */
  if (!(polynomial & 1))
    return 0;

  /*
   * A binary primitive polynomial of degree > 1 has an odd number of terms.
   * temp ^= temp >> 16; temp ^= temp >> 8; ... would be sligthly faster.
   * Compute the degree at the same time.
   */
  for (; temp; degree++, temp >>= 1)
    weight += temp & 1;
  if (!(weight & 1))
    return 0;

  /*
   * Check if the period is 2^degree-1.
   * Sufficient if 2^degree-1 is prime.
   */
  if (modpow(pelement, 1UL << degree, polynomial, degree) != pelement)
    return 0;

  if (divisors[degree][0] != 1)
    /* Primitivity test */
    for (k = 0; divisors[degree][k]; k++)
      if (modpow(pelement, divisors[degree][k], polynomial, degree) == 1)
        return 0;

  return 1;
}
