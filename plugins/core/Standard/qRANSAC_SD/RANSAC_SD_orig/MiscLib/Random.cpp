/*
 * random.c
 *
 * implementiert lagged Fibonacci random sequence
 *   exakt nach Knuth TAOCP Vol.2 pp.186f
 *
 */
#include <stdio.h>
#include "Random.h"

using namespace MiscLib;

#define KK 100
#define LL 37
#define MM MiscLib_RN_RAND_MOD
#define TT 70

#define mod_diff(x,y) ( (x) - (y) & (MM-1) )
#define is_odd(x)     ( (x) & 1 )
#define evenize(x)    ( (x) & (MM-2) )

size_t MiscLib::rn_buf[MiscLib_RN_BUFSIZE];
size_t MiscLib::rn_point = MiscLib_RN_BUFSIZE;

void MiscLib::rn_setseed(size_t seed)
{
  register int t, j;
  size_t x[KK+KK-1];
  register size_t ss = evenize(seed+2);
  for (j=0;j<KK;j++) {
    x[j]=ss;
    ss<<=1;
    if (ss>=MM) ss-=MM-2;
  }
  for (;j<KK+KK-1;j++) x[j]=0;
  x[1]++;
  ss=seed&(MM-1);
  t=TT-1;
  while (t) {
    for (j=KK-1; j>0; j--) x[j+j]=x[j];
    for (j=KK+KK-2; j>KK-LL; j-=2) x[KK+KK-1-j]=evenize(x[j]);
    for (j=KK+KK-2; j>=KK; j--) if (is_odd(x[j])) {
      x[j-(KK-LL)]=mod_diff(x[j-(KK-LL)],x[j]);
      x[j-KK]=mod_diff(x[j-KK],x[j]);
    }
    if(is_odd(ss)) {
      for (j=KK;j>0;j--) x[j]=x[j-1];
      x[0]=x[KK];
      if (is_odd(x[KK])) x[LL]=mod_diff(x[LL],x[KK]);
    }
    if (ss) ss>>=1; else t--;
  }
  for (j=0;j<LL;j++) rn_buf[j+KK-LL]=x[j];
  for (;j<KK;j++) rn_buf[j-LL]=x[j];  
}

size_t MiscLib::rn_refresh()
{
/* You remember Duff's device? If it would help then it should be used here */
  rn_point=1;

  register int i, j;
  for (j=KK;j<MiscLib_RN_BUFSIZE;j++) 
    rn_buf[j]=mod_diff(rn_buf[j-KK],rn_buf[j-LL]);
  for (i=0;i<LL;i++,j++) rn_buf[i]=mod_diff(rn_buf[j-KK],rn_buf[j-LL]);
  for (;i<KK;i++,j++) rn_buf[i]=mod_diff(rn_buf[j-KK],rn_buf[i-LL]);
 

  return *rn_buf;
}
