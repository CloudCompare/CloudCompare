#ifndef MiscLib__RANDOM_HEADER__
#define MiscLib__RANDOM_HEADER__
/*
 * random.h -- Random number generation interface
 *
 *  for further details see Knuth TAOCP Vol.2 pp. 186f
 *
 */

#define MiscLib_RN_BUFSIZE 500
#define MiscLib_RN_RAND_MOD (1L << 30)
#define MiscLib_RN_CONST 6180339887L 

namespace MiscLib
{
	extern size_t rn_buf[];
	extern size_t rn_point;
	void rn_setseed(size_t);
	size_t rn_refresh(void);
	inline size_t rn_rand()
	{
		size_t idx = rn_point++;
		return (MiscLib_RN_BUFSIZE > idx)?
			rn_buf[idx] : rn_refresh();
	}
	inline size_t rn_urand(size_t m)
	{
		return rn_rand() % m;
	}
	inline float rn_frand()
	{
		return (float)rn_rand() / MiscLib_RN_RAND_MOD;
	}
};

#endif
