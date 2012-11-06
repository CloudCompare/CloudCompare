#ifndef MiscLib__PERFORMANCE_HEADER__
#define MiscLib__PERFORMANCE_HEADER__
#ifdef WIN32
#include <windows.h>
#undef max
#undef min
#endif

namespace MiscLib
{
#ifdef WIN32
typedef __int64 performance_t;
inline performance_t GetPerformanceCounter()
{
	LARGE_INTEGER count;
	QueryPerformanceCounter(&count);
	return count.QuadPart;
}
inline double GetPerformanceFreq()
{
	LARGE_INTEGER dummy_performance_freq;
	QueryPerformanceFrequency(&dummy_performance_freq);
	performance_t performance_freq = dummy_performance_freq.QuadPart;
	return double(performance_freq);
}
#else
typedef clock_t performance_t;
inline performance_t GetPerformanceCounter()
{
	return clock();
}
inline double GetPerformanceFreq() { return double(CLOCKS_PER_SEC); }
#endif
};

#endif
