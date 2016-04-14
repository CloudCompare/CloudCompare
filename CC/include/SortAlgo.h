#ifndef SORT_ALGO_HEADER
#define SORT_ALGO_HEADER

#ifndef SortAlgo

	#if (_MSC_VER >= 1800)

		//Parallel Patterns Library (for parallel sort)
		#include <ppl.h>
		#define SortAlgo Concurrency::parallel_sort

	#else

		//TODO: find a portable parallel sort algorithm
		#define SortAlgo std::sort

	#endif

#endif //#ifndef SortAlgo

#endif //SORT_ALGO_HEADER