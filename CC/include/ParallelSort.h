//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#ifndef PARALLEL_SORT_HEADER
#define PARALLEL_SORT_HEADER

#ifdef ParallelSort
#undef ParallelSort
#pragma message "Replacing preprocessor symbol 'ParallelSort' with the one defined in Parallel.h"
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1800)

	//Parallel Patterns Library (for parallel sort)
	#include <ppl.h>

	#define ParallelSort Concurrency::parallel_sort

#elif USE_TBB

	#include <tbb/parallel_sort.h>

	#define ParallelSort tbb::parallel_sort

#else

	#include <algorithm>

	#define ParallelSort std::sort

#endif

#endif
