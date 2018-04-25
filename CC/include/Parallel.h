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

#ifndef CCCORE_PARALLEL_HEADER
#define CCCORE_PARALLEL_HEADER
#include <atomic>

#if defined(_MSC_VER) && (_MSC_VER >= 1800)

#define USE_PARALLEL
//Parallel Patterns Library (for parallel sort)
#include <ppl.h>
#define CCParallelSort Concurrency::parallel_sort
#define CCParallelFor Concurrency::parallel_for
#define CCParallelForEach Concurrency::parallel_for_each

#elif USE_TBB

#define USE_PARALLEL
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/parallel_sort.h>
#define CCParallelSort tbb::parallel_sort
#define CCParallelFor tbb::parallel_for
#define CCParallelForEach tbb::parallel_for_each

#else

#define CCParallelSort std::sort

#endif

#endif // CCCORE_PARALLEL_HEADER
