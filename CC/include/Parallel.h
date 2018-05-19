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

#if defined(_MSC_VER) && (_MSC_VER >= 1800)

#define USE_PARALLEL
//Parallel Patterns Library (for parallel sort)
#include <ppl.h>
#define CCParallelSort Concurrency::parallel_sort
#define CCParallelFor Concurrency::parallel_for
#define CCParallelForEach Concurrency::parallel_for_each
#define CCCriticalSection Concurrency::critical_section
//Limit the number of thread of a parallel template function (or task)
#define CCParallelWithLimitedThreads(task, num_threads) { \
	Concurrency::CurrentScheduler::Create( Concurrency::SchedulerPolicy( 2, Concurrency::MinConcurrency, num_threads, Concurrency::MaxConcurrency, num_threads ) ); \
	Concurrency::structured_task_group tasks; \
	auto taskLambda = Concurrency::make_task([&] { task; }); \
	tasks.run(taskLambda); \
	tasks.wait(); \
	Concurrency::CurrentScheduler::Detach(); \
	} \

#elif USE_TBB

#define USE_PARALLEL
/*
 * TODO: Task arena was in preview state prior to TBB 4.3 release.
 * It's very unlikely that end user will compile with 4.2 but
 * this version is notably used by ubuntu 14.04 on Travis. */
#define TBB_PREVIEW_TASK_ARENA 1
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/parallel_sort.h>
#include <tbb/critical_section.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#define CCParallelSort tbb::parallel_sort
#define CCParallelFor tbb::parallel_for
#define CCParallelForEach tbb::parallel_for_each
#define CCCriticalSection tbb::critical_section
//Limit the number of thread of a parallel template function (or task)
#define CCParallelWithLimitedThreads(task, num_threads) { \
	tbb::task_arena limited_arena(num_threads, 1); \
	limited_arena.execute([&]{ \
		task; \
	}); \
}

#else

#define CCParallelSort std::sort

#endif

#endif // CCCORE_PARALLEL_HEADER
