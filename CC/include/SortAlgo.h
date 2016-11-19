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

#ifndef SORT_ALGO_HEADER
#define SORT_ALGO_HEADER

#ifndef SortAlgo

	#if defined(_MSC_VER) && (_MSC_VER >= 1800)

		//Parallel Patterns Library (for parallel sort)
		#include <ppl.h>
		#define SortAlgo Concurrency::parallel_sort

	#else

		//TODO: find a portable parallel sort algorithm
		#define SortAlgo std::sort

	#endif

#endif //#ifndef SortAlgo

#endif //SORT_ALGO_HEADER