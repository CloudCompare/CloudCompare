/*
 * ccTrace.h
 *
 *  Created on: 14 mars 2020
 *      Author: paul
 */

#ifndef CC_INCLUDE_CCTRACE_H_
#define CC_INCLUDE_CCTRACE_H_

#include <iostream>
#include <sstream>

#ifdef _CCDEBUG_
#define CCTRACE(msg) {std::cerr<<std::flush<<__FILE__<<" ["<<__LINE__<<"] : "<<msg<<std::endl<<std::flush;}
#else
#define CCTRACE(msg)
#endif

#endif /* CC_INCLUDE_CCTRACE_H_ */
