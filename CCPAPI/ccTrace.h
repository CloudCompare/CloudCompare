/*
 * ccTrace.h
 *
 *  Created on: 14 mars 2020
 *      Author: paul
 */

#ifndef CCPAPI_CCTRACE_H_
#define CCPAPI_CCTRACE_H_

#include <iostream>
#include <sstream>

#ifdef _CCDEBUG_
#define CCTRACE(msg) {std::cerr<<std::flush<<__FILE__<<" ["<<__LINE__<<"] : "<<msg<<std::endl<<std::flush;}
#else
#define CCTRACE(msg)
#endif

#endif /* CCPAPI_CCTRACE_H_ */
