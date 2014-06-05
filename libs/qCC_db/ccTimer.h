//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_TIMER_HEADER
#define CC_TIMER_HEADER

//Local
#include "qCC_db.h"

//! Absolute timer
/** Timer must be initialized once (with Init) at application/plugin start.
**/
class QCC_DB_LIB_API ccTimer
{
public:

	//! Inits static object
	/** Must be called once before any call to ccTimer.
	**/
	static void Init();

	//! Returns number of seconds since timer was initialized
	static int Sec();
	//! Returns number of milli-seconds since timer was initialized
	static int Msec();

};

#endif //CC_TIMER_HEADER
