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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef CC_TIMER_HEADER
#define CC_TIMER_HEADER

//Qt
#include <QElapsedTimer>

//! Absolute timer
/** Now based on QElapsedTimer for 64 bits portability and 
	sharing same time reference with plugins.
	Timer must still be initialized once (with Init) at 
	application/plugin start.
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccTimer
#else
class ccTimer
#endif
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

protected:

};

#endif
