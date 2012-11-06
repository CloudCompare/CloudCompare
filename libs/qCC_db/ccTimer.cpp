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

#include "ccTimer.h"

//Qt
#include <QElapsedTimer>

//System
#include <assert.h>

//unique instance
static QElapsedTimer* s_eTimer = 0;

void ccTimer::Init()
{
    if (!s_eTimer)
	{
        s_eTimer = new QElapsedTimer();
		s_eTimer->start();
	}
}

qint64 ccTimer::Sec()
{
	assert(s_eTimer && s_eTimer->isValid());
	return (s_eTimer->msecsSinceReference()+s_eTimer->elapsed())/1000; //elapsed = ms precision
}

qint64 ccTimer::Msec()
{
	assert(s_eTimer && s_eTimer->isValid());
	return s_eTimer->msecsSinceReference()+s_eTimer->elapsed(); //elapsed = ms precision
}
