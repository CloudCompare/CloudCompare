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
#include <QTime>

//System
#include <assert.h>

//unique instance
static QTime* s_eTimer = 0;

void ccTimer::Init()
{
    if (!s_eTimer)
	{
        s_eTimer = new QTime();
		s_eTimer->start();
	}
}

int ccTimer::Sec()
{
	assert(s_eTimer && s_eTimer->isValid());
	return (s_eTimer ? s_eTimer->elapsed()/1000 : 0); //QTime::elapsed = ms precision
}

int ccTimer::Msec()
{
	assert(s_eTimer && s_eTimer->isValid());
	return (s_eTimer ? s_eTimer->elapsed() : 0); //QTime::elapsed = ms precision
}
