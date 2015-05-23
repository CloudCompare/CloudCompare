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

#include "ccTimer.h"

//Local
#include "ccSingleton.h"

//Qt
#include <QTime>

//System
#include <assert.h>

//unique instance
static ccSingleton<QTime> s_timer;

void ccTimer::Init()
{
	if (!s_timer.instance)
	{
		s_timer.instance = new QTime();
		s_timer.instance->start();
	}
}

int ccTimer::Sec()
{
	assert(s_timer.instance && s_timer.instance->isValid());
	return (s_timer.instance ? s_timer.instance->elapsed()/1000 : 0);
}

int ccTimer::Msec()
{
	assert(s_timer.instance && s_timer.instance->isValid());
	return (s_timer.instance ? s_timer.instance->elapsed() : 0);
}
