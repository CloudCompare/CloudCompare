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

//Qt
#include <QTime>

//System
#include <assert.h>

//encapsulation of a (pointer on a) Qt timer (for proper deletion on app exit)
struct Timer
{
	QTime* m_timer;
	Timer() : m_timer(0) {}
	~Timer() { if (m_timer) delete m_timer; }
};
//unique instance
static Timer s_timer;

void ccTimer::Init()
{
	if (!s_timer.m_timer)
	{
        s_timer.m_timer = new QTime();
		s_timer.m_timer->start();
	}
}

int ccTimer::Sec()
{
	assert(s_timer.m_timer && s_timer.m_timer->isValid());
	return (s_timer.m_timer ? s_timer.m_timer->elapsed()/1000 : 0);
}

int ccTimer::Msec()
{
	assert(s_timer.m_timer && s_timer.m_timer->isValid());
	return (s_timer.m_timer ? s_timer.m_timer->elapsed() : 0);
}
