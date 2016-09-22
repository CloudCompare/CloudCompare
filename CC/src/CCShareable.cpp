//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "CCShareable.h"

#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4530 )
#endif

//system
#include <vector>

//set of all currently 'alived' shared objects
static std::vector<CCShareable*> s_aliveSharedObjects;

unsigned CCShareable::GetAliveCount()
{
	return s_aliveSharedObjects.size();
}

#endif

CCShareable::CCShareable() : m_linkCount(0)
{
#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS
	s_aliveSharedObjects.push_back(this);
#endif
}

void CCShareable::link()
{
	++m_linkCount;
}

void CCShareable::release()
{
	if (m_linkCount > 1)
		--m_linkCount;
	else
		delete this;
}

CCShareable::~CCShareable()
{
#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS
	std::vector<CCShareable*>::iterator it;
	for (it=s_aliveSharedObjects.begin(); it!=s_aliveSharedObjects.end(); ++it)
	{
		if (*it == this)
		{
			std::swap(*it,s_aliveSharedObjects.back());
			s_aliveSharedObjects.pop_back();
			return;
		}
	}
#endif
}
