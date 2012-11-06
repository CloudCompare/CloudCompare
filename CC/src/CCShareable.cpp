#include "CCShareable.h"

#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4530 )
#endif

#include <vector>
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
	if (m_linkCount>1)
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

