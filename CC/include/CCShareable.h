#ifndef CC_SHAREABLE_HEADER
#define CC_SHAREABLE_HEADER

#include <assert.h>

//Activate shared objects tracking (for debug purposes)
#ifdef _DEBUG
//#define CC_TRACK_ALIVE_SHARED_OBJECTS
#endif

//CCShareable object (with counter)
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API CCShareable
#else
class CCShareable
#endif
{
public:

	//! Default constructor
	CCShareable();

	//! Increase counter
	/** Should be called when this object is 'attached' to another one.
	**/
	virtual void link();

	//! Decrease counter and deletes object when 0
	/** Should be called when this object is 'detached'.
	**/
	virtual void release();

	//! Returns the current link count
	/** \return current link count
	**/
	inline virtual unsigned getLinkCount() const { return m_linkCount; }

#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS
	//! Get alive shared objects count
	static unsigned GetAliveCount();
#endif

protected:

	//! Destructor
	/** private to avoid deletion with 'delete' operator
	**/
	virtual ~CCShareable();

	//! Links counter
	unsigned m_linkCount;
};

#endif
