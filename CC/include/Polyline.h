//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_POLYLINE_HEADER
#define CC_POLYLINE_HEADER

#include "ReferenceCloud.h"

namespace CCLib
{

//! A simple polyline class
/** The polyline is considered as a cloud of points
	(in a specific order) with a open/closed state
	information.
**/
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API Polyline : public ReferenceCloud
#else
class Polyline : public ReferenceCloud
#endif
{
	public:

		//! Polyline constructor
		Polyline(GenericIndexedCloudPersist* associatedCloud);

		//! Returns whether the polyline is closed or not
		inline bool isClosed() const { return m_isClosed; }

		//! Sets whether the polyline is closed or not
		inline void setClosed(bool state) { m_isClosed = state; }

		//inherited from ReferenceCloud
		virtual void clear();

	protected:

		//! Closing state
		bool m_isClosed;
};

}

#endif //CC_POLYLINE_HEADER
