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

#ifndef DEBUG_PROGRESS_CALLBACK_HEADER
#define DEBUG_PROGRESS_CALLBACK_HEADER

//Local
#include "CCCoreLib.h"
#include "GenericProgressCallback.h"

namespace CCLib
{

//! Displays the progress status and other information about an ongoing process in a text console
/** Implements the GenericProgressCallback interface.
**/
class CC_CORE_LIB_API DebugProgressCallback : public GenericProgressCallback
{
public:

	//! DebugProgressCallback constructor
	DebugProgressCallback();

	//! DebugProgressCallback destructor
	virtual ~DebugProgressCallback();

	//inherited methods (see GenericProgressCallback)
	virtual void reset();
	virtual void update(float percent);
	virtual void setMethodTitle(const char* methodTitle);
	virtual void setInfo(const char* infoStr);
	virtual bool isCancelRequested() { return false; } //no cancel request mechanism available in "console" mode
	virtual void start();
	virtual void stop();

protected:

	//! Process name
	char methodTitle[256];
	//! Some information about the ongoing process
	char displayedInfos[256];
	//! Progress status
	float currentPercentage;
	//! Tenth of total progression
	int decade;
};

}

#endif //DEBUG_PROGRESS_CALLBACK_HEADER
