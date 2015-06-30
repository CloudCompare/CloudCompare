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

#ifdef _MSC_VER
//To get rid of the really annoying warnings about unsafe methods
#pragma warning( disable: 4996 )
#endif

#include "DebugProgressCallback.h"

//system
#include <stdio.h>
#include <string.h>

using namespace CCLib;

DebugProgressCallback::DebugProgressCallback()
{
	reset();
}

DebugProgressCallback::~DebugProgressCallback()
{
}

void DebugProgressCallback::reset()
{
	setMethodTitle("Undefined Method");
	setInfo("No info");
	currentPercentage = 0;
	decade = 0;
}

void DebugProgressCallback::update(float percent)
{
	while (percent - static_cast<float>(10*decade) > 10.0)
	{
		++decade;
		printf(".");
	}

	currentPercentage = percent;
}

void DebugProgressCallback::setMethodTitle(const char* _methodTitle)
{
	strcpy(methodTitle,_methodTitle);
}

void DebugProgressCallback::setInfo(const char* _infoStr)
{
	strcpy(displayedInfos,_infoStr);
}

void DebugProgressCallback::start()
{
	printf("[Start : %s]\nInfos: %s\n-->   Progress [",methodTitle,displayedInfos);
}

void DebugProgressCallback::stop()
{
	printf("OK]\n");
}
