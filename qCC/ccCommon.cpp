//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This project has been initiated under funding from ANR/CIFRE          #
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

#include "ccCommon.h"

//CCLib
#include <CCPlatform.h>

#define CC_VER_NUM "2.7"
#define CC_SUB_VER "beta" //201?-??-??

//! Returns current version as a string
QString ccCommon::GetCCVersion(bool full/*=true*/)
{
	QString verStr = QString("%1.%2").arg(CC_VER_NUM).arg(CC_SUB_VER);

#if defined(CC_ENV_64)
	QString arch = "64 bits";
#elif defined(CC_ENV_32)
	QString arch = "32 bits";
#else
	QString arch = "?? bits";
#endif

	if (full)
	{
#if defined(CC_WINDOWS)
		QString platform = "Windows";
#elif defined(CC_MAC_OS)
		QString platform = "Mac OS";
#elif defined(CC_LINUX)
		QString platform = "Linux";
#else
		QString platform = "Unknown OS";
#endif

		verStr += QString(" [%1 %2]").arg(platform).arg(arch);
	}
	else
	{
		verStr += QString(" [%1]").arg(arch);
	}

#ifdef _DEBUG
	verStr += QString(" [DEBUG]");
#endif

	return verStr;
};
