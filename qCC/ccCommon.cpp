//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This project has been initated under funding from ANR/CIFRE           #
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

#include "ccPlatform.h"
#include "ccCommon.h"

#define CC_VER_NUM 2.5
#define CC_SUB_VER 2 //2013-10-24

//! Returns current version as string
QString ccCommon::GetCCVersion()
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

#if defined(CC_ENV_64)
	QString format = "64 bits";
#elif defined(CC_ENV_32)
	QString format = "32 bits";
#else
	QString format = "?? bits";
#endif

	return QString("%1.%2").arg(CC_VER_NUM).arg(CC_SUB_VER)+QString(" [%1 %2]").arg(platform).arg(format);
};
