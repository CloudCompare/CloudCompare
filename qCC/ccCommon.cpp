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

#include "ccCommon.h"

#define CC_VER_NUM 2.5
#define CC_SUB_VER 0 //2013-07-10

//! Returns current version as string
QString ccCommon::GetCCVersion()
{
#if defined(_W64) || defined(__x86_64__) || defined(__ppc64__)
	QString format = "64 bits";
#else
	QString format = "32 bits";
#endif

#if defined(_WIN32) || defined(WIN32)
	QString platform = "Windows";
#elif defined(__APPLE__)
	QString platform = "Mac OS";
#else
	QString platform = "Linux";
#endif

	return QString("%1.%2").arg(CC_VER_NUM).arg(CC_SUB_VER)+QString(" [%1 %2]").arg(platform).arg(format);
};
