#ifndef CCFILEUTILS_H
#define CCFILEUTILS_H
//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include <QStandardPaths>

namespace ccFileUtils
{
	//! Shortcut for getting the documents location path
	inline QString	defaultDocPath()
	{
		// note that according to the docs the QStandardPaths::DocumentsLocation path is never empty
		return QStandardPaths::standardLocations( QStandardPaths::DocumentsLocation ).first();
	}
}
#endif
