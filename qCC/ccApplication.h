#ifndef CCAPPLICATION_H
#define CCAPPLICATION_H

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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

//Common
#include <ccApplicationBase.h>

class ccApplication : public ccApplicationBase
{
	Q_OBJECT

public:
	ccApplication( int &argc, char **argv, bool isCommandLine );

	//! Returns the minimum version of CC required to load a given file version
	static QString GetMinCCVersionForFileVersion(short fileVersion);

protected:
	bool event( QEvent *inEvent ) override;
};

#endif
