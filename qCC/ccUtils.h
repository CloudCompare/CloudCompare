#pragma once

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

class QString;

//CCCoreLib
#include <CCGeom.h>

namespace ccUtils
{
	//! Display a warning or error for locked verts
	void DisplayLockedVerticesWarning(const QString &meshName, bool displayAsError);

	//! Tries to convert the current contents of the clipboard into a vector (3 values)
	/** Supported separators: white spaces, comma or semicolon
	**/
	bool GetVectorFromClipboard(CCVector3d& vector, bool sendErrors = true);
}
