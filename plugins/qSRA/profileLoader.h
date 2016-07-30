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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef PROFILE_LOADER_HEADER
#define PROFILE_LOADER_HEADER

class ccPolyline;
class ccMainAppInterface;

//Qt
#include <QString>

//CCLib
#include <CCGeom.h>

//! Loads a 2D profile form a custom (ASCII) file
class ProfileLoader
{
public:

	//! Loads a 2D profile from a file
	/** The file must have a particular organization (see the code for more details).
		Notably the profile is associated to a 3D origin.
		Only the X and Y coordinates of the polyline's vertices will be used:
		X = radius and Y = height (Z = 0).
		\param[in] filename filename
		\param[out] origin profile origin
		\param[in] app main application handle for displaying messages (optional)
		\return loaded polyline (or 0 if an error occurred)
	**/
	static ccPolyline* Load(QString filename, CCVector3& origin, ccMainAppInterface* app = 0);

};

#endif //PROFILE_LOADER_HEADER
