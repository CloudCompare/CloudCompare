//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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

#ifndef PROFILE_LOADER_HEADER
#define PROFILE_LOADER_HEADER

class ccPolyline;
class ccMainAppInterface;

//Qt
#include <QString>

//! Loads a 2D profile form a custom (ASCII) file
class ProfileLoader
{
public:

	//! Loads a 2D profile from a file
	/** The file must have a particular organization (see method for more detail).
		The profile should be associated with an absolute 3D origin (and an axis,
		specified outside). This origin will be saved as 'global shift' with the
		polyline's vertices (see ccPointCloud::setOriginalShift).
		Only the X and Y coordinates of the polyline's vertices will be used:
		X = radius and Y = height (Z = 0).
		\param filename filename
		\param ignoreAxisShift whether heights are expressed relatively to 0 (true) or to the input center position (false)
		\param heightDim height dimension
		\param app main application handle for displaying messages
		\return loaded polyline (or 0 if an error occurred)
	**/
	static ccPolyline* Load(QString filename, bool ignoreAxisShift = true, int heightDim = 2, ccMainAppInterface* app = 0);

};

#endif //PROFILE_LOADER_HEADER
