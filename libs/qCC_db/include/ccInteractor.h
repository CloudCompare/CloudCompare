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

#ifndef CC_INTERACTOR_HEADER
#define CC_INTERACTOR_HEADER

//Local
#include "qCC_db.h"

#include "CCGeom.h"

//Qt
#include <Qt>

//! Interactor interface (entity that can be dragged or clicked in a 3D view)
class QCC_DB_LIB_API ccInteractor
{
public:

	virtual ~ccInteractor() = default;
	
	//! Called on mouse click
	virtual bool acceptClick(int x, int y, Qt::MouseButton button) { return false; }

	//! Called on mouse move (for 2D interactors)
	/** \return true if a movement occurs
	**/
	virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) { return false; }

	//! Called on mouse move (for 3D interactors)
	/** \return true if a movement occurs
	**/
	virtual bool move3D(const CCVector3d& u) { return false; }

};

#endif //CC_INTERACTOR_HEADER
