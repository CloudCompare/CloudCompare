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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//CCCoreLib
#include <CCGeom.h>

//qCC_gl
#include <ccGLDrawContext.h>

//! Interface for a planar entity
class ccPlanarEntityInterface
{
public:
	
	//! Default constructor
	ccPlanarEntityInterface();

	//! Show normal vector
	inline void showNormalVector(bool state) { m_showNormalVector = state; }
	//! Whether normal vector is shown or not
	inline bool normalVectorIsShown() const { return m_showNormalVector; }

	//! Returns the entity normal
	virtual CCVector3d getNormal() const = 0;

protected: //members

	//! Draws a normal vector (OpenGL)
	void glDrawNormal(CC_DRAW_CONTEXT& context, const CCVector3d& pos, double scale, const ccColor::Rgb* color = nullptr);

	//! Whether the facet normal vector should be displayed or not
	bool m_showNormalVector;

};
