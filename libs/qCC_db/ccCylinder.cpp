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

#include "ccCylinder.h"

ccCylinder::ccCylinder(	PointCoordinateType radius,
						PointCoordinateType height,
						const ccGLMatrix* transMat/*=0*/,
						QString name/*=QString("Cylinder")*/,
						unsigned precision/*=DEFAULT_DRAWING_PRECISION*/,
						unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccCone(radius, radius, height, 0, 0, transMat, name, precision, uniqueID)
{
}

ccCylinder::ccCylinder(QString name/*=QString("Cylinder")*/)
	: ccCone(name)
{
}

ccGenericPrimitive* ccCylinder::clone() const
{
	return finishCloneJob(new ccCylinder(m_bottomRadius, m_height, &m_transformation, getName(), m_drawPrecision));
}

void ccCylinder::setBottomRadius(PointCoordinateType radius)
{
	//we set the top radius as well!
	m_topRadius = radius;
	ccCone::setBottomRadius(radius);
}

