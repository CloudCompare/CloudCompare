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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1856                                                              $
//$LastChangedDate:: 2011-05-21 21:34:24 +0200 (sam., 21 mai 2011)         $
//**************************************************************************
//

#include "ccCylinder.h"

ccCylinder::ccCylinder(PointCoordinateType radius,
					   PointCoordinateType height,
					   const ccGLMatrix* transMat/*=0*/,
					   QString name/*=QString("Cylinder")*/,
					   unsigned precision/*=24*/)
	: ccCone(radius,radius,height,0,0,transMat,name,precision)
{
}

ccCylinder::ccCylinder(QString name/*=QString("Cylinder")*/)
	: ccCone(name)
{
}

ccGenericPrimitive* ccCylinder::clone() const
{
	return finishCloneJob(new ccCylinder(m_bottomRadius,m_height,&m_transformation,getName(),m_drawPrecision));
}
