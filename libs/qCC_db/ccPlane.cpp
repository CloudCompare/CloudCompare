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

#include "ccPlane.h"

//qCC_db
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

ccPlane::ccPlane(PointCoordinateType xWidth, PointCoordinateType yWidth, const ccGLMatrix* transMat /*= 0*/, QString name/*=QString("Plane")*/)
	: ccGenericPrimitive(name,transMat)
	, m_xWidth(xWidth)
	, m_yWidth(yWidth)
{
	buildUp();
	applyTransformationToVertices();
}

bool ccPlane::buildUp()
{
	if (!init(4,false,2,1))
	{
		ccLog::Error("[ccPlane::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	verts->addPoint(CCVector3(-m_xWidth*0.5f,-m_yWidth*0.5f, 0));
	verts->addPoint(CCVector3(-m_xWidth*0.5f, m_yWidth*0.5f, 0));
	verts->addPoint(CCVector3( m_xWidth*0.5f, m_yWidth*0.5f, 0));
	verts->addPoint(CCVector3( m_xWidth*0.5f,-m_yWidth*0.5f, 0));

	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0,0.0,1.0).u));

	addTriangle(0,2,1);
	addTriangleNormalIndexes(0,0,0);
	addTriangle(0,3,2);
	addTriangleNormalIndexes(0,0,0);

	return true;
}

ccPlane::ccPlane(QString name /*=QString("Plane")*/)
	: ccGenericPrimitive(name)
	, m_xWidth(0)
	, m_yWidth(0)
{
}

ccGenericPrimitive* ccPlane::clone() const
{
	return finishCloneJob(new ccPlane(m_xWidth,m_yWidth,&m_transformation,getName()));
}

bool ccPlane::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_xWidth;
    outStream << m_yWidth;

	return true;
}

bool ccPlane::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	inStream >> m_xWidth;
	inStream >> m_yWidth;

	return true;
}
