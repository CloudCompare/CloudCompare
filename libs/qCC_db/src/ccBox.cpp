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

#include "ccBox.h"

//qCC_db
#include "ccPlane.h"
#include "ccPointCloud.h"

ccBox::ccBox(	const CCVector3& dims,
				const ccGLMatrix* transMat/*= 0*/,
				QString name/*=QString("Box")*/)
	: ccGenericPrimitive(name, transMat)
	, m_dims(dims)
{
	updateRepresentation();
}

ccBox::ccBox(QString name/*=QString("Box")*/)
	: ccGenericPrimitive(name)
	, m_dims(0, 0, 0)
{
}

bool ccBox::buildUp()
{
	//clear triangles indexes
	if (m_triVertIndexes)
	{
		m_triVertIndexes->clear();
	}
	//clear per triangle normals
	removePerTriangleNormalIndexes();
	if (m_triNormals)
	{
		m_triNormals->clear();
	}
	//clear vertices
	ccPointCloud* verts = vertices();
	if (verts)
	{
		verts->clear();
	}

	//upper plane
	ccGLMatrix upperMat;
	upperMat.getTranslation()[2] = m_dims.z / 2;
	*this += ccPlane(m_dims.x, m_dims.y, &upperMat);
	//lower plane
	ccGLMatrix lowerMat;
	lowerMat.initFromParameters(-static_cast<PointCoordinateType>(M_PI), CCVector3(1, 0, 0), CCVector3(0, 0, -m_dims.z / 2));
	*this += ccPlane(m_dims.x, m_dims.y, &lowerMat);
	//left plane
	ccGLMatrix leftMat;
	leftMat.initFromParameters(-static_cast<PointCoordinateType>(M_PI / 2), CCVector3(0, 1, 0), CCVector3(-m_dims.x / 2, 0, 0));
	*this += ccPlane(m_dims.z, m_dims.y, &leftMat);
	//right plane
	ccGLMatrix rightMat;
	rightMat.initFromParameters(static_cast<PointCoordinateType>(M_PI / 2), CCVector3(0, 1, 0), CCVector3(m_dims.x / 2, 0, 0));
	*this += ccPlane(m_dims.z, m_dims.y, &rightMat);
	//front plane
	ccGLMatrix frontMat;
	frontMat.initFromParameters(static_cast<PointCoordinateType>(M_PI / 2), CCVector3(1, 0, 0), CCVector3(0, -m_dims.y / 2, 0));
	*this += ccPlane(m_dims.x, m_dims.z, &frontMat);
	//back plane
	ccGLMatrix backMat;
	backMat.initFromParameters(-static_cast<PointCoordinateType>(M_PI / 2), CCVector3(1, 0, 0), CCVector3(0, m_dims.y / 2, 0));
	*this += ccPlane(m_dims.x, m_dims.z, &backMat);

	return (vertices() && vertices()->size() == 24 && this->size() == 12);
}

ccGenericPrimitive* ccBox::clone() const
{
	return finishCloneJob(new ccBox(m_dims, &m_transformation, getName()));
}

bool ccBox::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_dims.x;
	outStream << m_dims.y;
	outStream << m_dims.z;

	return true;
}

bool ccBox::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_dims.u, 3);

	return true;
}
