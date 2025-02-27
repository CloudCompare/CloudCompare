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
				const ccGLMatrix* transMat/*=nullptr*/,
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
	if (!init(8, false, 12, 6))
	{
		ccLog::Error("[ccPlane::buildUp] Not enough memory");
		return false;
	}
	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	//   5 -- 6
	// 1/-- 2/|
	// | 4 -|-7
	// 0/-- 3/
	verts->addPoint(CCVector3(-m_dims.x / 2, -m_dims.y / 2,  m_dims.z / 2));
	verts->addPoint(CCVector3(-m_dims.x / 2,  m_dims.y / 2,  m_dims.z / 2));
	verts->addPoint(CCVector3( m_dims.x / 2,  m_dims.y / 2,  m_dims.z / 2));
	verts->addPoint(CCVector3( m_dims.x / 2, -m_dims.y / 2,  m_dims.z / 2));
	verts->addPoint(CCVector3(-m_dims.x / 2, -m_dims.y / 2, -m_dims.z / 2));
	verts->addPoint(CCVector3(-m_dims.x / 2,  m_dims.y / 2, -m_dims.z / 2));
	verts->addPoint(CCVector3( m_dims.x / 2,  m_dims.y / 2, -m_dims.z / 2));
	verts->addPoint(CCVector3( m_dims.x / 2, -m_dims.y / 2, -m_dims.z / 2));

	//front plane
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 0, 1)));
	addTriangle(0, 2, 1);
	addTriangleNormalIndexes(0, 0, 0);
	addTriangle(0, 3, 2);
	addTriangleNormalIndexes(0, 0, 0);

	//left plane
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(-1, 0, 0)));
	addTriangle(4, 1, 5);
	addTriangleNormalIndexes(1, 1, 1);
	addTriangle(4, 0, 1);
	addTriangleNormalIndexes(1, 1, 1);

	//back plane
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 0, -1)));
	addTriangle(7, 5, 6);
	addTriangleNormalIndexes(2, 2, 2);
	addTriangle(7, 4, 5);
	addTriangleNormalIndexes(2, 2, 2);

	//right plane
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(1, 0, 0)));
	addTriangle(3, 6, 2);
	addTriangleNormalIndexes(3, 3, 3);
	addTriangle(3, 7, 6);
	addTriangleNormalIndexes(3, 3, 3);

	//lower plane
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, -1, 0)));
	addTriangle(4, 3, 0);
	addTriangleNormalIndexes(4, 4, 4);
	addTriangle(4, 7, 3);
	addTriangleNormalIndexes(4, 4, 4);

	//upper plane
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 1, 0)));
	addTriangle(1, 6, 5);
	addTriangleNormalIndexes(5, 5, 5);
	addTriangle(1, 2, 6);
	addTriangleNormalIndexes(5, 5, 5);

	return true;
}

ccGenericPrimitive* ccBox::clone() const
{
	return finishCloneJob(new ccBox(m_dims, &m_transformation, getName()));
}

bool ccBox::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 21)
	{
		assert(false);
		return false;
	}

	if (!ccGenericPrimitive::toFile_MeOnly(out, dataVersion))
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

short ccBox::minimumFileVersion_MeOnly() const
{
	return std::max(static_cast<short>(21), ccGenericPrimitive::minimumFileVersion_MeOnly());
}
