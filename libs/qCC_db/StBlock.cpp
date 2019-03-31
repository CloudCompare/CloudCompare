//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
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

#include "StBlock.h"

//qCC_db
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccNormalVectors.h"

//CCLib
#include <Delaunay2dMesh.h>

//system
#include <string.h>


StBlock::StBlock(const std::vector<CCVector2>& profile,
	PointCoordinateType bottom_height, 
	PointCoordinateType top_height,
	const ccGLMatrix * transMat,
	QString name)
	: ccGenericPrimitive(name, transMat)
{
	assert(profile.size() > 2);

	for (auto & pt : profile) {
		
	}

	updateRepresentation();
}

StBlock::StBlock(const std::vector<CCVector3>& top, 
	const std::vector<CCVector3>& bottom,
	const ccGLMatrix * transMat,
	QString name)
	: ccGenericPrimitive(name, transMat)
{
	assert(top.size() > 2);
	assert(top.size() == bottom.size());

	m_top = ccFacet::CreateFromContour(top, "top");
	m_bottom = ccFacet::CreateFromContour(bottom, "bottom");

	updateRepresentation();
}

StBlock::StBlock(CCLib::GenericIndexedCloudPersist * top_cloud,
	const std::vector<int>& top_index, 
	CCLib::GenericIndexedCloudPersist * bottom_cloud, 
	const std::vector<int>& bottom_index,
	const ccGLMatrix * transMat, QString name)
	: ccGenericPrimitive(name, transMat)
{
	assert(top_index.size() > 2);
	assert(top_index.size() == bottom_index.size());

	

	updateRepresentation();
}

StBlock::StBlock(ccFacet * top, ccFacet * bottom, const ccGLMatrix * transMat, QString name)
{
	m_top = top->clone();
	m_bottom = bottom->clone();
	updateRepresentation();
}

StBlock::StBlock(QString name/*="Block"*/)
	: ccGenericPrimitive(name)
{
}

ccGenericPrimitive* StBlock::clone() const
{
	return finishCloneJob(new StBlock(m_top, m_bottom, &m_transformation, getName()));
}

std::vector<CCVector3> StBlock::getTop() 
{
	return m_top->getContour()->getPoints(false);
}

std::vector<CCVector3> StBlock::getBottom() 
{
	return m_bottom->getContour()->getPoints(false);
}

std::vector<CCVector2> StBlock::getProfile()
{
	std::vector<CCVector2> profile;
	std::vector<CCVector3> points = m_top->getContour()->getPoints(false);
	for (auto & pt : points) {
		profile.push_back(CCVector2(pt.x, pt.y));
	}
	return profile;
}

void StBlock::TopHeightAdd(double val)
{
	ccPointCloud* cloud = m_top->getContourVertices();
	
	for (size_t i = 0; i < cloud->size(); i++) {
		CCVector3& P = const_cast<CCVector3&>(*cloud->getPoint(i));
		P.z += val;
	}
	cloud->invalidateBoundingBox();
}

void StBlock::BottomHeightAdd(double val)
{
	ccPointCloud* cloud = m_bottom->getContourVertices();

	for (size_t i = 0; i < cloud->size(); i++) {
		CCVector3& P = const_cast<CCVector3&>(*cloud->getPoint(i));
		P.z += val;
	}
	cloud->invalidateBoundingBox();
}

bool StBlock::buildUp()
{

	unsigned count = static_cast<unsigned>(m_top->getContour()->size());
	if (count < 3)
		return false;

	CCLib::Delaunay2dMesh mesh;

	//DGM: we check that last vertex is different from the first one!
	//(yes it happens ;)
 	

	std::vector<CCVector2> profile = getProfile();

	if (profile.back().x == profile.front().x &&  profile.back().y == profile.front().y) {
		profile.pop_back();
		--count;
	}		

	char errorStr[1024];
	if (!mesh.buildMesh(profile, profile.size(), errorStr))
	{
		ccLog::Warning(QString("[ccPlane::buildUp] Profile triangulation failed (CClib said: '%1'").arg(errorStr));
		return false;
	}

	unsigned numberOfTriangles = mesh.size();
	int* triIndexes = mesh.getTriangleVertIndexesArray();

	if (numberOfTriangles == 0)
		return false;

	//vertices
	unsigned vertCount = 2 * count;
	//faces
	unsigned faceCount = 2 * numberOfTriangles + 2 * count;
	//faces normals
	unsigned faceNormCount = 2 + count;

	if (!init(vertCount, false, faceCount, faceNormCount))
	{
		ccLog::Error("[ccPlane::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	//bottom & top faces normals
	if (0){
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0, 0.0, -1.0).u));
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0, 0.0, 1.0).u));
	} 	

	//add profile vertices & normals
	for (unsigned i = 0; i < count; ++i)
	{
		verts->addPoint(*(m_top->getContourVertices()->getPoint(i)));
		verts->addPoint(*(m_bottom->getContourVertices()->getPoint(i)));

		const CCVector2& P = profile[i];	
		const CCVector2& PNext = profile[(i + 1) % count];
		CCVector2 N(-(PNext.y - P.y), PNext.x - P.x);
		N.normalize();
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(N.x, N.y, 0.0).u));
	}

	//add faces
	{
		//side faces
		if (0) //! represented by facet
		{
			const int* _triIndexes = triIndexes;
			for (unsigned i = 0; i < numberOfTriangles; ++i, _triIndexes += 3)
			{
				addTriangle(_triIndexes[0] * 2, _triIndexes[2] * 2, _triIndexes[1] * 2);
				addTriangleNormalIndexes(0, 0, 0);
				addTriangle(_triIndexes[0] * 2 + 1, _triIndexes[1] * 2 + 1, _triIndexes[2] * 2 + 1);
				addTriangleNormalIndexes(1, 1, 1);
			}
		}

		//thickness
		{
			for (unsigned i = 0; i < count; ++i)
			{
				unsigned iNext = ((i + 1) % count);
				addTriangle(i * 2, i * 2 + 1, iNext * 2);
				addTriangleNormalIndexes(/*2 + */i, /*2 + */i, /*2 + */i);
				addTriangle(iNext * 2, i * 2 + 1, iNext * 2 + 1);
				addTriangleNormalIndexes(/*2 + */i, /*2 + */i, /*2 + */i);
			}
		}
	}

	return true;
}


bool StBlock::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	{
		uint32_t top = (m_top ? static_cast<uint32_t>(m_top->getUniqueID()) : 0);
		if (out.write((const char*)&top, 4) < 0)
			return WriteError();
	}

	{
		uint32_t bottom = (m_bottom ? static_cast<uint32_t>(m_bottom->getUniqueID()) : 0);
		if (out.write((const char*)&bottom, 4) < 0)
			return WriteError();
	}

	return true;
}

bool StBlock::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	{
		uint32_t top = 0;
		if (in.read((char*)&top, 4) < 0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'm_originPoints' pointer!!!
		*(uint32_t*)(&m_top) = top;
	}

	{
		uint32_t bottom = 0;
		if (in.read((char*)&bottom, 4) < 0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'm_originPoints' pointer!!!
		*(uint32_t*)(&m_bottom) = bottom;
	}

	return true;
}
