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
		m_top.push_back(CCVector3(pt.x, pt.y, top_height));
		m_bottom.push_back(CCVector3(pt.x, pt.y, bottom_height));
	}
	StBlock(m_top, m_bottom, transMat, name);
}

StBlock::StBlock(const std::vector<CCVector3>& top, 
	const std::vector<CCVector3>& bottom,
	const ccGLMatrix * transMat,
	QString name)
	: m_top(top)
	, m_bottom(bottom)
	, ccGenericPrimitive(name, transMat)
{
	assert(top.size() > 2);
	assert(top.size() == bottom.size());
	
	if (!updateRepresentation()) {
		throw std::runtime_error("internal error");
	}
}

StBlock::StBlock(QString name/*="Block"*/)
	: ccGenericPrimitive(name)
{
}

ccGenericPrimitive* StBlock::clone() const
{
	return finishCloneJob(new StBlock(m_top, m_bottom, &m_transformation, getName()));
}

CCVector3 StBlock::getCenterTop()
{
	CCVector3 center(0, 0, 0);
	for (auto & pt : m_top)	{
		center += pt;
	}
	center /= m_top.size();
	return center;
}

CCVector3 StBlock::getCenterBottom()
{
	CCVector3 center(0, 0, 0);
	for (auto & pt : m_bottom) {
		center += pt;
	}
	center /= m_bottom.size();
	return center;
}

std::vector<CCVector2> StBlock::getProfile()
{
	std::vector<CCVector2> profile;
	for (auto & pt : m_top) {
		profile.push_back(CCVector2(pt.x, pt.y));
	}
	return profile;
}

ccFacet * StBlock::getTopFacet()
{
	for (unsigned int i = 0; i < getChildrenNumber(); i++) {
		if (getChild(i)->getName()=="top" && getChild(i)->isA(CC_TYPES::FACET)) {
			return static_cast<ccFacet*>(getChild(i));
		}
	}
	return nullptr;
}

ccFacet * StBlock::getBottomFacet()
{
	for (unsigned int i = 0; i < getChildrenNumber(); i++) {
		if (getChild(i)->getName() == "bottom" && getChild(i)->isA(CC_TYPES::FACET)) {
			return static_cast<ccFacet*>(getChild(i));
		}
	}
	return nullptr;
}

void StBlock::setTopHeight(double val)
{
	double center_height = 0;
	for (size_t i = 0; i < m_top.size(); i++) {
		center_height += m_top.at(i).z;		
	}
	center_height /= m_top.size();
	double add = val - center_height;

	for (size_t i = 0; i < m_top.size(); i++) {
		m_top.at(i).z += add;
	}

	ccPointCloud* verts = vertices();
	if (!verts) { return; }
	for (unsigned int i = 0; i < verts->size() / 2; i++) {
		CCVector3& P = const_cast<CCVector3&>(*verts->getPoint(i * 2));
		P.z += add;
	}
	verts->invalidateBoundingBox();

	ccFacet* top = getTopFacet();
	if (!top) return;
	ccPointCloud* cloud = top->getContourVertices();
	if (!cloud) return;
	for (unsigned int i = 0; i < cloud->size(); i++) {
		CCVector3& P = const_cast<CCVector3&>(*cloud->getPoint(i));
		P.z += add;
	}
	cloud->invalidateBoundingBox();
}

void StBlock::setBottomHeight(double val)
{
	double center_height = 0;
	for (size_t i = 0; i < m_bottom.size(); i++) {
		center_height += m_bottom.at(i).z;
	}
	center_height /= m_bottom.size();
	double add = val - center_height;

	for (size_t i = 0; i < m_bottom.size(); i++) {
		m_bottom.at(i).z += add;
	}

	ccPointCloud* verts = vertices();
	if (!verts) { return; }
	for (unsigned int i = 0; i < verts->size() / 2; i++) {
		CCVector3& P = const_cast<CCVector3&>(*verts->getPoint(i * 2 + 1));
		P.z += add;
	}
	verts->invalidateBoundingBox();

	ccFacet* bottom = getBottomFacet();
	if (!bottom) return;
	ccPointCloud* cloud = bottom->getContourVertices();
	if (!cloud) return;
	for (unsigned int i = 0; i < cloud->size(); i++) {
		CCVector3& P = const_cast<CCVector3&>(*cloud->getPoint(i));
		P.z += add;
	}
	cloud->invalidateBoundingBox();
}
#include <iostream>
bool StBlock::buildUp()
{
// 	unsigned count = static_cast<unsigned>(m_top.size());
// 	assert(count >= 3);
 	if (m_top.size() < 3) { return false; }
	ccFacet* top_facet = ccFacet::CreateFromContour(m_top, "top", true);
	if (!top_facet) { return false; }
	ccFacet* bottom_facet = ccFacet::CreateFromContour(m_bottom, "bottom", true);
	if (!bottom_facet) { return false; }
	bottom_facet->invertNormal();
	
	addChild(top_facet);
	addChild(bottom_facet);

	std::vector<CCVector2> profile = getProfile();
	ccMesh* mesh = top_facet->getPolygon();	
	
	if (!mesh) {
		ccLog::Warning(QString("[ccPlane::buildUp] Profile triangulation failed"));
		return false;
	}

	unsigned count = mesh->getAssociatedCloud()->size();
	unsigned numberOfTriangles = mesh->size();
//	int* triIndexes = mesh->getTriangleVertIndexesArray();
	//determine if the triangles must be flipped or not
	bool flip = false;
// 	{
// 		for (unsigned i = 0; i < numberOfTriangles; ++i, triIndexes += 3) {
// 			int i1 = triIndexes[0];
// 			int i2 = triIndexes[1];
// 			int i3 = triIndexes[2];
// 			//by definition the first edge of the original polygon
// 			//should be in the same 'direction' of the triangle that uses it
// 			if ((i1 == 0 || i2 == 0 || i3 == 0)
// 				&& (i1 == 1 || i2 == 1 || i3 == 1))	{
// 				if ((i1 == 1 && i2 == 0)
// 					|| (i2 == 1 && i3 == 0)
// 					|| (i3 == 1 && i1 == 0)) {
// 					flip = true;
// 				}
// 				break;
// 			}
// 		}
// 	}

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

	// top & bottom faces normals
	{
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(top_facet->getNormal().u));
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(bottom_facet->getNormal().u));
	} 	

	//add profile vertices & normals
	for (unsigned i = 0; i < count; ++i)
	{
		verts->addPoint(m_top.at(i));
		verts->addPoint(m_bottom.at(i));

		const CCVector2& P = profile[i];	
		const CCVector2& PNext = profile[(i + 1) % count];
		CCVector2 N(PNext.y - P.y, -(PNext.x - P.x));
		N.normalize();
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(N.x, N.y, 0.0).u));
	}

	//add faces
	{
		//side faces
		{
			int first_top = 1, second_top = 2;
			if (ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(0)).z < 0)
				std::swap(first_top, second_top);
			if (flip)
				std::swap(first_top, second_top);

			int first_bot = 1, second_bot = 2;
			if (ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(1)).z < 0)
				std::swap(first_bot, second_bot);
			if (flip)
				std::swap(first_bot, second_bot);

			//std::cout << "top: " << first_top << " " << second_top << std::endl;
			//std::cout << "top: " << first_bot << " " << second_bot << std::endl;

			for (unsigned ti = 0; ti < numberOfTriangles; ++ti) {
				unsigned int* _triIndexes = mesh->getTriangleVertIndexes(ti)->i;
				
				addTriangle(_triIndexes[0] * 2, _triIndexes[first_top] * 2, _triIndexes[second_top] * 2);
				addTriangleNormalIndexes(0, 0, 0);

				addTriangle(_triIndexes[0] * 2 + 1, _triIndexes[first_bot] * 2 + 1, _triIndexes[second_bot] * 2 + 1);
				addTriangleNormalIndexes(1, 1, 1);
			}
		}

		//thickness
		{
			for (unsigned i = 0; i < count; ++i)
			{
				unsigned iNext = ((i + 1) % count);
				addTriangle(i * 2, i * 2 + 1, iNext * 2);
				addTriangleNormalIndexes(2 + i, 2 + i, 2 + i);
				addTriangle(iNext * 2, i * 2 + 1, iNext * 2 + 1);
				addTriangleNormalIndexes(2 + i, 2 + i, 2 + i);
			}
		}
	}
	setVisible(true);
	enableStippling(false);	
	showNormals(true);


	return true;
}

bool StBlock::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	//m_top size
	outStream << (qint32)m_top.size();
	//m_top points (3D)
	for (unsigned i = 0; i < m_top.size(); ++i)
	{
		outStream << m_top[i].x;
		outStream << m_top[i].y;
		outStream << m_top[i].z;
	}

	//m_bottom size
	outStream << (qint32)m_bottom.size();
	//m_bottom points (3D)
	for (unsigned i = 0; i < m_bottom.size(); ++i)
	{
		outStream << m_bottom[i].x;
		outStream << m_bottom[i].y;
		outStream << m_bottom[i].z;
	}

	return true;
}

bool StBlock::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	//m_top size
	qint32 vertCount;
	inStream >> vertCount;
	if (vertCount > 0) {
		m_top.resize(vertCount);
		//m_top points (2D)
		for (unsigned i = 0; i < m_top.size(); ++i)	{
			ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_top[i].u, 3);
		}
	}		

	//m_bottom size
	inStream >> vertCount;
	if (vertCount > 0) {
		m_bottom.resize(vertCount);
		//m_bottom points (2D)
		for (unsigned i = 0; i < m_bottom.size(); ++i) {
			ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_bottom[i].u, 3);
		}
	}	

	return true;
}
