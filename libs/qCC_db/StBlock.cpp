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
#include "ccPlane.h"

//CCLib
#include <Delaunay2dMesh.h>

//system
#include <string.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////

StBlock::StBlock(ccPlane* mainPlane,
	PointCoordinateType top_height,	CCVector3 top_normal, 
	PointCoordinateType bottom_height, CCVector3 bottom_normal,
	QString name)
	: m_mainPlane(mainPlane)
	, m_top_height(top_height)
	, m_top_normal(top_normal)
	, m_bottom_height(bottom_height)
	, m_bottom_normal(bottom_normal)
	, ccGenericPrimitive(name, nullptr/*&mainPlane->getTransformation()*/)
{
	if (!updateRepresentation()) {
		throw std::runtime_error("internal error");
	}
}

StBlock::StBlock(ccPlane* mainPlane,
	ccFacet* top_facet, ccFacet* bottom_facet,
	QString name)
	: m_mainPlane(mainPlane)
	, m_top_facet(top_facet)
	, m_bottom_facet(bottom_facet)
	, ccGenericPrimitive(name, nullptr/*&mainPlane->getTransformation()*/)
{
	m_top_height = (top_facet->getCenter() - mainPlane->getCenter()).norm();
	m_top_normal = top_facet->getNormal();
	m_bottom_height = (bottom_facet->getCenter() - mainPlane->getCenter()).norm();
	m_bottom_normal = bottom_facet->getNormal();

	if (!buildFromFacet()) {
		throw std::runtime_error("internal error");
	}
}

StBlock* StBlock::Create(const std::vector<CCVector3>& top,
	const PointCoordinateType bottom_height, QString name)
{
	std::vector<CCVector3> profile_points;
	for (auto & pt : top) {
		profile_points.push_back(CCVector3(pt.x, pt.y, bottom_height));
	}
	ccPlane* mainPlane = ccPlane::Fit(profile_points);
	ccFacet* top_facet = ccFacet::CreateFromContour(top, "top", true);
	ccFacet* bottom_facet = ccFacet::CreateFromContour(profile_points, "bottom", true);
	bottom_facet->invertNormal();

	//! 
	StBlock* block = new StBlock(mainPlane, top_facet, bottom_facet, name);
	return block;
}

bool StBlock::buildFromFacet()
{
	if (!m_top_facet || !m_bottom_facet || !m_mainPlane) {
		return false;
	}

	std::vector<CCVector3> profile = m_mainPlane->getProfile();
	ccMesh* mesh = m_top_facet->getPolygon();

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
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(m_top_facet->getNormal().u));
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(m_bottom_facet->getNormal().u));
	}

	ccPointCloud* top_points = m_top_facet->getContourVertices();
	ccPointCloud* bottom_points = m_bottom_facet->getContourVertices();
	
	//add profile vertices & normals
	for (unsigned i = 0; i < count; ++i)
	{
		verts->addPoint(*top_points->getPoint(i));
		verts->addPoint(*bottom_points->getPoint(i));

		const CCVector2& P = CCVector2(profile[i].x, profile[i].y);	// TODO: Project to plane
		const CCVector2& PNext = CCVector2(profile[(i + 1) % count].x, profile[(i + 1) % count].y);
		CCVector2 N(PNext.y - P.y, -(PNext.x - P.x));
		N.normalize();
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(N.x, N.y, 0.0).u));	// TODO: SHOULD CONSIDER MAINPLANE
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

bool StBlock::buildUp()
{
// 	unsigned count = static_cast<unsigned>(m_top.size());
// 	assert(count >= 3);

	//! firstly, deduce top facet and bottom facet from height and normal
	std::vector<CCVector3> top_points;
	std::vector<CCVector3> bottom_points;

	if (m_top_facet && m_bottom_facet) {
		//! change the facet contours

	}
	else {
		if (m_top_facet) { delete m_top_facet; m_top_facet = nullptr; }
		if (m_bottom_facet) { delete m_bottom_facet; m_bottom_facet = nullptr; }
		m_top_facet = ccFacet::CreateFromContour(top_points, "top", true);
		m_bottom_facet = ccFacet::CreateFromContour(bottom_points, "bottom", true);
		m_bottom_facet->invertNormal();

		addChild(m_top_facet);
		addChild(m_bottom_facet);
	}
 	
	return buildFromFacet();
}

CCVector3 StBlock::getTopCenter()
{
	return m_mainPlane->getCenter() + m_mainPlane->getNormal() * m_top_height;
}

CCVector3 StBlock::getBottomCenter()
{
	return m_mainPlane->getCenter() + m_mainPlane->getNormal() * m_bottom_height;
}

StBlock::StBlock(QString name/*="Block"*/)
	: ccGenericPrimitive(name)
	, m_mainPlane(nullptr)
	, m_top_facet(nullptr)
	, m_bottom_facet(nullptr)
{
}

ccGenericPrimitive* StBlock::clone() const
{
	return finishCloneJob(new StBlock(m_mainPlane, m_top_facet, m_bottom_facet, getName()));
}

// CCVector3 StBlock::getCenterTop()
// {
// 	CCVector3 center(0, 0, 0);
// 	for (auto & pt : m_top)	{
// 		center += pt;
// 	}
// 	center /= m_top.size();
// 	return center;
// }
// 
// CCVector3 StBlock::getCenterBottom()
// {
// 	CCVector3 center(0, 0, 0);
// 	for (auto & pt : m_bottom) {
// 		center += pt;
// 	}
// 	center /= m_bottom.size();
// 	return center;
// }

// std::vector<CCVector2> StBlock::getProfile()
// {
// 	std::vector<CCVector2> profile;
// 	for (auto & pt : m_top) {
// 		profile.push_back(CCVector2(pt.x, pt.y));
// 	}
// 	return profile;
// }

ccFacet * StBlock::getTopFacet()
{
	return m_top_facet;
}

ccFacet * StBlock::getBottomFacet()
{
	return m_bottom_facet;
}

void StBlock::setTopHeight(double val)
{
	double add = val - m_top_height;

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

	m_top_height = val;
}

void StBlock::setBottomHeight(double val)
{
	double add = val - m_bottom_height;
	
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

	m_bottom_height = val;
}

inline CCVector3 StBlock::getNormal() const
{
	return m_mainPlane->getNormal();
}

CCVector3 StBlock::getCenter() const
{
	return m_mainPlane->getCenter();
}

void StBlock::notifyPlanarEntityChanged(ccGLMatrix mat, bool trans)
{
	//! main plane
	m_mainPlane->applyGLTransformation_recursive(&mat);

	//! for mesh
	applyGLTransformation_recursive(&mat);

	//! for m_top_normal
	m_top_normal = getTopFacet()->getNormal();

	//! for m_bottom_normal
	m_bottom_normal = getBottomFacet()->getNormal();
}

//! onUpdateOf(ccFacet)

bool StBlock::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return WriteError();

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);

	//! top
	if (out.write((const char*)m_top_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return WriteError();
	if (out.write((const char*)&m_top_height, sizeof(double)) < 0)
		return WriteError();

	//! bottom
	if (out.write((const char*)m_bottom_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return WriteError();
	if (out.write((const char*)&m_bottom_height, sizeof(double)) < 0)
		return WriteError();

	//! plane
	if (!m_mainPlane->toFile(out)) {
		return WriteError();
	}

	return true;
}

bool StBlock::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags))
		return ReadError();

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);

	//! top
	if (in.read((char*)m_top_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return ReadError();
	if (in.read((char*)&m_top_height, sizeof(double)) < 0)
		return ReadError();

	//! bottom
	if (in.read((char*)m_bottom_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return ReadError();
	if (in.read((char*)&m_bottom_height, sizeof(double)) < 0)
		return ReadError();

	//! plane
	if (!m_mainPlane) {
		m_mainPlane = new ccPlane;
	}

	CC_CLASS_ENUM classID = ReadClassIDFromFile(in, dataVersion);
	if (classID != CC_TYPES::PLANE) {
		return CorruptError();
	}
	if (!m_mainPlane->fromFile(in, dataVersion, flags)) {
		return ReadError();
	}

	return true;
}
