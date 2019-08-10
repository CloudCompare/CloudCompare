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

#include "vcg/space/intersection3.h"
#include "vcg/space/plane3.h"
#include "vcg/space/line3.h"

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
	, m_top_height(0)
	, m_top_normal(0,0,1)
	, m_bottom_height(0)
	, m_bottom_normal(0,0,1)
	, ccGenericPrimitive(name, nullptr/*&mainPlane->getTransformation()*/)
{
	setTopFacet(top_facet);
	setBottomFacet(bottom_facet);

	paramFromFacet();

	if (!buildFromFacet()) {
//		throw std::runtime_error("internal error");
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

	verts->clear();
	m_triNormals->clear();

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

void StBlock::updateFacet(ccFacet * facet)
{
	//! update normal and height
	if (facet == m_top_facet || facet == m_bottom_facet) {
		paramFromFacet();
	}
	else {
		return;
	}
	
	//! reset facet
	buildUp();
}

void StBlock::setFacetPoints(ccFacet * facet, std::vector<CCVector3> points, bool computePlane)
{
	if (!(facet == m_top_facet) && !(facet == m_bottom_facet)) {
		return;
	}
	facet->FormByContour(points, true, computePlane ? nullptr : facet->getPlaneEquation());

	//! change the mainPlane
	std::vector<CCVector3> plane_points = m_mainPlane->projectTo3DGlobal(points);
	m_mainPlane->setProfile(plane_points, true);

	if (facet == m_top_facet) {
		std::vector<CCVector3> bottom_points;

		std::vector<CCVector3> profiles = m_mainPlane->getProfile();
		CCVector3 plane_normal = m_mainPlane->getNormal();

		vcg::Plane3d bot_plane;
		CCVector3 bot_center = getBottomCenter();
		bot_plane.Init({ bot_center.x,bot_center.y,bot_center.z }, { m_bottom_normal.x,m_bottom_normal.y,m_bottom_normal.z });

		for (auto & pt : profiles) {
			vcg::Line3d line;
			line.SetOrigin({ pt.x, pt.y, pt.z });
			line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

			vcg::Point3d facet_pt;
			if (!vcg::IntersectionLinePlane(line, bot_plane, facet_pt)) {
				return;
			}
			bottom_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
		}
		m_bottom_facet->FormByContour(bottom_points);
	}
	else if (facet == m_bottom_facet) {
		std::vector<CCVector3> top_points;

		std::vector<CCVector3> profiles = m_mainPlane->getProfile();
		CCVector3 plane_normal = m_mainPlane->getNormal();

		vcg::Plane3d top_plane;
		CCVector3 top_center = getTopCenter();
		top_plane.Init({ top_center.x,top_center.y,top_center.z }, { m_top_normal.x,m_top_normal.y,m_top_normal.z });

		for (auto & pt : profiles) {
			vcg::Line3d line;
			line.SetOrigin({ pt.x, pt.y, pt.z });
			line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

			vcg::Point3d facet_pt;
			if (!vcg::IntersectionLinePlane(line, top_plane, facet_pt)) {
				return;
			}
			top_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
		}
		m_top_facet->FormByContour(top_points);
	}

	paramFromFacet();
	buildFromFacet();
}

bool StBlock::buildUp()
{
// 	unsigned count = static_cast<unsigned>(m_top.size());
// 	assert(count >= 3);

	//! firstly, deduce top facet and bottom facet from height and normal
	std::vector<CCVector3> top_points = deduceTopPoints();
	std::vector<CCVector3> bottom_points = deduceBottomPoints();

	if (top_points.size() < 3 || bottom_points.size() < 3) { return false; }

	if (m_top_facet && m_bottom_facet) {
		//! change the facet contours
		m_top_facet->FormByContour(top_points, true);
		m_bottom_facet->FormByContour(bottom_points, true);
		m_bottom_facet->invertNormal();
	}
	else {
		if (m_top_facet) { delete m_top_facet; m_top_facet = nullptr; }
		if (m_bottom_facet) { delete m_bottom_facet; m_bottom_facet = nullptr; }
		ccFacet* top_facet = ccFacet::CreateFromContour(top_points, "top", true);
		ccFacet* bottom_facet = ccFacet::CreateFromContour(bottom_points, "bottom", true);
		if (bottom_facet) {
			bottom_facet->invertNormal();
		}

		setTopFacet(top_facet);
		setBottomFacet(bottom_facet);
	}
 	
	return buildFromFacet();
}

void StBlock::paramFromFacet()
{
	if (!m_top_facet || !m_bottom_facet || !m_mainPlane) {
		return;
	}
	m_top_height = (m_top_facet->getCenter() - m_mainPlane->getProfileCenter()).norm();
	m_top_normal = m_top_facet->getNormal();
	m_bottom_height = (m_bottom_facet->getCenter() - m_mainPlane->getProfileCenter()).norm();
	m_bottom_normal = m_bottom_facet->getNormal();
}

CCVector3 StBlock::getTopCenter()
{
	return m_mainPlane->getProfileCenter() + m_mainPlane->getNormal() * m_top_height;
}

CCVector3 StBlock::getBottomCenter()
{
	return m_mainPlane->getProfileCenter() + m_mainPlane->getNormal() * m_bottom_height;
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

ccFacet * StBlock::getTopFacet()
{
	return m_top_facet;
}

void StBlock::setTopFacet(ccFacet * facet)
{
	m_top_facet = facet;
	if (m_top_facet && !m_top_facet->isAncestorOf(this) && !m_top_facet->getParent()) {
		addChild(m_top_facet);
	//	m_top_facet->addDependency(this, ccHObject::DP_NOTIFY_OTHER_ON_DELETE | ccHObject::DP_NOTIFY_OTHER_ON_UPDATE);
	}
}

ccFacet * StBlock::getBottomFacet()
{
	return m_bottom_facet;
}

void StBlock::setBottomFacet(ccFacet * facet)
{
	m_bottom_facet = facet;
	if (m_bottom_facet && !m_bottom_facet->isAncestorOf(this)) {
		addChild(m_bottom_facet);
	//	m_bottom_facet->addDependency(this, ccHObject::DP_NOTIFY_OTHER_ON_DELETE | ccHObject::DP_NOTIFY_OTHER_ON_UPDATE);
	}
}

void StBlock::setTopHeight(double val)
{
	double add = val - m_top_height;
	m_top_height = val;

	//! get top points
	std::vector<CCVector3> top_points;
	ccPointCloud* verts = vertices();
	if (!verts) { return; }
	for (unsigned int i = 0; i < verts->size() / 2; i++) {
		CCVector3& P = const_cast<CCVector3&>(*verts->getPoint(i * 2));
		P += m_mainPlane->getNormal() * add;
		top_points.push_back(P);
	}
	verts->invalidateBoundingBox();

	ccFacet* top = getTopFacet();
	if (!top) return;
	top->FormByContour(top_points, true);
}

void StBlock::setBottomHeight(double val)
{
	double add = val - m_bottom_height;
	m_bottom_height = val;

	//! get bottom points
	std::vector<CCVector3> bot_points;
	ccPointCloud* verts = vertices();
	if (!verts) { return; }
	for (unsigned int i = 0; i < verts->size() / 2; i++) {
		CCVector3& P = const_cast<CCVector3&>(*verts->getPoint(i * 2 + 1));
		P += m_mainPlane->getNormal() * add;
		bot_points.push_back(P);
	}
	verts->invalidateBoundingBox();

	ccFacet* bottom = getBottomFacet();
	if (!bottom) return;
	bottom->FormByContour(bot_points);
}

std::vector<CCVector3> StBlock::deduceTopPoints()
{
	std::vector<CCVector3> top_points;
	if (!m_mainPlane) {
		return top_points;
	}

	std::vector<CCVector3> profiles = m_mainPlane->getProfile();
	CCVector3 plane_normal = m_mainPlane->getNormal();
	if (profiles.size() < 3) { return top_points; }

	vcg::Plane3d top_plane;
	CCVector3 top_center = getTopCenter();
	top_plane.Init({ top_center.x,top_center.y,top_center.z }, { m_top_normal.x,m_top_normal.y,m_top_normal.z });

	for (auto & pt : profiles) {
		vcg::Line3d line;
		line.SetOrigin({ pt.x, pt.y, pt.z });
		line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

		vcg::Point3d facet_pt;
		if (!vcg::IntersectionLinePlane(line, top_plane, facet_pt)) {
			top_points.clear(); return top_points;
		}
		top_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
	}
	return top_points;
}

std::vector<CCVector3> StBlock::deduceBottomPoints()
{
	std::vector<CCVector3> bottom_points;
	if (!m_mainPlane) {
		return bottom_points;
	}

	std::vector<CCVector3> profiles = m_mainPlane->getProfile();
	CCVector3 plane_normal = m_mainPlane->getNormal();
	if (profiles.size() < 3) { return bottom_points; }

	vcg::Plane3d bot_plane;
	CCVector3 bot_center = getBottomCenter();
	bot_plane.Init({ bot_center.x,bot_center.y,bot_center.z }, { m_bottom_normal.x,m_bottom_normal.y,m_bottom_normal.z });

	for (auto & pt : profiles) {
		vcg::Line3d line;
		line.SetOrigin({ pt.x, pt.y, pt.z });
		line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

		vcg::Point3d facet_pt;
		if (!vcg::IntersectionLinePlane(line, bot_plane, facet_pt)) {
			bottom_points.clear();
			return bottom_points;
		}
		bottom_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
	}
	return bottom_points;
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

void StBlock::getEquation(CCVector3 & N, PointCoordinateType & constVal) const
{
	m_mainPlane->getEquation(N, constVal);
}

//! onUpdateOf(ccFacet)

void StBlock::drawMeOnly(CC_DRAW_CONTEXT & context)
{
	if (isVisible()) {
		ccGenericPrimitive::drawMeOnly(context);
	}
	if (getNormalEditState() && m_mainPlane && MACRO_Draw3D(context)) {
		m_mainPlane->draw(context);
	}
}

bool StBlock::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return WriteError();

	//we can't save the associated facet here
	//so instead we save it's unique ID (dataVersion>=20)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	uint32_t topUniqueID = (m_top_facet ? static_cast<uint32_t>(m_top_facet->getUniqueID()) : 0);
	if (out.write((const char*)&topUniqueID, 4) < 0)
		return WriteError();

	uint32_t botUniqueID = (m_bottom_facet ? static_cast<uint32_t>(m_bottom_facet->getUniqueID()) : 0);
	if (out.write((const char*)&botUniqueID, 4) < 0)
		return WriteError();

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

	//as the associated facet can't be saved directly
	//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_top_facet' pointer!!!
	*(uint32_t*)(&m_top_facet) = vertUniqueID;

	if (in.read((char*)&vertUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_bottom_facet' pointer!!!
	*(uint32_t*)(&m_bottom_facet) = vertUniqueID;

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

	//! plane fake loading, the associate point cloud is not saved
	CC_CLASS_ENUM classID = ReadClassIDFromFile(in, dataVersion);
	if (classID != CC_TYPES::PLANE) {
		return CorruptError();
	}
	ccPlane* plane = new ccPlane;
	if (!plane->fromFile(in, dataVersion, flags)) {
		return ReadError();
	}
	m_mainPlane = new ccPlane(plane->getXWidth(), plane->getYWidth(), &plane->getTransformation(), plane->getName());

	plane->setAssociatedCloud(0);
	plane->setTriNormsTable(0, false);
	plane->setTexCoordinatesTable(0, false);
	delete plane;
	plane = nullptr;

	return true;
}
