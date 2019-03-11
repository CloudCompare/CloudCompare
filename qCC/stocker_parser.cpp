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

#include "stocker_parser.h"

#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccPlane.h"
#include "ccHObjectCaster.h"
#include "ccDBRoot.h"

#include "QFileInfo"

#ifdef USE_STOCKER
using namespace stocker;
#endif // USE_STOCKER

ccHObject* FitPlaneAndAddChild(ccPointCloud* cloud)
{
	ccHObject* cc_plane = nullptr;
	double rms = 0;
	ccPlane* pPlane = ccPlane::Fit(cloud, &rms);
	if (pPlane) {
		cc_plane = static_cast<ccHObject*>(pPlane);
		pPlane->setColor(cloud->getPointColor(0));
		pPlane->enableStippling(true);
	}
	if (cc_plane) {
		cc_plane->setName("Plane");
		cc_plane->applyGLTransformation_recursive();
		cc_plane->showColors(true);
		cc_plane->setVisible(true);

		cloud->addChild(cc_plane);
		cc_plane->setDisplay(cloud->getDisplay());
		cc_plane->prepareDisplayForRefresh_recursive();
	}
	return cc_plane;
}

stocker::Contour3d GetPointsFromCloud(ccHObject* entity) {
	stocker::Contour3d points;
	if (!entity->isEnabled()) {
		return points;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!cloud) return points;

	for (unsigned i = 0; i < cloud->size(); i++) {
		CCVector3 pt = *cloud->getPoint(i);
		points.push_back({ pt.x, pt.y, pt.z });
	}
	return points;
}

stocker::Polyline3d GetPolylineFromEntities(ccHObject::Container entities)
{
	stocker::Polyline3d polyline;
	for (auto & polyline_entity : entities) {
		ccPolyline* ccpolyline = ccHObjectCaster::ToPolyline(polyline_entity);
		unsigned lastvert = ccpolyline->isClosed() ? ccpolyline->size() : ccpolyline->size() - 1;
		for (size_t i = 0; i < lastvert; i++) {
			stocker::Seg3d seg;
			CCVector3 P0 = *(ccpolyline->getPoint(i));
			CCVector3 P1 = *(ccpolyline->getPoint((i + 1) % ccpolyline->size()));
			seg.P0() = stocker::parse_xyz(P0);
			seg.P1() = stocker::parse_xyz(P1);
			polyline.push_back(seg);
		}
	}
	return polyline;
}

ccHObject::Container GetEnabledObjFromGroup(ccHObject* entity, CC_CLASS_ENUM type, bool check_enable)
{
	if (entity) {
		ccHObject::Container group;
		entity->filterChildren(group, true, type, true, entity->getDisplay());
		if (check_enable) {
			ccHObject::Container group_enabled;
			for (auto & gp : group) {
				if ((gp->getParent()) && (!gp->getParent()->isEnabled())) {
					continue;
				}
				if (gp->isEnabled())
					group_enabled.push_back(gp);
			}
			return group_enabled;
		}
		else {
			return group;
		}		
	}
	return ccHObject::Container();
}

ccHObject* AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col)
{
	if (lines.empty()) {
		return nullptr;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);

	ccPointCloud* line_vert = new ccPointCloud(name);
	int i(0);
	for (auto & ln : lines) {
		ccPolyline* cc_polyline = new ccPolyline(line_vert);
		cc_polyline->setDisplay(entity->getDisplay());
		cc_polyline->setColor(col);
		cc_polyline->showColors(true);
		char ln_name[32];
		sprintf(ln_name, "%s%d", name.toStdString().c_str(), i);
		cc_polyline->setName(ln_name);
		cc_polyline->setWidth(1);
		if (cloud) {
			cc_polyline->setGlobalShift(cloud->getGlobalShift());
			cc_polyline->setGlobalScale(cloud->getGlobalScale());
		}
		cc_polyline->reserve(2);

		line_vert->addPoint(CCVector3(vcgXYZ(ln.P0())));
		cc_polyline->addPointIndex(line_vert->size() - 1);

		line_vert->addPoint(CCVector3(vcgXYZ(ln.P1())));
		cc_polyline->addPointIndex(line_vert->size() - 1);

		cc_polyline->setClosed(false);
		line_vert->addChild(cc_polyline);
		i++;
	}

	entity->addChild(line_vert);
	return line_vert;
}

void CalcPlaneIntersections(ccHObject::Container entity_planes, double distance)
{
#ifdef USE_STOCKER
	stocker::PlaneData plane_units;
	for (size_t i = 0; i < entity_planes.size(); i++) {
		if (!entity_planes[i]->isEnabled()) continue;

		stocker::Contour3d cur_plane_points = GetPointsFromCloud(entity_planes[i]->getParent());
		if (cur_plane_points.size() < 3) continue;

		char name[32]; sprintf(name, "%d", i);
		plane_units.push_back(FormPlaneUnit(cur_plane_points, name, true));
	}
	//////////////////////////////////////////////////////////////////////////
	stocker::Polyline3d ints_all; vector<stocker::Polyline3d> ints_per_plane;
	ints_per_plane.resize(plane_units.size());
	for (size_t i = 0; i < plane_units.size() - 1; i++) {
		for (size_t j = i + 1; j < plane_units.size(); j++) {
			stocker::Seg3d cur_ints;
			if (!stocker::IntersectionPlanePlaneStrict(plane_units[i], plane_units[j], cur_ints, distance))
				continue;

			ints_per_plane[i].push_back(cur_ints);
			ints_per_plane[j].push_back(cur_ints);
			ints_all.push_back(cur_ints);
		}
	}
	//////////////////////////////////////////////////////////////////////////
	for (size_t i = 0; i < plane_units.size(); i++) {
		int num;
		sscanf(plane_units[i].GetName().Str().c_str(), "%d", &num);
		AddSegmentsAsChildVertices(entity_planes[num]->getParent(), ints_per_plane[i], "Intersection", ccColor::red);
	}
#endif // USE_STOCKER
}

void CalcPlaneBoundary(ccHObject* planeObj, double p2l_distance, double boundary_minpts, MainWindow* win)
{
#ifdef USE_STOCKER
	/// get boundary points
	Contour2d boundary_points_2d;
	Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
	PlaneUnit plane_unit = FormPlaneUnit(cur_plane_points, "temp", true);
	Contour2d points_2d = Point3dToPlpoint2d(plane_unit, cur_plane_points);
	vector<bool>bd_check;
	ComputeBoundaryPts2d(points_2d, bd_check, 32, true);
	assert(points_2d.size() == bd_check.size());
	for (size_t i = 0; i < bd_check.size(); i++) {
		if (bd_check[i]) {
			boundary_points_2d.push_back(points_2d[i]);
		}
	}

	/// get boundary lines
	Contour3d boundary_points_3d = Plpoint2dToPoint3d(plane_unit, boundary_points_2d);
	Polyline3d ransac_lines; IndexGroup line_index_group;
	LineRansacfromPoints(boundary_points_3d, ransac_lines, line_index_group, p2l_distance, boundary_minpts);

	ccHObject* line_vert = AddSegmentsAsChildVertices(planeObj->getParent(), ransac_lines, "Boundary Lines", ccColor::yellow);

	if (!line_vert) {
		return;
	}
	ccPointCloud* line_cloud = ccHObjectCaster::ToPointCloud(line_vert);
	for (auto & pt : boundary_points_3d) {
		line_cloud->addPoint(CCVector3(vcgXYZ(pt)));
	}
	line_cloud->setRGBColor(ccColor::yellow);
	line_cloud->showColors(true);
	win->addToDB(line_vert);
#endif // USE_STOCKER
}

void CalcPlaneOutlines(ccHObject* planeObj, double alpha)
{
#ifdef USE_STOCKER

	stocker::Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
	if (cur_plane_points.size() < 3) {
		return;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(planeObj->getParent());

	//! get boundary
	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, alpha, false, 2);
	for (vector<stocker::Contour3d> & component : contours_points) {
		for (stocker::Contour3d & st_contours : component) {
			ccPointCloud* line_vert = new ccPointCloud("Vertices");
			ccPolyline* cc_polyline = new ccPolyline(line_vert);
			cc_polyline->setDisplay(planeObj->getDisplay());
			cc_polyline->setColor(ccColor::green);
			cc_polyline->showColors(true);
			cc_polyline->addChild(line_vert);
			cc_polyline->setName("Outline");
			cc_polyline->setWidth(2);
			cc_polyline->setGlobalShift(cloud->getGlobalShift());
			cc_polyline->setGlobalScale(cloud->getGlobalScale());
			cc_polyline->reserve(static_cast<unsigned>(st_contours.size() + 1));
			for (auto & pt : st_contours) {
				line_vert->addPoint(CCVector3(pt.X(), pt.Y(), pt.Z()));
				cc_polyline->addPointIndex(line_vert->size() - 1);
			}
			cc_polyline->setClosed(true);
			cloud->addChild(cc_polyline);
		}
	}
#endif // USE_STOCKER
}
#include "vcg/space/intersection2.h"
void ShrinkPlaneToOutline(ccHObject * planeObj, double alpha, MainWindow* win)
{
#ifdef USE_STOCKER
	ccHObject* parent_cloud = planeObj->getParent();
	if (!parent_cloud) {
		std::cout << "failed to shrink plane" << planeObj->getName().toStdString() << std::endl;
		return;
	}
	stocker::Contour3d cur_plane_points = GetPointsFromCloud(parent_cloud);
	if (cur_plane_points.size() < 3) {
		parent_cloud->setEnabled(false);
		return;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(parent_cloud);
	vcg::Plane3d plane;
	PlaneUnit plane_unit = FormPlaneUnit(cur_plane_points, "temp", true);
 	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, alpha * 3, false, 2);
 	Contour3d concave_contour = contours_points.front().front();
	Contour2d concave_2d = Point3dToPlpoint2d(plane_unit, concave_contour);
	Polyline2d concave_polygon = MakeLoopPolylinefromContour2d(concave_2d);
	
	CCLib::ReferenceCloud remained(cloud);
	int size = cloud->size();
	for (size_t i = 0; i < size; i++) {
		CCVector3 point = *cloud->getPoint(i);
		vcg::Point2d pt_2d = plane_unit.Point3dPrjtoPlpoint2d({ parse_xyz(point) });
		if (vcg::PointInsidePolygon(pt_2d, concave_polygon)) {
			remained.addPointIndex(i);			
		}
	}
	ccPointCloud* newCloud = cloud->partialClone(&remained);
	newCloud->setName(cloud->getName());
	cloud->setName(cloud->getName() + "-delete");
	parent_cloud->setEnabled(false);
	ccHObject* parent = parent_cloud->getParent();
	parent->addChild(newCloud);
	
	FitPlaneAndAddChild(newCloud);	

	int index_old = parent->getChildIndex(cloud);
	int index_new = parent->getChildIndex(newCloud);
	parent->swapChildren(index_old, index_new);
	win->addToDB(newCloud);

	win->removeFromDB(cloud);
//	win->db()->removeElement(cloud);
	
#endif // USE_STOCKER
}

void PlaneFrameOptimization(ccHObject* planeObj)
{
#ifdef USE_STOCKER
	ccPlane* ccPlane = ccHObjectCaster::ToPlane(planeObj);
	if (!ccPlane) return;

	CCVector3 N; float constVal;
	ccPlane->getEquation(N, constVal);

	vcg::Plane3d vcgPlane;
	vcgPlane.SetDirection({ N.x, N.y, N.z });
	vcgPlane.SetOffset(constVal);


#endif
}


ccHObject::Container BDBaseHObject::GetHObjContainer(CC_CLASS_ENUM type, QString suffix, bool check_enable)
{
	ccHObject::Container entities = GetEnabledObjFromGroup(this, type, check_enable);
	ccHObject::Container output;
	for (auto & entity : entities) {
		if (entity->getName().endsWith(suffix)) {
			output.push_back(entity);
		}
	}
	return output;
}
ccHObject * BDBaseHObject::GetHObj(CC_CLASS_ENUM type, QString suffix, QString basename, bool check_enable)
{
	ccHObject::Container entities = GetEnabledObjFromGroup(this, type, check_enable);
	ccHObject* output = nullptr;
	for (auto & entity : entities) {
		QFileInfo name(entity->getName());	
		if (entity->getName().endsWith(suffix) &&
			name.baseName() == basename) {
			return entity;
		}
	}
	return nullptr;
}
ccHObject* BDBaseHObject::GetBuildingGroup(QString building_name, bool check_enable) {
	for (size_t i = 0; i < getChildrenNumber(); i++) 
		if (getChild(i)->getName() == building_name) 
			return getChild(i);
	return nullptr;
}
ccHObject::Container BDBaseHObject::GetOriginPointCloud(bool check_enable) {
	return GetHObjContainer(CC_TYPES::POINT_CLOUD, BDDB_ORIGIN_CLOUD_SUFFIX);
}
ccHObject * BDBaseHObject::GetOriginPointCloud(QString building_name, bool check_enable) {
	return GetHObj(CC_TYPES::POINT_CLOUD, BDDB_ORIGIN_CLOUD_SUFFIX, building_name);
}
ccHObject * BDBaseHObject::GetPrimitiveGroup(QString building_name, bool check_enable) {
	return GetHObj(CC_TYPES::HIERARCHY_OBJECT, BDDB_PRIMITIVE_SUFFIX, building_name);
}
BDBaseHObject* GetRootBDBase(ccHObject* obj) {
	do {
		BDBaseHObject* bd_obj = static_cast<BDBaseHObject*>(obj->getParent());
		if (bd_obj->valid) {
			return bd_obj;
		}
	} while (obj->getParent());

	return nullptr;
}