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

QString GetBaseName(QString name) {	
	return name.mid(0, name.indexOf('.'));
}

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
		cc_plane->showNormals(cloud->hasNormals());

		cloud->addChild(cc_plane);
		cc_plane->setDisplay(cloud->getDisplay());
		cc_plane->prepareDisplayForRefresh_recursive();
	}
	return cc_plane;
}

stocker::Contour3d GetPointsFromCloud(ccHObject* entity) {
	stocker::Contour3d points;
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

vector<vector<stocker::Contour3d>> GetOutlinesFromOutlineParent(ccHObject* entity)
{
	ccHObject::Container container_find;
	entity->filterChildren(container_find, false, CC_TYPES::POLY_LINE, true);
	QString name = QString();
	vector<vector<stocker::Contour3d>> contours_points;
	for (auto & component : container_find) {
		if (component->getName() != name) {
			contours_points.push_back(vector<stocker::Contour3d>());
			name = component->getName();
		}
		ccPolyline* poly_line = ccHObjectCaster::ToPolyline(component);
		vector<CCVector3> outline_points = poly_line->getPoints(true);
		Contour3d outline_points_;
		for (auto & pt : outline_points) {
			outline_points_.push_back(parse_xyz(pt));
		}
		contours_points.back().push_back(outline_points_);
	}
	return contours_points;
}

ccHObject::Container GetEnabledObjFromGroup(ccHObject* entity, CC_CLASS_ENUM type, bool check_enable, bool recursive)
{
	if (entity) {
		ccHObject::Container group;
		entity->filterChildren(group, recursive, type, true, entity->getDisplay());
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

vcg::Plane3d GetVcgPlane(ccHObject* planeObj)
{
	ccPlane* ccPlane = ccHObjectCaster::ToPlane(planeObj);
	CCVector3 N; float constVal;
	ccPlane->getEquation(N, constVal);
	vcg::Plane3d vcgPlane;
	vcgPlane.SetDirection({ N.x, N.y, N.z });
	vcgPlane.SetOffset(constVal);
	return vcgPlane;
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
		cc_polyline->setName(name + QString::number(i));
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

ccHObject* AddPlanesPointsAsNewGroup(QString name, std::vector<stocker::Contour3d> planes_points)
{
	ccHObject* group = new ccHObject(name);

	for (size_t i = 0; i < planes_points.size(); i++) {
		ccPointCloud* plane_cloud = new ccPointCloud(BDDB_PLANESEG_PREFIX + QString::number(i));// TODO

		//! get plane points
		for (auto & pt : planes_points[i]) {
			plane_cloud->addPoint(CCVector3(vcgXYZ(pt)));
		}
		ccColor::Rgb col = ccColor::Generator::Random();
		plane_cloud->setRGBColor(col);
		plane_cloud->showColors(true);

		//! add plane
		FitPlaneAndAddChild(plane_cloud);

		group->addChild(plane_cloud);
	}
	return group;
}

ccHObject* PlaneSegmentationRgGrow(ccHObject* entity,
	int min_pts, double distance_epsilon, double seed_raius,
	double growing_radius,
	double merge_threshold, double split_threshold)
{
	ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!entity_cloud->hasNormals()) {
		return nullptr;
	}

	std::vector<vcg::Plane3d> planes;
	std::vector<stocker::Contour3d> planes_points;

	stocker::BuilderLOD2 builder_3d4em(true);
	std::vector<stocker::BdPoint3d> point_cloud;
	for (unsigned i = 0; i < entity_cloud->size(); i++) {
		CCVector3 pt = *entity_cloud->getPoint(i);
		point_cloud.push_back(stocker::BdPoint3d(pt.x, pt.y, pt.z));
	}
	builder_3d4em.SetBuildingPoints(point_cloud);
	builder_3d4em.SetPlaneSegOption(min_pts, distance_epsilon, seed_raius, growing_radius);
	builder_3d4em.PlaneSegmentation();
	std::vector<std::vector<stocker::BdPoint3d>> pp_3d4em = builder_3d4em.GetSegmentedPoints();

	for (auto & pl : pp_3d4em) {
		stocker::Contour3d pl_pts;
		for (auto & pt : pl) {
			pl_pts.push_back(parse_xyz(pt));			
		}
		vcg::Plane3d plane;
		stocker::Vec3d cen;
		stocker::FitPlane(pl_pts, plane, cen);
		planes.push_back(plane);
		planes_points.push_back(pl_pts);
	}

	if (merge_threshold > 0 || split_threshold > 0) {
		stocker::Option_PlaneRefinement option_refine;
		option_refine.merge_threshold = merge_threshold;
		option_refine.split_threshold = split_threshold;
		if (!stocker::PlaneRefinement(planes, planes_points, option_refine)) {
			return nullptr;
		}
	}

	ccHObject* group = AddPlanesPointsAsNewGroup(entity->getName() + BDDB_PRIMITIVE_SUFFIX, planes_points);
	entity->getParent()->addChild(group);
	return group;
}

ccHObject* PlaneSegmentationRansac(ccHObject* entity,
	int min_pts, double distance_epsilon, double seed_raius,
	double normal_threshold, double ransac_probability,
	double merge_threshold, double split_threshold)
{
	ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!entity_cloud->hasNormals()) {
		return nullptr;
	}

	stocker::GLMesh mesh;
	for (unsigned i = 0; i < entity_cloud->size(); i++) {
		CCVector3 pt = *entity_cloud->getPoint(i);
		CCVector3 normal = entity_cloud->getPointNormal(i);
		stocker::GLMeshAL::AddVertex(mesh, parse_xyz(pt), parse_xyz(normal));
	}

	std::vector<vcg::Plane3d> planes;
	std::vector<stocker::Contour3d> planes_points;
	std::vector<Point_Normal> unassigned_points;
	
	stocker::Option_PlaneSegmentation option;
	option.min_points = min_pts;
	option.distance_epsilon = distance_epsilon;
	option.cluster_epsilon = seed_raius;
	option.normal_threshold = normal_threshold;
	option.ransac_probability = ransac_probability;
	if (!stocker::PlaneSegmentation(mesh, planes, planes_points, unassigned_points, option)) {
		return nullptr;
	}
	
	if (merge_threshold > 0 || split_threshold > 0) {
		stocker::Option_PlaneRefinement option_refine;
		option_refine.merge_threshold = merge_threshold;
		option_refine.split_threshold = split_threshold;
		if (!stocker::PlaneRefinement(planes, planes_points, option_refine)) {
			return nullptr;
		}
	}

	ccHObject* group = AddPlanesPointsAsNewGroup(entity->getName() + BDDB_PRIMITIVE_SUFFIX, planes_points);
	entity->getParent()->addChild(group);
	return group;	
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

ccHObject* CalcPlaneBoundary(ccHObject* planeObj, double p2l_distance, double boundary_minpts)
{
#ifdef USE_STOCKER
	/// get boundary points
	Contour2d boundary_points_2d;
	Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
	PlaneUnit plane_unit = FormPlaneUnit(cur_plane_points, "temp", true);
	Contour2d points_2d = Point3dToPlpoint2d(plane_unit, cur_plane_points);
	vector<bool>bd_check;
	stocker::ComputeBoundaryPts2d(points_2d, bd_check, 32, true);
	assert(points_2d.size() == bd_check.size());
	for (size_t i = 0; i < bd_check.size(); i++) {
		if (bd_check[i]) {
			boundary_points_2d.push_back(points_2d[i]);
		}
	}

	/// get boundary lines
	Polyline3d detected_lines;
	stocker::LineFromPlanePoints(cur_plane_points, detected_lines);

 	Contour3d boundary_points_3d = Plpoint2dToPoint3d(plane_unit, boundary_points_2d);
// 	IndexGroup line_index_group;
// 	LineRansacfromPoints(boundary_points_3d, detected_lines, line_index_group, p2l_distance, boundary_minpts);

	ccHObject* line_vert = AddSegmentsAsChildVertices(planeObj->getParent(), detected_lines, BDDB_BOUNDARY_PREFIX, ccColor::yellow);

	if (!line_vert) {
		return nullptr;
	}
	ccPointCloud* line_cloud = ccHObjectCaster::ToPointCloud(line_vert);
	for (auto & pt : boundary_points_3d) {
		line_cloud->addPoint(CCVector3(vcgXYZ(pt)));
	}
	line_cloud->setRGBColor(ccColor::yellow);
	line_cloud->showColors(true);

	return line_vert;
#endif // USE_STOCKER
}

ccHObject* AddOutlinesAsChild(vector<vector<stocker::Contour3d>> contours_points, ccHObject* parent)
{
	if (contours_points.empty()) return nullptr;
	ccPointCloud* line_vert = new ccPointCloud(BDDB_OUTLINE_PREFIX);
	int component_number = 0;
	for (vector<stocker::Contour3d> & component : contours_points) {
		for (stocker::Contour3d & st_contours : component) {
			ccPolyline* cc_polyline = new ccPolyline(line_vert);
// 			cc_polyline->setDisplay(planeObj->getDisplay());
			cc_polyline->setColor(ccColor::green);
			cc_polyline->showColors(true);
			line_vert->addChild(cc_polyline);
			cc_polyline->setName(BDDB_OUTLINE_PREFIX + QString::number(component_number));
			cc_polyline->setWidth(2);
// 			cc_polyline->setGlobalShift(cloud->getGlobalShift());
// 			cc_polyline->setGlobalScale(cloud->getGlobalScale());
			cc_polyline->reserve(static_cast<unsigned>(st_contours.size() + 1));
			for (auto & pt : st_contours) {
				line_vert->addPoint(CCVector3(pt.X(), pt.Y(), pt.Z()));
				cc_polyline->addPointIndex(line_vert->size() - 1);
			}
			cc_polyline->setClosed(true);
		}
		component_number++;
	}
	parent->addChild(line_vert);
	return line_vert;
}

ccHObject* CalcPlaneOutlines(ccHObject* planeObj, double alpha)
{
#ifdef USE_STOCKER
	stocker::Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
	if (cur_plane_points.size() < 3) {
		return nullptr;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(planeObj->getParent());

	//! get boundary
	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, alpha, false, 2);
	return AddOutlinesAsChild(contours_points, planeObj->getParent());
//	if (contours_points.empty()) return nullptr;
// 	ccPointCloud* line_vert = new ccPointCloud(BDDB_OUTLINE_PREFIX);
// 	int component_number = 0;
// 	for (vector<stocker::Contour3d> & component : contours_points) {
// 		for (stocker::Contour3d & st_contours : component) {
// 			ccPolyline* cc_polyline = new ccPolyline(line_vert);
// 			cc_polyline->setDisplay(planeObj->getDisplay());
// 			cc_polyline->setColor(ccColor::green);
// 			cc_polyline->showColors(true);
// 			line_vert->addChild(cc_polyline);
// 			cc_polyline->setName(BDDB_OUTLINE_PREFIX + QString::number(component_number));
// 			cc_polyline->setWidth(2);
// 			cc_polyline->setGlobalShift(cloud->getGlobalShift());
// 			cc_polyline->setGlobalScale(cloud->getGlobalScale());
// 			cc_polyline->reserve(static_cast<unsigned>(st_contours.size() + 1));
// 			for (auto & pt : st_contours) {
// 				line_vert->addPoint(CCVector3(pt.X(), pt.Y(), pt.Z()));
// 				cc_polyline->addPointIndex(line_vert->size() - 1);
// 			}
// 			cc_polyline->setClosed(true);
// 		}
// 		component_number++;
// 	}
// 	planeObj->getParent()->addChild(line_vert);
// 	return line_vert;
#endif // USE_STOCKER
}
#include "vcg/space/intersection2.h"
void ShrinkPlaneToOutline(ccHObject * planeObj, double alpha, double distance_epsilon, MainWindow* win)
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

	vcg::Plane3d vcgPlane = GetVcgPlane(planeObj);
	PlaneUnit plane_unit = FormPlaneUnit("temp", vcgPlane, cur_plane_points, true);
 	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, alpha * 3, false, 2);
 	Contour3d concave_contour = contours_points.front().front();
	Contour2d concave_2d = Point3dToPlpoint2d(plane_unit, concave_contour);
	Polyline2d concave_polygon = MakeLoopPolylinefromContour2d(concave_2d);
		
	vector<size_t> inside_index;
	stocker::Contour3d inside_points;
	for (unsigned int i = 0; i < cloud->size(); i++) {
		CCVector3 point = *cloud->getPoint(i);
		vcg::Point2d pt_2d = plane_unit.Point3dPrjtoPlpoint2d({ parse_xyz(point) });
		if (vcg::PointInsidePolygon(pt_2d, concave_polygon)) {
			inside_index.push_back(i);
			inside_points.push_back(parse_xyz(point));					
		}
	}
	PlaneUnit plane_unit_inside = FormPlaneUnit(inside_points, "temp", true);
	CCLib::ReferenceCloud remained(cloud);
	for (size_t i = 0; i < inside_index.size(); i++) {
		if (plane_unit_inside.IsInPlane(parse_xyz(*cloud->getPoint(i)), distance_epsilon)) {
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

	vector<vector<stocker::Contour3d>> contours_points_remained;
	contours_points_remained.push_back(contours_points.front());
	ccHObject* outlines_add = AddOutlinesAsChild(contours_points_remained, newCloud);
	win->addToDB(outlines_add);
//	win->db()->removeElement(cloud);
	
#endif // USE_STOCKER
}

ccHObject*  PlaneFrameOptimization(ccHObject* planeObj)
{
#ifdef USE_STOCKER
	//////////////////////////////////////////////////////////////////////////
	// frame optimization
	stocker::FrameOptmzt frame_opt(planeObj->getParent()->getName().toStdString());
	vcg::Plane3d vcgPlane = GetVcgPlane(planeObj);

	// prepare plane points
	Contour3d plane_points = GetPointsFromCloud(planeObj->getParent());

	// prepare boundary lines
	Polyline3d boundary_lines; {
		ccHObject::Container container_find, container_objs;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_BOUNDARY_PREFIX, true);
		container_find.back()->filterChildren(container_objs, false, CC_TYPES::POLY_LINE, true);
		boundary_lines = GetPolylineFromEntities(container_objs);
	}

	// prepare outline
	Polyline3d outline; {
		ccHObject::Container container_find;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_OUTLINE_PREFIX, true);
		auto outlines_points_all = GetOutlinesFromOutlineParent(container_find.back());
		if (!outlines_points_all.empty()) {
			Contour3d outline_points = outlines_points_all.front().front();
			outline = MakeLoopPolylinefromContour3d(outline_points);
		}		
	}

	// prepare image lines
	Polyline3d image_lines;	{
		ccHObject::Container container_find, container_objs;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_IMAGELINE_PREFIX, true);
		container_find.back()->filterChildren(container_objs, false, CC_TYPES::POLY_LINE, true);
		image_lines = GetPolylineFromEntities(container_objs);
	}

	// boundary loop
	ccHObject* plane_frame = new ccHObject(BDDB_PLANEFRAME_PREFIX);
	return plane_frame;
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
		if (entity->getName().endsWith(suffix) &&
			GetBaseName(entity->getName()) == basename) {
			return entity;
		}
	}
	return nullptr;
}
ccHObject* BDBaseHObject::GetBuildingGroup(QString building_name, bool check_enable) {
	for (unsigned int i = 0; i < getChildrenNumber(); i++)
		if (getChild(i)->getName() == building_name) 
			return getChild(i);
	return nullptr;
}
ccHObject::Container BDBaseHObject::GetOriginPointCloud(bool check_enable) {
	return GetHObjContainer(CC_TYPES::POINT_CLOUD, BDDB_ORIGIN_CLOUD_SUFFIX, check_enable);
}
ccHObject * BDBaseHObject::GetOriginPointCloud(QString building_name, bool check_enable) {
	return GetHObj(CC_TYPES::POINT_CLOUD, BDDB_ORIGIN_CLOUD_SUFFIX, building_name, check_enable);
}
ccHObject * BDBaseHObject::GetPrimitiveGroup(QString building_name, bool check_enable) {
	return GetHObj(CC_TYPES::HIERARCHY_OBJECT, BDDB_PRIMITIVE_SUFFIX, building_name, check_enable);
}
BDBaseHObject* GetRootBDBase(ccHObject* obj) {
	ccHObject* bd_obj_ = obj;
	do {
		BDBaseHObject* bd_obj = static_cast<BDBaseHObject*>(bd_obj_);
		if (bd_obj->valid) {
			return bd_obj;
		}
		bd_obj_ = bd_obj_->getParent();
	} while (bd_obj_);

	return nullptr;
}