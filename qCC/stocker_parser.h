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

#ifndef __STOCKER_PARSER_HEADER__
#define __STOCKER_PARSER_HEADER__

#include "ccHObject.h"
#include "mainwindow.h"

#ifdef USE_STOCKER
#include "builderlod3/builderlod3.h"
#include "builderlod2/builderlod2.h"
#include "buildercore/StBuilder.h"
#include "ioctrl/StFileOperator.hpp"
#endif // USE_STOCKER

class BDBaseHObject;

QString GetBaseName(QString name);

ccHObject* FitPlaneAndAddChild(ccPointCloud* cloud);

stocker::Contour3d GetPointsFromCloud(ccHObject* entity);
stocker::Polyline3d GetPolylineFromEntities(ccHObject::Container entities);
ccHObject::Container GetEnabledObjFromGroup(ccHObject* entity, CC_CLASS_ENUM type, bool check_enable = true, bool recursive = true);
ccHObject* AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col);
ccHObject* PlaneSegmentationRansac(ccHObject* entity, int min_pts, double distance_epsilon, double seed_raius, double normal_threshold, double ransac_probability, double merge_threshold = -1, double split_threshold = -1);
ccHObject* PlaneSegmentationRgGrow(ccHObject* entity, int min_pts, double distance_epsilon, double seed_raius, double growing_radius, double merge_threshold = -1, double split_threshold = -1);
ccHObject::Container CalcPlaneIntersections(ccHObject::Container entity_planes, double distance);
ccHObject* CalcPlaneBoundary(ccHObject* planeObj);
ccHObject* CalcPlaneOutlines(ccHObject* planeObj, double alpha);
void ShrinkPlaneToOutline(ccHObject* planeObj, double alpha, double distance_epsilon, MainWindow* win);
ccHObject* PlaneFrameOptimization(ccHObject* planeObj);

#define BDDB_ORIGIN_CLOUD_SUFFIX	".original"
#define BDDB_PRIMITIVE_SUFFIX		".primitive"
#define BDDB_IMAGELINE_SUFFIX		".imageline"
#define BDDB_PLANESEG_PREFIX		"Plane"
#define BDDB_BOUNDARY_PREFIX		"Boundary"
#define BDDB_INTERSECT_PREFIX		"Intersection"
#define BDDB_OUTLINE_PREFIX			"Outline"
#define BDDB_IMAGELINE_PREFIX		"Imageline"
#define BDDB_PLANEFRAME_PREFIX		"Frame"

class BDBaseHObject : public ccHObject
{
public:
	BDBaseHObject(QString name = QString()) :
		ccHObject(name), valid(false) {}
	BDBaseHObject(const ccHObject& s) :
		ccHObject(s), valid(false) {}
	~BDBaseHObject() {}

	using Container = std::vector<BDBaseHObject *>;
public:
	stocker::BlockProj block_prj;
	stocker::Vec3d global_shift;
	double global_scale;
	bool valid;
private:
	ccHObject::Container GetHObjContainer(CC_CLASS_ENUM type, QString suffix, bool check_enable = false);
	ccHObject* GetHObj(CC_CLASS_ENUM type, QString suffix, QString basename = QString(), bool check_enable = false);

public:	
	ccHObject* GetBuildingGroup(QString building_name, bool check_enable = false);
	ccHObject::Container GetOriginPointCloud(bool check_enable = false);
	ccHObject* GetOriginPointCloud(QString building_name, bool check_enable = false);
	ccHObject* GetPrimitiveGroup(QString building_name, bool check_enable = false);

	stocker::Vec3d ToLocal(stocker::Vec3d pt) { return (pt + global_shift)*global_scale; }
	stocker::Vec3d ToGlobal(stocker::Vec3d pt) { return pt / global_scale - global_shift; }
};

BDBaseHObject* GetRootBDBase(ccHObject* obj);

#endif
