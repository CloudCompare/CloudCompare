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

#include "StBlock.h"
#include "StBlockGroup.h"
#include "StBuilding.h"
#include "StFootPrint.h"
#include "StModel.h"
#include "StPrimGroup.h"

#ifdef USE_STOCKER
#include "builderlod3/builderlod3.h"
#include "builderlod2/builderlod2.h"
#include "buildercore/StBuilder.h"
#include "ioctrl/StFileOperator.hpp"

#include "polyfit/method/hypothesis_generator.h"
#include "polyfit/method/face_selection.h"
#include "polyfit/method/method_global.h"
#include "polyfit/model/point_set_io.h"
#include "polyfit/model/point_set.h"
#include "polyfit/model/map_geometry.h"
#include "polyfit/model/map_io.h"
#endif // USE_STOCKER

class BDBaseHObject;
class PolyFitObj;

QString GetBaseName(QString name);

stocker::Contour3d GetPointsFromCloud(ccHObject* entity);
stocker::Contour3d GetPointsFromCloudInsidePolygonXY(ccHObject * entity, stocker::Polyline3d polygon, double height);
std::vector<stocker::Contour3d> GetPointsFromCloudInsidePolygonXY(ccHObject::Container entities, stocker::Polyline3d polygon, double height, bool skip_empty = true);
stocker::Contour3d GetPointsFromCloudInsidePolygon3d(ccHObject * entity, stocker::Polyline3d polygon, stocker::Contour3d & remained, double distance_threshold);
stocker::Polyline3d GetPolygonFromPolyline(ccHObject * entity);
stocker::Polyline3d GetPolylineFromEntities(ccHObject::Container entities);
vector<vector<stocker::Contour3d>> GetOutlinesFromOutlineParent(ccHObject * entity);
ccHObject::Container GetEnabledObjFromGroup(ccHObject* entity, CC_CLASS_ENUM type, bool check_enable = true, bool recursive = true);
ccHObject::Container GetPlaneEntitiesBySelected(ccHObject * entity);
bool isPlaneCloud(ccHObject* entity);
//! return -1 if no child exists
int GetMaxNumberExcludeChildPrefix(ccHObject * obj, QString prefix);

ccPlane* FitPlaneAndAddChild(ccPointCloud* cloud);

ccPointCloud* AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col);

void AddSegmentsToVertices(ccPointCloud * cloud, stocker::Polyline3d lines, QString Prefix, ccColor::Rgb col);

ccPointCloud * AddPointsAsPlane(stocker::Contour3d points, QString name, ccColor::Rgb col);

ccPointCloud* AddSegmentsAsPlane(stocker::Polyline3d lines, QString lines_prefix, ccColor::Rgb col, ccHObject* _exist_cloud = nullptr);

StPrimGroup * AddPlanesPointsAsNewGroup(QString name, std::vector<stocker::Contour3d> planes_points);

ccHObject* PlaneSegmentationRansac(ccHObject* entity, int min_pts, double distance_epsilon, double seed_raius, double normal_threshold, double ransac_probability, double merge_threshold = -1, double split_threshold = -1, ccPointCloud* todo_cloud = nullptr);
void RetrieveUnassignedPoints(ccHObject * original_cloud, ccHObject * prim_group, ccPointCloud * todo_point);
void RetrieveAssignedPoints(ccPointCloud * todo_cloud, ccPointCloud * plane_cloud, double distance_threshold);
ccHObject* PlaneSegmentationRgGrow(ccHObject* entity, int min_pts, double distance_epsilon, double seed_raius, double growing_radius, double merge_threshold = -1, double split_threshold = -1);
ccHObject::Container CalcPlaneIntersections(ccHObject::Container entity_planes, double distance);
ccHObject* CalcPlaneBoundary(ccHObject* planeObj);
ccHObject * DetectLineRansac(ccHObject * entity, double distance, double minpts, double radius);
ccHObject* CalcPlaneOutlines(ccHObject* planeObj, double alpha);
void ShrinkPlaneToOutline(ccHObject* planeObj, double alpha, double distance_epsilon);
ccHObject* PlaneFrameOptimization(ccHObject* planeObj, stocker::FrameOption option);

//! polyfit
ccHObject* PolyfitGenerateHypothesis(ccHObject* primitive_group, PolyFitObj* polyfit_obj);

void PolyfitComputeConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj);

void UpdateConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj);

ccHObject * PolyfitFaceSelection(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj);

#define BDDB_PROJECTNAME_PREFIX		""
#define BDDB_PLANESEG_PREFIX		"Plane"
#define BDDB_BOUNDARY_PREFIX		"Boundary"
#define BDDB_INTERSECT_PREFIX		"Intersection"
#define BDDB_OUTLINE_PREFIX			"Outline"
#define BDDB_IMAGELINE_PREFIX		"Imageline"
#define BDDB_PLANEFRAME_PREFIX		"Frame"
#define BDDB_FOOTPRINT_PREFIX		"Footprint"
#define BDDB_BLOCK_PREFIX			"Block"
#define BDDB_LOD1MODEL_PREFIX		"LoD1_"
#define BDDB_LOD2MODEL_PREFIX		"LoD2_"
#define BDDB_LOD3MODEL_PREFIX		"LoD3_"
#define BDDB_TODOPOINT_PREFIX		"TodoPoint"
#define BDDB_TODOLINE_PREFIX		"TodoLine"
#define BDDB_DEDUCED_PREFIX			"Deduced"

#define BDDB_CAMERA_SUFFIX			".camera"
#define BDDB_ORIGIN_CLOUD_SUFFIX	".original"
#define BDDB_PRIMITIVE_SUFFIX		".primitive"
#define BDDB_POLYFITHYPO_SUFFIX		".hypothesis"
#define BDDB_POLYFITOPTM_SUFFIX		".optimized"
#define BDDB_FINALMODEL_SUFFIX		".model"
#define BDDB_IMAGELINE_SUFFIX		".imageline"
#define BDDB_BLOCKGROUP_SUFFIX		".block"
#define BDDB_LOD1MODEL_SUFFIX		".lod1.model"
#define BDDB_LOD2MODEL_SUFFIX		".lod2.model"
#define BDDB_LOD3MODEL_SUFFIX		".lod3.model"
#define BDDB_TODOGROUP_SUFFIX		".todo"

class BDBaseHObject : public BDBaseHObject_
{
public:
	BDBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {}
	BDBaseHObject(const ccHObject& s) :
		BDBaseHObject_(s) {}
	~BDBaseHObject() {}

	using Container = std::vector<BDBaseHObject *>;

public:
	stocker::BlockProj block_prj;
	stocker::Vec3d global_shift;
	double global_scale;

	std::map<std::string, Map*> building_hypomesh;
//	std::map<std::string, HypothesisGenerator*> building_hypothesis;
private:
	ccHObject::Container GetHObjContainer(CC_CLASS_ENUM type, QString suffix, bool check_enable = false);
	ccHObject* GetHObj(CC_CLASS_ENUM type, QString suffix, QString basename = QString(), bool check_enable = false);

public:	
	StBuilding* GetBuildingGroup(QString building_name, bool check_enable);
	ccPointCloud* GetOriginPointCloud(QString building_name, bool check_enable);
	StPrimGroup* GetPrimitiveGroup(QString building_name, bool check_enable);
	StBlockGroup * GetBlockGroup(QString building_name, bool check_enable);
	ccHObject * GetHypothesisGroup(QString building_name, bool check_enable);
	ccHObject* GetCameraGroup();
	ccHObject* GetTodoGroup(QString building_name, bool check_enable);
	ccPointCloud* GetTodoPoint(QString buildig_name, bool check_enable);
	ccPointCloud* GetTodoLine(QString buildig_name, bool check_enable);
	stocker::Vec3d ToLocal(stocker::Vec3d pt) { return (pt + global_shift)*global_scale; }
	stocker::Vec3d ToGlobal(stocker::Vec3d pt) { return pt / global_scale - global_shift; }

public:
	//! file path

	std::string GetPathModelObj(std::string building_name);

	stocker::BuildUnit GetBuildingUnit(std::string building_name);

};

StBuilding * GetParentBuilding(ccHObject * obj);

bool IsBDBaseObj(ccHObject * obj);

BDBaseHObject* GetRootBDBase(ccHObject* obj);

ccHObject * GetPlaneCloud(ccHObject * planeObj);

bool SetGlobalShiftAndScale(ccHObject * obj);

void filterCameraByName(ccHObject* camera_group, QStringList name_list);

class PolyFitObj
{
public:
	PolyFitObj();
	~PolyFitObj();

	void clear();

	void initGenerator(ccHObject::Container planeObjs);

	void GenerateHypothesis();

	//! all the facets are calculated
	void ComputeConfidence();

	void FacetOptimization();

	void AutoFilterByDistanceAndFitting();

	void UpdateValidFacet(std::vector<stocker::String_String> valid);

	//! only update valid planes
	void UpdateConfidence(ccHObject::Container facetObjs);

	bool OutputResultToObjFile(BDBaseHObject* baseObj, std::string & file_path);
	
private:

	bool FindValidFacet(std::string name_plane, std::string name_facet);

public:
//	PointSet::Ptr			point_set_;
	HypothesisGenerator*	hypothesis_;
	Map::Ptr				hypothesis_mesh_;
	Map::Ptr				optimized_mesh_;
	std::vector<stocker::String_String> valid_group_facet_name;

	std::string building_name;

	double data_fitting;
	double model_coverage;
	double model_complexity;
	bool use_confidence;
	bool strict_intersection;
	double snap_intersection;
	bool auto_filter;
	enum POLYFIT_STUTAS
	{
		STT_prepared,
		STT_hypomesh,
		STT_confidence,
		STT_optimized,
	};
	POLYFIT_STUTAS status;	// 0-prepared, 1-hypomesh, 2-confidence, 3-optimized
};

#include <concurrent_vector.h>
#include <ppl.h>
#define conc_index concurr_index
#ifndef DISABLE_PARRALLEL_FOR
#define ConcPair(x) std::pair<size_t, x>
#define ConcVector(x) Concurrency::concurrent_vector<ConcPair(x)>
#define ConcParForBegin(x) Concurrency::parallel_for((size_t)0, (size_t)x, [&](size_t conc_index)
#define ConcParForEnd );
#define ConcPairObj(x) { conc_index, x }
#define ConcSort(x, v) sort(begin(v), end(v), [](ConcPair(x) _l, ConcPair(x) _r) {return _l.first < _r.first; });
#define GetConcObj(x) x.second
#else
#define ConcVector(x) std::vector<x>
#define ConcParForBegin(x) for (size_t conc_ind = 0; conc_ind < x; conc_ind++)
#define ConcParForEnd 
#define ConcPairObj(x) x
#define ConcSort(x, v)
#define GetConcObj(x) x
#endif // USE_PARRALEL_FOR

#endif

bool FastPlanarTextureMapping(ccHObject * planeObj);

ccHObject * ConstrainedMesh(ccHObject * planeObj);

ccHObject * LoD1FromFootPrint(ccHObject * buildingObj);

ccHObject * LoD2FromFootPrint(ccHObject * buildingObj, double ground_height = DBL_MAX);
