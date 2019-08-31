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

#ifndef __STOCKER_DATABASE_HEADER__
#define __STOCKER_DATABASE_HEADER__

#include "ccHObject.h"
#include "StBlock.h"
#include "StBlockGroup.h"
#include "StBuilding.h"
#include "StFootPrint.h"
#include "StModel.h"
#include "StPrimGroup.h"

#include "buildercore/StBuilder.h"

//class BDBaseHObject;

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

class DataBaseHObject : public BDBaseHObject_
{
public:
	DataBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	DataBaseHObject(const ccHObject& s) :
		BDBaseHObject_(s) {
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	~DataBaseHObject() {}

	using Container = std::vector<DataBaseHObject *>;

public:
	ccHObject* getProductGroup();
	ccHObject* getProductFiltered();
	ccHObject* getProductClassified();
	ccHObject* getProductBuildingSeg();
};

class BDBaseHObject : public BDBaseHObject_
{
public:
	BDBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {
		setDBSourceType(CC_TYPES::DB_BUILDING);
	}
	BDBaseHObject(const ccHObject& s) :
		BDBaseHObject_(s) {
		setDBSourceType(CC_TYPES::DB_BUILDING);
	}
	~BDBaseHObject() {}

	using Container = std::vector<BDBaseHObject *>;

public:
	stocker::BlockProj block_prj;
	stocker::Vec3d global_shift;
	double global_scale;
	
public:	

	StBuilding* GetBuildingGroup(QString building_name, bool check_enable);
	ccPointCloud* GetOriginPointCloud(QString building_name, bool check_enable);
	StPrimGroup* GetPrimitiveGroup(QString building_name);
	StBlockGroup * GetBlockGroup(QString building_name);
	StPrimGroup * GetHypothesisGroup(QString building_name);
	
	ccHObject* GetTodoGroup(QString building_name);
	ccPointCloud* GetTodoPoint(QString buildig_name);
	ccPointCloud* GetTodoLine(QString buildig_name);
	stocker::Vec3d ToLocal(stocker::Vec3d pt) { return (pt + global_shift)*global_scale; }
	stocker::Vec3d ToGlobal(stocker::Vec3d pt) { return pt / global_scale - global_shift; }

public:
	//! file path

	std::string GetPathModelObj(std::string building_name);

	stocker::BuildUnit GetBuildingUnit(std::string building_name);
};

class BDImageBaseHObject : public BDBaseHObject_
{
public:
	BDImageBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {
		setDBSourceType(CC_TYPES::DB_IMAGE);
	}
	BDImageBaseHObject(const ccHObject& s) :
		BDBaseHObject_(s) {
		setDBSourceType(CC_TYPES::DB_IMAGE);
	}
	~BDImageBaseHObject() {}

	using Container = std::vector<BDImageBaseHObject *>;
};

inline bool isDatabaseProject(ccHObject* object) {
	return (object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_MAINDB);
}
inline bool isBuildingProject(ccHObject* object) {
	return (object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_BUILDING);
}
inline bool isImageProject(ccHObject* object) {
	return (object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_IMAGE);
}

DataBaseHObject* GetRootDataBase(ccHObject* obj);
BDBaseHObject* GetRootBDBase(ccHObject* obj);
BDImageBaseHObject* GetRootImageBase(ccHObject* obj);


#endif