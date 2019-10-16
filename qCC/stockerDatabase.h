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
#include "BlockDBaseIO.h"

//class BDBaseHObject;

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

#define BDDB_BUILDING_PREFIX		"bd"

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

enum importDataType
{
	IMPORT_POINTS,
	IMPORT_IMAGES,
	IMPORT_MISCS,
	IMPORT_POSTGIS,
	IMPORT_TYPE_END,
};

class DataBaseHObject : public BDBaseHObject_
{
public:
	DataBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	DataBaseHObject(const ccHObject& s) :
		BDBaseHObject_(s) {
		setPath(s.getPath());
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	~DataBaseHObject() {}

	using Container = std::vector<DataBaseHObject *>;

public:
	ccHObject* getPointCloudGroup();
	ccHObject* getImagesGroup();
	ccHObject* getMiscsGroup();
	ccHObject* getProductGroup();
	ccHObject* getProductItem(QString name);
	ccHObject* getProductFiltered();
	ccHObject* getProductClassified();
	ccHObject* getProductSegmented();
	ccHObject* getProductModels();

	static DataBaseHObject* Create(QString absolute_path);
	bool addData(ccHObject* obj, BlockDB::blkDataInfo info);
	bool addData(ccHObject* obj, importDataType type, QString str_level);
	void clear();
	bool load();
	bool save();

private:
	std::map<ccHObject*, BlockDB::blkDataInfo> m_obj_blkInfo;
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

	const stocker::BuildUnit GetBuildingUnit(std::string building_name);
	stocker::BuilderBase::SpBuild GetBuildingSp(std::string building_name);
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

ccHObject* getChildGroupByName(ccHObject* group, QString name, bool auto_create = true, bool add_to_db = false);

ccHObject * findChildByName(ccHObject * parent, bool recursive, QString filter, bool strict, CC_CLASS_ENUM type_filter = CC_TYPES::OBJECT, bool auto_create = false, ccGenericGLDisplay * inDisplay = 0);

inline QString BuildingNameByNumber(int number) {
	char name[256];
	sprintf(name, "%s%08d", BDDB_BUILDING_PREFIX, number);
	return name;
}

//! return -1 if no child exists
int GetMaxNumberExcludeChildPrefix(ccHObject * obj, QString prefix);

bool StCreatDir(QString dir);

QStringList moveFilesToDir(QStringList list, QString dir);
QStringList copyFilesToDir(QStringList list, QString dir);

#endif