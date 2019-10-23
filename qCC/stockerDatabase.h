
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
	IMPORT_MODELS,
	IMPORT_POSTGIS,
	IMPORT_TYPE_END,
};

Q_DECLARE_METATYPE(BlockDB::blkDataInfo*)
Q_DECLARE_METATYPE(BlockDB::blkCameraInfo)

#define BLK_DATA_METAKEY "BlkDataInfo"

#define PtCld_Dir_NAME "pointClouds"
#define IMAGE_Dir_NAME "images"
#define MISCS_Dir_NAME "miscs"
#define PRODS_Dir_NAME "products"

class DataBaseHObject : public BDBaseHObject_
{
public:
	DataBaseHObject(QString name = QString()) 
		: BDBaseHObject_(name)
		, m_blkData(new BlockDB::BlockDBaseIO)
	{
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	DataBaseHObject(const StHObject& s) 
		: BDBaseHObject_(s) 
		, m_blkData(new BlockDB::BlockDBaseIO) 
	{
		setPath(s.getPath());
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	~DataBaseHObject() {
		if (m_blkData) { delete m_blkData; m_blkData = nullptr; }
	}
	virtual inline void setPath(const QString& tp) override;

	using Container = std::vector<DataBaseHObject *>;

public:
	StHObject* getPointCloudGroup();
	StHObject* getImagesGroup();
	StHObject* getMiscsGroup();
	StHObject* getProductGroup();
	StHObject* getProductItem(QString name);
	StHObject* getProductFiltered();
	StHObject* getProductClassified();
	StHObject* getProductSegmented();
	StHObject* getProductModels();

	static DataBaseHObject* Create(QString absolute_path);
	bool addData(StHObject* obj, BlockDB::blkDataInfo* info, bool exist_info);
	bool addDataExist(BlockDB::blkDataInfo* info);
	void clear();
	bool load();
	bool save();
	bool parseResults(BlockDB::BLOCK_TASK_ID task_id, QStringList results, int copy_mode);
	bool retrieveResults(BlockDB::BLOCK_TASK_ID task_id);
	// copy mode, 0 - copy, 1 - move, 2 - use the origin path
	bool retrieveResults(BlockDB::BLOCK_TASK_ID task_id, QStringList results, int copy_mode);

	BlockDB::BlockDBaseIO* m_blkData;
};

StHObject * createObjectFromBlkDataInfo(BlockDB::blkDataInfo * info, bool return_scene = false);

class BDBaseHObject : public BDBaseHObject_
{
public:
	BDBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {
		setDBSourceType(CC_TYPES::DB_BUILDING);
	}
	BDBaseHObject(const StHObject& s) :
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
	
	StHObject* GetTodoGroup(QString building_name);
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
	BDImageBaseHObject(const StHObject& s) :
		BDBaseHObject_(s) {
		setDBSourceType(CC_TYPES::DB_IMAGE);
	}
	~BDImageBaseHObject() {}

	using Container = std::vector<BDImageBaseHObject *>;
};

inline bool isDatabaseProject(StHObject* object) {
	return object && object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_MAINDB;
}
inline DataBaseHObject* ToDatabaseProject(StHObject* object) {
	return isDatabaseProject(object) ? static_cast<DataBaseHObject*>(object) : nullptr;
}
inline bool isBuildingProject(StHObject* object) {
	return object && object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_BUILDING;
}
inline BDBaseHObject* ToBuildingProject(StHObject* object) {
	return isBuildingProject(object) ? static_cast<BDBaseHObject*>(object) : nullptr;
}
inline bool isImageProject(StHObject* object) {
	return object && object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_IMAGE;
}
inline BDImageBaseHObject* ToImageProject(StHObject* object) {
	return isImageProject(object) ? static_cast<BDImageBaseHObject*>(object) : nullptr;
}

DataBaseHObject* GetRootDataBase(StHObject* obj);
BDBaseHObject* GetRootBDBase(StHObject* obj);
BDImageBaseHObject* GetRootImageBase(StHObject* obj);

StHObject* getChildGroupByName(StHObject* group, QString name, bool auto_create = false, bool add_to_db = false, bool keep_dir_hier = false);

StHObject * findChildByName(StHObject * parent, bool recursive, QString filter, bool strict, CC_CLASS_ENUM type_filter = CC_TYPES::OBJECT, bool auto_create = false, ccGenericGLDisplay * inDisplay = 0);

inline QString BuildingNameByNumber(int number) {
	char name[256];
	sprintf(name, "%s%08d", BDDB_BUILDING_PREFIX, number);
	return name;
}

//! return -1 if no child exists
int GetMaxNumberExcludeChildPrefix(StHObject * obj, QString prefix);

bool StCreatDir(QString dir);

// return new result files, if force success, will return all the existing files whatever the files are successfully moved or not
QStringList moveFilesToDir(QStringList list, QString dir, bool remove_old, QStringList* failed_files = nullptr, bool force_success = false);

#endif