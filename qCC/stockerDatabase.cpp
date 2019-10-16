#include "stockerDatabase.h"
#include "ioctrl/StFileOperator.hpp"

#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccDBRoot.h"

#include "mainwindow.h"
#include "QFileInfo"
#include <QImageReader>
#include <QFileDialog>
#include "FileIOFilter.h"
#include <QDir>
#include <QStringLiteral>

#include "BlockDBaseIO.h"

using namespace stocker;

DataBaseHObject * GetRootDataBase(ccHObject * obj)
{
	ccHObject* bd_obj_ = obj;
	do {
		if (isDatabaseProject(bd_obj_)) {
			return static_cast<DataBaseHObject*>(bd_obj_);
		}
		bd_obj_ = bd_obj_->getParent();
	} while (bd_obj_);

	return nullptr;
}

BDBaseHObject* GetRootBDBase(ccHObject* obj) {
	ccHObject* bd_obj_ = obj;
	do {
		if (isBuildingProject(bd_obj_)) {
			return static_cast<BDBaseHObject*>(bd_obj_);
		}
		bd_obj_ = bd_obj_->getParent();
	} while (bd_obj_);

	return nullptr;
}

BDImageBaseHObject* GetRootImageBase(ccHObject* obj) {
	ccHObject* bd_obj_ = obj;
	do {
		if (isImageProject(bd_obj_)) {
			return static_cast<BDImageBaseHObject*>(bd_obj_);
		}
		bd_obj_ = bd_obj_->getParent();
	} while (bd_obj_);

	return nullptr;
}

ccHObject* getChildGroupByName(ccHObject* group, QString name, bool auto_create, bool add_to_db)
{
	ccHObject* find_obj = nullptr;
	for (size_t i = 0; i < group->getChildrenNumber(); i++) {
		ccHObject* child = group->getChild(i);
		if (child->isGroup() && child->getName() == name) {
			find_obj = child;
		}
	}
	if (!find_obj && auto_create) {
		find_obj = new ccHObject(name);
		QString path = group->getPath() + "/" + name;
		if (StCreatDir(path)) {
			find_obj->setPath(path);
		}
		else {
			delete find_obj; find_obj = nullptr;
			return nullptr;
		}
		group->addChild(find_obj);
		if (add_to_db) {
			MainWindow::TheInstance()->addToDB(find_obj, group->getDBSourceType());
		}
	}
	else {
		QString path = find_obj->getPath();
		if (path.isEmpty() ||
			(!QFileInfo(path).exists() && !StCreatDir(path))) {
			return nullptr;
		}
	}
	return find_obj;
}

ccHObject * DataBaseHObject::getPointCloudGroup()
{
	return getChildGroupByName(this, "pointClouds");
}

ccHObject * DataBaseHObject::getImagesGroup()
{
	return getChildGroupByName(this, "images");
}

ccHObject * DataBaseHObject::getMiscsGroup()
{
	return getChildGroupByName(this, "miscs");
}

ccHObject* DataBaseHObject::getProductGroup() {
	return getChildGroupByName(this, "products");
}

ccHObject * DataBaseHObject::getProductItem(QString name)
{
	ccHObject* products = getProductGroup();
	if (!products) { return nullptr; }
	return getChildGroupByName(products, name);
}
ccHObject* DataBaseHObject::getProductFiltered() 
{
	return getProductItem("filtered");
}
ccHObject* DataBaseHObject::getProductClassified() 
{
	return getProductItem("classified");
}
ccHObject * DataBaseHObject::getProductSegmented()
{
	return getProductItem("segmented");
}
ccHObject * DataBaseHObject::getProductModels()
{
	return getProductItem("models");
}

DataBaseHObject * DataBaseHObject::Create(QString absolute_path)
{
	DataBaseHObject* new_database = new DataBaseHObject(QFileInfo(absolute_path).completeBaseName());
	new_database->setPath(QFileInfo(absolute_path).absoluteFilePath());

	//! point clouds
	{
		ccHObject* points = new_database->getPointCloudGroup();
		if (!points) {
			return nullptr;
		}
		points->setLocked(true);
	}

	//! images
	{
		ccHObject* images = new_database->getImagesGroup();
		if (!images) {
			return nullptr;
		}
		images->setLocked(true);
	}

	//! miscellaneous
	{
		ccHObject* misc = new_database->getMiscsGroup();
		if (!misc) {
			return nullptr;
		}
		misc->setLocked(true);
	}

	//! products
	{
		ccHObject* products = new_database->getProductGroup();
		if (!products) {
			return nullptr;
		}
		products->setLocked(true);

		ccHObject* groundFilter = new_database->getProductFiltered();
		if (!groundFilter) {
			return nullptr;
		}
		groundFilter->setLocked(true);

		ccHObject* classified = new_database->getProductClassified();
		if (!classified) {
			return nullptr;
		}
		classified->setLocked(true);

		ccHObject* segments = new_database->getProductSegmented();
		if (!segments) {
			return nullptr;
		}
		segments->setLocked(true);

		ccHObject* models = new_database->getProductModels();
		if (!models) {
			return nullptr;
		}
		models->setLocked(true);
	}
	return new_database;
}

bool DataBaseHObject::addData(ccHObject * obj, BlockDB::blkDataInfo info)
{
	if (info.dataType() == BlockDB::Blk_unset) {
		return false;
	}
	m_obj_blkInfo.insert(std::make_pair(obj, info));
	return true;
}

bool DataBaseHObject::addData(ccHObject * obj, importDataType type, QString str_level)
{
	ccHObject* importObj = nullptr;
	switch (type)
	{
	case IMPORT_POINTS:
		BlockDB::BLOCK_PtCldLevel level;
		for (size_t i = 0; i < BlockDB::PCLEVEL_END; i++) {
			if (str_level == QString::fromLocal8Bit(BlockDB::g_strPtCldLevelName[i])) {
				level = BlockDB::BLOCK_PtCldLevel(i);
			}
		}
		if (level >= BlockDB::PCLEVEL_STRIP && level <= BlockDB::PCLEVEL_TILE) {
			importObj = getPointCloudGroup();
		}
		else if (level == BlockDB::PCLEVEL_FILTER) {
			importObj = getProductFiltered();
		}
		else if (level == BlockDB::PCLEVEL_CLASS) {
			importObj = getProductClassified();
		}
		else if (level == BlockDB::PCLEVEL_BUILD) {
			importObj = getProductSegmented();
		}
		
		break;
	case IMPORT_IMAGES:
		importObj = getImagesGroup();
		break;
	case IMPORT_TYPE_END:
		break;
	default:
		break;
	}
	if (importObj) {
		importObj->addChild(obj);
	}
	return true;
}

void DataBaseHObject::clear()
{
	removeAllChildren();
}

bool DataBaseHObject::load()
{
	QString xml_file = getPath() + "/" + QFileInfo(getPath()).completeBaseName() + ".xml";
	BlockDB::BlockDBaseIO blkDBase;
	if (!blkDBase.loadProject(xml_file.toLocal8Bit())) {
		return false;
	}
	//! add 
	BlockDB::blkImageInfo* images = blkDBase.images();
	blkDBase.images();
	blkDBase.cameras();
	

	return true;
}

bool DataBaseHObject::save()
{
	QString xml_file = getPath() + "/" + QFileInfo(getPath()).completeBaseName() + ".xml";
	BlockDB::BlockDBaseIO blkDBase;

	getPointCloudGroup();
	getImagesGroup();
	
	blkDBase.ptClds();
	blkDBase.images();
	blkDBase.cameras();

	// other information in the prj.ini

	if (!blkDBase.saveProject(xml_file.toLocal8Bit())) {
		return false;
	}
	return true;
}

StBuilding* BDBaseHObject::GetBuildingGroup(QString building_name, bool check_enable) {
	ccHObject* obj = GetHObj(CC_TYPES::ST_BUILDING, "", building_name, check_enable);
	if (obj) return static_cast<StBuilding*>(obj);
	return nullptr;
}
ccPointCloud * BDBaseHObject::GetOriginPointCloud(QString building_name, bool check_enable) {
	ccHObject* obj = GetHObj(CC_TYPES::POINT_CLOUD, BDDB_ORIGIN_CLOUD_SUFFIX, building_name, check_enable);
	if (obj) return static_cast<ccPointCloud*>(obj);
	return nullptr;
}
StPrimGroup * BDBaseHObject::GetPrimitiveGroup(QString building_name) {
	ccHObject* obj = GetHObj(CC_TYPES::ST_PRIMGROUP, BDDB_PRIMITIVE_SUFFIX, building_name, false);
	if (obj) return static_cast<StPrimGroup*>(obj);
	StPrimGroup* group = new StPrimGroup(building_name + BDDB_PRIMITIVE_SUFFIX);
	if (group) {
		ccHObject* bd = GetBuildingGroup(building_name, false);
		if (bd) { bd->addChild(group); MainWindow::TheInstance()->addToDB(group, this->getDBSourceType()); return group; }
		else { delete group; group = nullptr; }
	}
	return nullptr;
}
StBlockGroup * BDBaseHObject::GetBlockGroup(QString building_name) {
	ccHObject* obj = GetHObj(CC_TYPES::ST_BLOCKGROUP, BDDB_BLOCKGROUP_SUFFIX, building_name, false);
	if (obj) return static_cast<StBlockGroup*>(obj);
	StBlockGroup* group = new StBlockGroup(building_name + BDDB_BLOCKGROUP_SUFFIX);
	if (group) {
		ccHObject* bd = GetBuildingGroup(building_name, false);
		if (bd) { bd->addChild(group); MainWindow::TheInstance()->addToDB(group, this->getDBSourceType()); return group; }
		else { delete group; group = nullptr; }
	}
	return nullptr;
}
StPrimGroup * BDBaseHObject::GetHypothesisGroup(QString building_name) {
	ccHObject* obj = GetHObj(CC_TYPES::ST_PRIMGROUP, BDDB_POLYFITHYPO_SUFFIX, building_name, false);
	if (obj) return static_cast<StPrimGroup*>(obj);
	StPrimGroup* group = new StPrimGroup(building_name + BDDB_POLYFITHYPO_SUFFIX);
	if (group) {
		ccHObject* bd = GetBuildingGroup(building_name, false);
		if (bd) { bd->addChild(group); MainWindow::TheInstance()->addToDB(group, this->getDBSourceType()); return group; }
		else { delete group; group = nullptr; }
	}
	return nullptr;
}
ccHObject * BDBaseHObject::GetTodoGroup(QString building_name)
{
	ccHObject* obj = GetHObj(CC_TYPES::HIERARCHY_OBJECT, BDDB_TODOGROUP_SUFFIX, building_name, false);
	if (obj) return static_cast<StBlockGroup*>(obj);
	StBlockGroup* group = new StBlockGroup(building_name + BDDB_TODOGROUP_SUFFIX);
	if (group) {
		group->setDisplay(getDisplay());
		ccHObject* bd = GetBuildingGroup(building_name, false);
		if (bd) { bd->addChild(group); MainWindow::TheInstance()->addToDB(group, this->getDBSourceType()); return group; }
		else { delete group; group = nullptr; }
	}
	return nullptr;
}
ccPointCloud * BDBaseHObject::GetTodoPoint(QString buildig_name)
{
	ccHObject* todo_group = GetTodoGroup(buildig_name);
	if (!todo_group) { throw std::runtime_error("internal error"); return nullptr; }
	ccHObject::Container todo_children;
	todo_group->filterChildrenByName(todo_children, false, BDDB_TODOPOINT_PREFIX, true, CC_TYPES::POINT_CLOUD);
	if (!todo_children.empty()) {
		return ccHObjectCaster::ToPointCloud(todo_children.front());
	}
	else {
		ccPointCloud* todo_point = new ccPointCloud(BDDB_TODOPOINT_PREFIX);
		todo_point->setGlobalScale(global_scale);
		todo_point->setGlobalShift(CCVector3d(vcgXYZ(global_shift)));
		todo_point->setDisplay(todo_group->getDisplay());
		todo_point->showColors(true);
		todo_group->addChild(todo_point);
		MainWindow* win = MainWindow::TheInstance();
		assert(win);
		win->addToDB(todo_point, this->getDBSourceType(), false, false);
		return todo_point;
	}
	return nullptr;
}
ccPointCloud * BDBaseHObject::GetTodoLine(QString buildig_name)
{
	ccHObject* todo_group = GetTodoGroup(buildig_name);
	if (!todo_group) { throw std::runtime_error("internal error"); return nullptr; }
	ccHObject::Container todo_children;
	todo_group->filterChildrenByName(todo_children, false, BDDB_TODOLINE_PREFIX, true, CC_TYPES::POINT_CLOUD);
	if (!todo_children.empty()) {
		return ccHObjectCaster::ToPointCloud(todo_children.front());
	}
	else {
		ccPointCloud* todo_point = new ccPointCloud(BDDB_TODOLINE_PREFIX);
		todo_point->setGlobalScale(global_scale);
		todo_point->setGlobalShift(CCVector3d(vcgXYZ(global_shift)));
		todo_point->setDisplay(getDisplay());
		todo_group->addChild(todo_point);
		MainWindow* win = MainWindow::TheInstance();
		assert(win);
		win->addToDB(todo_point, this->getDBSourceType(), false, false);
		return todo_point;
	}
	return nullptr;
}
std::string BDBaseHObject::GetPathModelObj(std::string building_name)
{
	auto& bd = block_prj.m_builder.sbuild.find(stocker::BuilderBase::BuildNode::Create(building_name));
	if (bd == block_prj.m_builder.sbuild.end()) {
		throw std::runtime_error("cannot find building!" + building_name);
	}
	return std::string((*bd)->data.file_path.model_dir + building_name + MODEL_LOD3_OBJ_SUFFIX);
}

const stocker::BuildUnit BDBaseHObject::GetBuildingUnit(std::string building_name) {
	auto iter = block_prj.m_builder.sbuild.find(BuilderBase::BuildNode::Create(building_name));
	if (iter == block_prj.m_builder.sbuild.end()) {
		throw runtime_error("internal error: cannot find building");
		return stocker::BuildUnit("invalid");
	}
	else return (*iter)->data;
}

stocker::BuilderBase::SpBuild BDBaseHObject::GetBuildingSp(std::string building_name)
{
	auto iter = block_prj.m_builder.sbuild.find(BuilderBase::BuildNode::Create(building_name));
	if (iter == block_prj.m_builder.sbuild.end())
		return nullptr;
	else
		return *iter;
	//return stocker::BuilderSet::SpBuild();
}

ccHObject* findChildByName(ccHObject* parent,
	bool recursive,
	QString filter,
	bool strict,
	CC_CLASS_ENUM type_filter/*=CC_TYPES::OBJECT*/,
	bool auto_create /*= false*/,
	ccGenericGLDisplay* inDisplay/*=0*/)
{
	ccHObject::Container children;
	parent->filterChildrenByName(children, recursive, filter, strict, type_filter, inDisplay);

	if (children.empty()) {
		if (auto_create) {
			ccHObject* obj = ccHObject::New(type_filter, filter.toStdString().c_str());
			if (parent->getDisplay()) {
				obj->setDisplay(parent->getDisplay());
			}
			parent->addChild(obj);
			MainWindow::TheInstance()->addToDB(obj, parent->getDBSourceType(), false, false);
			return obj;
		}
		else
			return nullptr;
	}
	else {
		return children.front();
	}
}

int GetNumberExcludePrefix(ccHObject * obj, QString prefix)
{
	QString name = obj->getName();
	if (name.startsWith(prefix) && name.length() > prefix.length()) {
		QString number = name.mid(prefix.length(), name.length());
		return number.toInt();
	}
	return -1;
}

int GetMaxNumberExcludeChildPrefix(ccHObject * obj, QString prefix/*, CC_CLASS_ENUM type = CC_TYPES::OBJECT*/)
{
	if (!obj) { return -1; }
	set<int> name_numbers;
	for (size_t i = 0; i < obj->getChildrenNumber(); i++) {
		QString name = obj->getChild(i)->getName();
		int number = GetNumberExcludePrefix(obj->getChild(i), prefix);
		if (number >= 0) {
			name_numbers.insert(number);
		}
	}
	if (!name_numbers.empty()) {
		return *name_numbers.rbegin();
	}
	return -1;
}

bool StCreatDir(QString dir)
{
	if (QDir(dir).exists()) {
		return true;
	}
	return CreateDir(dir.toStdString().c_str());
}

QStringList moveFilesToDir(QStringList list, QString dir)
{
	QStringList new_files;
	if (StCreatDir(dir)) {
		for (size_t i = 0; i < list.size(); i++) {
			QString & file = const_cast<QString&>(list[i]);
			QFileInfo file_info(file);
			QString new_file = dir + "/" + file_info.fileName();
			if (QFile::copy(file, new_file)) {
				QFile::remove(file);
				new_files.append(new_file);
			}
			else {
				new_files.append(file);
			}
		}
	}
	else {
		return list;
	}

	return new_files;
}

QStringList copyFilesToDir(QStringList list, QString dir)
{
	QStringList new_files;
	if (StCreatDir(dir)) {
		for (size_t i = 0; i < list.size(); i++) {
			QString & file = const_cast<QString&>(list[i]);
			QFileInfo file_info(file);
			QString new_file = dir + "/" + file_info.fileName();
			if (QFile::copy(file, new_file)) {
				new_files.append(new_file);
			}
			else {
				new_files.append(file);
			}
		}
	}
	else {
		return list;
	}

	return new_files;
}
