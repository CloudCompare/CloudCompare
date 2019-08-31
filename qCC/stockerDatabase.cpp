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

#include "stockerDatabase.h"

#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccDBRoot.h"

#include "QFileInfo"
#include <QImageReader>
#include <QFileDialog>
#include "FileIOFilter.h"
#include <QDir>

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

BDBaseHObject* GetRootImageBase(ccHObject* obj) {
	ccHObject* bd_obj_ = obj;
	do {
		if (isImageProject(bd_obj_)) {
			return static_cast<BDBaseHObject*>(bd_obj_);
		}
		bd_obj_ = bd_obj_->getParent();
	} while (bd_obj_);

	return nullptr;
}

ccHObject* DataBaseHObject::getProductGroup() {
	ccHObject* products = nullptr;
	for (size_t i = 0; i < getChildrenNumber(); i++) {
		ccHObject* child = getChild(i);
		if (child->isGroup() && child->getName() == "products") {
			products = child;
		}
	}
	if (!products) {
		products = new ccHObject("products");
		QString path = getPath() + "/products";
		if (QDir().mkdir(path))
			products->setPath(path);
		else {
			delete products; products = nullptr;
			return nullptr;
		}
		addChild(products);
	}
	return products;
}
ccHObject* DataBaseHObject::getProductFiltered() {
	ccHObject* products = getProductGroup();
	if (!products) { return nullptr; }
	ccHObject* find_obj = nullptr;
	for (size_t i = 0; i < products->getChildrenNumber(); i++) {
		ccHObject* child = getChild(i);
		if (child->isGroup() && child->getName() == "filtered") {
			find_obj = child;
		}
	}
	if (!find_obj) {
		find_obj = new ccHObject("filtered");
		QString path = products->getPath() + "/filtered";
		if (QDir().mkdir(path))
			find_obj->setPath(path);
		else {
			delete find_obj; find_obj = nullptr;
			return;
		}
		products->addChild(find_obj);
	}
	return find_obj;
}
ccHObject* DataBaseHObject::getProductClassified() {
	ccHObject* products = getProductGroup();
	if (!products) { return nullptr; }
	ccHObject* find_obj = nullptr;
	for (size_t i = 0; i < products->getChildrenNumber(); i++) {
		ccHObject* child = getChild(i);
		if (child->isGroup() && child->getName() == "classified") {
			find_obj = child;
		}
	}
	if (!find_obj) {
		find_obj = new ccHObject("classified");
		QString path = products->getPath() + "/classified";
		if (QDir().mkdir(path))
			find_obj->setPath(path);
		else {
			delete find_obj; find_obj = nullptr;
			return;
		}
		products->addChild(find_obj);
	}
	return find_obj;
}
ccHObject* DataBaseHObject::getProductBuildingSeg() {
	ccHObject* products = getProductGroup();
	if (!products) { return nullptr; }
	ccHObject* find_obj = nullptr;
	for (size_t i = 0; i < products->getChildrenNumber(); i++) {
		ccHObject* child = getChild(i);
		if (child->isGroup() && child->getName() == "buildings") {
			find_obj = child;
		}
	}
	if (!find_obj) {
		find_obj = new ccHObject("buildings");
		QString path = products->getPath() + "/buildings";
		if (QDir().mkdir(path))
			find_obj->setPath(path);
		else {
			delete find_obj; find_obj = nullptr;
			return;
		}
		products->addChild(find_obj);
	}
	return find_obj;
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

stocker::BuildUnit BDBaseHObject::GetBuildingUnit(std::string building_name) {
	auto iter = block_prj.m_builder.sbuild.find(BuilderBase::BuildNode::Create(building_name));
	if (iter == block_prj.m_builder.sbuild.end()) {
		throw runtime_error("internal error: cannot find building");
		return stocker::BuildUnit("invalid");
	}
	else return (*iter)->data;
}