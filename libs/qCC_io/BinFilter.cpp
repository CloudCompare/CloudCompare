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

#include "BinFilter.h"

//Qt
#include <QApplication>
#include <QFileInfo>
#include <QMessageBox>
#include <QtConcurrentRun>

//qCC_db
#include <cc2DLabel.h>
#include <ccCameraSensor.h>
#include <ccFacet.h>
#include <ccFlags.h>
#include <ccGenericPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccImage.h>
#include <ccMaterialSet.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccSensor.h>
#include <ccSubMesh.h>

//system
#include <cassert>
#include <cstring>
#include <unordered_set>

#if defined(CC_WINDOWS)
#include <windows.h>
#else
#include <ctime>
#include <unistd.h>
#endif


BinFilter::BinFilter()
	: FileIOFilter( {
					"_CloudCompare BIN Filter",
					1.0f,	// priority
					QStringList{ "bin" },
					"bin",
					QStringList{ GetFileFilter() },
					QStringList{ GetFileFilter() },
					Import | Export | BuiltIn
					} )	
{
}

bool BinFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//we list the entities that CAN'T be saved as BIN file (easier ;)
	switch (type)
	{
	//these entities can't be serialized
	case CC_TYPES::POINT_OCTREE:
	case CC_TYPES::POINT_KDTREE:
	case CC_TYPES::CLIPPING_BOX:
		return false;

	//these entities shouldn't be saved alone (but it's possible!)
	case CC_TYPES::MATERIAL_SET:
	case CC_TYPES::ARRAY:
	case CC_TYPES::NORMALS_ARRAY:
	case CC_TYPES::NORMAL_INDEXES_ARRAY:
	case CC_TYPES::RGB_COLOR_ARRAY:
	case CC_TYPES::RGBA_COLOR_ARRAY:
	case CC_TYPES::TEX_COORDS_ARRAY:
	case CC_TYPES::LABEL_2D:
	case CC_TYPES::TRANS_BUFFER:
		break;

	default:
		//nothing to do
		break;
	}

	multiple = true;
	exclusive = false;
	return true;
}

//! Per-cloud header flags (old style)
union HeaderFlags
{
	struct
	{
		bool bit1;			//bit 1
		bool colors;		//bit 2
		bool normals;		//bit 3
		bool scalarField;	//bit 4
		bool name;			//bit 5
		bool sfName;		//bit 6
		bool bit7;			//bit 7
		bool bit8;			//bit 8
	};
	ccFlags flags;

	//! Default constructor
	HeaderFlags()
	{
		flags.reset();
		bit1=true; //bit '1' is always ON!
	}
};

//specific methods (old style)
static int ReadEntityHeader(QFile& in, unsigned &numberOfPoints, HeaderFlags& header)
{
	assert(in.isOpen());

	//number of points
	uint32_t ptsCount;
	if (in.read((char*)&ptsCount,4) < 0)
		return -1;
	numberOfPoints = (unsigned)ptsCount;

	//flags (colors, etc.)
	uint8_t flag;
	if (in.read((char*)&flag,1) < 0)
		return -1;

	header.flags.fromByte((unsigned char)flag);
	//assert(header.bit1 == true); //should always be 0!

	return 0;
}

static QFile* s_file = nullptr;
static int s_flags = 0;
static ccHObject* s_container = nullptr;

CC_FILE_ERROR _LoadFileV2()
{
	return (s_file && s_container ? BinFilter::LoadFileV2(*s_file,*s_container,s_flags) : CC_FERR_BAD_ARGUMENT);
}

CC_FILE_ERROR _SaveFileV2()
{
	return (s_file && s_container ? BinFilter::SaveFileV2(*s_file,s_container) : CC_FERR_BAD_ARGUMENT);
}

CC_FILE_ERROR BinFilter::saveToFile(ccHObject* root, const QString& filename, const SaveParameters& parameters)
{
	if (!root || filename.isNull())
		return CC_FERR_BAD_ARGUMENT;

	QFile out(filename);
	if (!out.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(false, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("BIN file"));
		pDlg->setInfo(QObject::tr("Please wait... saving in progress"));
		pDlg->setRange(0, 0);
		pDlg->setModal(true);
		pDlg->start();
	}

	//concurrent call
	s_file = &out;
	s_container = root;

	QFuture<CC_FILE_ERROR> future = QtConcurrent::run(_SaveFileV2);

	while (!future.isFinished())
	{
#if defined(CC_WINDOWS)
		::Sleep(500);
#else
		usleep(500 * 1000);
#endif
		if (pDlg)
		{
			pDlg->setValue(pDlg->value() + 1);
		}
		QApplication::processEvents();
	}
	
	s_file = nullptr;
	s_container = nullptr;

	CC_FILE_ERROR result = future.result();

	return result;
}

CC_FILE_ERROR BinFilter::SaveFileV2(QFile& out, ccHObject* object)
{
	if (!object)
		return CC_FERR_BAD_ARGUMENT;

	//About BIN versions:
	//- 'original' version (file starts by the number of clouds - no header)
	//- 'new' evolutive version, starts by 4 bytes ("CCB2") + save the current ccObject version

	//header
	//Since ver 2.5.2, the 4th character of the header corresponds to
	//'deserialization flags' (see ccSerializableObject::DeserializationFlags)
	char firstBytes[5] = "CCB2";
	{
		char flags = 0;
		if (sizeof(PointCoordinateType) == 8)
			flags |= static_cast<char>(ccSerializableObject::DF_POINT_COORDS_64_BITS);
		if (sizeof(ScalarType) == 4)
			flags |= static_cast<char>(ccSerializableObject::DF_SCALAR_VAL_32_BITS);
		assert(flags <= 8);
		firstBytes[3] = 48 + flags; //48 = ASCII("0")
	}

	if (out.write(firstBytes,4) < 0)
		return CC_FERR_WRITING;

	// Current BIN file version
	uint32_t binVersion_u32 = static_cast<uint32_t>(ccObject::GetCurrentDBVersion());
	if (out.write((char*)&binVersion_u32, 4) < 0)
		return CC_FERR_WRITING;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//we check if all linked entities are in the sub tree we are going to save
	//(such as vertices for a mesh!)
	ccHObject::Container toCheck;
	toCheck.push_back(object);
	while (!toCheck.empty())
	{
		ccHObject* currentObject = toCheck.back();
		assert(currentObject);
		toCheck.pop_back();

		//we check objects that have links to other entities (meshes, polylines, etc.)
		std::unordered_set<const ccHObject*> dependencies;
		if (currentObject->isA(CC_TYPES::MESH) || currentObject->isKindOf(CC_TYPES::PRIMITIVE))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(currentObject);
			if (mesh->getAssociatedCloud())
				dependencies.insert(mesh->getAssociatedCloud());
			if (mesh->getMaterialSet())
				dependencies.insert(mesh->getMaterialSet());
			if (mesh->getTexCoordinatesTable())
				dependencies.insert(mesh->getTexCoordinatesTable());
			if (mesh->getTexCoordinatesTable())
				dependencies.insert(mesh->getTexCoordinatesTable());
		}
		else if (currentObject->isA(CC_TYPES::SUB_MESH))
		{
			dependencies.insert(currentObject->getParent());
		}
		else if (currentObject->isKindOf(CC_TYPES::POLY_LINE))
		{
			CCLib::GenericIndexedCloudPersist* cloud = static_cast<ccPolyline*>(currentObject)->getAssociatedCloud();
			ccPointCloud* pc = dynamic_cast<ccPointCloud*>(cloud);
			if (pc)
				dependencies.insert(pc);
			else
				ccLog::Warning(QString("[BIN] Poyline '%1' is associated to an unhandled vertices structure?!").arg(currentObject->getName()));
		}
		else if (currentObject->isKindOf(CC_TYPES::SENSOR))
		{
			ccIndexedTransformationBuffer* buffer = static_cast<ccSensor*>(currentObject)->getPositions();
			if (buffer)
				dependencies.insert(buffer);
		}
		else if (currentObject->isA(CC_TYPES::LABEL_2D))
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(currentObject);
			for (unsigned i = 0; i < label->size(); ++i)
			{
				const cc2DLabel::PickedPoint& pp = label->getPickedPoint(i);
				if (pp._cloud)
					dependencies.insert(pp._cloud);
				else if (pp._mesh)
					dependencies.insert(pp._mesh);
			}
		}
		else if (currentObject->isA(CC_TYPES::FACET))
		{
			ccFacet* facet = static_cast<ccFacet*>(currentObject);
			if (facet->getOriginPoints())
				dependencies.insert(facet->getOriginPoints());
			if (facet->getContourVertices())
				dependencies.insert(facet->getContourVertices());
			if (facet->getPolygon())
				dependencies.insert(facet->getPolygon());
			if (facet->getContour())
				dependencies.insert(facet->getContour());
		}
		else if (currentObject->isKindOf(CC_TYPES::IMAGE))
		{
			ccImage* image = static_cast<ccImage*>(currentObject);
			if (image->getAssociatedSensor())
				dependencies.insert(image->getAssociatedSensor());
		}

		for (std::unordered_set<const ccHObject*>::const_iterator it = dependencies.begin(); it != dependencies.end(); ++it)
		{
			if (!object->find((*it)->getUniqueID()))
			{
				ccLog::Warning(QString("[BIN] Dependency broken: entity '%1' must also be in selection in order to save '%2'").arg((*it)->getName(), currentObject->getName()));
				result = CC_FERR_BROKEN_DEPENDENCY_ERROR;
			}
		}
		//release some memory...
		dependencies.clear();

		for (unsigned i = 0; i < currentObject->getChildrenNumber(); ++i)
			toCheck.push_back(currentObject->getChild(i));
	}

	if (result == CC_FERR_NO_ERROR)
		if (!object->toFile(out))
			result = CC_FERR_CONSOLE_ERROR;

	out.close();

	return result;
}

CC_FILE_ERROR BinFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	ccLog::Print(QString("[BIN] Opening file '%1'...").arg(filename));

	//opening file
	QFile in(filename);
	if (!in.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	uint32_t firstBytes = 0;
	if (in.read((char*)&firstBytes, 4) < 0)
		return CC_FERR_READING;
	bool v1 = (strncmp((char*)&firstBytes, "CCB", 3) != 0);

	if (v1)
	{
		return LoadFileV1(in, container, static_cast<unsigned>(firstBytes), parameters); //firstBytes == number of scans for V1 files!
	}
	else
	{
		//Since ver 2.5.2, the 4th character of the header corresponds to 'load flags'
		int flags = 0;
		{
			QChar c(reinterpret_cast<char*>(&firstBytes)[3]);
			bool ok;
			flags = QString(c).toInt(&ok);
			if (!ok || flags > 8)
			{
				ccLog::Error(QString("Invalid file header (4th byte is '%1'?!)").arg(c));
				return CC_FERR_WRONG_FILE_TYPE;
			}
		}

		//if (sizeof(PointCoordinateType) == 8 && strncmp((char*)&firstBytes,"CCB3",4) != 0)
		//{
		//	QMessageBox::information(nullptr, QString("Wrong version"), QString("This file has been generated with the standard 'float' version!\nAt this time it cannot be read with the 'double' version."),QMessageBox::Ok);
		//	return CC_FERR_WRONG_FILE_TYPE;
		//}
		//else if (sizeof(PointCoordinateType) == 4 && strncmp((char*)&firstBytes,"CCB2",4) != 0)
		//{
		//	QMessageBox::information(nullptr, QString("Wrong version"), QString("This file has been generated with the new 'double' version!\nAt this time it cannot be read with the standard 'float' version."),QMessageBox::Ok);
		//	return CC_FERR_WRONG_FILE_TYPE;
		//}

		if (parameters.alwaysDisplayLoadDialog)
		{
			QScopedPointer<ccProgressDialog> pDlg(nullptr);
			if (parameters.parentWidget)
			{
				pDlg.reset(new ccProgressDialog(false, parameters.parentWidget));
				pDlg->setMethodTitle(QObject::tr("BIN file"));
				pDlg->setInfo(QObject::tr("Loading: %1").arg(QFileInfo(filename).fileName()));
				pDlg->setRange(0, 0);
				pDlg->show();
			}

			//concurrent call in a separate thread
			s_file = &in;
			s_container = &container;
			s_flags = flags;

			QFuture<CC_FILE_ERROR> future = QtConcurrent::run(_LoadFileV2);

			while (!future.isFinished())
			{
#if defined(CC_WINDOWS)
				::Sleep(500);
#else
				usleep(500 * 1000);
#endif
				if (pDlg)
				{
					pDlg->setValue(pDlg->value() + 1);
				}
				//pDlg.setValue(static_cast<int>(in.pos())); //DGM: in fact, the file reading part is just half of the work!
				QApplication::processEvents();
			}
	
			s_file = nullptr;
			s_container = nullptr;

			return future.result();
		}
		else
		{
			return BinFilter::LoadFileV2(in, container, flags);
		}
	}
}

inline bool Match(ccHObject* object, unsigned uniqueID, CC_CLASS_ENUM expectedType)
{
	return object && object->getUniqueID() == uniqueID && object->isKindOf(expectedType);
}

ccHObject* FindRobust(ccHObject* root, ccHObject* source, const ccObject::LoadedIDMap& oldToNewIDMap, unsigned oldUniqueID, CC_CLASS_ENUM expectedType)
{
	ccObject::LoadedIDMap::const_iterator it = oldToNewIDMap.find(oldUniqueID);
	while (it != oldToNewIDMap.end() && it.key() == oldUniqueID)
	{
		unsigned uniqueID = it.value();
		++it;

		if (source)
		{
			//1st test the parent
			ccHObject* parent = source->getParent();
			if (Match(parent, uniqueID, expectedType))
				return parent;

			//now test the children
			for (unsigned i = 0; i < source->getChildrenNumber(); ++i)
			{
				ccHObject* child = source->getChild(i);
				if (Match(child, uniqueID, expectedType))
					return child;
			}
		}

		//now test the whole DB
		ccHObject* object = root->find(uniqueID);
		//if we've found an object, we must also test its type!
		if (object && object->isKindOf(expectedType))
		{
			return object;
		}
	}

	//no entity found!
	return nullptr;
}

CC_FILE_ERROR BinFilter::LoadFileV2(QFile& in, ccHObject& container, int flags)
{
	assert(in.isOpen());

	uint32_t binVersion = 20;
	if (in.read((char*)&binVersion, 4) < 0)
		return CC_FERR_READING;

	if (binVersion < 20) //should be superior to 2.0!
		return CC_FERR_MALFORMED_FILE;

	QString coordsFormat = ((flags & ccSerializableObject::DF_POINT_COORDS_64_BITS) ? "double" : "float");
	QString scalarFormat = ((flags & ccSerializableObject::DF_SCALAR_VAL_32_BITS) ? "float" : "double");
	ccLog::Print(QString("[BIN] Version %1.%2 (coords: %3 / scalar: %4)").arg(binVersion / 10).arg(binVersion % 10).arg(coordsFormat).arg(scalarFormat));

	if (ccObject::GetCurrentDBVersion() < binVersion)
	{
		ccLog::Error("This version of CloudCompare is too old and can't load this file, sorry");
		return CC_FERR_CONSOLE_ERROR;
	}

	//we keep track of the last unique ID before load
	unsigned lastUniqueIDBeforeLoad = ccObject::GetLastUniqueID();

	//we read first entity type
	CC_CLASS_ENUM classID = ccObject::ReadClassIDFromFile(in, static_cast<short>(binVersion));
	if (classID == CC_TYPES::OBJECT)
	{
		return CC_FERR_CONSOLE_ERROR;
	}

	//call the CC object factory
	ccHObject* root = ccHObject::New(classID);
	if (!root)
	{
		return CC_FERR_MALFORMED_FILE;
	}

	ccObject::LoadedIDMap oldToNewIDMap;

	if (classID == CC_TYPES::CUSTOM_H_OBJECT)
	{
		// store seeking position
		size_t original_pos = in.pos();
		// we need to load it as plain ccCustomHobject
		root->fromFileNoChildren(in, static_cast<short>(binVersion), flags, oldToNewIDMap); // this will load it
		in.seek(original_pos); // reseek back the file

		QString classId = root->getMetaData("class_name").toString();
		QString pluginId = root->getMetaData("plugin_name").toString();

		// try to get a new object from external factories
		ccHObject* new_child = ccHObject::New(pluginId, classId);
		if (new_child) // found a plugin that can deserialize it
			root = new_child;
		else
			return CC_FERR_FILE_WAS_WRITTEN_BY_UNKNOWN_PLUGIN;
	}

	if (!root->fromFile(in, static_cast<short>(binVersion), flags, oldToNewIDMap))
	{
		//delete root; //DGM: can't delete it, too dangerous (bad pointers ;)
		ccLog::Error(QString("Failed to read file (file position: %1 / %2").arg(in.pos()).arg(in.size()));
		return CC_FERR_CONSOLE_ERROR;
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//re-link objects (and check errors)
	bool checkErrors = true;
	ccHObject* orphans = new ccHObject("Orphans (CORRUPTED FILE)");
	ccHObject::Container toCheck;
	toCheck.push_back(root);
	while (!toCheck.empty())
	{
		ccHObject* currentObject = toCheck.back();
		toCheck.pop_back();

		assert(currentObject);

		//we check objects that have links to other entities (meshes, polylines, etc.)
		if (currentObject->isKindOf(CC_TYPES::MESH))
		{
			//specific case: mesh groups are deprecated!
			if (currentObject->isA(CC_TYPES::MESH_GROUP))
			{
				//TODO
				ccLog::Warning(QString("[BIN] Mesh groups are deprecated! Entity %1 should be ignored...").arg(currentObject->getName()));
			}
			else if (currentObject->isA(CC_TYPES::SUB_MESH))
			{
				ccSubMesh* subMesh = ccHObjectCaster::ToSubMesh(currentObject);

				//normally, the associated mesh should be the sub-mesh's parent!
				//however we have its ID so we will look for it just to be sure
				intptr_t meshID = (intptr_t)subMesh->getAssociatedMesh();
				if (meshID > 0)
				{
					ccHObject* mesh = FindRobust(root, subMesh, oldToNewIDMap, meshID, CC_TYPES::MESH);
					if (mesh)
					{
						subMesh->setAssociatedMesh(ccHObjectCaster::ToMesh(mesh), false); //'false' because previous mesh is not null (= real mesh ID)!!!
					}
					else
					{
						//we have a problem here ;)
						//normally, the associated mesh should be the sub-mesh's parent!
						if (subMesh->getParent() && subMesh->getParent()->isA(CC_TYPES::MESH))
						{
							subMesh->setAssociatedMesh(ccHObjectCaster::ToMesh(subMesh->getParent()), false); //'false' because previous mesh is not null (= real mesh ID)!!!
						}
						else
						{
							subMesh->setAssociatedMesh(nullptr, false); //'false' because previous mesh is not null (= real mesh ID)!!!
							//DGM: can't delete it, too dangerous (bad pointers ;)
							//delete subMesh;
							ccLog::Warning(QString("[BIN] Couldn't find associated mesh (ID=%1) for sub-mesh '%2' in the file!").arg(meshID).arg(subMesh->getName()));
							return CC_FERR_MALFORMED_FILE;
						}
					}
				}
			}
			else if (currentObject->isA(CC_TYPES::MESH) || currentObject->isKindOf(CC_TYPES::PRIMITIVE)) //CC_TYPES::MESH or CC_TYPES::PRIMITIVE!
			{
				ccMesh* mesh = ccHObjectCaster::ToMesh(currentObject);
				assert(mesh);

				//vertices
				intptr_t cloudID = (intptr_t)mesh->getAssociatedCloud();
				if (cloudID > 0)
				{
					ccHObject* cloud = FindRobust(root, mesh, oldToNewIDMap, cloudID, CC_TYPES::POINT_CLOUD);
					if (cloud)
					{
						mesh->setAssociatedCloud(ccHObjectCaster::ToGenericPointCloud(cloud));
					}
					else
					{
						//we have a problem here ;)
						mesh->setAssociatedCloud(nullptr);
						if (mesh->getMaterialSet())
							mesh->setMaterialSet(nullptr, false);
						//DGM: can't delete it, too dangerous (bad pointers ;)
						//delete mesh;
						if (mesh->getParent())
						{
							mesh->getParent()->removeDependencyWith(mesh);
							mesh->getParent()->removeChild(mesh);
						}
						ccLog::Warning(QString("[BIN] Couldn't find vertices (ID=%1) for mesh '%2' in the file!").arg(cloudID).arg(mesh->getName()));
						if (root == mesh)
						{
							return CC_FERR_MALFORMED_FILE;
						}
						currentObject = mesh = nullptr;
					}
				}
				else
				{
					ccLog::Warning(QString("[BIN] Mesh '%1' has no vertices! It will be ignored.").arg(mesh->getName()));
					if (mesh->getParent())
					{
						mesh->getParent()->removeChild(mesh);
					}
					else
					{
						if (root == mesh)
						{
							delete mesh;
							return CC_FERR_MALFORMED_FILE;
						}
						else
						{
							//we'll try to live with that
							delete mesh;
						}
					}
					currentObject = mesh = nullptr;
				}

				if (mesh)
				{
					//materials
					ccHObject* materials = nullptr;
					intptr_t matSetID = (intptr_t)mesh->getMaterialSet();
					if (matSetID > 0)
					{
						materials = FindRobust(root, mesh, oldToNewIDMap, matSetID, CC_TYPES::MATERIAL_SET);
						if (materials)
						{
							mesh->setMaterialSet(static_cast<ccMaterialSet*>(materials), false);
						}
						else
						{
							//we have a (less severe) problem here ;)
							mesh->setMaterialSet(nullptr, false);
							mesh->showMaterials(false);
							ccLog::Warning(QString("[BIN] Couldn't find shared materials set (ID=%1) for mesh '%2' in the file!").arg(matSetID).arg(mesh->getName()));
							result = CC_FERR_BROKEN_DEPENDENCY_ERROR;

							//add it to the 'orphans' set
							if (materials)
								orphans->addChild(materials);
							materials = nullptr;
						}
					}
					//per-triangle normals
					ccHObject* triNormsTable = nullptr;
					intptr_t triNormsTableID = (intptr_t)mesh->getTriNormsTable();
					if (triNormsTableID > 0)
					{
						triNormsTable = FindRobust(root, mesh, oldToNewIDMap, triNormsTableID, CC_TYPES::NORMAL_INDEXES_ARRAY);
						if (triNormsTable)
						{
							mesh->setTriNormsTable(static_cast<NormsIndexesTableType*>(triNormsTable), false);
						}
						else
						{
							//we have a (less severe) problem here ;)
							mesh->setTriNormsTable(nullptr, false);
							mesh->showTriNorms(false);
							ccLog::Warning(QString("[BIN] Couldn't find shared normals (ID=%1) for mesh '%2' in the file!").arg(triNormsTableID).arg(mesh->getName()));
							result = CC_FERR_BROKEN_DEPENDENCY_ERROR;

							//add it to the 'orphans' set
							if (triNormsTable)
								orphans->addChild(triNormsTable);
							triNormsTable = nullptr;
						}
					}
					//per-triangle texture coordinates
					ccHObject* texCoordsTable = nullptr;
					intptr_t texCoordArrayID = (intptr_t)mesh->getTexCoordinatesTable();
					if (texCoordArrayID > 0)
					{
						texCoordsTable = FindRobust(root, mesh, oldToNewIDMap, texCoordArrayID, CC_TYPES::TEX_COORDS_ARRAY);
						if (texCoordsTable)
						{
							mesh->setTexCoordinatesTable(static_cast<TextureCoordsContainer*>(texCoordsTable), false);
						}
						else
						{
							//we have a (less severe) problem here ;)
							mesh->setTexCoordinatesTable(nullptr, false);
							ccLog::Warning(QString("[BIN] Couldn't find shared texture coordinates (ID=%1) for mesh '%2' in the file!").arg(texCoordArrayID).arg(mesh->getName()));
							result = CC_FERR_BROKEN_DEPENDENCY_ERROR;

							//add it to the 'orphans' set
							if (texCoordsTable)
								orphans->addChild(texCoordsTable);
							texCoordsTable = nullptr;
						}
					}

					if (checkErrors)
					{
						ccGenericPointCloud* pc = mesh->getAssociatedCloud();
						unsigned faceCount = mesh->size();
						unsigned vertCount = pc->size();
						for (unsigned i = 0; i < faceCount; ++i)
						{
							const CCLib::VerticesIndexes* tri = mesh->getTriangleVertIndexes(i);
							if (	tri->i1 >= vertCount
								||	tri->i2 >= vertCount
								||	tri->i3 >= vertCount )
							{
								ccLog::Warning(QString("[BIN] File is corrupted: missing vertices for mesh '%1'!").arg(mesh->getName()));

								//add cloud to the 'orphans' set
								pc->setName(mesh->getName() + QString(".") + pc->getName());
								orphans->addChild(pc);
								if (texCoordsTable)
								{
									texCoordsTable->setName(mesh->getName() + QString(".") + texCoordsTable->getName());
									orphans->addChild(texCoordsTable);
								}
								if (triNormsTable)
								{
									triNormsTable->setName(mesh->getName() + QString(".") + triNormsTable->getName());
									orphans->addChild(triNormsTable);
								}
								if (materials)
								{
									materials->setName(mesh->getName() + QString(".") + materials->getName());
									orphans->addChild(materials);
								}

								//delete corrupted mesh
								mesh->setMaterialSet(nullptr, false);
								mesh->setTriNormsTable(nullptr, false);
								mesh->setTexCoordinatesTable(nullptr, false);
								if (mesh->getParent())
									mesh->getParent()->removeChild(mesh);
								currentObject = mesh = nullptr;

								break;
							}
						}
					}
				}
			}
		}
		else if (currentObject->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(currentObject);
			intptr_t cloudID = (intptr_t)poly->getAssociatedCloud();
			ccHObject* cloud = FindRobust(root, poly, oldToNewIDMap, cloudID, CC_TYPES::POINT_CLOUD);
			if (cloud)
			{
				poly->setAssociatedCloud(ccHObjectCaster::ToGenericPointCloud(cloud));
			}
			else
			{
				//we have a problem here ;)
				poly->setAssociatedCloud(nullptr);
				//DGM: can't delete it, too dangerous (bad pointers ;)
				//delete root;
				currentObject = nullptr;
				ccLog::Warning(QString("[BIN] Couldn't find vertices (ID=%1) for polyline '%2' in the file!").arg(cloudID).arg(poly->getName()));
				return CC_FERR_MALFORMED_FILE;
			}
		}
		else if (currentObject->isKindOf(CC_TYPES::SENSOR))
		{
			ccSensor* sensor = ccHObjectCaster::ToSensor(currentObject);
			intptr_t bufferID = (intptr_t)sensor->getPositions();
			if (bufferID > 0)
			{
				ccHObject* buffer = FindRobust(root, sensor, oldToNewIDMap, bufferID, CC_TYPES::TRANS_BUFFER);
				if (buffer)
				{
					sensor->setPositions(ccHObjectCaster::ToTransBuffer(buffer));
				}
				else
				{
					//we have a problem here ;)
					sensor->setPositions(nullptr);

					//DGM: can't delete it, too dangerous (bad pointers ;)
					//delete root;

					currentObject = nullptr;
					ccLog::Warning(QString("[BIN] Couldn't find trans. buffer (ID=%1) for sensor '%2' in the file!").arg(bufferID).arg(sensor->getName()));

					//positions are optional, so we can simply set them to nullptr and go ahead, we do not need to return.
					//return CC_FERR_MALFORMED_FILE;
				}
			}
		}
		else if (currentObject->isA(CC_TYPES::LABEL_2D))
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(currentObject);
			std::vector<cc2DLabel::PickedPoint> correctedPickedPoints;
			//we must check all label 'points'!
			bool invalidLabel = false;
			for (unsigned i = 0; !invalidLabel && i < label->size(); ++i)
			{
				const cc2DLabel::PickedPoint& pp = label->getPickedPoint(i);
				if (pp._cloud)
				{
					intptr_t cloudID = (intptr_t)pp._cloud;
					ccHObject* cloud = FindRobust(root, label, oldToNewIDMap, cloudID, CC_TYPES::POINT_CLOUD);
					if (cloud)
					{
						ccGenericPointCloud* genCloud = ccHObjectCaster::ToGenericPointCloud(cloud);
						assert(genCloud->size() > pp.index);
						correctedPickedPoints.push_back(cc2DLabel::PickedPoint(genCloud, pp.index, pp.entityCenterPoint));
					}
					else
					{
						//we have a problem here ;)
						ccLog::Warning(QString("[BIN] Couldn't find cloud (ID=%1) associated to label ID=%2 in the file!").arg(cloudID).arg(label->getUniqueID())); //we can't call getName as it relies on the cloud/mesh pointers!
						if (label->getParent())
							label->getParent()->removeChild(label);
						//DGM: can't delete it, too dangerous (bad pointers ;)
						//delete label;
						currentObject = label = nullptr;
						invalidLabel = true;
						break;
					}
				}
				else if (pp._mesh)
				{
					intptr_t meshID = (intptr_t)pp._mesh;
					ccHObject* mesh = FindRobust(root, label, oldToNewIDMap, meshID, CC_TYPES::MESH);
					if (mesh)
					{
						ccGenericMesh* genMesh = ccHObjectCaster::ToGenericMesh(mesh);
						assert(genMesh->size() > pp.index);
						correctedPickedPoints.push_back(cc2DLabel::PickedPoint(genMesh, pp.index, pp.uv, pp.entityCenterPoint));
					}
					else
					{
						//we have a problem here ;)
						ccLog::Warning(QString("[BIN] Couldn't find mesh (ID=%1) associated to label ID=%2 in the file!").arg(meshID).arg(label->getUniqueID())); //we can't call getName as it relies on the cloud/mesh pointers!
						if (label->getParent())
							label->getParent()->removeChild(label);
						//DGM: can't delete it, too dangerous (bad pointers ;)
						//delete label;
						currentObject = label = nullptr;
						invalidLabel = true;
						break;
					}
				}
			}

			if (label) //correct label data
			{
				assert(correctedPickedPoints.size() == label->size());
				bool visible = label->isVisible();
				QString originalName(label->getRawName());
				label->clear(true);
				for (const cc2DLabel::PickedPoint& cpp : correctedPickedPoints)
				{
					if (cpp._cloud)
						label->addPickedPoint(cpp._cloud, cpp.index, cpp.entityCenterPoint);
					else if (cpp._mesh)
						label->addPickedPoint(cpp._mesh, cpp.index, cpp.uv, cpp.entityCenterPoint);
				}
				label->setVisible(visible);
				label->setName(originalName);
			}
		}
		else if (currentObject->isA(CC_TYPES::FACET))
		{
			ccFacet* facet = ccHObjectCaster::ToFacet(currentObject);

			//origin points
			{
				intptr_t cloudID = (intptr_t)facet->getOriginPoints();
				if (cloudID > 0)
				{
					ccHObject* cloud = FindRobust(root, facet, oldToNewIDMap, cloudID, CC_TYPES::POINT_CLOUD);
					if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
					{
						facet->setOriginPoints(ccHObjectCaster::ToPointCloud(cloud));
					}
					else
					{
						//we have a problem here ;)
						facet->setOriginPoints(nullptr);
						currentObject = nullptr;
						ccLog::Warning(QString("[BIN] Couldn't find origin points (ID=%1) for facet '%2' in the file!").arg(cloudID).arg(facet->getName()));
					}
				}
			}
			//contour points
			{
				intptr_t cloudID = (intptr_t)facet->getContourVertices();
				if (cloudID > 0)
				{
					ccHObject* cloud = FindRobust(root, facet, oldToNewIDMap, cloudID, CC_TYPES::POINT_CLOUD);
					if (cloud)
					{
						facet->setContourVertices(ccHObjectCaster::ToPointCloud(cloud));
					}
					else
					{
						//we have a problem here ;)
						facet->setContourVertices(nullptr);
						currentObject = nullptr;
						ccLog::Warning(QString("[BIN] Couldn't find contour points (ID=%1) for facet '%2' in the file!").arg(cloudID).arg(facet->getName()));
					}
				}
			}
			//contour polyline
			{
				intptr_t polyID = (intptr_t)facet->getContour();
				if (polyID > 0)
				{
					ccHObject* poly = FindRobust(root, facet, oldToNewIDMap, polyID, CC_TYPES::POLY_LINE);
					if (poly)
					{
						facet->setContour(ccHObjectCaster::ToPolyline(poly));
					}
					else
					{
						//we have a problem here ;)
						facet->setContourVertices(nullptr);
						currentObject = nullptr;
						ccLog::Warning(QString("[BIN] Couldn't find contour polyline (ID=%1) for facet '%2' in the file!").arg(polyID).arg(facet->getName()));
					}
				}
			}
			//polygon mesh
			{
				intptr_t polyID = (intptr_t)facet->getPolygon();
				if (polyID > 0)
				{
					ccHObject* poly = FindRobust(root, facet, oldToNewIDMap, polyID, CC_TYPES::MESH);
					if (poly)
					{
						facet->setPolygon(ccHObjectCaster::ToMesh(poly));
					}
					else
					{
						//we have a problem here ;)
						facet->setPolygon(nullptr);
						currentObject = nullptr;
						ccLog::Warning(QString("[BIN] Couldn't find polygon mesh (ID=%1) for facet '%2' in the file!").arg(polyID).arg(facet->getName()));
					}
				}
			}
		}
		else if (currentObject->isKindOf(CC_TYPES::IMAGE))
		{
			ccImage* image = ccHObjectCaster::ToImage(currentObject);

			intptr_t sensorID = (intptr_t)image->getAssociatedSensor();
			if (sensorID > 0)
			{
				ccHObject* sensor = FindRobust(root, image, oldToNewIDMap, sensorID, CC_TYPES::CAMERA_SENSOR);
				if (sensor)
				{
					image->setAssociatedSensor(ccHObjectCaster::ToCameraSensor(sensor));
				}
				else
				{
					//we have a problem here ;)
					image->setAssociatedSensor(nullptr);

					//DGM: can't delete it, too dangerous (bad pointers ;)
					//delete root;

					currentObject = nullptr;
					ccLog::Warning(QString("[BIN] Couldn't find camera sensor (ID=%1) for image '%2' in the file!").arg(sensorID).arg(image->getName()));
					//return CC_FERR_MALFORMED_FILE;
				}
			}
		}

		if (currentObject)
		{
			const ccShiftedObject* shifted = ccHObjectCaster::ToShifted(currentObject);
			if (shifted)
			{
				//it may be interesting to re-use the Global Shift when loading other files
				ccGlobalShiftManager::StoreShift(shifted->getGlobalShift(), shifted->getGlobalScale());

				//TODO: we should also check that other entities with global shift not too far away
				//have not already been loaded. In which case we should 'translate' the current entity?
			}

			for (unsigned i = 0; i < currentObject->getChildrenNumber(); ++i)
			{
				toCheck.push_back(currentObject->getChild(i));
			}
		}
	}

	if (root->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//transfer children to container
		root->transferChildren(container, true);
		delete root;
		root = nullptr;
	}
	else
	{
		container.addChild(root);
	}

	//orphans
	if (orphans)
	{
		if (orphans->getChildrenNumber() != 0)
		{
			orphans->setEnabled(false);
			container.addChild(orphans);
		}
		else
		{
			delete orphans;
			orphans = nullptr;
		}
	}

	return result;
}

CC_FILE_ERROR BinFilter::LoadFileV1(QFile& in, ccHObject& container, unsigned nbScansTotal, const LoadParameters& parameters)
{
	ccLog::Print("[BIN] Version 1.0");

	if (nbScansTotal > 99)
	{
		if (QMessageBox::question(nullptr, QString("Oups"), QString("Hum, do you really expect %1 point clouds?").arg(nbScansTotal), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			return CC_FERR_WRONG_FILE_TYPE;
	}
	else if (nbScansTotal == 0)
	{
		return CC_FERR_NO_LOAD;
	}

	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Open Bin file (old style)"));
		pDlg->setAutoClose(false);
	}

	for (unsigned k = 0; k < nbScansTotal; k++)
	{
		HeaderFlags header;
		unsigned nbOfPoints = 0;
		if (ReadEntityHeader(in, nbOfPoints, header) < 0)
		{
			return CC_FERR_READING;
		}

		//Console::print("[BinFilter::loadModelFromBinaryFile] Entity %i : %i points, color=%i, norms=%i, dists=%i\n",k,nbOfPoints,color,norms,distances);

		if (nbOfPoints == 0)
		{
			//Console::print("[BinFilter::loadModelFromBinaryFile] rien a faire !\n");
			continue;
		}

		//progress for this cloud
		CCLib::NormalizedProgress nprogress(pDlg.data(), nbOfPoints);
		if (pDlg)
		{
			pDlg->reset();
			pDlg->setInfo(QObject::tr("cloud %1/%2 (%3 points)").arg(k + 1).arg(nbScansTotal).arg(nbOfPoints));
			pDlg->start();
			QApplication::processEvents();
		}

		//Cloud name
		char cloudName[256] = "unnamed";
		if (header.name)
		{
			for (int i = 0; i < 256; ++i)
			{
				if (in.read(cloudName + i, 1) < 0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the cloud name!\n");
					return CC_FERR_READING;
				}
				if (cloudName[i] == 0)
				{
					break;
				}
			}
			//we force the end of the name in case it is too long!
			cloudName[255] = 0;
		}
		else
		{
			sprintf(cloudName,"unnamed - Cloud #%u",k);
		}

		//Cloud name
		char sfName[1024] = "unnamed";
		if (header.sfName)
		{
			for (int i=0; i<1024; ++i)
			{
				if (in.read(sfName+i,1) < 0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the cloud name!\n");
					return CC_FERR_READING;
				}
				if (sfName[i] == 0)
					break;
			}
			//we force the end of the name in case it is too long!
			sfName[1023] = 0;
		}
		else
		{
			strcpy(sfName,"Loaded scalar field");
		}
		
		//Creation
		ccPointCloud* loadedCloud = new ccPointCloud(cloudName);
		CCLib::ScalarField* loadedCloudSF = nullptr;
		if (!loadedCloud)
			return CC_FERR_NOT_ENOUGH_MEMORY;

		unsigned fileChunkPos = 0;
		unsigned fileChunkSize = std::min(nbOfPoints, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);

		loadedCloud->reserveThePointsTable(fileChunkSize);
		if (header.colors)
		{
			loadedCloud->reserveTheRGBTable();
			loadedCloud->showColors(true);
		}
		if (header.normals)
		{
			loadedCloud->reserveTheNormsTable();
			loadedCloud->showNormals(true);
		}
		if (header.scalarField)
		{
			if (loadedCloud->enableScalarField())
			{
				loadedCloudSF = loadedCloud->getCurrentInScalarField();
			}
			else
			{
				ccLog::Warning(QString("Failed to allocate scalar field on cloud '%1'").arg(loadedCloud->getName()));
			}
		}

		unsigned lineRead = 0;
		unsigned parts = 0;

		const ScalarType FORMER_HIDDEN_POINTS = static_cast<ScalarType>(-1.0);

		//read the file
		for (unsigned i = 0; i < nbOfPoints; ++i)
		{
			if (lineRead == fileChunkPos + fileChunkSize)
			{
				if (loadedCloudSF)
				{
					loadedCloudSF->computeMinAndMax();
					loadedCloudSF = nullptr;
				}

				//create a new cloud
				container.addChild(loadedCloud);
				fileChunkPos = lineRead;
				fileChunkSize = std::min(nbOfPoints - lineRead, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
				char partName[sizeof(cloudName) + 16];
				++parts;
				sprintf(partName, "%s.part_%u", cloudName, parts);
				loadedCloud = new ccPointCloud(partName);
				if (!loadedCloud->reserveThePointsTable(fileChunkSize))
				{
					delete loadedCloud;
					return CC_FERR_NOT_ENOUGH_MEMORY;
				}

				if (header.colors)
				{
					loadedCloud->reserveTheRGBTable();
					loadedCloud->showColors(true);
				}
				if (header.normals)
				{
					loadedCloud->reserveTheNormsTable();
					loadedCloud->showNormals(true);
				}
				if (header.scalarField)
				{
					if (loadedCloud->enableScalarField())
					{
						loadedCloudSF = loadedCloud->getCurrentInScalarField();
					}
					else
					{
						ccLog::Warning(QString("Failed to allocate scalar field on cloud '%1'").arg(loadedCloud->getName()));
					}
				}
			}

			float Pf[3];
			if (in.read((char*)Pf, sizeof(float) * 3) < 0)
			{
				//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity point !\n",k);
				return CC_FERR_READING;
			}
			loadedCloud->addPoint(CCVector3::fromArray(Pf));

			if (header.colors)
			{
				ccColor::Rgb C;
				if (in.read((char*)C.rgb, sizeof(ColorCompType) * 3) < 0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity colors !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addColor(C);
			}

			if (header.normals)
			{
				CCVector3 N;
				if (in.read((char*)N.u, sizeof(float) * 3) < 0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity norms !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addNorm(N);
			}

			if (header.scalarField)
			{
				double D;
				if (in.read((char*)&D, sizeof(double)) < 0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity distance!\n",k);
					return CC_FERR_READING;
				}
				if (loadedCloudSF)
				{
					ScalarType d = static_cast<ScalarType>(D);
					loadedCloudSF->addElement(d);
				}
			}

			lineRead++;

			if (parameters.alwaysDisplayLoadDialog && !nprogress.oneStep())
			{
				loadedCloud->resize(i + 1 - fileChunkPos);
				k = nbScansTotal;
				i = nbOfPoints;
			}
		}

		if (pDlg)
		{
			pDlg->stop();
			QApplication::processEvents();
		}

		if (loadedCloudSF)
		{
			loadedCloudSF->setName(sfName);

			//replace HIDDEN_VALUES by NAN_VALUES
			for (unsigned i = 0; i < loadedCloudSF->currentSize(); ++i)
			{
				if (loadedCloudSF->getValue(i) == FORMER_HIDDEN_POINTS)
					loadedCloudSF->setValue(i, NAN_VALUE);
			}
			loadedCloudSF->computeMinAndMax();

			loadedCloud->setCurrentDisplayedScalarField(loadedCloud->getCurrentInScalarFieldIndex());
			loadedCloud->showSF(true);
		}

		container.addChild(loadedCloud);
	}

	return CC_FERR_NO_ERROR;
}
