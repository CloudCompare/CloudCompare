// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "BinFilter.h"

// Qt
#include <QApplication>
#include <QFileInfo>
#include <QMessageBox>

// qCC_db
#include <cc2DLabel.h>
#include <ccBackgroundTask.h>
#include <ccCameraSensor.h>
#include <ccCircle.h>
#include <ccFacet.h>
#include <ccFlags.h>
#include <ccGenericPointCloud.h>
#include <ccGenericPrimitive.h>
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

// system
#include <cassert>
#include <cstring>
#include <unordered_set>

//! Last saved file version
static short s_lastSavedFileBinVersion = 0;

short BinFilter::GetLastSavedFileVersion()
{
	return s_lastSavedFileBinVersion;
}

BinFilter::BinFilter()
    : FileIOFilter({"_CloudCompare BIN Filter",
                    1.0f, // priority
                    QStringList{"bin"},
                    "bin",
                    QStringList{GetFileFilter()},
                    QStringList{GetFileFilter()},
                    Import | Export | BuiltIn})
{
}

bool BinFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	// we list the entities that CAN'T be saved as BIN file (easier ;)
	switch (type)
	{
	// these entities can't be serialized
	case CC_TYPES::POINT_OCTREE:
	case CC_TYPES::POINT_KDTREE:
	case CC_TYPES::CLIPPING_BOX:
		return false;

	// these entities shouldn't be saved alone (but it's possible!)
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
		// nothing to do
		break;
	}

	multiple  = true;
	exclusive = false;
	return true;
}

//! Per-cloud header flags (old style)
union HeaderFlags
{
	struct
	{
		bool bit1;        // bit 1
		bool colors;      // bit 2
		bool normals;     // bit 3
		bool scalarField; // bit 4
		bool name;        // bit 5
		bool sfName;      // bit 6
		bool bit7;        // bit 7
		bool bit8;        // bit 8
	};
	ccFlags flags;

	//! Default constructor
	HeaderFlags()
	{
		flags.reset();
		bit1 = true; // bit '1' is always ON!
	}
};

// specific methods (old style)
static int ReadEntityHeader(QFile& in, unsigned& numberOfPoints, HeaderFlags& header)
{
	assert(in.isOpen());

	// number of points
	uint32_t ptsCount;
	if (in.read((char*)&ptsCount, 4) < 0)
		return -1;
	numberOfPoints = (unsigned)ptsCount;

	// flags (colors, etc.)
	uint8_t flag;
	if (in.read((char*)&flag, 1) < 0)
		return -1;

	header.flags.fromByte((unsigned char)flag);
	// assert(header.bit1 == true); //should always be 0!

	return 0;
}

CC_FILE_ERROR BinFilter::saveToFile(ccHObject* root, const QString& filename, const SaveParameters& parameters)
{
	s_lastSavedFileBinVersion = 0;

	if (!root || filename.isNull())
		return CC_FERR_BAD_ARGUMENT;

	QFile out(filename);
	if (!out.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	std::unique_ptr<ccProgressDialog> pDlg;
	if (parameters.parentWidget)
	{
		pDlg = std::make_unique<ccProgressDialog>(false, parameters.parentWidget);
		pDlg->setMethodTitle(QObject::tr("BIN file"));
		pDlg->setInfo(QObject::tr("Please wait... saving in progress"));
		pDlg->setRange(0, 0);
		pDlg->setModal(true);
		pDlg->start();
	}

	// concurrent call, so that the progress dialog keeps refreshing
	CC_FILE_ERROR result = ccBackgroundTask::Run([&]()
	                                             { return BinFilter::SaveFileV2(out, root); });

	return result;
}

CC_FILE_ERROR BinFilter::SaveFileV2(QFile& out, ccHObject* object)
{
	if (!object)
		return CC_FERR_BAD_ARGUMENT;

	// About BIN versions:
	//- 'original' version (file starts by the number of clouds - no header)
	//- 'new' evolutive version, starts by 4 bytes ("CCB2") + save the current ccObject version

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	// we check if all linked entities are in the sub tree we are going to save
	//(such as vertices for a mesh!)
	ccHObject::Container toCheck;
	toCheck.push_back(object);
	while (!toCheck.empty())
	{
		ccHObject* currentObject = toCheck.back();
		assert(currentObject);
		toCheck.pop_back();

		// we check objects that have links to other entities (meshes, polylines, etc.)
		std::unordered_set<const ccHObject*> dependencies;
		if (currentObject->isA(CC_TYPES::MESH) || currentObject->isKindOf(CC_TYPES::PRIMITIVE))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(currentObject);
			if (mesh->getAssociatedCloud())
				dependencies.insert(mesh->getAssociatedCloud());
			if (mesh->getMaterialSet())
				dependencies.insert(mesh->getMaterialSet().get());
			if (mesh->getTriNormsTable())
				dependencies.insert(mesh->getTriNormsTable().get());
			if (mesh->getTexCoordinatesTable())
				dependencies.insert(mesh->getTexCoordinatesTable().get());
		}
		else if (currentObject->isA(CC_TYPES::SUB_MESH))
		{
			dependencies.insert(currentObject->getParent());
		}
		else if (currentObject->isKindOf(CC_TYPES::POLY_LINE))
		{
			CCCoreLib::GenericIndexedCloudPersist* cloud = static_cast<ccPolyline*>(currentObject)->getAssociatedCloud();
			ccPointCloud*                          pc    = dynamic_cast<ccPointCloud*>(cloud);
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
		// release some memory...
		dependencies.clear();

		for (unsigned i = 0; i < currentObject->getChildrenNumber(); ++i)
			toCheck.push_back(currentObject->getChild(i));
	}

	if (result != CC_FERR_NO_ERROR)
	{
		return result;
	}

	// header
	// Since ver 2.5.2, the 4th character of the header corresponds to
	//'deserialization flags' (see ccSerializableObject::DeserializationFlags)
	char firstBytes[5] = "CCB2";
	{
		char flags = 0;
		if (sizeof(PointCoordinateType) == 8)
		{
			flags |= static_cast<char>(ccSerializableObject::DF_POINT_COORDS_64_BITS);
		}
		flags |= static_cast<char>(ccSerializableObject::DF_SCALAR_VAL_32_BITS); // internal representation of scalar fields is now always floats
		assert(flags <= 8);
		firstBytes[3] = 48 + flags; // 48 = ASCII("0")
	}

	if (out.write(firstBytes, 4) < 0)
		return CC_FERR_WRITING;

	// Current BIN file version
	short dataVersion = object->minimumFileVersion();
	{
		ccLog::Print(QString("[BIN] Output file version: %1.%2 (automatically deduced from selected entities)").arg(dataVersion / 10).arg(dataVersion % 10));
		uint32_t binVersion_u32 = dataVersion;
		if (out.write((char*)&binVersion_u32, 4) < 0)
			return CC_FERR_WRITING;
	}

	if (!object->toFile(out, dataVersion))
	{
		result = CC_FERR_CONSOLE_ERROR;
	}

	s_lastSavedFileBinVersion = dataVersion;

	out.close();

	return result;
}

CC_FILE_ERROR BinFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	ccLog::Print(QString("[BIN] Opening file '%1'...").arg(filename));

	// opening file
	QFile in(filename);
	if (!in.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	uint32_t firstBytes = 0;
	if (in.read((char*)&firstBytes, 4) < 0)
		return CC_FERR_READING;
	bool v1 = (strncmp((char*)&firstBytes, "CCB", 3) != 0);

	if (v1)
	{
		return LoadFileV1(in, container, static_cast<unsigned>(firstBytes), parameters); // firstBytes == number of scans for V1 files!
	}

	// Since ver 2.5.2, the 4th character of the header corresponds to 'load flags'
	int flags = 0;
	{
		QChar c(reinterpret_cast<char*>(&firstBytes)[3]);
		bool  ok;
		flags = QString(c).toInt(&ok);
		if (!ok || flags > 8)
		{
			ccLog::Error(QString("Invalid file header (4th byte is '%1'?!)").arg(c));
			return CC_FERR_WRONG_FILE_TYPE;
		}
	}

	return BinFilter::LoadFileV2(in,
	                             container,
	                             flags,
	                             parameters.alwaysDisplayLoadDialog,
	                             parameters.parentWidget);
}

static bool Match(ccHObject* object, unsigned uniqueID, CC_CLASS_ENUM expectedType)
{
	return object && object->getUniqueID() == uniqueID && object->isKindOf(expectedType);
}

static ccHObject* FindRobust(ccHObject*                                               root,
                             ccHObject*                                               source,
                             const ccSerializableObject::LoadingContext::LoadedIDMap& oldToNewIDMap,
                             unsigned                                                 oldUniqueID,
                             CC_CLASS_ENUM                                            expectedType)
{
	auto it = oldToNewIDMap.find(oldUniqueID);
	while (it != oldToNewIDMap.end() && it.key() == oldUniqueID)
	{
		unsigned uniqueID = it.value();
		++it;

		if (source)
		{
			// 1st test the parent
			ccHObject* parent = source->getParent();
			if (Match(parent, uniqueID, expectedType))
				return parent;

			// now test the children
			for (unsigned i = 0; i < source->getChildrenNumber(); ++i)
			{
				ccHObject* child = source->getChild(i);
				if (Match(child, uniqueID, expectedType))
					return child;
			}
		}

		// now test the whole DB
		ccHObject* object = root->find(uniqueID);
		// if we've found an object, we must also test its type!
		if (object && object->isKindOf(expectedType))
		{
			return object;
		}
	}

	// no entity found!
	return nullptr;
}

//! Context to tracker errors or events during the incomplete entities linking process
struct IncompleteEntityLinkerContext
{
	IncompleteEntityLinkerContext(ccHObject*& _root, ccHObject::LoadingContext& loadingContext)
	    : root(_root)
	    , loadingContext(loadingContext)
	    , incompleteEntity(nullptr)
	    , dependencies()
	    , result(CC_FERR_NO_ERROR)
	    , checkErrors(true)
	    , hasBrokenDependencies(false)
	    , forceLoadAfterError(false)
	{
	}

	void setIncompleteEntity(ccHObject*                                                           _incompleteEntity,
	                         const std::vector<ccSerializableObject::LoadingContext::Dependency>& _dependencies)
	{
		incompleteEntity = _incompleteEntity;
		dependencies     = _dependencies;
	}

	//! Detach and delete an object from the DB (and its children from the loading context if the parent was incomplete)
	void deleteIncompleteEntity()
	{
		if (!incompleteEntity)
		{
			assert(false);
			return;
		}

		// make sure to remove any children from the loading context if the parent was incomplete
		for (auto it = loadingContext.incompleteEntities.begin(); root && it != loadingContext.incompleteEntities.end(); ++it)
		{
			if (it.key() != incompleteEntity && !it.value().empty())
			{
				ccHObject* otherIncompleteEntity = static_cast<ccHObject*>(it.key());
				if (incompleteEntity->isAncestorOf(otherIncompleteEntity))
				{
					it.value().clear();
				}
			}
		}

		auto* parent = incompleteEntity->getParent();
		if (parent)
		{
			// detach the object from its parent (and remove the dependency link if any)
			parent->removeDependencyWith(incompleteEntity);
			parent->removeChild(incompleteEntity);
		}
		else if (root == incompleteEntity)
		{
			root = nullptr;
		}
		delete incompleteEntity;
		incompleteEntity = nullptr;
	}

	ccHObject*&                root;           //!< Reference to the root of the loaded DB
	ccHObject::LoadingContext& loadingContext; //!< Reference to the loading context

	ccHObject*                                                    incompleteEntity; //!< Reference to the incomplete entity (we may want to delete it if we can't find its dependencies)
	std::vector<ccSerializableObject::LoadingContext::Dependency> dependencies;     //!< Copy of the dependencies for the current incomplete entity

	CC_FILE_ERROR result; //!< Result of the loading process

	bool checkErrors;           //!< Whether we should check for errors or not
	bool hasBrokenDependencies; //!< Whether we have detected broken dependencies or not
	bool forceLoadAfterError;   //!< Whether we should force loading the entity after an error (we may want to ask the user if he wants to continue loading the file or not)
};

static bool ContinueAfterError(IncompleteEntityLinkerContext& context, bool couldBeAMemoryIssue = false)
{
	if (!context.forceLoadAfterError)
	{
		// If forceLoadAfterError is false, it means we haven't asked the question yet, so let's do it
		if (QMessageBox::Yes == QMessageBox::critical(nullptr, QObject::tr("Reading error"), couldBeAMemoryIssue ? "The file couldn't be completely loaded, but some entities were loaded.\nDo you want to take the risk to load them? (CC could crash)" : "The file seems corrupted, but some entities were loaded.\nDo you want to take the risk to load them? (CC could crash)", QMessageBox::Yes, QMessageBox::No))
		{
			context.forceLoadAfterError = true;
		}
	}

	return context.forceLoadAfterError;
}

static void HandleMeshGroup(IncompleteEntityLinkerContext& linkerContext)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}
	ccLog::Warning(QString("[BIN] Mesh groups are deprecated! Entity %1 should be ignored...").arg(linkerContext.incompleteEntity->getName()));
}

static void HandleSubMesh(IncompleteEntityLinkerContext& linkerContext)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}

	ccSubMesh* subMesh = ccHObjectCaster::ToSubMesh(linkerContext.incompleteEntity);
	if (!subMesh)
	{
		assert(false);
		return;
	}

	bool hasAssociatedMesh = false;
	for (auto depIt = linkerContext.dependencies.begin(); depIt != linkerContext.dependencies.end(); ++depIt)
	{
		switch (depIt->type)
		{
		case ccSerializableObject::LoadingContext::Dependency::SUBMESH_ASSOCIATED_MESH:
		{
			ccHObject* mesh = FindRobust(linkerContext.root, subMesh, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::MESH);
			if (mesh)
			{
				subMesh->setAssociatedMesh(ccHObjectCaster::ToMesh(mesh));
				hasAssociatedMesh = true;
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find associated mesh (ID=%1) for sub-mesh '%2' in the file!").arg(depIt->objectID).arg(subMesh->getName()));
			}
		}
		break;

		default:
			ccLog::Warning(QString("[BIN] Unexpected dependency type (%1) for sub-mesh '%2' in the file!").arg(depIt->type).arg(subMesh->getName()));
			assert(false);
			break;
		}
	}

	if (!hasAssociatedMesh)
	{
		ccLog::Warning(QString("[BIN] No associated mesh found for sub-mesh '%1' in the file!").arg(subMesh->getName()));
		linkerContext.hasBrokenDependencies = true;

		if (!ContinueAfterError(linkerContext))
		{
			linkerContext.deleteIncompleteEntity();
			linkerContext.result           = CC_FERR_MALFORMED_FILE;
			linkerContext.incompleteEntity = nullptr;
			return;
		}

		auto subMeshParent = subMesh->getParent();
		if (subMeshParent && subMeshParent->isA(CC_TYPES::MESH))
		{
			ccLog::Warning(QString("[BIN] Automatically replacing it by its parent '%1'...").arg(subMeshParent->getName()));
			subMesh->setAssociatedMesh(ccHObjectCaster::ToMesh(subMeshParent));
		}
		else
		{
			linkerContext.deleteIncompleteEntity();
			linkerContext.incompleteEntity = nullptr;
		}
	}
}

static void HandleMeshOrPrimitive(IncompleteEntityLinkerContext& linkerContext,
                                  ccHObject*                     orphans)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}

	ccMesh* mesh = ccHObjectCaster::ToMesh(linkerContext.incompleteEntity);
	if (!mesh)
	{
		assert(false);
		return;
	}

	if (mesh->isKindOf(CC_TYPES::PRIMITIVE))
	{
		auto vertices = mesh->getAssociatedCloud();
		if (vertices)
		{
			mesh->setAssociatedCloud(nullptr);
			mesh->removeChild(vertices);
		}
	}

	bool                           hasAssociatedVertices = false;
	ccMaterialSet::Shared          materials;
	NormsIndexesTableType::Shared  triNormsTable;
	TextureCoordsContainer::Shared texCoordsTable;

	for (auto depIt = linkerContext.dependencies.begin(); depIt != linkerContext.dependencies.end(); ++depIt)
	{
		switch (depIt->type)
		{
		case ccSerializableObject::LoadingContext::Dependency::MESH_VERTICES_CLOUD:
		{
			ccHObject* cloud = FindRobust(linkerContext.root, mesh, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::POINT_CLOUD);
			if (cloud)
			{
				mesh->setAssociatedCloud(ccHObjectCaster::ToGenericPointCloud(cloud));
				hasAssociatedVertices = true;
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find vertices (ID=%1) for mesh '%2' in the file!").arg(depIt->objectID).arg(mesh->getName()));
				if (mesh->isKindOf(CC_TYPES::PRIMITIVE))
				{
					static_cast<ccGenericPrimitive*>(mesh)->updateRepresentation();
				}
			}
		}
		break;

		case ccSerializableObject::LoadingContext::Dependency::MESH_MATERIALS:
		{
			materials.reset(static_cast<ccMaterialSet*>(FindRobust(linkerContext.root, mesh, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::MATERIAL_SET)));
			if (materials)
			{
				mesh->setMaterialSet(materials);
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find shared materials set (ID=%1) for mesh '%2' in the file!").arg(depIt->objectID).arg(mesh->getName()));
				linkerContext.hasBrokenDependencies = true;
				mesh->showMaterials(false);
			}
		}
		break;

		case ccSerializableObject::LoadingContext::Dependency::MESH_TRI_NORMALS:
		{
			triNormsTable.reset(static_cast<NormsIndexesTableType*>(FindRobust(linkerContext.root, mesh, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::NORMAL_INDEXES_ARRAY)));
			if (triNormsTable)
			{
				mesh->setTriNormsTable(triNormsTable);
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find shared normals (ID=%1) for mesh '%2' in the file!").arg(depIt->objectID).arg(mesh->getName()));
				linkerContext.hasBrokenDependencies = true;
				mesh->showTriNorms(false);
			}
		}
		break;

		case ccSerializableObject::LoadingContext::Dependency::MESH_TEXTURE_COORDS:
		{
			texCoordsTable.reset(static_cast<TextureCoordsContainer*>(FindRobust(linkerContext.root, mesh, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::TEX_COORDS_ARRAY)));
			if (texCoordsTable)
			{
				mesh->setTexCoordinatesTable(texCoordsTable);
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find shared texture coordinates (ID=%1) for mesh '%2' in the file!").arg(depIt->objectID).arg(mesh->getName()));
				linkerContext.hasBrokenDependencies = true;
				mesh->showMaterials(false);
			}
		}
		break;

		default:
			ccLog::Warning(QString("[BIN] Unexpected dependency type (%1) for mesh '%2' in the file!").arg(depIt->type).arg(mesh->getName()));
			assert(false);
			break;
		}
	}

	if (!hasAssociatedVertices)
	{
		linkerContext.deleteIncompleteEntity();
		linkerContext.incompleteEntity = nullptr;
		if (!ContinueAfterError(linkerContext))
		{
			linkerContext.result = CC_FERR_MALFORMED_FILE;
		}
		return;
	}

	if (mesh && linkerContext.checkErrors)
	{
		ccGenericPointCloud* pc = mesh->getAssociatedCloud();
		assert(pc);

		unsigned faceCount = mesh->size();
		unsigned vertCount = pc->size();

		for (unsigned i = 0; i < faceCount; ++i)
		{
			const CCCoreLib::VerticesIndexes* tri = mesh->getTriangleVertIndexes(i);
			if (tri->i1 >= vertCount || tri->i2 >= vertCount || tri->i3 >= vertCount)
			{
				ccLog::Warning(QString("[BIN] File is corrupted: some vertices are missing for mesh '%1'!").arg(mesh->getName()));

				if (mesh->isAncestorOf(pc) && orphans)
				{
					pc->setName(mesh->getName() + QString(".") + pc->getName());
					orphans->addChild(pc);
				}
				pc->setVisible(true);
				mesh->detachAllChildren();

				if (materials && orphans)
				{
					materials->setName(mesh->getName() + QString(".") + materials->getName());
					orphans->addChild(materials.get());
				}
				if (triNormsTable && orphans)
				{
					triNormsTable->setName(mesh->getName() + QString(".") + triNormsTable->getName());
					orphans->addChild(triNormsTable.get());
				}
				if (texCoordsTable && orphans)
				{
					texCoordsTable->setName(mesh->getName() + QString(".") + texCoordsTable->getName());
					orphans->addChild(texCoordsTable.get());
				}

				linkerContext.deleteIncompleteEntity();
				linkerContext.incompleteEntity = nullptr;
				break;
			}
		}
	}
}

static void HandlePolyline(
    IncompleteEntityLinkerContext& linkerContext)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}

	ccPolyline* poly = ccHObjectCaster::ToPolyline(linkerContext.incompleteEntity);
	if (!poly)
	{
		assert(false);
		return;
	}

	bool hasAssociatedVertices = false;
	for (auto depIt = linkerContext.dependencies.begin(); depIt != linkerContext.dependencies.end(); ++depIt)
	{
		switch (depIt->type)
		{
		case ccSerializableObject::LoadingContext::Dependency::POLYLINE_VERTICES_CLOUD:
		{
			ccHObject* cloudEntity = FindRobust(linkerContext.root, poly, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::POINT_CLOUD);
			if (cloudEntity)
			{
				ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(cloudEntity);
				poly->setAssociatedCloud(cloud);
				hasAssociatedVertices = true;
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find vertices (ID=%1) for polyline '%2' in the file!").arg(depIt->objectID).arg(poly->getName()));
			}
		}
		break;

		default:
			ccLog::Warning(QString("[BIN] Unexpected dependency type (%1) for polyline '%2' in the file!").arg(depIt->type).arg(poly->getName()));
			assert(false);
			break;
		}
	}

	if (!hasAssociatedVertices)
	{
		linkerContext.hasBrokenDependencies = true;
		linkerContext.deleteIncompleteEntity();
		linkerContext.incompleteEntity = nullptr;
		if (!ContinueAfterError(linkerContext))
		{
			linkerContext.result = CC_FERR_MALFORMED_FILE;
		}
		return;
	}

	if (poly)
	{
		unsigned pointCount = poly->getAssociatedCloud()->size();
		for (unsigned i = 0; i < poly->size(); ++i)
		{
			if (poly->getPointGlobalIndex(i) >= pointCount)
			{
				ccLog::Warning(QString("[BIN] Polyline '%1' (ID=%2) seems corrupted!").arg(poly->getName()).arg(poly->getUniqueID()));
				linkerContext.deleteIncompleteEntity();
				linkerContext.incompleteEntity = nullptr;
				break;
			}
		}
	}
}

static void HandleSensor(IncompleteEntityLinkerContext& linkerContext)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}

	ccSensor* sensor = ccHObjectCaster::ToSensor(linkerContext.incompleteEntity);
	if (!sensor)
	{
		assert(false);
		return;
	}

	bool hasAssociatedBuffer = false;
	for (auto depIt = linkerContext.dependencies.begin(); depIt != linkerContext.dependencies.end(); ++depIt)
	{
		switch (depIt->type)
		{
		case ccSerializableObject::LoadingContext::Dependency::SENSOR_POSITIONS_BUFFER:
		{
			hasAssociatedBuffer = true;
			ccHObject* buffer   = FindRobust(linkerContext.root, sensor, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::TRANS_BUFFER);
			if (buffer)
			{
				sensor->setPositions(ccHObjectCaster::ToTransBuffer(buffer));
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find transformation buffer (ID=%1) for sensor '%2' in the file!").arg(depIt->objectID).arg(sensor->getName()));
				linkerContext.hasBrokenDependencies = true;
			}
		}
		break;

		default:
			ccLog::Warning(QString("[BIN] Unexpected dependency type (%1) for sensor '%2' in the file!").arg(depIt->type).arg(sensor->getName()));
			assert(false);
			break;
		}
	}

	if (!hasAssociatedBuffer)
	{
		assert(false);
		ccLog::Warning(QString("[BIN] Missing transformation buffer for sensor '%1' in the file!").arg(sensor->getName()));
	}
}

static void HandleLabel2D(
    IncompleteEntityLinkerContext& linkerContext)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}

	cc2DLabel* label = ccHObjectCaster::To2DLabel(linkerContext.incompleteEntity);
	if (!label)
	{
		assert(false);
		return;
	}

	std::vector<cc2DLabel::PickedPoint> correctedPickedPoints;
	correctedPickedPoints.reserve(linkerContext.dependencies.size());
	for (size_t index = 0; index < linkerContext.dependencies.size(); ++index)
	{
		const auto& dep = linkerContext.dependencies[index];
		switch (dep.type)
		{
		case ccSerializableObject::LoadingContext::Dependency::LABEL_SOURCE_CLOUD:
		{
			const cc2DLabel::PickedPoint& pp    = label->getPickedPoint(static_cast<unsigned>(index));
			ccHObject*                    cloud = FindRobust(linkerContext.root, label, linkerContext.loadingContext.oldToNewIDMap, dep.objectID, CC_TYPES::POINT_CLOUD);
			if (cloud)
			{
				ccGenericPointCloud* genCloud = ccHObjectCaster::ToGenericPointCloud(cloud);
				assert(genCloud && genCloud->size() > pp.index);
				correctedPickedPoints.push_back(cc2DLabel::PickedPoint(genCloud, pp.index, pp.entityCenterPoint));
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find cloud (ID=%1) associated to label ID=%2 in the file!").arg(dep.objectID).arg(label->getUniqueID()));
				index = linkerContext.dependencies.size();
			}
		}
		break;

		case ccSerializableObject::LoadingContext::Dependency::LABEL_SOURCE_MESH:
		{
			const cc2DLabel::PickedPoint& pp   = label->getPickedPoint(static_cast<unsigned>(index));
			ccHObject*                    mesh = FindRobust(linkerContext.root, label, linkerContext.loadingContext.oldToNewIDMap, dep.objectID, CC_TYPES::MESH);
			if (mesh)
			{
				ccGenericMesh* genMesh = ccHObjectCaster::ToGenericMesh(mesh);
				assert(genMesh && genMesh->size() > pp.index);
				correctedPickedPoints.push_back(cc2DLabel::PickedPoint(genMesh, pp.index, pp.uv, pp.entityCenterPoint));
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find mesh (ID=%1) associated to label ID=%2 in the file!").arg(dep.objectID).arg(label->getUniqueID()));
				index = linkerContext.dependencies.size();
			}
		}
		break;

		default:
			ccLog::Warning(QString("[BIN] Unexpected dependency type (%1) for label '%2' in the file!").arg(dep.type).arg(label->getName()));
			assert(false);
			index = linkerContext.dependencies.size();
			break;
		}
	}

	if (correctedPickedPoints.size() == label->size())
	{
		bool    visible      = label->isVisible();
		QString originalName = label->getRawName();
		label->clear(true);
		for (const cc2DLabel::PickedPoint& cpp : correctedPickedPoints)
		{
			if (cpp._cloud)
			{
				label->addPickedPoint(cpp._cloud, cpp.index, cpp.entityCenterPoint);
			}
			else if (cpp._mesh)
			{
				label->addPickedPoint(cpp._mesh, cpp.index, cpp.uv, cpp.entityCenterPoint);
			}
			else
			{
				assert(false);
			}
		}
		label->setVisible(visible);
		label->setName(originalName);
	}
	else
	{
		linkerContext.hasBrokenDependencies = true;
		ccLog::Warning(QString("[BIN] Label '%1' (ID=%2) seems corrupted!").arg(label->getName()).arg(label->getUniqueID()));
		linkerContext.deleteIncompleteEntity();
		linkerContext.incompleteEntity = nullptr;
		if (!ContinueAfterError(linkerContext))
		{
			linkerContext.result = CC_FERR_MALFORMED_FILE;
		}
	}
}

static void HandleFacet(
    IncompleteEntityLinkerContext& linkerContext)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}

	ccFacet* facet = ccHObjectCaster::ToFacet(linkerContext.incompleteEntity);
	if (!facet)
	{
		assert(false);
		return;
	}

	bool hasOriginPoints    = false;
	bool hasContourVertices = false;
	bool hasContourPolyline = false;
	bool hasPolygon         = false;

	for (auto depIt = linkerContext.dependencies.begin(); depIt != linkerContext.dependencies.end(); ++depIt)
	{
		switch (depIt->type)
		{
		case ccSerializableObject::LoadingContext::Dependency::FACET_ORIGIN_POINTS:
		{
			ccHObject* cloud = FindRobust(linkerContext.root, facet, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::POINT_CLOUD);
			if (cloud)
			{
				facet->setOriginPoints(ccHObjectCaster::ToPointCloud(cloud));
				hasOriginPoints = true;
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find origin points (ID=%1) for facet '%2' in the file!").arg(depIt->objectID).arg(facet->getName()));
				linkerContext.hasBrokenDependencies = true;
			}
		}
		break;

		case ccSerializableObject::LoadingContext::Dependency::FACET_CONTOUR_VERTICES:
		{
			ccHObject* cloud = FindRobust(linkerContext.root, facet, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::POINT_CLOUD);
			if (cloud)
			{
				facet->setContourVertices(ccHObjectCaster::ToPointCloud(cloud));
				hasContourVertices = true;
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find contour points (ID=%1) for facet '%2' in the file!").arg(depIt->objectID).arg(facet->getName()));
				linkerContext.hasBrokenDependencies = true;
			}
		}
		break;

		case ccSerializableObject::LoadingContext::Dependency::FACET_CONTOUR_POLYLINE:
		{
			ccHObject* poly = FindRobust(linkerContext.root, facet, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::POLY_LINE);
			if (poly)
			{
				facet->setContour(ccHObjectCaster::ToPolyline(poly));
				hasContourPolyline = true;
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find contour polyline (ID=%1) for facet '%2' in the file!").arg(depIt->objectID).arg(facet->getName()));
				linkerContext.hasBrokenDependencies = true;
			}
		}
		break;

		case ccSerializableObject::LoadingContext::Dependency::FACET_POLYGON_MESH:
		{
			ccHObject* poly = FindRobust(linkerContext.root, facet, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::MESH);
			if (poly)
			{
				facet->setPolygon(ccHObjectCaster::ToMesh(poly));
				hasPolygon = true;
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find polygon mesh (ID=%1) for facet '%2' in the file!").arg(depIt->objectID).arg(facet->getName()));
				linkerContext.hasBrokenDependencies = true;
			}
		}
		break;

		default:
			ccLog::Warning(QString("[BIN] Unexpected dependency type (%1) for facet '%2' in the file!").arg(depIt->type).arg(facet->getName()));
			assert(false);
			break;
		}
	}

	if (!hasOriginPoints && !hasContourVertices && !hasContourPolyline && !hasPolygon)
	{
		linkerContext.deleteIncompleteEntity();
		linkerContext.incompleteEntity = nullptr;
		if (!ContinueAfterError(linkerContext))
		{
			linkerContext.result = CC_FERR_MALFORMED_FILE;
		}
	}
}

static void HandleImage(
    IncompleteEntityLinkerContext& linkerContext)
{
	if (!linkerContext.incompleteEntity)
	{
		assert(false);
		return;
	}

	ccImage* image = ccHObjectCaster::ToImage(linkerContext.incompleteEntity);
	if (!image)
	{
		assert(false);
		return;
	}

	bool hasAssociatedSensor = false;

	for (auto depIt = linkerContext.dependencies.begin(); depIt != linkerContext.dependencies.end(); ++depIt)
	{
		switch (depIt->type)
		{
		case ccSerializableObject::LoadingContext::Dependency::IMAGE_SENSOR:
		{
			hasAssociatedSensor = true;
			ccHObject* sensor   = FindRobust(linkerContext.root, image, linkerContext.loadingContext.oldToNewIDMap, depIt->objectID, CC_TYPES::CAMERA_SENSOR);
			if (sensor)
			{
				image->setAssociatedSensor(ccHObjectCaster::ToCameraSensor(sensor));
			}
			else
			{
				ccLog::Warning(QString("[BIN] Couldn't find associated sensor (ID=%1) for image '%2' in the file!").arg(depIt->objectID).arg(image->getName()));
				ccHObject::Container children;
				if (image->filterChildren(children, false, CC_TYPES::CAMERA_SENSOR, true) > 0)
				{
					ccLog::Warning(QString("[BIN] Automatically replacing it by its child '%1'...").arg(children.front()->getName()));
					image->setAssociatedSensor(ccHObjectCaster::ToCameraSensor(children.front()));
				}
				linkerContext.hasBrokenDependencies = true;
			}
		}
		break;

		default:
			ccLog::Warning(QString("[BIN] Unexpected dependency type (%1) for image '%2' in the file!").arg(depIt->type).arg(image->getName()));
			assert(false);
			break;
		}
	}

	if (!hasAssociatedSensor)
	{
		assert(false);
		ccLog::Warning(QString("[BIN] No associated sensor found for image '%1' in the file!").arg(image->getName()));
	}
}

CC_FILE_ERROR BinFilter::LoadFileV2(QFile& in, ccHObject& container, int flags, bool parallel, QWidget* parentWidget /*=nullptr*/)
{
	assert(in.isOpen());

	uint32_t binVersion = 20;
	if (in.read((char*)&binVersion, 4) < 0)
	{
		return CC_FERR_READING;
	}

	if (binVersion < 20) // should be superior to 2.0!
	{
		return CC_FERR_MALFORMED_FILE;
	}

	QString coordsFormat = ((flags & ccSerializableObject::DF_POINT_COORDS_64_BITS) ? "double" : "float");
	QString scalarFormat = ((flags & ccSerializableObject::DF_SCALAR_VAL_32_BITS) ? "float" : "double");
	ccLog::Print(QString("[BIN] Version %1.%2 (coords: %3 / scalar: %4)").arg(binVersion / 10).arg(binVersion % 10).arg(coordsFormat).arg(scalarFormat));

	if (ccObject::GetCurrentDBVersion() < binVersion)
	{
		ccLog::Error("This version of CloudCompare is too old and can't load this file, sorry");
		return CC_FERR_CONSOLE_ERROR;
	}

	// we read the first entity type
	CC_CLASS_ENUM classID = ccObject::ReadClassIDFromFile(in, static_cast<short>(binVersion));
	if (classID == CC_TYPES::OBJECT)
	{
		return CC_FERR_CONSOLE_ERROR;
	}

	// call the CC object factory
	ccHObject* root = ccHObject::New(classID);
	if (!root)
	{
		return CC_FERR_MALFORMED_FILE;
	}

	std::unique_ptr<ccProgressDialog> pDlg;
	if (parallel && parentWidget)
	{
		pDlg = std::make_unique<ccProgressDialog>(false, parentWidget);
		pDlg->setMethodTitle(QObject::tr("BIN file"));
		pDlg->setInfo(QObject::tr("Loading: %1").arg(in.fileName()));
		pDlg->setRange(0, 0);
		pDlg->show();
	}

	ccHObject::LoadingContext loadingContext(static_cast<short>(binVersion), flags);

	if (classID == CC_TYPES::CUSTOM_H_OBJECT)
	{
		// store seeking position
		size_t original_pos = in.pos();
		// we need to load it as plain ccCustomHobject
		root->fromFileNoChildren(in, loadingContext); // this will load it, should be pretty quick
		in.seek(original_pos);                        // back to the beginning of the file

		QString classId  = root->getMetaData("class_name").toString();
		QString pluginId = root->getMetaData("plugin_name").toString();

		// get rid of the previous ccCustomHobject instance
		delete root;
		root = nullptr;

		// try to get a new object from external factories
		ccHObject* new_child = ccHObject::New(pluginId, classId);
		if (new_child)
		{
			// found a plugin that can deserialize it
			root = new_child;
		}
		else
		{
			return CC_FERR_FILE_WAS_WRITTEN_BY_UNKNOWN_PLUGIN;
		}
	}

	bool success = false;

	if (parallel)
	{
		// concurrent call in a separate thread, so that the progress dialog keeps refreshing
		success = ccBackgroundTask::Run([&]()
		                                { return root->fromFile(in, loadingContext); });
	}
	else
	{
		success = root->fromFile(in, loadingContext);
	}

	IncompleteEntityLinkerContext linkerContext(root, loadingContext);

	if (!success)
	{
		// delete root; //DGM: can't delete it, too dangerous (bad pointers ;)
		ccLog::Error(QString("Failed to read file (file position: %1 / %2").arg(in.pos()).arg(in.size()));

		if (!root->isA(CC_TYPES::HIERARCHY_OBJECT) || root->getChildrenNumber() != 0)
		{
			ContinueAfterError(linkerContext, true);
		}

		if (!linkerContext.forceLoadAfterError)
		{
			return CC_FERR_CONSOLE_ERROR;
		}
	}

	// re-link incomplete objects (and check errors)
	std::unique_ptr<ccHObject> orphans(new ccHObject("Orphans (CORRUPTED FILE)"));

	for (auto it = loadingContext.incompleteEntities.begin(); root && it != loadingContext.incompleteEntities.end(); ++it)
	{
		// initialize the linker context for the current incomplete entity
		{
			ccHObject* incompleteEntity = static_cast<ccHObject*>(it.key());
			assert(incompleteEntity);

			std::vector<ccSerializableObject::LoadingContext::Dependency>& dependencies = it.value();
			if (dependencies.empty())
			{
				// means the entity has already been processed or has been deleted already (see IncompleteEntityLinkerContext::deleteIncompleteEntity())
				continue;
			}

			linkerContext.setIncompleteEntity(incompleteEntity, dependencies); // dependencies will be copied
			dependencies.clear();                                              // remove the dependencies so as to mark the entity as 'processed'
			                                                                   //(important for IncompleteEntityLinkerContext::deleteIncompleteEntity())
		}

		if (linkerContext.result == CC_FERR_MALFORMED_FILE)
		{
			// we don't have the time to check dependencies, we just remove the incomplete entity from the tree
			assert(!linkerContext.forceLoadAfterError);
			linkerContext.deleteIncompleteEntity();
			continue;
		}

		// Replace the large if/else chain by calls to the new handlers
		if (linkerContext.incompleteEntity->isA(CC_TYPES::MESH_GROUP))
		{
			HandleMeshGroup(linkerContext);
		}
		else if (linkerContext.incompleteEntity->isA(CC_TYPES::SUB_MESH))
		{
			HandleSubMesh(linkerContext);
		}
		else if (linkerContext.incompleteEntity->isA(CC_TYPES::MESH) || linkerContext.incompleteEntity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			HandleMeshOrPrimitive(linkerContext, orphans.get());
		}
		else if (linkerContext.incompleteEntity->isKindOf(CC_TYPES::POLY_LINE))
		{
			HandlePolyline(linkerContext);
		}
		else if (linkerContext.incompleteEntity->isKindOf(CC_TYPES::SENSOR))
		{
			HandleSensor(linkerContext);
		}
		else if (linkerContext.incompleteEntity->isA(CC_TYPES::LABEL_2D))
		{
			HandleLabel2D(linkerContext);
		}
		else if (linkerContext.incompleteEntity->isA(CC_TYPES::FACET))
		{
			HandleFacet(linkerContext);
		}
		else if (linkerContext.incompleteEntity->isA(CC_TYPES::IMAGE))
		{
			HandleImage(linkerContext);
		}
		else
		{
			assert(false);
			ccLog::Warning(QString("[BIN] Unexpected entity type (%1) for entity '%2' in the file!").arg(linkerContext.incompleteEntity->getClassID()).arg(linkerContext.incompleteEntity->getName()));
		}

		// if we still have an incomplete entity at this point, it means that we were able to fix all its dependencies
		if (linkerContext.incompleteEntity)
		{
			const ccShiftedObject* shifted = ccHObjectCaster::ToShifted(linkerContext.incompleteEntity);
			if (shifted)
			{
				// it may be interesting to re-use the Global Shift when loading other files
				ccGlobalShiftManager::StoreShift(shifted->getGlobalShift(), shifted->getGlobalScale());

				// TODO: we should also check that other entities with global shift not too far away
				// have not already been loaded. In which case we should 'translate' the current entity?
			}
		}
	}

	if (linkerContext.result == CC_FERR_NO_ERROR && linkerContext.hasBrokenDependencies)
	{
		// minor error
		linkerContext.result = CC_FERR_BROKEN_DEPENDENCY_ERROR;
	}

	if (root)
	{
		if (root->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			// transfer children to container
			root->transferChildren(container, true);
			delete root;
			root = nullptr;
		}
		else
		{
			container.addChild(root);
		}
	}

	// orphans
	if (orphans && orphans->getChildrenNumber() != 0)
	{
		orphans->setEnabled(false);
		container.addChild(orphans.release());
	}

	return linkerContext.result;
}

CC_FILE_ERROR BinFilter::LoadFileV1(QFile& in, ccHObject& container, unsigned nbScansTotal, const LoadParameters& parameters)
{
	ccLog::Print("[BIN] Version 1.0");

	if (nbScansTotal > 99)
	{
		if (QMessageBox::question(nullptr, QString("Oops"), QString("Hum, do you really expect to load %1 point clouds?").arg(nbScansTotal), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			return CC_FERR_WRONG_FILE_TYPE;
	}
	else if (nbScansTotal == 0)
	{
		return CC_FERR_NO_LOAD;
	}

	std::unique_ptr<ccProgressDialog> pDlg;
	if (parameters.parentWidget)
	{
		pDlg = std::make_unique<ccProgressDialog>(true, parameters.parentWidget);
		pDlg->setMethodTitle(QObject::tr("Open Bin file (old style)"));
		pDlg->setAutoClose(false);
	}

	for (unsigned k = 0; k < nbScansTotal; k++)
	{
		HeaderFlags header;
		unsigned    nbOfPoints = 0;
		if (ReadEntityHeader(in, nbOfPoints, header) < 0)
		{
			return CC_FERR_READING;
		}

		// Console::print("[BinFilter::loadModelFromBinaryFile] Entity %i : %i points, color=%i, norms=%i, dists=%i\n",k,nbOfPoints,color,norms,distances);

		if (nbOfPoints == 0)
		{
			// Console::print("[BinFilter::loadModelFromBinaryFile] rien a faire !\n");
			continue;
		}

		// progress for this cloud
		CCCoreLib::NormalizedProgress nprogress(pDlg.get(), nbOfPoints);
		if (pDlg)
		{
			pDlg->reset();
			pDlg->setInfo(QObject::tr("cloud %1/%2 (%3 points)").arg(k + 1).arg(nbScansTotal).arg(nbOfPoints));
			pDlg->start();
			QApplication::processEvents();
		}

		// Cloud name
		char cloudName[256] = "unnamed";
		if (header.name)
		{
			for (int i = 0; i < 256; ++i)
			{
				if (in.read(cloudName + i, 1) < 0)
				{
					// Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the cloud name!\n");
					return CC_FERR_READING;
				}
				if (cloudName[i] == 0)
				{
					break;
				}
			}
			// we force the end of the name in case it is too long!
			cloudName[255] = 0;
		}
		else
		{
			snprintf(cloudName, 256, "unnamed - Cloud #%u", k);
		}

		// Cloud name
		char sfName[1024] = "unnamed";
		if (header.sfName)
		{
			for (int i = 0; i < 1024; ++i)
			{
				if (in.read(sfName + i, 1) < 0)
				{
					// Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the cloud name!\n");
					return CC_FERR_READING;
				}
				if (sfName[i] == 0)
					break;
			}
			// we force the end of the name in case it is too long!
			sfName[1023] = 0;
		}
		else
		{
			strncpy(sfName, "Loaded scalar field", 1024);
		}

		// Creation
		ccPointCloud* loadedCloud = new ccPointCloud(cloudName);
		if (!loadedCloud)
		{
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		unsigned fileChunkPos  = 0;
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

		CCCoreLib::ScalarField::Shared loadedCloudSF;
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
		unsigned parts    = 0;

		const ScalarType FORMER_HIDDEN_POINTS = static_cast<ScalarType>(-1.0);

		// read the file
		for (unsigned i = 0; i < nbOfPoints; ++i)
		{
			if (lineRead == fileChunkPos + fileChunkSize)
			{
				if (loadedCloudSF)
				{
					loadedCloudSF->computeMinAndMax();
					loadedCloudSF = nullptr;
				}

				// create a new cloud
				container.addChild(loadedCloud);
				fileChunkPos     = lineRead;
				fileChunkSize    = std::min(nbOfPoints - lineRead, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
				QString partName = QString("%1.%2").arg(cloudName).arg(parts);
				loadedCloud      = new ccPointCloud(partName);
				if (!loadedCloud->reserveThePointsTable(fileChunkSize))
				{
					delete loadedCloud;
					return CC_FERR_NOT_ENOUGH_MEMORY;
				}

				if (header.colors)
				{
					if (loadedCloud->reserveTheRGBTable())
					{
						loadedCloud->showColors(true);
					}
					else
					{
						ccLog::Warning(QString("Failed to allocate RGB colors on cloud '%1'").arg(loadedCloud->getName()));
						delete loadedCloud;
						return CC_FERR_NOT_ENOUGH_MEMORY;
					}
				}
				if (header.normals)
				{
					if (loadedCloud->reserveTheNormsTable())
					{
						loadedCloud->showNormals(true);
					}
					else
					{
						ccLog::Warning(QString("Failed to allocate normals on cloud '%1'").arg(loadedCloud->getName()));
						delete loadedCloud;
						return CC_FERR_NOT_ENOUGH_MEMORY;
					}
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
				// Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity point !\n",k);
				return CC_FERR_READING;
			}
			loadedCloud->addPoint(CCVector3::fromArray(Pf));

			if (header.colors)
			{
				ccColor::Rgb C;
				if (in.read((char*)C.rgb, sizeof(ColorCompType) * 3) < 0)
				{
					// Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity colors !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addColor(C);
			}

			if (header.normals)
			{
				CCVector3 N;
				if (in.read((char*)N.u, sizeof(float) * 3) < 0)
				{
					// Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity norms !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addNorm(N);
			}

			if (header.scalarField)
			{
				double D;
				if (in.read((char*)&D, sizeof(double)) < 0)
				{
					// Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity distance!\n",k);
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

			// replace HIDDEN_VALUES by NAN_VALUES
			for (unsigned i = 0; i < loadedCloudSF->currentSize(); ++i)
			{
				if (loadedCloudSF->getValue(i) == FORMER_HIDDEN_POINTS)
					loadedCloudSF->setValue(i, CCCoreLib::NAN_VALUE);
			}
			loadedCloudSF->computeMinAndMax();

			loadedCloud->setCurrentDisplayedScalarField(loadedCloud->getCurrentInScalarFieldIndex());
			loadedCloud->showSF(true);
		}

		container.addChild(loadedCloud);
	}

	return CC_FERR_NO_ERROR;
}
