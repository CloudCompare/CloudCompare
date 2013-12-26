//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "BinFilter.h"

//Qt
#include <QMessageBox>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QtConcurrentRun>

//CCLib
#include <ScalarField.h>

//qCC_db
#include <ccFlags.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccMesh.h>
#include <ccSubMesh.h>
#include <ccPolyline.h>
#include <ccMaterialSet.h>
#include <cc2DLabel.h>
#include <ccFacet.h>

//system
#include <set>
#include <assert.h>
#if defined(CC_WINDOWS)
#include <Windows.h>
#else
#include <time.h>
#include <unistd.h>
#endif

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
	if (in.read((char*)&ptsCount,4)<0)
		return -1;
	numberOfPoints = (unsigned)ptsCount;

	//flags (colors, etc.)
	uint8_t flag;
	if (in.read((char*)&flag,1)<0)
		return -1;

	header.flags.fromByte((unsigned char)flag);
	//assert(header.bit1 == true); //should always be 0!

	return 0;
}

static QFile* s_file = 0;
static int s_flags = 0;
static ccHObject* s_container = 0;

CC_FILE_ERROR _LoadFileV2()
{
	return (s_file && s_container ? BinFilter::LoadFileV2(*s_file,*s_container,s_flags) : CC_FERR_BAD_ARGUMENT);
}

CC_FILE_ERROR _SaveFileV2()
{
	return (s_file && s_container ? BinFilter::SaveFileV2(*s_file,s_container) : CC_FERR_BAD_ARGUMENT);
}

CC_FILE_ERROR BinFilter::saveToFile(ccHObject* root, const char* filename)
{
	if (!root || !filename)
		return CC_FERR_BAD_ARGUMENT;

	QFile out(filename);
	if (!out.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	QProgressDialog pDlg(QString("Please wait... saving in progress"),QString(),0,0);
	pDlg.setWindowTitle("BIN file");
	pDlg.setModal(true);
	pDlg.show();

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
		pDlg.setValue(pDlg.value()+1);
		QApplication::processEvents();
	}
	
	s_file = 0;
	s_container = 0;

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
		firstBytes[3] = 48+flags; //48 = ASCII("0")
	}

	if (out.write(firstBytes,4)<0)
		return CC_FERR_WRITING;

	// Current BIN file version
	uint32_t binVersion_u32 = static_cast<uint32_t>(ccObject::GetCurrentDBVersion());
	if (out.write((char*)&binVersion_u32,4)<0)
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
		std::set<const ccHObject*> dependencies;
		if (currentObject->isA(CC_MESH) || currentObject->isKindOf(CC_PRIMITIVE))
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
		else if (currentObject->isA(CC_SUB_MESH))
		{
			dependencies.insert(currentObject->getParent());
		}
		else if (currentObject->isKindOf(CC_POLY_LINE))
		{
			CCLib::GenericIndexedCloudPersist* cloud = static_cast<ccPolyline*>(currentObject)->getAssociatedCloud();
			ccPointCloud* pc = dynamic_cast<ccPointCloud*>(cloud);
			if (pc)
				dependencies.insert(pc);
			else
				ccLog::Warning(QString("[BIN] Poyline '%1' is associated to an unhandled vertices structure?!").arg(currentObject->getName()));
		}
		else if (currentObject->isA(CC_2D_LABEL))
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(currentObject);
			for (unsigned i=0;i<label->size();++i)
			{
				const cc2DLabel::PickedPoint& pp = label->getPoint(i);
				dependencies.insert(pp.cloud);
			}
		}
		else if (currentObject->isA(CC_FACET))
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

		for (std::set<const ccHObject*>::const_iterator it = dependencies.begin(); it != dependencies.end(); ++it)
		{
			if (!object->find((*it)->getUniqueID()))
			{
				ccLog::Warning(QString("[BIN] Dependency broken: entity '%1' must also be in selection in order to save '%2'").arg((*it)->getName()).arg(currentObject->getName()));
				result = CC_FERR_BROKEN_DEPENDENCY_ERROR;
			}
		}
		//release some memory...
		dependencies.clear();

		for (unsigned i=0; i<currentObject->getChildrenNumber(); ++i)
			toCheck.push_back(currentObject->getChild(i));
	}

	if (result == CC_FERR_NO_ERROR)
		if (!object->toFile(out))
			result = CC_FERR_CONSOLE_ERROR;

	out.close();

	return result;
}

CC_FILE_ERROR BinFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
	ccLog::Print("[BIN] Opening file '%s'...",filename);


	//opening file
	QFile in(filename);
	if (!in.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	uint32_t firstBytes = 0;
	if (in.read((char*)&firstBytes,4)<0)
		return CC_FERR_READING;
	bool v1 = (strncmp((char*)&firstBytes,"CCB",3) != 0);

	if (v1)
	{
		return LoadFileV1(in,container,static_cast<unsigned>(firstBytes),alwaysDisplayLoadDialog); //firstBytes == number of scans for V1 files!
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
		//	QMessageBox::information(0, QString("Wrong version"), QString("This file has been generated with the standard 'float' version!\nAt this time it cannot be read with the 'double' version."),QMessageBox::Ok);
		//	return CC_FERR_WRONG_FILE_TYPE;
		//}
		//else if (sizeof(PointCoordinateType) == 4 && strncmp((char*)&firstBytes,"CCB2",4) != 0)
		//{
		//	QMessageBox::information(0, QString("Wrong version"), QString("This file has been generated with the new 'double' version!\nAt this time it cannot be read with the standard 'float' version."),QMessageBox::Ok);
		//	return CC_FERR_WRONG_FILE_TYPE;
		//}

		if (alwaysDisplayLoadDialog)
		{
			QProgressDialog pDlg(QString("Loading: %1").arg(QFileInfo(filename).fileName()),QString(),0,0/*static_cast<int>(in.size())*/);
			pDlg.setWindowTitle("BIN file");
			pDlg.show();

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
				if (alwaysDisplayLoadDialog)
				{
					pDlg.setValue(pDlg.value()+1);
					//pDlg.setValue(static_cast<int>(in.pos())); //DGM: in fact, the file reading part is just half of the work!
					QApplication::processEvents();
				}
			}
	
			s_file = 0;
			s_container = 0;

			return future.result();
		}
		else
		{
			return BinFilter::LoadFileV2(in,container,flags);
		}
	}
}

CC_FILE_ERROR BinFilter::LoadFileV2(QFile& in, ccHObject& container, int flags)
{
	assert(in.isOpen());

	uint32_t binVersion = 20;
	if (in.read((char*)&binVersion,4) < 0)
		return CC_FERR_READING;

	if (binVersion < 20) //should be superior to 2.0!
		return CC_FERR_MALFORMED_FILE;

	QString coordsFormat = (flags & ccSerializableObject::DF_POINT_COORDS_64_BITS ? "double" : "float");
	QString scalarFormat = (flags & ccSerializableObject::DF_SCALAR_VAL_32_BITS ? "float" : "double");
	ccLog::Print(QString("[BIN] Version %1.%2 (coords: %3 / scalar: %4)").arg(binVersion/10).arg(binVersion%10).arg(coordsFormat).arg(scalarFormat));

	//we keep track of the last unique ID before load
	unsigned lastUniqueIDBeforeLoad = ccObject::GetLastUniqueID();

	//we read first entity type
	unsigned classID = 0;
	if (!ccObject::ReadClassIDFromFile(classID, in, static_cast<short>(binVersion)))
		return CC_FERR_CONSOLE_ERROR;

	ccHObject* root = ccHObject::New(classID);
	if (!root)
		return CC_FERR_MALFORMED_FILE;

	if (!root->fromFile(in,static_cast<short>(binVersion),flags))
	{
		//DGM: can't delete it, too dangerous (bad pointers ;)
		//delete root;
		return CC_FERR_CONSOLE_ERROR;
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//re-link objects (and check errors)
	bool checkErrors = true;
	ccHObject* orphans = new ccHObject("Orphans (CORRUPTED FILE)");;
	ccHObject::Container toCheck;
	toCheck.push_back(root);
	while (!toCheck.empty())
	{
		ccHObject* currentObject = toCheck.back();
		toCheck.pop_back();

		assert(currentObject);
		//we check objects that have links to other entities (meshes, polylines, etc.)
		if (currentObject->isKindOf(CC_MESH))
		{
			//specific case: mesh groups are deprecated!
			if (currentObject->isA(CC_MESH_GROUP))
			{
				//TODO
				ccLog::Warning(QString("Mesh groups are deprecated! Entity %1 should be ignored...").arg(currentObject->getName()));
			}
			else if (currentObject->isA(CC_SUB_MESH))
			{
				ccSubMesh* subMesh = ccHObjectCaster::ToSubMesh(currentObject);

				//normally, the associated mesh should be the sub-mesh's parent!
				//however we have its ID so we will look for it just to be sure
				intptr_t meshID = (intptr_t)subMesh->getAssociatedMesh();
				if (meshID > 0)
				{
					ccHObject* mesh = root->find(static_cast<int>(meshID));
					if (mesh && mesh->isA(CC_MESH))
					{
						subMesh->setAssociatedMesh(ccHObjectCaster::ToMesh(mesh));
					}
					else
					{
						//we have a problem here ;)
						//normally, the associated mesh should be the sub-mesh's parent!
						if (subMesh->getParent() && subMesh->getParent()->isA(CC_MESH))
						{
							subMesh->setAssociatedMesh(ccHObjectCaster::ToMesh(subMesh->getParent()));
						}
						else
						{
							subMesh->setAssociatedMesh(0);
							//DGM: can't delete it, too dangerous (bad pointers ;)
							//delete subMesh;
							ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find associated mesh (ID=%1) for sub-mesh '%2' in the file!").arg(meshID).arg(subMesh->getName()));
							return CC_FERR_MALFORMED_FILE;
						}
					}
				}
			}
			else if (currentObject->isA(CC_MESH) || currentObject->isKindOf(CC_PRIMITIVE)) //CC_MESH or CC_PRIMITIVE!
			{
				ccMesh* mesh = ccHObjectCaster::ToMesh(currentObject);
				assert(mesh);

				//vertices
				intptr_t cloudID = (intptr_t)mesh->getAssociatedCloud();
				if (cloudID > 0)
				{
					ccHObject* cloud = root->find(static_cast<int>(cloudID));
					if (cloud && cloud->isKindOf(CC_POINT_CLOUD))
					{
						mesh->setAssociatedCloud(ccHObjectCaster::ToGenericPointCloud(cloud));
					}
					else
					{
						//we have a problem here ;)
						mesh->setAssociatedCloud(0);
						if (mesh->getMaterialSet())
							mesh->setMaterialSet(0,false);
						//DGM: can't delete it, too dangerous (bad pointers ;)
						//delete mesh;
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find vertices (ID=%1) for mesh '%2' in the file!").arg(cloudID).arg(mesh->getName()));
						return CC_FERR_MALFORMED_FILE;
					}
				}

				//materials
				ccHObject* materials = 0;
				intptr_t matSetID = (intptr_t)mesh->getMaterialSet();
				if (matSetID > 0)
				{
					materials = root->find(static_cast<int>(matSetID));
					if (materials && materials->isA(CC_MATERIAL_SET))
						mesh->setMaterialSet(static_cast<ccMaterialSet*>(materials),false);
					else
					{
						//we have a (less severe) problem here ;)
						mesh->setMaterialSet(0,false);
						mesh->showMaterials(false);
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find shared materials set (ID=%1) for mesh '%2' in the file!").arg(matSetID).arg(mesh->getName()));
						result = CC_FERR_BROKEN_DEPENDENCY_ERROR;

						//add it to the 'orphans' set
						if (materials)
							orphans->addChild(materials);
						materials = 0;
					}
				}
				//per-triangle normals
				ccHObject* triNormsTable = 0;
				intptr_t triNormsTableID = (intptr_t)mesh->getTriNormsTable();
				if (triNormsTableID > 0)
				{
					triNormsTable = root->find(static_cast<int>(triNormsTableID));
					if (triNormsTable && triNormsTable->isA(CC_NORMAL_INDEXES_ARRAY))
					{
						mesh->setTriNormsTable(static_cast<NormsIndexesTableType*>(triNormsTable),false);
					}
					else
					{
						//we have a (less severe) problem here ;)
						mesh->setTriNormsTable(0,false);
						mesh->showTriNorms(false);
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find shared normals (ID=%1) for mesh '%2' in the file!").arg(triNormsTableID).arg(mesh->getName()));
						result = CC_FERR_BROKEN_DEPENDENCY_ERROR;

						//add it to the 'orphans' set
						if (triNormsTable)
							orphans->addChild(triNormsTable);
						triNormsTable = 0;
					}
				}
				//per-triangle texture coordinates
				ccHObject* texCoordsTable = 0;
				intptr_t texCoordArrayID = (intptr_t)mesh->getTexCoordinatesTable();
				if (texCoordArrayID > 0)
				{
					texCoordsTable = root->find(static_cast<int>(texCoordArrayID));
					if (texCoordsTable && texCoordsTable->isA(CC_TEX_COORDS_ARRAY))
						mesh->setTexCoordinatesTable(static_cast<TextureCoordsContainer*>(texCoordsTable),false);
					else
					{
						//we have a (less severe) problem here ;)
						mesh->setTexCoordinatesTable(0,false);
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find shared texture coordinates (ID=%1) for mesh '%2' in the file!").arg(texCoordArrayID).arg(mesh->getName()));
						result = CC_FERR_BROKEN_DEPENDENCY_ERROR;

						//add it to the 'orphans' set
						if (texCoordsTable)
							orphans->addChild(texCoordsTable);
						texCoordsTable = 0;
					}
				}

				if (checkErrors)
				{
					ccGenericPointCloud* pc = mesh->getAssociatedCloud();
					unsigned faceCount = mesh->size();
					unsigned vertCount = pc->size();
					for (unsigned i=0; i<faceCount; ++i)
					{
						const CCLib::TriangleSummitsIndexes* tri = mesh->getTriangleIndexes(i);
						if (	tri->i1 >= vertCount
							||	tri->i2 >= vertCount
							||	tri->i3 >= vertCount )
						{
							ccLog::Warning(QString("[BinFilter::loadFileV2] File is corrupted: missing vertices for mesh '%1'!").arg(mesh->getName()));

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
							mesh->setMaterialSet(0,false);
							mesh->setTriNormsTable(0,false);
							mesh->setTexCoordinatesTable(0,false);
							if (mesh->getParent())
								mesh->getParent()->removeChild(mesh);
							mesh = 0;

							break;
						}
					}
				}
			}
		}
		else if (currentObject->isKindOf(CC_POLY_LINE))
		{
			ccPolyline* poly = static_cast<ccPolyline*>(currentObject);
			intptr_t cloudID = (intptr_t)poly->getAssociatedCloud();
			ccHObject* cloud = root->find(static_cast<int>(cloudID));
			if (cloud && cloud->isKindOf(CC_POINT_CLOUD))
				poly->setAssociatedCloud(ccHObjectCaster::ToGenericPointCloud(cloud));
			else
			{
				//we have a problem here ;)
				poly->setAssociatedCloud(0);
				//DGM: can't delete it, too dangerous (bad pointers ;)
				//delete root;
				ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find vertices (ID=%1) for polyline '%2' in the file!").arg(cloudID).arg(poly->getName()));
				return CC_FERR_MALFORMED_FILE;
			}
		}
		else if (currentObject->isA(CC_2D_LABEL))
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(currentObject);
			std::vector<cc2DLabel::PickedPoint> correctedPickedPoints;
			//we must check all label 'points'!
			for (unsigned i=0;i<label->size();++i)
			{
				const cc2DLabel::PickedPoint& pp = label->getPoint(i);
				intptr_t cloudID = (intptr_t)pp.cloud;
				ccHObject* cloud = root->find(static_cast<int>(cloudID));
				if (cloud && cloud->isKindOf(CC_POINT_CLOUD))
				{
					ccGenericPointCloud* genCloud = ccHObjectCaster::ToGenericPointCloud(cloud);
					assert(genCloud->size()>pp.index);
					correctedPickedPoints.push_back(cc2DLabel::PickedPoint(genCloud,pp.index));
				}
				else
				{
					//we have a problem here ;)
					ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find cloud (ID=%1) associated to label '%2' in the file!").arg(cloudID).arg(label->getName()));
					if (label->getParent())
						label->getParent()->removeChild(label);
					if (!label->getFlagState(CC_FATHER_DEPENDENT))
					{
						//DGM: can't delete it, too dangerous (bad pointers ;)
						//delete label;
					}
					label=0;
					break;
				}
			}

			if (label) //correct label data
			{
				assert(correctedPickedPoints.size() == label->size());
				bool visible = label->isVisible();
				QString originalName(label->getRawName());
				label->clear();
				for (unsigned i=0;i<correctedPickedPoints.size();++i)
					label->addPoint(correctedPickedPoints[i].cloud,correctedPickedPoints[i].index);
				label->setVisible(visible);
				label->setName(originalName);
			}
		}
		else if (currentObject->isA(CC_FACET))
		{
			ccFacet* facet = ccHObjectCaster::ToFacet(currentObject);

			//origin points
			{
				intptr_t cloudID = (intptr_t)facet->getOriginPoints();
				if (cloudID > 0)
				{
					ccHObject* cloud = root->find(static_cast<int>(cloudID));
					if (cloud && cloud->isA(CC_POINT_CLOUD))
						facet->setOriginPoints(ccHObjectCaster::ToPointCloud(cloud));
					else
					{
						//we have a problem here ;)
						facet->setOriginPoints(0);
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find origin points (ID=%1) for facet '%2' in the file!").arg(cloudID).arg(facet->getName()));
					}
				}
			}
			//contour points
			{
				intptr_t cloudID = (intptr_t)facet->getContourVertices();
				if (cloudID > 0)
				{
					ccHObject* cloud = root->find(static_cast<int>(cloudID));
					if (cloud && cloud->isA(CC_POINT_CLOUD))
						facet->setContourVertices(ccHObjectCaster::ToPointCloud(cloud));
					else
					{
						//we have a problem here ;)
						facet->setContourVertices(0);
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find contour points (ID=%1) for facet '%2' in the file!").arg(cloudID).arg(facet->getName()));
					}
				}
			}
			//contour polyline
			{
				intptr_t polyID = (intptr_t)facet->getContour();
				if (polyID > 0)
				{
					ccHObject* poly = root->find(static_cast<int>(polyID));
					if (poly && poly->isA(CC_POLY_LINE))
						facet->setContour(ccHObjectCaster::ToPolyline(poly));
					else
					{
						//we have a problem here ;)
						facet->setContourVertices(0);
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find contour polyline (ID=%1) for facet '%2' in the file!").arg(polyID).arg(facet->getName()));
					}
				}
			}
			//polygon mesh
			{
				intptr_t polyID = (intptr_t)facet->getPolygon();
				if (polyID > 0)
				{
					ccHObject* poly = root->find(static_cast<int>(polyID));
					if (poly && poly->isA(CC_MESH))
					{
						facet->setPolygon(ccHObjectCaster::ToMesh(poly));
					}
					else
					{
						//we have a problem here ;)
						facet->setContourVertices(0);
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find polygon mesh (ID=%1) for facet '%2' in the file!").arg(polyID).arg(facet->getName()));
					}
				}
			}
		}

		if (currentObject)
			for (unsigned i=0;i<currentObject->getChildrenNumber();++i)
				toCheck.push_back(currentObject->getChild(i));
	}

	//update 'unique IDs'
	toCheck.push_back(root);
	while (!toCheck.empty())
	{
		ccHObject* currentObject = toCheck.back();
		toCheck.pop_back();

		currentObject->setUniqueID(lastUniqueIDBeforeLoad+currentObject->getUniqueID());

		for (unsigned i=0;i<currentObject->getChildrenNumber();++i)
			toCheck.push_back(currentObject->getChild(i));
	}

	if (root->isA(CC_HIERARCHY_OBJECT))
	{
		//transfer children to container
		root->transferChildren(container,true);
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
			orphans = 0;
		}
	}

	return result;
}

CC_FILE_ERROR BinFilter::LoadFileV1(QFile& in, ccHObject& container, unsigned nbScansTotal, bool alwaysDisplayLoadDialog)
{
	ccLog::Print("[BIN] Version 1.0");

	if (nbScansTotal > 99)
	{
		if (QMessageBox::question(0, QString("Oups"), QString("Hum, do you really expect %1 point clouds?").arg(nbScansTotal), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			return CC_FERR_WRONG_FILE_TYPE;
	}
	else if (nbScansTotal == 0)
	{
		return CC_FERR_NO_LOAD;
	}

	ccProgressDialog pdlg(true);
	pdlg.setMethodTitle("Open Bin file (old style)");

	for (unsigned k=0; k<nbScansTotal; k++)
	{
		HeaderFlags header;
		unsigned nbOfPoints=0;
		if (ReadEntityHeader(in,nbOfPoints,header) < 0)
			return CC_FERR_READING;

		//Console::print("[BinFilter::loadModelFromBinaryFile] Entity %i : %i points, color=%i, norms=%i, dists=%i\n",k,nbOfPoints,color,norms,distances);

		if (nbOfPoints == 0)
		{
			//Console::print("[BinFilter::loadModelFromBinaryFile] rien a faire !\n");
			continue;
		}

		//progress for this cloud
		CCLib::NormalizedProgress* nprogress = 0;
		if (alwaysDisplayLoadDialog)
		{
			nprogress = new CCLib::NormalizedProgress(&pdlg,nbOfPoints);
			pdlg.reset();
			char buffer[256];
			sprintf(buffer,"cloud %u/%u (%u points)",k+1,nbScansTotal,nbOfPoints);
			pdlg.setInfo(buffer);
			pdlg.start();
			QApplication::processEvents();
		}

		//Cloud name
		char cloudName[256] = "unnamed";
		if (header.name)
		{
			for (int i=0; i<256; ++i)
			{
				if (in.read(cloudName+i,1)<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the cloud name!\n");
					return CC_FERR_READING;
				}
				if (cloudName[i] == 0)
					break;
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
				if (in.read(sfName+i,1)<0)
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
		if (!loadedCloud)
			return CC_FERR_NOT_ENOUGH_MEMORY;

		unsigned fileChunkPos = 0;
		unsigned fileChunkSize = std::min(nbOfPoints,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);

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
			loadedCloud->enableScalarField();

		unsigned lineRead = 0;
		int parts = 0;

		const ScalarType FORMER_HIDDEN_POINTS = (ScalarType)-1.0;

		//lecture du fichier
		for (unsigned i=0; i<nbOfPoints; ++i)
		{
			if (lineRead == fileChunkPos+fileChunkSize)
			{
				if (header.scalarField)
					loadedCloud->getCurrentInScalarField()->computeMinAndMax();

				container.addChild(loadedCloud);
				fileChunkPos = lineRead;
				fileChunkSize = std::min(nbOfPoints-lineRead,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
				char partName[64];
				++parts;
				sprintf(partName,"%s.part_%i",cloudName,parts);
				loadedCloud = new ccPointCloud(partName);
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
					loadedCloud->enableScalarField();
			}

			float Pf[3];
			if (in.read((char*)Pf,sizeof(float)*3)<0)
			{
				//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity point !\n",k);
				return CC_FERR_READING;
			}
			loadedCloud->addPoint(CCVector3::fromArray(Pf));

			if (header.colors)
			{
				unsigned char C[3];
				if (in.read((char*)C,sizeof(colorType)*3)<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity colors !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addRGBColor(C);
			}

			if (header.normals)
			{
				CCVector3 N;
				if (in.read((char*)N.u,sizeof(float)*3)<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity norms !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addNorm(N.u);
			}

			if (header.scalarField)
			{
				double D;
				if (in.read((char*)&D,sizeof(double))<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity distance!\n",k);
					return CC_FERR_READING;
				}
				ScalarType d = static_cast<ScalarType>(D);
				loadedCloud->setPointScalarValue(i,d);
			}

			lineRead++;

			if (nprogress && !nprogress->oneStep())
			{
				loadedCloud->resize(i+1-fileChunkPos);
				k=nbScansTotal;
				i=nbOfPoints;
			}
		}

		if (nprogress)
		{
			pdlg.stop();
			QApplication::processEvents();
			delete nprogress;
			nprogress = 0;
		}

		if (header.scalarField)
		{
			CCLib::ScalarField* sf = loadedCloud->getCurrentInScalarField();
			assert(sf);
			sf->setName(sfName);

			//replace HIDDEN_VALUES by NAN_VALUES
			for (unsigned i=0; i<sf->currentSize(); ++i)
			{
				if (sf->getValue(i) == FORMER_HIDDEN_POINTS)
					sf->setValue(i,NAN_VALUE);
			}
			sf->computeMinAndMax();

			loadedCloud->setCurrentDisplayedScalarField(loadedCloud->getCurrentInScalarFieldIndex());
			loadedCloud->showSF(true);
		}

		container.addChild(loadedCloud);
	}

	return CC_FERR_NO_ERROR;
}
