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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//
#include "BinFilter.h"

//Qt
#include <QMessageBox>
#include <QApplication>

//CCLib
#include <ScalarField.h>
#include <CCMiscTools.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccGenericMesh.h>
#include <ccPolyline.h>
#include <ccMaterialSet.h>
#include <cc2DLabel.h>

#include <assert.h>

// Current BIN file version
/** Versions:
- 'original' version (file starts by the number of clouds - no header)
* V1.0 = prior to 05/04/2012 = old version
- 'new' evolutive version, starts by 4 bytes ("CCB2")
* V2.0 - 05/04/2012 - upgrade to serialized version
* V2.1 - 07/02/2012 - points & 2D labels upgraded
* V2.2 - 11/26/2012 - object name is now a QString
**/
static unsigned s_currentBinVersion = 22; //2.2

int BinFilter::ReadEntityHeader(QFile& in, unsigned &numberOfPoints, HeaderFlags& header)
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
	assert(header.bit1 == true); //should always be 0!

	return 0;
}

CC_FILE_ERROR BinFilter::saveToFile(ccHObject* root, const char* filename)
{
	if (!root || !filename)
		return CC_FERR_BAD_ARGUMENT;

	QFile out(filename);
	if (!out.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	//header
	char firstBytes[5] = "CCB2";
	if (out.write(firstBytes,4)<0)
		return CC_FERR_WRITING;

	//version
	uint32_t binVersion = (uint32_t)s_currentBinVersion;
	if (out.write((char*)&binVersion,4)<0)
		return CC_FERR_WRITING;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//we check if all linked entities are in the sub tree we are going to save
	//(such as vertices for a mesh!)
	ccHObject::Container toCheck;
	toCheck.push_back(root);
	while (!toCheck.empty())
	{
		ccHObject* currentObject = toCheck.back();
		assert(currentObject);
		toCheck.pop_back();

		//we check objects that have links to other entities (meshes, polylines, etc.)
		std::vector<const ccHObject*> dependencies;
		if (currentObject->isKindOf(CC_MESH))
		{
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(currentObject);
			if (mesh->getAssociatedCloud())
				dependencies.push_back(mesh->getAssociatedCloud());
			if (mesh->getMaterialSet())
				dependencies.push_back(mesh->getMaterialSet());
		}
		else if (currentObject->isKindOf(CC_POLY_LINE))
		{
			CCLib::GenericIndexedCloudPersist* cloud = static_cast<ccPolyline*>(currentObject)->getAssociatedCloud();
			//TODO: dirty!
			dependencies.push_back(dynamic_cast<ccPointCloud*>(cloud));
		}
		else if (currentObject->isA(CC_2D_LABEL))
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(currentObject);
			for (unsigned i=0;i<label->size();++i)
			{
				const cc2DLabel::PickedPoint& pp = label->getPoint(i);
				if (dependencies.empty() || pp.cloud != dependencies.back())
					dependencies.push_back(pp.cloud);
			}
		}

		while (!dependencies.empty())
		{
			if (!root->find(dependencies.back()->getUniqueID()))
			{
				ccLog::Warning(QString("Dependency broken: entity '%1' must also be in selection in order to save '%2'").arg(dependencies.back()->getName()).arg(currentObject->getName()));
				result = CC_FERR_BROKEN_DEPENDENCY_ERROR;
			}
			dependencies.pop_back();
		}

		for (unsigned i=0;i<currentObject->getChildrenNumber();++i)
			toCheck.push_back(currentObject->getChild(i));
	}

	if (result == CC_FERR_NO_ERROR)
		if (!root->toFile(out))
			result = CC_FERR_CONSOLE_ERROR;

	out.close();

	return result;
}

CC_FILE_ERROR BinFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	//ccConsole::Print("[BinFilter::loadFile] Opening binary file '%s'...\n",filename);

	//opening file
	QFile in(filename);
	if (!in.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	uint32_t firstBytes = 0;
	if (in.read((char*)&firstBytes,4)<0)
		return CC_FERR_READING;

	//old style?
	if (strncmp((char*)&firstBytes,"CCB2",4)!=0)
		return loadFileV1(in,container,(unsigned)firstBytes); //firstBytes==number of scans!

	//new style otherwise (serialized)
	return loadFileV2(in,container);
}

CC_FILE_ERROR BinFilter::loadFileV2(QFile& in, ccHObject& container)
{
	assert(in.isOpen());

	uint32_t binVersion = 20;
	if (in.read((char*)&binVersion,4)<0)
		return CC_FERR_READING;

	if (binVersion<20) //should be superior to 2.0!
		return CC_FERR_MALFORMED_FILE;

	ccProgressDialog pdlg(true);
	pdlg.setMethodTitle(qPrintable(QString("Open Bin file (V%1.%2)").arg(binVersion/10).arg(binVersion%10)));

	//we keep track of the last unique ID before load
	unsigned lastUniqueIDBeforeLoad = ccObject::GetLastUniqueID();

	//we read first entity type
	unsigned classID=0;
	if (!ccObject::ReadClassIDFromFile(classID, in, binVersion))
		return CC_FERR_CONSOLE_ERROR;

	ccHObject* root = ccHObject::New(classID);
	if (!root)
		return CC_FERR_MALFORMED_FILE;

	if (!root->fromFile(in,binVersion))
	{
		//DGM: can't delete it, too dangerous (bad pointers ;)
		//delete root;
		return CC_FERR_CONSOLE_ERROR;
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//re-link objects
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
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(currentObject);
			//vertices
			//special case: if the parent is a mesh group, then the job has already be done once and for all!
			if (!mesh->getParent() || !mesh->getParent()->isA(CC_MESH_GROUP))
			{
				intptr_t cloudID = (intptr_t)mesh->getAssociatedCloud();
				if (cloudID>0)
				{
					ccHObject* cloud = root->find(cloudID);
					if (cloud && cloud->isKindOf(CC_POINT_CLOUD))
						mesh->setAssociatedCloud(static_cast<ccGenericPointCloud*>(cloud));
					else
					{
						//we have a problem here ;)
						mesh->setAssociatedCloud(0);
						if (mesh->getMaterialSet())
							mesh->setMaterialSet(0,false);
						//DGM: can't delete it, too dangerous (bad pointers ;)
						//delete root;
						ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find vertices (ID=%1) for mesh '%2' in the file!").arg(cloudID).arg(mesh->getName()));
						return CC_FERR_MALFORMED_FILE;
					}
				}
			}
			//materials
			intptr_t matSetID = (intptr_t)mesh->getMaterialSet();
			if (matSetID>0)
			{
				ccHObject* materials = root->find(matSetID);
				if (materials && materials->isA(CC_MATERIAL_SET))
					mesh->setMaterialSet(static_cast<ccMaterialSet*>(materials),false);
				else
				{
					//we have a (less severe) problem here ;)
					mesh->setMaterialSet(0,false);
					mesh->showMaterials(false);
					ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find shared materials set (ID=%1) for mesh '%2' in the file!").arg(matSetID).arg(mesh->getName()));
					result = CC_FERR_BROKEN_DEPENDENCY_ERROR;
				}
			}
			//per-triangle normals
			intptr_t triNormsTableID = (intptr_t)mesh->getTriNormsTable();
			if (triNormsTableID>0)
			{
				ccHObject* triNormsTable = root->find(triNormsTableID);
				if (triNormsTable && triNormsTable->isA(CC_NORMAL_INDEXES_ARRAY))
					mesh->setTriNormsTable(static_cast<NormsIndexesTableType*>(triNormsTable),false);
				else
				{
					//we have a (less severe) problem here ;)
					mesh->setTriNormsTable(0,false);
					mesh->showTriNorms(false);
					ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find shared normals (ID=%1) for mesh '%2' in the file!").arg(matSetID).arg(mesh->getName()));
					result = CC_FERR_BROKEN_DEPENDENCY_ERROR;
				}
			}
			//per-triangle texture coordinates
			intptr_t texCoordArrayID = (intptr_t)mesh->getTexCoordinatesTable();
			if (texCoordArrayID>0)
			{
				ccHObject* texCoordsTable = root->find(texCoordArrayID);
				if (texCoordsTable && texCoordsTable->isA(CC_TEX_COORDS_ARRAY))
					mesh->setTexCoordinatesTable(static_cast<TextureCoordsContainer*>(texCoordsTable),false);
				else
				{
					//we have a (less severe) problem here ;)
					mesh->setTexCoordinatesTable(0,false);
					ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find shared texture coordinates (ID=%1) for mesh '%2' in the file!").arg(matSetID).arg(mesh->getName()));
					result = CC_FERR_BROKEN_DEPENDENCY_ERROR;
				}
			}
		}
		else if (currentObject->isKindOf(CC_POLY_LINE))
		{
			ccPolyline* poly = static_cast<ccPolyline*>(currentObject);
			intptr_t cloudID = (intptr_t)poly->getAssociatedCloud();
			ccHObject* cloud = root->find(cloudID);
			if (cloud && cloud->isKindOf(CC_POINT_CLOUD))
				poly->setAssociatedCloud(static_cast<ccGenericPointCloud*>(cloud));
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
				ccHObject* cloud = root->find(cloudID);
				if (cloud && cloud->isKindOf(CC_POINT_CLOUD))
				{
					ccGenericPointCloud* genCloud = static_cast<ccGenericPointCloud*>(cloud);
					assert(genCloud->size()>pp.index);
					correctedPickedPoints.push_back(cc2DLabel::PickedPoint(genCloud,pp.index));
				}
				else
				{
					//we have a problem here ;)
					ccLog::Warning(QString("[BinFilter::loadFileV2] Couldn't find cloud (ID=%1) associated to label '%2' in the file!").arg(cloudID).arg(label->getName()));
					if (label->getParent())
						label->getParent()->removeChild(label);
					if (!label->getFlagState(CC_FATHER_DEPENDANT))
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
				QString originalName(label->getName());
				label->clear();
				for (unsigned i=0;i<correctedPickedPoints.size();++i)
					label->addPoint(correctedPickedPoints[i].cloud,correctedPickedPoints[i].index);
				label->setVisible(visible);
				label->setName(originalName);
			}
		}

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

	return result;
}

CC_FILE_ERROR BinFilter::loadFileV1(QFile& in, ccHObject& container, unsigned nbScansTotal)
{
	if (nbScansTotal>99)
	{
		if (QMessageBox::question(0, QString("Oups"), QString("Hum, do you really expect %1 point clouds?").arg(nbScansTotal))==QMessageBox::No)
			return CC_FERR_WRONG_FILE_TYPE;
	}
	else if (nbScansTotal==0)
	{
		return CC_FERR_NO_LOAD;
	}

	ccProgressDialog pdlg(true);
	pdlg.setMethodTitle("Open Bin file (old style)");

	for (unsigned k=0;k<nbScansTotal;k++)
	{
		HeaderFlags header;
		unsigned nbOfPoints=0;
		if (ReadEntityHeader(in,nbOfPoints,header) < 0)
			return CC_FERR_READING;

		//Console::print("[BinFilter::loadModelFromBinaryFile] Entity %i : %i points, color=%i, norms=%i, dists=%i\n",k,nbOfPoints,color,norms,distances);

		//progress for this cloud
		CCLib::NormalizedProgress nprogress(&pdlg,nbOfPoints);
		pdlg.reset();
		char buffer[256];
		sprintf(buffer,"cloud %i/%i (%i points)",k+1,nbScansTotal,nbOfPoints);
		pdlg.setInfo(buffer);
		pdlg.start();
		QApplication::processEvents();

		if (nbOfPoints==0)
		{
			//Console::print("[BinFilter::loadModelFromBinaryFile] rien a faire !\n");
			continue;
		}

		//Cloud name
		char cloudName[256]="unnamed";
		if (header.name)
		{
			for (int i=0;i<256;++i)
			{
				if (in.read(cloudName+i,1)<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the cloud name!\n");
					return CC_FERR_READING;
				}
				if (cloudName[i]==0)
					break;
			}
			//we force the end of the name in case it is too long!
			cloudName[255]=0;
		}
		else
		{
			sprintf(cloudName,"unnamed - Cloud #%i",k);
		}

		//Cloud name
		char sfName[1024]="unnamed";
		if (header.sfName)
		{
			for (int i=0;i<1024;++i)
			{
				if (in.read(sfName+i,1)<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the cloud name!\n");
					return CC_FERR_READING;
				}
				if (sfName[i]==0)
					break;
			}
			//we force the end of the name in case it is too long!
			sfName[1023]=0;
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
		unsigned fileChunkSize = ccMin(nbOfPoints,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);

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

		CCVector3 P;
		unsigned char C[3];
		double D;

		//does the associated scalar field is negative?
		bool negSF = false;

		unsigned lineReaded=0;
		int parts = 0;

		//lecture du fichier
		for (unsigned i=0;i<nbOfPoints;++i)
		{
			if (lineReaded == fileChunkPos+fileChunkSize)
			{
				if (header.scalarField)
					loadedCloud->getCurrentInScalarField()->computeMinAndMax();

				container.addChild(loadedCloud);
				fileChunkPos = lineReaded;
				fileChunkSize = ccMin(nbOfPoints-lineReaded,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
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

			if (in.read((char*)P.u,sizeof(float)*3)<0)
			{
				//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity point !\n",k);
				return CC_FERR_READING;
			}
			loadedCloud->addPoint(P);

			if (header.colors)
			{
				if (in.read((char*)C,sizeof(colorType)*3)<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity colors !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addRGBColor(C);
			}

			if (header.normals)
			{
				if (in.read((char*)P.u,sizeof(float)*3)<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity norms !\n",k);
					return CC_FERR_READING;
				}
				loadedCloud->addNorm(P.u);
			}

			if (header.scalarField)
			{
				if (in.read((char*)&D,sizeof(double))<0)
				{
					//Console::print("[BinFilter::loadModelFromBinaryFile] Error reading the %ith entity distance !\n",k);
					return CC_FERR_READING;
				}
				DistanceType d = (DistanceType)D;
				//if there are negative values, we test if they are particular values (HIDDEN_VALUE, etc.)
				//or not particular (in which case we have a non strictly positive SF)
				if (d<0.0 && !negSF)
				{
					//we must test if the value is a particular one
					if (d != HIDDEN_VALUE &&
						d != OUT_VALUE &&
						d != SEGMENTED_VALUE)
						negSF = true;
				}
				loadedCloud->setPointScalarValue(i,d);
			}

			lineReaded++;

			if (!nprogress.oneStep())
			{
				loadedCloud->resize(i+1-fileChunkPos);
				k=nbScansTotal;
				i=nbOfPoints;
			}
		}

		if (header.scalarField)
		{
			CCLib::ScalarField* sf = loadedCloud->getCurrentInScalarField();
			assert(sf);
			sf->setName(sfName);
			sf->setPositive(!negSF);
			sf->computeMinAndMax();

			loadedCloud->setCurrentDisplayedScalarField(loadedCloud->getCurrentInScalarFieldIndex());
			loadedCloud->showSF(true);
		}

		container.addChild(loadedCloud);
	}

	return CC_FERR_NO_ERROR;
}
