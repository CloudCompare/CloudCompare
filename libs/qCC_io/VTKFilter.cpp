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

#include "VTKFilter.h"
#include "ccCoordinatesShiftManager.h"

//CCLib
#include <ScalarField.h>

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccMesh.h>

//Qt
#include <QFile>
#include <QTextStream>

//System
#include <string.h>

CC_FILE_ERROR VTKFilter::saveToFile(ccHObject* entity, QString filename)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	//look for either a cloud or a mesh
	ccHObject::Container clouds,meshes;
	if (entity->isA(CC_TYPES::POINT_CLOUD))
		clouds.push_back(entity);
	else if (entity->isKindOf(CC_TYPES::MESH))
		meshes.push_back(entity);
	else //group?
	{
		for (unsigned i=0; i<entity->getChildrenNumber(); ++i)
		{
			ccHObject* child = entity->getChild(i);
			if (child->isKindOf(CC_TYPES::POINT_CLOUD))
				clouds.push_back(child);
			else if (child->isKindOf(CC_TYPES::MESH))
				meshes.push_back(child);
		}
	}

	if (clouds.empty() && meshes.empty())
	{
		ccLog::Error("No point cloud nor mesh in input selection!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}
	else if (clouds.size()+meshes.size()>1)
	{
		ccLog::Error("Can't save more than one entity per VTK file!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//the cloud to save
	ccGenericPointCloud* vertices = 0;
	ccMesh* mesh = 0;
	unsigned triCount = 0;
	if (!clouds.empty()) //1 cloud, no mesh
	{
		vertices = ccHObjectCaster::ToGenericPointCloud(clouds[0]);
	}
	else //1 mesh, with vertices as cloud
	{
		mesh = static_cast<ccMesh*>(meshes[0]);
		triCount = mesh->size();
		if (triCount == 0)
		{
			ccLog::Error("Mesh has no triangle?!");
			return CC_FERR_NO_SAVE;
		}
		vertices = mesh->getAssociatedCloud();
	}

	assert(vertices);
	unsigned ptsCount = vertices->size();
	if (!ptsCount)
	{
		ccLog::Error("No point/vertex to save?!");
		return CC_FERR_NO_SAVE;
	}

	//open ASCII file for writing
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return CC_FERR_WRITING;

	QTextStream outFile(&file);
	outFile.setRealNumberPrecision(sizeof(PointCoordinateType) == 4 ? 8 : 12);

	//write header
	outFile << "# vtk DataFile Version 3.0" << endl;
	outFile << "vtk output" << endl;
	outFile << "ASCII" << endl;
	outFile << "DATASET " << (mesh ? "POLYDATA" : "UNSTRUCTURED_GRID") << endl;

	//data type
	QString floatType = (sizeof(PointCoordinateType) == 4 ? "float" : "double");

	/*** what shall we save now? ***/

	// write the points
	{
		outFile << "POINTS " << ptsCount << " " << floatType << endl;
		for (unsigned i=0; i<ptsCount; ++i)
		{
			const CCVector3* P = vertices->getPoint(i);
			CCVector3d Pglobal = vertices->toGlobal3d<PointCoordinateType>(*P);
			outFile << Pglobal.x << " "
					<< Pglobal.y << " "
					<< Pglobal.z << endl;
		}
	}

	// write triangles
	if (mesh)
	{
		outFile << "POLYGONS " << triCount << " " <<  4*triCount << endl;
		mesh->placeIteratorAtBegining();
		for (unsigned i=0; i<triCount; ++i)
		{
			const CCLib::TriangleSummitsIndexes* tsi = mesh->getNextTriangleIndexes(); //DGM: getNextTriangleIndexes is faster for mesh groups!
			outFile << "3 " << tsi->i1 << " " << tsi->i2  << " " << tsi->i3 << endl;
		}
	}
	else
	{
		// write cell data
		outFile << "CELLS " << ptsCount << " " <<  2*ptsCount << endl;
		for (unsigned i=0; i<ptsCount; ++i)
			outFile << "1 " << i << endl;

		outFile << "CELL_TYPES " << ptsCount  << endl;
		for (unsigned i=0; i<ptsCount; ++i)
			outFile << "1 " << endl;
	}

	outFile << "POINT_DATA " << ptsCount << endl;

	// write normals
	if (vertices->hasNormals())
	{
		outFile << "NORMALS Normals "<< floatType << endl;
		for (unsigned i=0; i<ptsCount; ++i)
		{
			const CCVector3& N = vertices->getPointNormal(i);
			outFile << N.x << " " << N.y << " "  << N.z << endl;
		}
	}

	// write colors
	if (vertices->hasColors())
	{
		outFile << "COLOR_SCALARS RGB 3" << endl;
		for (unsigned i=0; i<ptsCount; ++i)
		{
			const colorType* C = vertices->getPointColor(i);
			outFile << (float)C[0]/(float)MAX_COLOR_COMP << " " << (float)C[1]/(float)MAX_COLOR_COMP << " "  << (float)C[2]/(float)MAX_COLOR_COMP << endl;
		}
	}

	// write scalar field(s)?
	if (vertices->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pointCloud = static_cast<ccPointCloud*>(vertices);
		unsigned sfCount = pointCloud->getNumberOfScalarFields();
		for (unsigned i=0;i<sfCount;++i)
		{
			CCLib::ScalarField* sf = pointCloud->getScalarField(i);

			outFile << "SCALARS " << QString(sf->getName()).replace(" ","_") << (sizeof(ScalarType)==4 ? " float" : " double") << " 1" << endl;
			outFile << "LOOKUP_TABLE default" << endl;

			for (unsigned j=0;j<ptsCount; ++j)
				outFile << sf->getValue(j) << endl;
		}
	}
	else //virtual point cloud, we only have access to its currently displayed scalar field
	{
		if (vertices->hasScalarFields())
		{
			outFile << "SCALARS ScalarField" << (sizeof(ScalarType)==4 ? " float" : " double") << " 1" << endl;
			outFile << "LOOKUP_TABLE default" << endl;

			for (unsigned j=0;j<ptsCount; ++j)
				outFile << vertices->getPointDisplayedDistance(j) << endl;
		}
	}

	file.close();

	return CC_FERR_NO_ERROR;
}

static bool GetNextNonEmptyLine(QTextStream& stream, QString& line)
{
	//allow blank lines
	line = stream.readLine();
	while (line.isEmpty())
	{
		//end of file?
		if (line.isNull())
			return false;
		line = stream.readLine().toUpper();
	}

	return true;
}

CC_FILE_ERROR VTKFilter::loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
	//open ASCII file for reading
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		return CC_FERR_READING;

	QTextStream inFile(&file);

	//read header
	QString nextline = inFile.readLine();
	if (!nextline.startsWith("# vtk"))
		return CC_FERR_MALFORMED_FILE;

	//comment
	nextline = inFile.readLine();
	ccLog::Print(QString("[VTK] ")+nextline);

	ccMesh* mesh = 0;
	ccPointCloud* vertices = 0;
	
	std::vector<int> indexes; //global so as to avoid unnecessary mem. allocations
	QString lastSfName;
	bool acceptLookupTables = true;

	QString fileType = inFile.readLine().toUpper();
	if (fileType.startsWith("BINARY"))
	{
		//binary not supported yet!
		ccLog::Error("VTK binary format not supported yet!");
		return CC_FERR_WRONG_FILE_TYPE;
	}
	else if (fileType.startsWith("ASCII"))
	{
		//allow blank lines
		QString dataType;
		if (!GetNextNonEmptyLine(inFile,dataType))
			return CC_FERR_MALFORMED_FILE;
		if (!dataType.startsWith("DATASET"))
			return CC_FERR_MALFORMED_FILE;
		dataType.remove(0,8);
		if (dataType.startsWith("POLYDATA"))
		{
			vertices = new ccPointCloud("vertices");
			mesh = new ccMesh(vertices);
		}
		else if (dataType.startsWith("UNSTRUCTURED_GRID"))
		{
			vertices = new ccPointCloud("unnamed - VTK unstructured grid");
		}
		else
		{
			ccLog::Error(QString("VTK entity '%1' is not supported!").arg(dataType));
			return CC_FERR_WRONG_FILE_TYPE;
		}
	}

	//loop on keywords/data
	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	CCVector3d Pshift(0,0,0);
	while (error == CC_FERR_NO_ERROR)
	{
		if (!GetNextNonEmptyLine(inFile,nextline))
			break; //end of file

		assert(!nextline.isEmpty());

		if (nextline.startsWith("POINTS"))
		{
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			if (parts.size() != 3)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}

			bool ok = false;
			unsigned ptsCount = parts[1].toInt(&ok);
			if (!ok)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			//QString dataFormat = parts[3].toUpper();
			//char buffer[8];
			//unsigned char datSize = 4;
			//if (dataFormat == "DOUBLE")
			//{
			//	datSize = 8;
			//}
			//else if (dataFormat != "FLOAT")
			//{
			//	ccLog::Error(QString("Non floating point data (%1) is not supported!").arg(dataFormat));
			//	error = CC_FERR_WRONG_FILE_TYPE;
			//	break;
			//}

			if (!vertices->reserve(ptsCount))
			{
				error = CC_FERR_NOT_ENOUGH_MEMORY;
				break;
			}

			for (unsigned i=0; i<ptsCount; ++i)
			{
				nextline = inFile.readLine();
				parts = nextline.split(" ",QString::SkipEmptyParts);
				if (parts.size() != 3)
				{
					error = CC_FERR_MALFORMED_FILE;
					break;
				}

				CCVector3d Pd(0,0,0);
				for (unsigned char j=0; j<3; ++j)
				{
					Pd.u[j] = parts[j].toDouble(&ok);
					if (!ok)
					{
						ccLog::Warning("[VTK] Element #%1 of POINTS data is corrupted!",i);
						error = CC_FERR_MALFORMED_FILE;
						break;
					}
				}

				//first point: check for 'big' coordinates
				if (i == 0)
				{
					bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
					if (shiftAlreadyEnabled)
						Pshift = *coordinatesShift;
					bool applyAll = false;
					if (	sizeof(PointCoordinateType) < 8
						&&	ccCoordinatesShiftManager::Handle(Pd,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,&applyAll))
					{
						vertices->setGlobalShift(Pshift);
						ccLog::Warning("[VTKFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift.x,Pshift.y,Pshift.z);

						//we save coordinates shift information
						if (applyAll && coordinatesShiftEnabled && coordinatesShift)
						{
							*coordinatesShiftEnabled = true;
							*coordinatesShift = Pshift;
						}
					}
				}

				CCVector3 P = CCVector3::fromArray((Pd + Pshift).u);
				vertices->addPoint(P);
			}
		//end POINTS
		}
		else if (nextline.startsWith("POLYGONS") || nextline.startsWith("TRIANGLE_STRIPS"))
		{
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			if (parts.size() != 3)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			//current type name (i.e. POLYGONS or TRIANGLE_STRIPS)
			QString typeName = parts[0];
			bool isPolygon = (typeName == "POLYGONS");

			bool ok = false;
			unsigned elemCount = parts[1].toUInt(&ok);
			if (!ok)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}
			
			unsigned totalElements = parts[2].toUInt(&ok);
			if (!ok)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			assert(mesh);
			if (!mesh)
			{
				ccLog::Warning(QString("[VTK] We found %1 data while file is not composed of POLYDATA!").arg(typeName));
				mesh = new ccMesh(vertices); //however, we can still try to load it?
			}

			for (unsigned i=0; i<elemCount; ++i)
			{
				nextline = inFile.readLine();
				parts = nextline.split(" ",QString::SkipEmptyParts);
				if (parts.empty()) 
				{
					error = CC_FERR_MALFORMED_FILE;
					break;
				}

				unsigned vertCount = parts[0].toUInt(&ok);
				if (!ok || static_cast<int>(vertCount) >= parts.size())
				{
					error = CC_FERR_MALFORMED_FILE;
					break;
				}
				else if (vertCount < 3)
				{
					ccLog::Warning(QString("[VTK] Element #%1 of %2 data is corrupted! (not enough indexes)").arg(i).arg(typeName));
				}

				if (isPolygon && (vertCount != 3 && vertCount != 4)) //quads are easy to handle as well!
				{
					ccLog::Warning(QString("[VTK] POLYGON element #%1 has an unhandled size (> 4 vertices)").arg(i));
					continue;
				}

				//reserve mem to. store indexes
				if (indexes.size() < vertCount)
				{
					try
					{
						indexes.resize(vertCount);
					}
					catch (std::bad_alloc)
					{
						error = CC_FERR_NOT_ENOUGH_MEMORY;
						break;
					}
				}
				//decode indexes
				for (unsigned j=0; j<vertCount; ++j)
				{
					indexes[j] = parts[j+1].toUInt(&ok);
					if (!ok)
					{
						ccLog::Warning(QString("[VTK] Element #%1 of %2 data is corrupted! (invalid index value)").arg(i).arg(typeName));
						error = CC_FERR_MALFORMED_FILE;
						break;
					}
				}

				//add the triangles
				{
					assert(vertCount > 2);
					unsigned triCount = vertCount-2;
					if (mesh->size() + triCount > mesh->maxSize())
					{
						if (!mesh->reserve(mesh->size()+triCount+256)) //take some advance to avoid too many allocations
						{
							error = CC_FERR_NOT_ENOUGH_MEMORY;
							break;
						}
					}

					if (isPolygon)
					{
						//triangle or quad
						mesh->addTriangle(indexes[0],indexes[1],indexes[2]);
						if (vertCount == 4)
							mesh->addTriangle(indexes[0],indexes[2],indexes[3]);
					}
					else
					{
						//triangle strip
						for (unsigned j=0; j<triCount; ++j)
							mesh->addTriangle(indexes[j],indexes[j+1],indexes[j+2]);
					}
				}
			}
			
			if (mesh->size() != 0 && mesh->size() < mesh->maxSize())
			{
				mesh->resize(mesh->size());
			}
		//end POLYGONS or TRIANGLE_STRIPS
		}
		else if (nextline.startsWith("NORMALS"))
		{
			unsigned ptsCount = vertices->size();
			if (vertices->size() == 0)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}
			else
			{
				bool loadNormals = vertices->reserveTheNormsTable();
				if (!loadNormals)
					ccLog::Warning("[VTK] Not enough memory to load normals!");
				for (unsigned i=0; i<ptsCount; ++i)
				{
					nextline = inFile.readLine();
					if (loadNormals)
					{
						QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
						if (parts.size() != 3)
						{
							error = CC_FERR_MALFORMED_FILE;
							break;
						}
						CCVector3 N;
						for (unsigned char j=0; j<3; ++j)
						{
							bool ok;
							N.u[j] = (PointCoordinateType)parts[j].toDouble(&ok);
							if (!ok)
							{
								ccLog::Warning("[VTK] Element #%1 of NORMALS data is corrupted!",i);
								error = CC_FERR_MALFORMED_FILE;
								break;
							}
						}
						vertices->addNorm(N);
					}
				}
			}
		//end NORMALS
		}
		else if (nextline.startsWith("COLOR_SCALARS"))
		{
			unsigned ptsCount = vertices->size();
			if (vertices->size() == 0)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}
			else
			{
				bool loadRGBColors = vertices->reserveTheRGBTable();
				if (!loadRGBColors)
					ccLog::Warning("[VTK] Not enough memory to load RGB colors!");
				for (unsigned i=0; i<ptsCount; ++i)
				{
					nextline = inFile.readLine();
					if (loadRGBColors)
					{
						QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
						if (parts.size() != 3)
						{
							error = CC_FERR_MALFORMED_FILE;
							break;
						}
						colorType rgb[3];
						for (unsigned char j=0; j<3; ++j)
						{
							bool ok;
							rgb[j] = (colorType)(parts[j].toDouble(&ok) * (double)MAX_COLOR_COMP);
							if (!ok)
							{
								ccLog::Warning("[VTK] Element #%1 of COLOR_SCALARS data is corrupted!",i);
								error = CC_FERR_MALFORMED_FILE;
								break;
							}
						}
						vertices->addRGBColor(rgb);
					}
				}
			}
		//end COLOR_SCALARS
		}
		else if (nextline.startsWith("SCALARS"))
		{
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			lastSfName = "ScalarField";
			if (parts.size() > 1)
				lastSfName = parts[1].replace("_"," ");

			//SF already exists?
			if (vertices->getScalarFieldIndexByName(qPrintable(lastSfName)) >= 0)
				lastSfName += QString(" (%1)").arg(vertices->getNumberOfScalarFields());
			//end of SCALARS
		}
		else if (nextline.startsWith("LOOKUP_TABLE") || nextline.startsWith("VECTORS"))
		{
			unsigned ptsCount = vertices->size();

			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			QString itemName = parts[0];
			if (parts.size() > 2)
			{
				bool ok = false;
				int valCount = parts[2].toUInt(&ok);
				if (ok)
					ptsCount = valCount;
			}

			bool createSF = (vertices->size() == ptsCount && vertices->size() != 0);
			if (acceptLookupTables && !createSF)
			{
				ccLog::Warning(QString("[VTK] field %1 has not the right number of points (will be ignored)").arg(itemName));
			}
			createSF &= acceptLookupTables;
			if (createSF && lastSfName.isNull())
			{
				ccLog::Warning(QString("[VTK] field %1 has no name (will be ignored)").arg(itemName));
				createSF = false;
			}

			//create scalar field?
			int newSFIndex = createSF ? vertices->addScalarField(qPrintable(lastSfName)) : -1;
			CCLib::ScalarField* sf = newSFIndex >= 0 ? vertices->getScalarField(newSFIndex) : 0;
			
			lastSfName.clear(); //name is "consumed"
				
			for (unsigned i=0; i<ptsCount; ++i)
			{
				nextline = inFile.readLine();
				if (sf) //otherwise we simply skip the line
				{
					QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
					if (parts.size() != 1)
					{
						//get rid of the scalar field :(
						vertices->deleteScalarField(newSFIndex);
						sf = 0;

						if (i == 0)
						{
							ccLog::Warning(QString("[VTK] %1 field with more than one element can't be imported as scalar fields!").arg(itemName));
						}
						else
						{
							error = CC_FERR_MALFORMED_FILE;
							break;
						}
					}
					else
					{
						bool ok;
						ScalarType d = static_cast<ScalarType>(nextline.toDouble(&ok));
						sf->setValue(i, ok ? d : NAN_VALUE);
					}
				}
			}

			if (sf)
			{
				sf->computeMinAndMax();
				vertices->setCurrentDisplayedScalarField(newSFIndex);
				vertices->showSF(true);
			}
		//end of SCALARS
		}
		else if (nextline.startsWith("POINT_DATA"))
		{
			//check that the number of 'point_data' match the number of points
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			acceptLookupTables = false;
			if (parts.size() > 1) 
			{
				bool ok;
				unsigned dataCount = parts[1].toUInt(&ok);
				if (ok && vertices && dataCount == vertices->size())
				{
					acceptLookupTables = true;
				}
			}

			if (!acceptLookupTables)
			{
				ccLog::Warning("[VTK] The number of 'POINT_DATA' doesn't match the number of loaded points... lookup tables will be ignored");
			}
		}
		else //unhandled property (CELLS, CELL_TYPES, etc.)
		{
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			if (parts.size() < 2) 
			{
				ccLog::Warning(QString("[VTK] Unhandled element: %1").arg(parts[0]));
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			bool ok;
			unsigned elements = parts[1].toUInt(&ok);
			if (!ok)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			for (unsigned i=0; i<elements; ++i)
			{
				inFile.readLine(); //ignore
			}
		//end unhandled property
		}

		if (error != CC_FERR_NO_ERROR)
			break;
	}

	if (error != CC_FERR_NO_ERROR)
	{
		if (mesh)
			delete mesh;
		if (vertices)
			delete vertices;
		return CC_FERR_MALFORMED_FILE;
	}

	file.close();

	if (mesh && mesh->size() == 0)
	{
		delete mesh;
		mesh = 0;
	}

	if (vertices->size() == 0)
	{
		delete vertices;
		return CC_FERR_NO_LOAD;
	}

	if (mesh)
	{
		container.addChild(mesh);
		mesh->setVisible(true);

		mesh->addChild(vertices);
		vertices->setVisible(false);
		vertices->setEnabled(false);
		vertices->setName("Vertices");
		vertices->setLocked(true); //DGM: no need to lock it as it is only used by one mesh!

		//DGM: normals can be per-vertex or per-triangle so it's better to let the user do it himself later
		//Moreover it's not always good idea if the user doesn't want normals (especially in ccViewer!)
		if (!mesh->hasNormals())
		{
			//	mesh->computeNormals();
			ccLog::Warning("[VTK] Mesh has no normal! You can manually compute them (select it then call \"Edit > Normals > Compute\")");
		}
		mesh->showNormals(mesh->hasNormals());
		if (vertices->hasScalarFields())
		{
			vertices->setCurrentDisplayedScalarField(0);
			mesh->showSF(true);
		}
		if (vertices->hasColors())
			mesh->showColors(true);
	}
	else
	{
		container.addChild(vertices);
		vertices->setVisible(true);
		if (vertices->hasNormals())
			vertices->showNormals(true);
		if (vertices->hasScalarFields())
		{
			vertices->setCurrentDisplayedScalarField(0);
			vertices->showSF(true);
		}
		if (vertices->hasColors())
			vertices->showColors(true);
	}

	return CC_FERR_NO_ERROR;
}
