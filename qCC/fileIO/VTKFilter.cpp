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
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//
#include "VTKFilter.h"
#include "../ccCoordinatesShiftManager.h"

//CCLib
#include <ScalarField.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>

//Qt
#include <QFile>
#include <QTextStream>

CC_FILE_ERROR VTKFilter::saveToFile(ccHObject* entity, const char* filename)
{
	if (!entity || !filename)
        return CC_FERR_BAD_ARGUMENT;

	//look for either a cloud or a mesh
	ccHObject::Container clouds,meshes;
	if (entity->isA(CC_POINT_CLOUD))
        clouds.push_back(entity);
    else if (entity->isKindOf(CC_MESH))
		meshes.push_back(entity);
	else //group?
	{
		for (unsigned i=0;i<entity->getChildrenNumber();++i)
		{
			ccHObject* child = entity->getChild(i);
			if (child->isKindOf(CC_POINT_CLOUD))
				clouds.push_back(child);
			else if (child->isKindOf(CC_MESH))
				meshes.push_back(child);
		}
	}

    if (clouds.empty() && meshes.empty())
    {
        ccConsole::Error("No point cloud nor mesh in input selection!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }
    else if (clouds.size()+meshes.size()>1)
    {
        ccConsole::Error("Can't save more than one entity per VTK file!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }

	//the cloud to save
    ccGenericPointCloud* vertices = 0;
    ccMesh* mesh = 0;
	unsigned triCount = 0;
	if (!clouds.empty()) //1 cloud, no mesh
	{
		vertices = static_cast<ccGenericPointCloud*>(clouds[0]);
	}
	else //1 mesh, with vertices as cloud
	{
		mesh = static_cast<ccMesh*>(meshes[0]);
		triCount = mesh->size();
		if (triCount == 0)
		{
			ccConsole::Error("Mesh has no triangle?!");
			return CC_FERR_NO_SAVE;
		}
		vertices = mesh->getAssociatedCloud();
	}

	assert(vertices);
	unsigned ptsCount = vertices->size();
	if (!ptsCount)
	{
		ccConsole::Error("No point/vertex to save?!");
		return CC_FERR_NO_SAVE;
	}
	const double* shift = vertices->getOriginalShift();

    //open ASCII file for writing
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return CC_FERR_WRITING;

	QTextStream outFile(&file);
	outFile.setRealNumberPrecision(sizeof(PointCoordinateType)==4 ? 8 : 12);

	//write header
	outFile << "# vtk DataFile Version 3.0" << endl;
	outFile << "vtk output" << endl;
	outFile << "ASCII" << endl;
	outFile << "DATASET " << (mesh ? "POLYDATA" : "UNSTRUCTURED_GRID") << endl;

	//data type
	QString floatType = (sizeof(PointCoordinateType)==4 ? "float" : "double");

    /*** what shall we save now? ***/

	// write the points
	{
		outFile << "POINTS " << ptsCount << " " << floatType << endl;
		for (unsigned i=0; i<ptsCount; ++i)
		{
			const CCVector3* P = vertices->getPoint(i);
			outFile << -shift[0]+(double)P->x << " " <<  -shift[1]+(double)P->y  << " " << -shift[2]+(double)P->z << endl;
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
			const PointCoordinateType* N = vertices->getPointNormal(i);
			outFile << N[0] << " " << N[1] << " "  << N[2] << endl;
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
	if (vertices->isA(CC_POINT_CLOUD))
	{
		ccPointCloud* pointCloud = static_cast<ccPointCloud*>(vertices);
		unsigned sfCount = pointCloud->getNumberOfScalarFields();
		for (unsigned i=0;i<sfCount;++i)
		{
			CCLib::ScalarField* sf = pointCloud->getScalarField(i);

			outFile << "SCALARS " << QString(sf->getName()).replace(" ","_") << (sizeof(DistanceType)==4 ? " float" : " double") << " 1" << endl;
			outFile << "LOOKUP_TABLE default" << endl;

			for (unsigned j=0;j<ptsCount; ++j)
				outFile << sf->getValue(j) << endl;
		}
	}
	else //virtual point cloud, we only have access to its currently displayed scalar field
	{
		if (vertices->hasScalarFields())
		{
			outFile << "SCALARS ScalarField" << (sizeof(DistanceType)==4 ? " float" : " double") << " 1" << endl;
			outFile << "LOOKUP_TABLE default" << endl;

			for (unsigned j=0;j<ptsCount; ++j)
				outFile << vertices->getPointDisplayedDistance(j) << endl;
		}
	}

	file.close();

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR VTKFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
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
	ccConsole::Print(QString("[VTK] ")+nextline);

	ccMesh* mesh=0;
	ccPointCloud* vertices=0;

	QString fileType = inFile.readLine().toUpper();
	if (fileType.startsWith("BINARY"))
	{
		//binary not supported yet!
		ccConsole::Error("VTK binary format not supported yet!");
		return CC_FERR_WRONG_FILE_TYPE;
	}
	else if (fileType.startsWith("ASCII"))
	{
		QString dataType = inFile.readLine().toUpper();
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
			ccConsole::Error(QString("VTK entity '%1' is not supported!").arg(dataType));
			return CC_FERR_WRONG_FILE_TYPE;
		}
	}

	//loop on keywords/data
	CC_FILE_ERROR error=CC_FERR_NO_ERROR;
	double Pshift[3]={0,0,0};
	while (error == CC_FERR_NO_ERROR)
	{
		nextline = inFile.readLine().toUpper();

		if (nextline.startsWith("POINTS"))
		{
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			if (parts.size() != 3)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}

			bool ok=false;
			unsigned ptsCount = parts[1].toInt(&ok);
			if (!ok)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}

			//QString dataFormat = parts[3].toUpper();
			//char buffer[8];
			//unsigned char datSize=4;
			//if (dataFormat == "DOUBLE")
			//{
			//	datSize=8;
			//}
			//else if (dataFormat != "FLOAT")
			//{
			//	ccConsole::Error(QString("Non floating point data (%1) is not supported!").arg(dataFormat));
			//	error=CC_FERR_WRONG_FILE_TYPE;
			//	break;
			//}

			if (!vertices->reserve(ptsCount))
			{
				error=CC_FERR_NOT_ENOUGH_MEMORY;
				break;
			}

			for (unsigned i=0;i<ptsCount;++i)
			{
				nextline = inFile.readLine();
				parts = nextline.split(" ",QString::SkipEmptyParts);
				if (parts.size() != 3)
				{
					error=CC_FERR_MALFORMED_FILE;
					break;
				}

				double Pd[3]={0,0,0};
				for (unsigned char j=0;j<3;++j)
				{
					Pd[j] = parts[j].toDouble(&ok);
					if (!ok)
					{
						ccConsole::Warning("[VTK] Element #%1 of POINTS data is corrupted!",i);
						error=CC_FERR_MALFORMED_FILE;
						break;
					}
				}
				//first point: check for 'big' coordinates
				if (i==0)
				{
					bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
					if (shiftAlreadyEnabled)
						memcpy(Pshift,coordinatesShift,sizeof(double)*3);
					bool applyAll=false;
					if (ccCoordinatesShiftManager::Handle(Pd,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,applyAll))
					{
						vertices->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
						ccConsole::Warning("[VTKFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift[0],Pshift[1],Pshift[2]);

						//we save coordinates shift information
						if (applyAll && coordinatesShiftEnabled && coordinatesShift)
						{
							*coordinatesShiftEnabled = true;
							coordinatesShift[0] = Pshift[0];
							coordinatesShift[1] = Pshift[1];
							coordinatesShift[2] = Pshift[2];
						}
					}
				}

				vertices->addPoint(CCVector3(Pd[0]+Pshift[0],Pd[1]+Pshift[1],Pd[2]+Pshift[2]));
			}
		//end POINTS
		}
		else if (nextline.startsWith("POLYGONS"))
		{
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			if (parts.size() != 3)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}

			bool ok=false;
			unsigned triCount = parts[1].toInt(&ok);
			if (!ok)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}
			
			unsigned totalElements = parts[2].toInt(&ok);
			if (!ok)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}

			if (totalElements != 4*triCount)
			{
				//TODO
				ccConsole::Warning("[VTK] Polygons other than triangle are not supported yet!");
			}

			assert(mesh);
			if (!mesh)
			{
				ccConsole::Warning("[VTK] We found polygon data while file is not composed of POLYDATA!");
				mesh= new ccMesh(vertices); //however, we can still try to load it?
			}

			if (!mesh->reserve(triCount))
			{
				error=CC_FERR_NOT_ENOUGH_MEMORY;
				break;
			}

			unsigned realTriCount=0;
			for (unsigned i=0;i<triCount;++i)
			{
				nextline = inFile.readLine();
				parts = nextline.split(" ",QString::SkipEmptyParts);
				if (parts.empty()) 
				{
					error=CC_FERR_MALFORMED_FILE;
					break;
				}

				int summits = parts[0].toInt(&ok);
				if (!ok || summits>=parts.size())
				{
					error=CC_FERR_MALFORMED_FILE;
					break;
				}

				if (summits == 3 || summits == 4) //quads are easy to handle as well!
				{
					int indexes[4];
					for (int j=0;j<summits;++j)
					{
						indexes[j] = parts[j+1].toInt(&ok);
						if (!ok)
						{
							ccConsole::Warning("[VTK] Element #%1 of POLYGONS data is corrupted!",i);
							error=CC_FERR_MALFORMED_FILE;
							break;
						}
					}

					realTriCount += (unsigned)(summits-2);
					if (realTriCount > mesh->maxSize())
					{
						if (!mesh->reserve(realTriCount+256))
						{
							error=CC_FERR_NOT_ENOUGH_MEMORY;
							break;
						}
					}

					mesh->addTriangle(indexes[0],indexes[1],indexes[2]);
					if (summits == 4)
						mesh->addTriangle(indexes[0],indexes[2],indexes[3]);
				}
			}
			
			if (realTriCount == 0)
			{
				delete mesh;
				mesh=0;
			}
			else if (realTriCount>mesh->maxSize())
			{
				mesh->resize(realTriCount);
			}
		//end POLYGONS
		}
		else if (nextline.startsWith("NORMALS"))
		{
			unsigned ptsCount = vertices->size();
			if (vertices->size()==0)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}
			else
			{
				bool loadNormals = vertices->reserveTheNormsTable();
				if (!loadNormals)
					ccConsole::Warning("[VTK] Not enough memory to load normals!");
				for (unsigned i=0;i<ptsCount;++i)
				{
					nextline = inFile.readLine();
					if (loadNormals)
					{
						QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
						if (parts.size() != 3)
						{
							error=CC_FERR_MALFORMED_FILE;
							break;
						}
						CCVector3 N;
						for (unsigned char j=0;j<3;++j)
						{
							bool ok;
							N.u[j] = (PointCoordinateType)parts[j].toDouble(&ok);
							if (!ok)
							{
								ccConsole::Warning("[VTK] Element #%1 of NORMALS data is corrupted!",i);
								error=CC_FERR_MALFORMED_FILE;
								break;
							}
						}
						vertices->addNorm(N.u);
					}
				}
			}
		//end NORMALS
		}
		else if (nextline.startsWith("COLOR_SCALARS"))
		{
			unsigned ptsCount = vertices->size();
			if (vertices->size()==0)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}
			else
			{
				bool loadRGBColors = vertices->reserveTheRGBTable();
				if (!loadRGBColors)
					ccConsole::Warning("[VTK] Not enough memory to load RGB colors!");
				for (unsigned i=0;i<ptsCount;++i)
				{
					nextline = inFile.readLine();
					if (loadRGBColors)
					{
						QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
						if (parts.size() != 3)
						{
							error=CC_FERR_MALFORMED_FILE;
							break;
						}
						colorType rgb[3];
						for (unsigned char j=0;j<3;++j)
						{
							bool ok;
							rgb[j] = (colorType)(parts[j].toDouble(&ok) * (double)MAX_COLOR_COMP);
							if (!ok)
							{
								ccConsole::Warning("[VTK] Element #%1 of COLOR_SCALARS data is corrupted!",i);
								error=CC_FERR_MALFORMED_FILE;
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
			unsigned ptsCount = vertices->size();
			if (vertices->size()==0)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}
			else
			{
				QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
				QString sfName = "ScalarField";
				if (vertices->getNumberOfScalarFields()>0)
					sfName = QString("ScalarField (%1)").arg(vertices->getNumberOfScalarFields());
				else
				{
					sfName = parts[1].replace("_"," ");
				}

				nextline = inFile.readLine();
				if (!nextline.startsWith("LOOKUP_TABLE"))
				{
					error=CC_FERR_MALFORMED_FILE;
					break;
				}
				parts = nextline.split(" ",QString::SkipEmptyParts);
				if (parts.size()>1 && parts[1].toUpper() != "DEFAULT")
					ccConsole::Warning(QString("[VTK] Lookup table other than default (%1) not supported!").arg(parts[1]));

				int newSFIndex = vertices->addScalarField(qPrintable(sfName),false);
				CCLib::ScalarField* sf = vertices->getScalarField(newSFIndex);
				for (unsigned i=0;i<ptsCount;++i)
				{
					nextline = inFile.readLine();
					if (sf)
					{
						bool ok;
						DistanceType d = (DistanceType)nextline.toDouble(&ok);
						sf->addElement(d);
					}
				}
				if (sf)
					sf->computeMinAndMax();
			}
		//end of SCALARS
		}
		else if (nextline.startsWith("POINT_DATA"))
		{
			//do nothing
		}
		else if (nextline.isEmpty()) //End of file?
		{
			break;
		}
		else //unhandled property (CELLS, CELL_TYPES, etc.)
		{
			QStringList parts = nextline.split(" ",QString::SkipEmptyParts);
			if (parts.size() < 2) 
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}

			bool ok;
			int elements = parts[1].toInt(&ok);
			if (!ok)
			{
				error=CC_FERR_MALFORMED_FILE;
				break;
			}

			for (int i=0;i<elements;++i)
				inFile.readLine(); //ignore
		//end unhandled property
		}
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

	if (mesh && mesh->size()==0)
	{
		delete mesh;
		mesh=0;
	}

	if (vertices->size()==0)
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

		if (!mesh->hasNormals())
			mesh->computeNormals();
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
