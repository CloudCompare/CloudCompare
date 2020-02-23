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

#include "VTKFilter.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QFile>
#include <QTextStream>

//System
#include <cstring>


VTKFilter::VTKFilter()
	: FileIOFilter( {
					"_VTK Filter",
					9.0f,	// priority
					QStringList{ "vtk" },
					"vtk",
					QStringList{ "VTK cloud or mesh (*.vtk)" },
					QStringList{ "VTK cloud or mesh (*.vtk)" },
					Import | Export
					} )
{
}

bool VTKFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::MESH
		|| type == CC_TYPES::POINT_CLOUD)
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR VTKFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	//look for either a cloud or a mesh
	ccMesh* mesh = ccHObjectCaster::ToMesh(entity);
	unsigned triCount = 0;
	ccGenericPointCloud* vertices = nullptr;
	if (mesh)
	{
		//input entity is a mesh
		triCount = mesh->size();
		if (triCount == 0)
		{
			ccLog::Warning("[VTK] Input mesh has no triangle?!");
			return CC_FERR_NO_SAVE;
		}
		vertices = mesh->getAssociatedCloud();
	}
	else
	{
		//no mesh? maybe the input entity is a cloud?
		vertices = ccHObjectCaster::ToGenericPointCloud(entity);
	}

	//in any case, we must have a valid 'vertices' entity now
	if (!vertices)
	{
		ccLog::Warning("[VTK] No point cloud nor mesh in input selection!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}
	unsigned ptsCount = vertices->size();
	if (!ptsCount)
	{
		ccLog::Warning("[VTK] No point/vertex to save?!");
		return CC_FERR_NO_SAVE;
	}

	//open ASCII file for writing
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return CC_FERR_WRITING;

	QTextStream outFile(&file);
	outFile.setRealNumberNotation(QTextStream::FixedNotation);
	outFile.setRealNumberPrecision(sizeof(PointCoordinateType) == 4 && !vertices->isShifted() ? 8 : 12);

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
		for (unsigned i = 0; i < ptsCount; ++i)
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
		outFile << "POLYGONS " << triCount << " " << 4 * triCount << endl;
		mesh->placeIteratorAtBeginning();
		for (unsigned i = 0; i < triCount; ++i)
		{
			const CCLib::VerticesIndexes* tsi = mesh->getNextTriangleVertIndexes(); //DGM: getNextTriangleVertIndexes is faster for mesh groups!
			outFile << "3 " << tsi->i1 << " " << tsi->i2 << " " << tsi->i3 << endl;
		}
	}
	else
	{
		// write cell data
		outFile << "CELLS " << ptsCount << " " << 2 * ptsCount << endl;
		for (unsigned i = 0; i < ptsCount; ++i)
			outFile << "1 " << i << endl;

		outFile << "CELL_TYPES " << ptsCount << endl;
		for (unsigned i = 0; i < ptsCount; ++i)
			outFile << "1 " << endl;
	}

	outFile << "POINT_DATA " << ptsCount << endl;

	// write normals
	if (vertices->hasNormals())
	{
		outFile << "NORMALS Normals " << floatType << endl;
		for (unsigned i = 0; i < ptsCount; ++i)
		{
			const CCVector3& N = vertices->getPointNormal(i);
			outFile << N.x << " " << N.y << " " << N.z << endl;
		}
	}

	// write colors
	if (vertices->hasColors())
	{
		outFile << "COLOR_SCALARS RGB 3" << endl;
		for (unsigned i = 0; i < ptsCount; ++i)
		{
			const ccColor::Rgb& C = vertices->getPointColor(i);
			outFile << static_cast<float>(C.r) / ccColor::MAX << " " << static_cast<float>(C.g) / ccColor::MAX << " " << static_cast<float>(C.b) / ccColor::MAX << endl;
		}
	}

	// write scalar field(s)?
	if (vertices->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pointCloud = static_cast<ccPointCloud*>(vertices);
		unsigned sfCount = pointCloud->getNumberOfScalarFields();
		for (unsigned i = 0; i < sfCount; ++i)
		{
			ccScalarField* sf = static_cast<ccScalarField*>(pointCloud->getScalarField(i));

			outFile << "SCALARS " << QString(sf->getName()).replace(" ", "_") << (sizeof(ScalarType) == 4 ? " float" : " double") << " 1" << endl;
			outFile << "LOOKUP_TABLE default" << endl;

			for (unsigned j = 0; j < ptsCount; ++j)
			{
				outFile << sf->getGlobalShift() + sf->getValue(j) << endl;
			}
		}
	}
	else //virtual point cloud, we only have access to its currently displayed scalar field
	{
		if (vertices->hasScalarFields())
		{
			outFile << "SCALARS ScalarField" << (sizeof(ScalarType) == 4 ? " float" : " double") << " 1" << endl;
			outFile << "LOOKUP_TABLE default" << endl;

			for (unsigned j = 0; j < ptsCount; ++j)
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

CC_FILE_ERROR VTKFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
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
	ccLog::Print(QString("[VTK] ") + nextline);

	ccMesh* mesh = nullptr;
	ccPointCloud* vertices = nullptr;

	std::vector<int> indexes; //global so as to avoid unnecessary mem. allocations
	QString lastSfName;
	bool acceptLookupTables = true;
	unsigned lastDataSize = 0;

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
		if (!GetNextNonEmptyLine(inFile, dataType))
			return CC_FERR_MALFORMED_FILE;
		if (!dataType.startsWith("DATASET"))
			return CC_FERR_MALFORMED_FILE;
		dataType.remove(0, 8);
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
	CCVector3d Pshift(0, 0, 0);
	bool skipReadLine = false;
	while (error == CC_FERR_NO_ERROR)
	{
		if (!skipReadLine && !GetNextNonEmptyLine(inFile, nextline))
			break; //end of file
		skipReadLine = false;

		assert(!nextline.isEmpty());

		if (nextline.startsWith("POINTS"))
		{
			QStringList parts = nextline.split(" ", QString::SkipEmptyParts);
			if (parts.size() != 3)
			{
				error = CC_FERR_MALFORMED_FILE;
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

			//warning: multiple points can be stored on a single line!
			unsigned iPt = 0;
			CCVector3d Pd(0, 0, 0);
			unsigned coordIndex = 0;
			while (iPt < ptsCount)
			{
				nextline = inFile.readLine();
				parts = nextline.split(" ", QString::SkipEmptyParts);

				for (int i = 0; i < parts.size(); ++i)
				{
					Pd.u[coordIndex] = parts[i].toDouble(&ok);
					if (!ok)
					{
						ccLog::Warning("[VTK] Element #%1 of POINTS data is corrupted!", iPt);
						error = CC_FERR_MALFORMED_FILE;
						iPt = ptsCount;
						break;
					}

					if (coordIndex == 2)
					{
						//first point: check for 'big' coordinates
						if (iPt == 0)
						{
							bool preserveCoordinateShift = true;
							if (HandleGlobalShift(Pd, Pshift, preserveCoordinateShift, parameters))
							{
								if (preserveCoordinateShift)
								{
									vertices->setGlobalShift(Pshift);
								}
								ccLog::Warning("[VTKFilter::loadFile] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
							}
						}

						CCVector3 P = CCVector3::fromArray((Pd + Pshift).u);
						vertices->addPoint(P);

						coordIndex = 0;
						++iPt;
					}
					else
					{
						++coordIndex;
					}
				}
			}
			//end POINTS
		}
		else if (nextline.startsWith("POLYGONS") || nextline.startsWith("TRIANGLE_STRIPS"))
		{
			QStringList parts = nextline.split(" ", QString::SkipEmptyParts);
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

			//			unsigned totalElements = parts[2].toUInt(&ok);
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

			for (unsigned i = 0; i < elemCount; ++i)
			{
				nextline = inFile.readLine();
				parts = nextline.split(" ", QString::SkipEmptyParts);
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
					catch (const std::bad_alloc&)
					{
						error = CC_FERR_NOT_ENOUGH_MEMORY;
						break;
					}
				}
				//decode indexes
				for (unsigned j = 0; j < vertCount; ++j)
				{
					indexes[j] = parts[j + 1].toUInt(&ok);
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
					unsigned triCount = vertCount - 2;
					if (mesh->size() + triCount > mesh->capacity())
					{
						if (!mesh->reserve(mesh->size() + triCount + 256)) //take some advance to avoid too many allocations
						{
							error = CC_FERR_NOT_ENOUGH_MEMORY;
							break;
						}
					}

					if (isPolygon)
					{
						//triangle or quad
						mesh->addTriangle(indexes[0], indexes[1], indexes[2]);
						if (vertCount == 4)
							mesh->addTriangle(indexes[0], indexes[2], indexes[3]);
					}
					else
					{
						//triangle strip
						for (unsigned j = 0; j < triCount; ++j)
							mesh->addTriangle(indexes[j], indexes[j + 1], indexes[j + 2]);
					}
				}
			}

			if (mesh->size() != 0)
			{
				mesh->shrinkToFit();
			}
			//end POLYGONS or TRIANGLE_STRIPS
		}
		else if (nextline.startsWith("NORMALS"))
		{
			if (lastDataSize == 0)
				lastDataSize = vertices->size();
			if (lastDataSize == 0)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			bool loadNormals = false;
			if (lastDataSize == vertices->size())
			{
				if (!vertices->reserveTheNormsTable())
					ccLog::Warning("[VTK] Not enough memory to load normals!");
				else
					loadNormals = true;
			}

			//warning: multiple normals can be stored on a single line!
			unsigned iNorm = 0;
			CCVector3 N;
			unsigned coordIndex = 0;
			while (iNorm < lastDataSize)
			{
				nextline = inFile.readLine();
				QStringList parts = nextline.split(" ", QString::SkipEmptyParts);

				for (int i = 0; i < parts.size(); ++i)
				{
					bool ok;
					N.u[coordIndex] = static_cast<PointCoordinateType>(parts[i].toDouble(&ok));
					if (!ok)
					{
						ccLog::Warning("[VTK] Element #%1 of NORMALS data is corrupted!", iNorm);
						error = CC_FERR_MALFORMED_FILE;
						iNorm = lastDataSize;
						break;
					}

					if (coordIndex == 2)
					{
						if (loadNormals)
							vertices->addNorm(N);
						coordIndex = 0;
						++iNorm;
					}
					else
					{
						++coordIndex;
					}
				}
			}
			lastDataSize = 0; //lastDataSize is consumed

			//end NORMALS
		}
		else if (nextline.startsWith("COLOR_SCALARS"))
		{
			if (lastDataSize == 0)
				lastDataSize = vertices->size();
			if (lastDataSize == 0)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			bool loadRGBColors = vertices->reserveTheRGBTable();
			if (!loadRGBColors)
				ccLog::Warning("[VTK] Not enough memory to load RGB colors!");

			//warning: multiple colors can be stored on a single line!
			unsigned iCol = 0;
			ccColor::Rgb rgb;
			unsigned coordIndex = 0;
			while (iCol < lastDataSize)
			{
				nextline = inFile.readLine();
				QStringList parts = nextline.split(" ", QString::SkipEmptyParts);

				for (int i = 0; i < parts.size(); ++i)
				{
					bool ok;
					rgb.rgb[coordIndex] = static_cast<ColorCompType>(parts[i].toDouble(&ok) * ccColor::MAX);
					if (!ok)
					{
						ccLog::Warning("[VTK] Element #%1 of COLOR_SCALARS data is corrupted!", iCol);
						error = CC_FERR_MALFORMED_FILE;
						iCol = lastDataSize;
						break;
					}

					if (coordIndex == 2)
					{
						if (loadRGBColors)
							vertices->addColor(rgb);
						coordIndex = 0;
						++iCol;
					}
					else
					{
						++coordIndex;
					}
				}
			}
			lastDataSize = 0; //lastDataSize is consumed

			//end COLOR_SCALARS
		}
		else if (nextline.startsWith("SCALARS"))
		{
			QStringList parts = nextline.split(" ", QString::SkipEmptyParts);
			lastSfName = "ScalarField";
			if (parts.size() > 1)
				lastSfName = parts[1].replace("_", " ");

			//SF already exists?
			if (vertices->getScalarFieldIndexByName(qPrintable(lastSfName)) >= 0)
				lastSfName += QString(" (%1)").arg(vertices->getNumberOfScalarFields());
			//end of SCALARS
		}
		else if (nextline.startsWith("LOOKUP_TABLE") || nextline.startsWith("VECTORS"))
		{
			bool expected = (lastDataSize != 0);
			assert(!acceptLookupTables || expected); //i.e. lastDataSize shouldn't be 0 for 'accepted' lookup tables

			QStringList parts = nextline.split(" ", QString::SkipEmptyParts);
			QString itemName = parts[0];
			if (parts.size() > 2)
			{
				bool ok = false;
				int valCount = parts[2].toUInt(&ok);
				if (ok)
					lastDataSize = valCount;
			}
			else if (!expected)
			{
				ccLog::Warning(QString("[VTK] field %1 has no size?!").arg(itemName));
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			bool createSF = (vertices->size() == lastDataSize && vertices->size() != 0);
			if (acceptLookupTables && !createSF)
			{
				ccLog::Warning(QString("[VTK] field %1 has not the right number of points (will be ignored)").arg(itemName));
			}
			createSF &= (acceptLookupTables || expected);
			if (createSF && lastSfName.isNull())
			{
				ccLog::Warning(QString("[VTK] field %1 has no name (will be ignored)").arg(itemName));
				createSF = false;
			}
			else if (!expected)
			{
				ccLog::Warning(QString("[VTK] field %1 was not expected (will be ignored)").arg(itemName));
			}

			//create scalar field?
			ccScalarField* sf = nullptr;
			if (createSF)
			{
				sf = new ccScalarField(qPrintable(lastSfName));
				if (!sf->reserveSafe(lastDataSize))
				{
					ccLog::Warning(QString("[VTK] Not enough memory to load scalar field' %1' (will be ignored)").arg(lastSfName));
					sf->release();
					sf = nullptr;
				}
			}

			lastSfName.clear(); //name is "consumed"

			//warning: multiple colors can be stored on a single line!
			unsigned iScal = 0;
			while (iScal < lastDataSize)
			{
				nextline = inFile.readLine();
				QStringList parts = nextline.split(" ", QString::SkipEmptyParts);

				if (expected)
				{
					for (int i = 0; i < parts.size(); ++i)
					{
						bool ok;
						ScalarType d = static_cast<ScalarType>(parts[i].toDouble(&ok));
						if (!ok)
						{
							ccLog::Warning("[VTK] Element #%1 of LOOKUP_TABLE/VECTORS data is corrupted!", iScal);
							error = CC_FERR_MALFORMED_FILE;
							if (sf)
							{
								sf->release();
								sf = nullptr;
							}
							iScal = lastDataSize;
							break;
						}

						if (sf)
							sf->addElement(d);
						++iScal;
					}
				}
				else
				{
					//hard to guess the right format, but an unexpected field seem to always be
					//organized as 'one element per line'
					++iScal;
				}
			}
			lastDataSize = 0; //lastDataSize is "consumed"
			acceptLookupTables = false;

			if (sf)
			{
				sf->computeMinAndMax();
				int newSFIndex = vertices->addScalarField(sf);
				if (newSFIndex == 0)
					vertices->setCurrentDisplayedScalarField(newSFIndex);
				vertices->showSF(true);
			}
			//end of SCALARS
		}
		else if (nextline.startsWith("POINT_DATA"))
		{
			//check that the number of 'point_data' match the number of points
			QStringList parts = nextline.split(" ", QString::SkipEmptyParts);
			acceptLookupTables = false;
			if (parts.size() > 1)
			{
				bool ok;
				lastDataSize = parts[1].toUInt(&ok);
				acceptLookupTables = ok && vertices;
			}
		}
		else if (nextline.startsWith("FIELD"))
		{
			QStringList parts = nextline.split(" ", QString::SkipEmptyParts);
			if (parts.size() < 2)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			bool ok;
			unsigned elements = parts[2].toUInt(&ok);
			if (!ok)
			{
				error = CC_FERR_MALFORMED_FILE;
				break;
			}

			elements *= 2;	//we don't know how to handle those properly but at least
			//we know that for FIELD elements, there's 2 lines per element...

			for (unsigned i = 0; i < elements; ++i)
			{
				inFile.readLine(); //ignore
			}
		}
		else //unhandled property (CELLS, CELL_TYPES, etc.)
		{
			QStringList parts = nextline.split(" ", QString::SkipEmptyParts);
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

			if (nextline.startsWith("CELL_DATA"))
			{
				//read next line (in case we actually know how to read it!
				if (!GetNextNonEmptyLine(inFile, nextline))
				{
					error = CC_FERR_MALFORMED_FILE;
					break;
				}
				skipReadLine = true;

				if (nextline.startsWith("SCALARS")
					|| nextline.startsWith("NORMALS")
					|| nextline.startsWith("COLOR_SCALARS"))
				{
					lastDataSize = elements;
					acceptLookupTables = false; //this property is for triangles!
					continue;
				}
			}
			//we'll try to blindly skip the elements...
			for (unsigned i = 0; i < elements; ++i)
			{
				inFile.readLine(); //ignore
			}
			//end unhandled property
		}

		if (error != CC_FERR_NO_ERROR)
			break;
	}

	file.close();

	if (vertices && vertices->size() == 0)
	{
		delete vertices;
		vertices = nullptr;
		if (error == CC_FERR_NO_ERROR)
			error = CC_FERR_NO_LOAD;
	}

	if (mesh && (mesh->size() == 0 || vertices == nullptr))
	{
		delete mesh;
		mesh = nullptr;
		if (error == CC_FERR_NO_ERROR)
			error = CC_FERR_NO_LOAD;
	}

	if (mesh)
	{
		container.addChild(mesh);
		mesh->setVisible(true);

		mesh->addChild(vertices);
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
	else if (vertices)
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

	return error;
}
