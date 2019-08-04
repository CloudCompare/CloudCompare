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

#include "OFFFilter.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccMesh.h>
#include <ccNormalVectors.h>
#include <ccOctree.h>
#include <ccPointCloud.h>

//System
#include <string>


OFFFilter::OFFFilter()
	: FileIOFilter( {
					"_OFF Filter",
					11.0f,	// priority
					QStringList{ "off" },
					"off",
					QStringList{ "OFF mesh (*.off)" },
					QStringList{ "OFF mesh (*.off)" },
					Import | Export
					} )
{
}

bool OFFFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::MESH)
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR OFFFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	Q_UNUSED( parameters );
	
	if (!entity)
		return CC_FERR_BAD_ARGUMENT;

	if (!entity->isKindOf(CC_TYPES::MESH))
	{
		ccLog::Warning("[OFF] This filter can only save one mesh at a time!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
	if (!mesh || mesh->size() == 0)
	{
		ccLog::Warning("[OFF] Input mesh is empty!");
		return CC_FERR_NO_SAVE;
	}
	ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
	if (!vertices || vertices->size() == 0)
	{
		ccLog::Warning("[OFF] Input mesh has no vertices?!");
		return CC_FERR_NO_SAVE;
	}

	//try to open file for saving
	QFile fp(filename);
	if (!fp.open(QIODevice::WriteOnly | QIODevice::Text))
		return CC_FERR_WRITING;

	QTextStream stream(&fp);
	stream.setRealNumberNotation(QTextStream::FixedNotation);
	stream.setRealNumberPrecision(12); //TODO: ask the user?

	//header: "OFF"
	stream << "OFF" << endl;

	//2nd line: vertices count / faces count / edges count
	unsigned vertCount = vertices->size();
	unsigned triCount = mesh->size();
	stream << vertCount << ' ' << triCount << ' ' << 0 << endl;

	//save vertices
	{
		for (unsigned i = 0; i < vertCount; ++i)
		{
			const CCVector3* P = vertices->getPoint(i);
			CCVector3d Pglobal = vertices->toGlobal3d<PointCoordinateType>(*P);
			stream << Pglobal.x << ' ' << Pglobal.y << ' ' << Pglobal.z << endl;
		}
	}

	//save triangles
	{
		for (unsigned i = 0; i < triCount; ++i)
		{
			const CCLib::VerticesIndexes* tsi = mesh->getTriangleVertIndexes(i);
			stream << "3 " << tsi->i1 << ' ' << tsi->i2 << ' ' << tsi->i3 << endl;
		}
	}

	return CC_FERR_NO_ERROR;
}


static QString GetNextLine(QTextStream& stream)
{
	QString currentLine;
	//skip comments
	do
	{
		currentLine = stream.readLine();
		//end of file?
		if (currentLine.isNull())
			return QString();
	}
	while (currentLine.startsWith("#") || currentLine.isEmpty());

	return currentLine;
}

CC_FILE_ERROR OFFFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	//try to open file
	QFile fp(filename);
	if (!fp.open(QIODevice::ReadOnly | QIODevice::Text))
		return CC_FERR_READING;

	QTextStream stream(&fp);

	QString currentLine = stream.readLine();
	if (!currentLine.toUpper().startsWith("OFF"))
		return CC_FERR_MALFORMED_FILE;

	//check if the number of vertices/faces/etc. are on the first line (yes it happens :( )
	QStringList tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
	if (tokens.size() == 4)
	{
		tokens.removeAt(0);
	}
	else
	{
		currentLine = GetNextLine(stream);

		//end of file already?!
		if (currentLine.isNull())
			return CC_FERR_MALFORMED_FILE;

		//read the number of vertices/faces
		tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
		if (tokens.size() < 2/*3*/) //should be 3 but we only use the 2 firsts...
			return CC_FERR_MALFORMED_FILE;
	}

	bool ok = false;
	unsigned vertCount = tokens[0].toUInt(&ok);
	if (!ok)
		return CC_FERR_MALFORMED_FILE;
	unsigned triCount = tokens[1].toUInt(&ok);
	if (!ok)
		return CC_FERR_MALFORMED_FILE;

	//create cloud and reserve some memory
	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertCount))
	{
		delete vertices;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	//read vertices
	{
		CCVector3d Pshift(0, 0, 0);
		for (unsigned i = 0; i < vertCount; ++i)
		{
			currentLine = GetNextLine(stream);
			tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
			if (tokens.size() < 3)
			{
				delete vertices;
				return CC_FERR_MALFORMED_FILE;
			}

			//read vertex
			CCVector3d Pd(0, 0, 0);
			{
				bool vertexIsOk = false;
				Pd.x = tokens[0].toDouble(&vertexIsOk);
				if (vertexIsOk)
				{
					Pd.y = tokens[1].toDouble(&vertexIsOk);
					if (vertexIsOk)
						Pd.z = tokens[2].toDouble(&vertexIsOk);
				}
				if (!vertexIsOk)
				{
					delete vertices;
					return CC_FERR_MALFORMED_FILE;
				}
			}

			//first point: check for 'big' coordinates
			if (i == 0)
			{
				bool preserveCoordinateShift = true;
				if (HandleGlobalShift(Pd, Pshift, preserveCoordinateShift, parameters))
				{
					if (preserveCoordinateShift)
					{
						vertices->setGlobalShift(Pshift);
					}
					ccLog::Warning("[OFF] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}
			}

			CCVector3 P = CCVector3::fromArray((Pd + Pshift).u);
			vertices->addPoint(P);
		}
	}

	ccMesh* mesh = new ccMesh(vertices);
	mesh->addChild(vertices);
	if (!mesh->reserve(triCount))
	{
		delete mesh;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	//load triangles
	{
		bool ignoredPolygons = false;
		for (unsigned i = 0; i < triCount; ++i)
		{
			currentLine = GetNextLine(stream);
			tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
			if (tokens.size() < 3)
			{
				delete mesh;
				return CC_FERR_MALFORMED_FILE;
			}

			unsigned polyVertCount = tokens[0].toUInt(&ok);
			if (!ok || static_cast<int>(polyVertCount) >= tokens.size())
			{
				delete mesh;
				return CC_FERR_MALFORMED_FILE;
			}
			if (polyVertCount == 3 || polyVertCount == 4)
			{
				//decode indexes
				unsigned indexes[4];
				for (unsigned j=0; j<polyVertCount; ++j)
				{
					indexes[j] = tokens[j+1].toUInt(&ok);
					if (!ok)
					{
						delete mesh;
						return CC_FERR_MALFORMED_FILE;
					}
				}

				//reserve memory if necessary
				unsigned polyTriCount = polyVertCount-2;
				if (mesh->size() + polyTriCount > mesh->capacity())
				{
					if (!mesh->reserve(mesh->size() + polyTriCount + 256)) //use some margin to avoid too many allocations
					{
						delete mesh;
						return CC_FERR_NOT_ENOUGH_MEMORY;
					}
				}

				//triangle or quad only
				mesh->addTriangle(indexes[0],indexes[1],indexes[2]);
				if (polyVertCount == 4)
					mesh->addTriangle(indexes[0],indexes[2],indexes[3]);

			}
			else
			{
				ignoredPolygons = true;
			}
		}

		if (ignoredPolygons)
		{
			ccLog::Warning("[OFF] Some polygons with an unhandled size (i.e. > 4) were ignored!");
		}
	}

	if (mesh->size() == 0)
	{
		ccLog::Warning("[OFF] Failed to load any polygon!");
		mesh->detachChild(vertices);
		delete mesh;
		mesh = nullptr;

		container.addChild(vertices);
		vertices->setEnabled(true);
	}
	else
	{
		mesh->shrinkToFit();

		//DGM: normals can be per-vertex or per-triangle so it's better to let the user do it himself later
		//Moreover it's not always good idea if the user doesn't want normals (especially in ccViewer!)
		//if (mesh->computeNormals())
		//	mesh->showNormals(true);
		//else
		//	ccLog::Warning("[OFF] Failed to compute per-vertex normals...");
		ccLog::Warning("[OFF] Mesh has no normal! You can manually compute them (select it then call \"Edit > Normals > Compute\")");

		vertices->setEnabled(false);
		//vertices->setLocked(true); //DGM: no need to lock it as it is only used by one mesh!
		container.addChild(mesh);
	}

	return CC_FERR_NO_ERROR;
}
