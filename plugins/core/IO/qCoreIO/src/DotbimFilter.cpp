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
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

#include "DotbimFilter.h"

// Qt
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QMap>

// qCC_db
#include <ccGLMatrix.h>
#include <ccLog.h>
#include <ccMesh.h>
#include <ccPointCloud.h>

// System
#include <array>
#include <vector>

DotbimFilter::DotbimFilter()
    : FileIOFilter({"_DOTBIM Filter",
                    DEFAULT_PRIORITY, // priority
                    QStringList{"bim"},
                    "bim",
                    QStringList{"dotBIM mesh (*.bim)"},
                    QStringList(),
                    Import})
{
}

CC_FILE_ERROR DotbimFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	Q_UNUSED(parameters);

	ccLog::Print(QString("[dotBIM] Loading '%1'").arg(filename));

	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
	{
		return CC_FERR_READING;
	}

	QJsonParseError parseError;
	QJsonDocument   doc = QJsonDocument::fromJson(file.readAll(), &parseError);
	file.close();

	if (parseError.error != QJsonParseError::NoError || !doc.isObject())
	{
		ccLog::Warning(QString("[dotBIM] Invalid JSON: %1").arg(parseError.errorString()));
		return CC_FERR_MALFORMED_FILE;
	}

	QJsonObject root = doc.object();
	if (!root.value("meshes").isArray() || !root.value("elements").isArray())
	{
		ccLog::Warning("[dotBIM] File is missing the 'meshes' or 'elements' array");
		return CC_FERR_MALFORMED_FILE;
	}

	// index the mesh geometries by mesh_id
	struct MeshGeometry
	{
		std::vector<CCVector3d>              vertices;
		std::vector<std::array<unsigned, 3>> triangles;
	};
	QMap<int, MeshGeometry> meshGeometries;

	for (const QJsonValue& meshVal : root.value("meshes").toArray())
	{
		QJsonObject meshObj = meshVal.toObject();
		int         meshId  = meshObj.value("mesh_id").toInt(-1);
		if (meshId < 0)
		{
			continue;
		}

		MeshGeometry geom;

		QJsonArray coordinates = meshObj.value("coordinates").toArray();
		geom.vertices.reserve(static_cast<size_t>(coordinates.size() / 3));
		for (int i = 0; i + 2 < coordinates.size(); i += 3)
		{
			geom.vertices.emplace_back(coordinates[i].toDouble(),
			                           coordinates[i + 1].toDouble(),
			                           coordinates[i + 2].toDouble());
		}

		QJsonArray indices = meshObj.value("indices").toArray();
		geom.triangles.reserve(static_cast<size_t>(indices.size() / 3));
		for (int i = 0; i + 2 < indices.size(); i += 3)
		{
			unsigned a = static_cast<unsigned>(indices[i].toInt(-1));
			unsigned b = static_cast<unsigned>(indices[i + 1].toInt(-1));
			unsigned c = static_cast<unsigned>(indices[i + 2].toInt(-1));
			if (a >= geom.vertices.size() || b >= geom.vertices.size() || c >= geom.vertices.size())
			{
				ccLog::Warning(QString("[dotBIM] mesh_id %1 has an out-of-range triangle index, skipped").arg(meshId));
				continue;
			}
			geom.triangles.push_back({a, b, c});
		}

		meshGeometries.insert(meshId, geom);
	}

	unsigned loadedCount = 0;

	for (const QJsonValue& elemVal : root.value("elements").toArray())
	{
		QJsonObject elemObj = elemVal.toObject();
		int         meshId  = elemObj.value("mesh_id").toInt(-1);
		QString     guid    = elemObj.value("guid").toString();

		auto geomIt = meshGeometries.constFind(meshId);
		if (geomIt == meshGeometries.constEnd() || geomIt->vertices.empty() || geomIt->triangles.empty())
		{
			ccLog::Warning(QString("[dotBIM] Element '%1' references unknown or empty mesh_id %2, skipped").arg(guid).arg(meshId));
			continue;
		}
		const MeshGeometry& geom = *geomIt;

		// rotate then translate (dotBIM applies rotation first, see the format's DeveloperTips)
		QJsonObject rotationObj = elemObj.value("rotation").toObject();
		double      quaternion[4] // (w, x, y, z)
		    = {rotationObj.value("qw").toDouble(1.0),
		       rotationObj.value("qx").toDouble(0.0),
		       rotationObj.value("qy").toDouble(0.0),
		       rotationObj.value("qz").toDouble(0.0)};
		ccGLMatrixd transform = ccGLMatrixd::FromQuaternion(quaternion);

		QJsonObject vectorObj = elemObj.value("vector").toObject();
		transform.setTranslation(CCVector3d(vectorObj.value("x").toDouble(),
		                                    vectorObj.value("y").toDouble(),
		                                    vectorObj.value("z").toDouble()));

		ccPointCloud* vertices = new ccPointCloud("vertices");
		if (!vertices->reserve(static_cast<unsigned>(geom.vertices.size())))
		{
			delete vertices;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		for (const CCVector3d& rawP : geom.vertices)
		{
			CCVector3d P = rawP;
			transform.apply(P);
			vertices->addPoint(P.toPC());
		}

		ccMesh* mesh = new ccMesh(vertices);
		mesh->addChild(vertices);
		QString elementType = elemObj.value("type").toString();
		mesh->setName(!elementType.isEmpty() ? elementType : (!guid.isEmpty() ? guid : "dotBIM element"));

		if (!mesh->reserve(static_cast<unsigned>(geom.triangles.size())))
		{
			delete mesh; // also deletes 'vertices', already added as its child
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		for (const std::array<unsigned, 3>& tri : geom.triangles)
		{
			mesh->addTriangle(tri[0], tri[1], tri[2]);
		}

		// mandatory 'color' field: applied as a uniform per-vertex color
		// ('face_colors', optional and taking precedence since schema 1.1.0, is not supported yet)
		QJsonObject colorObj = elemObj.value("color").toObject();
		if (!colorObj.isEmpty())
		{
			vertices->setColor(static_cast<ColorCompType>(colorObj.value("r").toInt(255)),
			                   static_cast<ColorCompType>(colorObj.value("g").toInt(255)),
			                   static_cast<ColorCompType>(colorObj.value("b").toInt(255)),
			                   static_cast<ColorCompType>(colorObj.value("a").toInt(255)));
			vertices->showColors(true);
			mesh->showColors(true);
		}

		vertices->setEnabled(false);
		container.addChild(mesh);
		++loadedCount;
	}

	if (loadedCount == 0)
	{
		return CC_FERR_NO_LOAD;
	}

	ccLog::Print(QString("[dotBIM] %1 element(s) loaded").arg(loadedCount));

	return CC_FERR_NO_ERROR;
}
