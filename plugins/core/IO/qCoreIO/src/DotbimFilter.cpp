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
#include <memory>
#include <new>

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
try
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

	QMap<int, QJsonObject> meshDefinitions;

	for (QJsonValue meshVal : root.value("meshes").toArray())
	{
		QJsonObject meshObj = meshVal.toObject();
		int         meshId  = meshObj.value("mesh_id").toInt(-1);
		if (meshId < 0)
		{
			continue;
		}

		if (meshDefinitions.contains(meshId))
		{
			ccLog::Warning(QString("[dotBIM] Duplicate mesh ID: %1").arg(meshId));
			return CC_FERR_MALFORMED_FILE;
		}

		meshDefinitions.insert(meshId, meshObj);
	}

	unsigned loadedCount = 0;

	for (QJsonValue elemVal : root.value("elements").toArray())
	{
		QJsonObject elemObj = elemVal.toObject();
		int         meshId  = elemObj.value("mesh_id").toInt(-1);
		QString     guid    = elemObj.value("guid").toString();

		auto meshIt = meshDefinitions.constFind(meshId);
		if (meshIt == meshDefinitions.constEnd())
		{
			ccLog::Warning(QString("[dotBIM] Element '%1' references unknown or empty mesh_id %2, skipped").arg(guid).arg(meshId));
			continue;
		}
		QJsonObject meshObj     = *meshIt;
		QJsonArray  coordinates = meshObj.value("coordinates").toArray();
		QJsonArray  indices     = meshObj.value("indices").toArray();
		if (coordinates.size() < 3 || indices.size() < 3)
		{
			ccLog::Warning(QString("[dotBIM] Element '%1' references unknown or empty mesh_id %2, skipped").arg(guid).arg(meshId));
			continue;
		}
		if (indices.size() % 3 != 0)
		{
			ccLog::Warning(QString("[dotBIM] mesh_id %1 has an invalid number of triangle indices, skipped").arg(meshId));
			continue;
		}
		if (coordinates.size() % 3 != 0)
		{
			ccLog::Warning(QString("[dotBIM] mesh_id %1 has an invalid number of vertex coordinates, skipped").arg(meshId));
			continue;
		}

		unsigned triCount    = static_cast<unsigned>(indices.size() / 3);
		unsigned vertexCount = static_cast<unsigned>(coordinates.size() / 3);

		ccPointCloud* vertices = new ccPointCloud("vertices");
		ccMesh*       mesh     = new ccMesh(vertices);
		mesh->addChild(vertices);
		vertices->setEnabled(false);

		if (!vertices->reserve(vertexCount) || !mesh->reserve(triCount))
		{
			delete mesh;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		QString elementType = elemObj.value("type").toString();
		mesh->setName(!elementType.isEmpty() ? elementType : (!guid.isEmpty() ? guid : "dotBIM element"));

		// transfer the vertex coordinates to the point cloud
		for (qsizetype i = 0; i + 2 < coordinates.size(); i += 3)
		{
			CCVector3d P(coordinates[i + 0].toDouble(),
			             coordinates[i + 1].toDouble(),
			             coordinates[i + 2].toDouble());
			vertices->addPoint(P.toPC());
		}

		if (elemObj.contains("rotation") || elemObj.contains("vector"))
		{
			// rotate then translate (dotBIM applies rotation first, see the format's DeveloperTips)
			QJsonObject rotationObj = elemObj.value("rotation").toObject();
			double      quaternion[4] // (w, x, y, z)
			    {rotationObj.value("qw").toDouble(1.0),
			     rotationObj.value("qx").toDouble(0.0),
			     rotationObj.value("qy").toDouble(0.0),
			     rotationObj.value("qz").toDouble(0.0)};
			ccGLMatrixd transform = ccGLMatrixd::FromQuaternion(quaternion);

			QJsonObject vectorObj = elemObj.value("vector").toObject();
			transform.setTranslation(CCVector3d(vectorObj.value("x").toDouble(),
			                                    vectorObj.value("y").toDouble(),
			                                    vectorObj.value("z").toDouble()));

			vertices->applyRigidTransformation(ccGLMatrix(transform.data()));
		}

		// transfer the triangle indexes to the mesh
		for (qsizetype i = 0; i + 2 < indices.size(); i += 3)
		{
			int a = indices[i + 0].toInt(-1);
			int b = indices[i + 1].toInt(-1);
			int c = indices[i + 2].toInt(-1);
			if (a < 0
			    || b < 0
			    || c < 0
			    || static_cast<unsigned>(a) >= vertexCount
			    || static_cast<unsigned>(b) >= vertexCount
			    || static_cast<unsigned>(c) >= vertexCount)
			{
				ccLog::Warning(QString("[dotBIM] mesh_id %1 has an out-of-range triangle index, skipped").arg(meshId));
				continue;
			}
			mesh->addTriangle(a, b, c);
		}

		if (mesh->size() == 0)
		{
			ccLog::Warning(QString("[dotBIM] Element '%1' references unknown or empty mesh_id %2, skipped").arg(guid).arg(meshId));
			delete mesh;
			continue;
		}

		// mandatory 'color' field: applied as a uniform per-vertex color
		// ('face_colors', optional and taking precedence since schema 1.1.0, is not supported yet)
		QJsonObject colorObj = elemObj.value("color").toObject();
		if (!colorObj.isEmpty())
		{
			if (vertices->setColor(static_cast<ColorCompType>(colorObj.value("r").toInt(255)),
			                       static_cast<ColorCompType>(colorObj.value("g").toInt(255)),
			                       static_cast<ColorCompType>(colorObj.value("b").toInt(255)),
			                       static_cast<ColorCompType>(colorObj.value("a").toInt(255))))
			{
				vertices->showColors(true);
				mesh->showColors(true);
			}
			else
			{
				ccLog::Warning(QString("[dotBIM] Not enough memory to set color for Element '%1' - mesh_id %2").arg(guid).arg(meshId));
			}
		}

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
catch (const std::bad_alloc&)
{
	return CC_FERR_NOT_ENOUGH_MEMORY;
}
