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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "../include/DRCFilter.h"
#include "../include/SaveDracoFileDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccMesh.h>

//CCCoreLib
#include <CCPlatform.h>

//draco
#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/mesh/mesh.h>
#include <draco/point_cloud/point_cloud.h>

DRCFilter::DRCFilter()
    : FileIOFilter( {
                    "_Draco DRC Filter",
                    12.0f,	// priority
                    QStringList{ "drc" },
                    "drc",
                    QStringList{ "DRC cloud or mesh (*.drc)" },
                    QStringList{ "DRC cloud or mesh (*.drc)" },
                    Import | Export
                    } )
{
}

bool DRCFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == static_cast<CC_CLASS_ENUM>(CC_TYPES::POINT_CLOUD)
		|| type == static_cast<CC_CLASS_ENUM>(CC_TYPES::MESH))
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

static CC_FILE_ERROR CCCloudToDraco(const ccGenericPointCloud& ccCloud, draco::PointCloud& dracoCloud)
{
	unsigned pointCount = ccCloud.size();
	dracoCloud.set_num_points(pointCount);

	draco::DataType dt = draco::DT_FLOAT32;
	bool shifted = ccCloud.isShifted();
	if (shifted)
	{
		dt = draco::DT_FLOAT64;
	}

	// create point attribute
	{
		draco::GeometryAttribute ga;
		ga.Init(draco::GeometryAttribute::POSITION, nullptr, 3, dt, false, DataTypeLength(dt) * 3, 0);
		const int pointAttributeID = dracoCloud.AddAttribute(ga, true, pointCount);
		// retrieve it
		draco::PointAttribute* pointAttribute = dracoCloud.attribute(pointAttributeID);
		if (nullptr == pointAttribute)
		{
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		if (dt == draco::DT_FLOAT32)
		{
			for (draco::PointIndex::ValueType i = 0; i < pointCount; ++i)
			{
				pointAttribute->SetAttributeValue(draco::AttributeValueIndex(i), ccCloud.getPoint(i)->u);
			}
		}
		else //draco::DT_FLOAT64
		{
			for (draco::PointIndex::ValueType i = 0; i < pointCount; ++i)
			{
				CCVector3 Plocal = *(ccCloud.getPoint(i));
				pointAttribute->SetAttributeValue(draco::AttributeValueIndex(i), ccCloud.toGlobal3d(Plocal).u);
			}
		}
	}

	// create normal attribute (if any)
	if (ccCloud.hasNormals())
	{
		draco::GeometryAttribute ga;
		ga.Init(draco::GeometryAttribute::NORMAL, nullptr, 3, draco::DT_FLOAT32, true, DataTypeLength(draco::DT_FLOAT32) * 3, 0);
		const int normalAttributeID = dracoCloud.AddAttribute(ga, true, pointCount);
		// retrieve it
		draco::PointAttribute* normalAttribute = dracoCloud.attribute(normalAttributeID);
		if (nullptr != normalAttribute)
		{
			for (draco::PointIndex::ValueType i = 0; i < pointCount; ++i)
			{
				normalAttribute->SetAttributeValue(draco::AttributeValueIndex(i), ccCloud.getPointNormal(i).u);
			}
		}
		else
		{
			ccLog::Warning("[DRACO] Failed to export normals");
		}
	}

	// create color attribute (if any)
	if (ccCloud.hasColors())
	{
		draco::GeometryAttribute ga;
		ga.Init(draco::GeometryAttribute::COLOR, nullptr, 4, draco::DT_UINT8, false, DataTypeLength(draco::DT_UINT8) * 4, 0);
		const int colorAttributeID = dracoCloud.AddAttribute(ga, true, pointCount);
		// retrieve it
		draco::PointAttribute* colorAttribute = dracoCloud.attribute(colorAttributeID);
		if (nullptr != colorAttribute)
		{
			for (draco::PointIndex::ValueType i = 0; i < pointCount; ++i)
			{
				colorAttribute->SetAttributeValue(draco::AttributeValueIndex(i), ccCloud.getPointColor(i).rgba);
			}
		}
		else
		{
			ccLog::Warning("[DRACO] Failed to export normals");
		}
	}

	// create generic attribute (if any)
	if (ccCloud.hasScalarFields())
	{
		if (ccCloud.isA(CC_TYPES::POINT_CLOUD))
		{
			const ccPointCloud& cc = static_cast<const ccPointCloud&>(ccCloud);
			// DGM: it seems DRACO supports only one "Generic" field
			if (cc.getNumberOfScalarFields() > 1)
			{
				ccLog::Warning("[DRACO] Cloud %1 has multiple scalar fields, however, only one can be saved (the active one by default)");
			}
		}

		draco::GeometryAttribute ga;
		ga.Init(draco::GeometryAttribute::GENERIC, nullptr, 1, draco::DT_FLOAT32, false, DataTypeLength(draco::DT_FLOAT32), 0);
		const int sfAttributeID = dracoCloud.AddAttribute(ga, true, pointCount);
		// retrieve it
		draco::PointAttribute* sfAttribute = dracoCloud.attribute(sfAttributeID);
		if (nullptr != sfAttribute)
		{
			for (draco::PointIndex::ValueType i = 0; i < pointCount; ++i)
			{
				float sfValue = ccCloud.getPointScalarValue(i);
				sfAttribute->SetAttributeValue(draco::AttributeValueIndex(i), &sfValue);
			}
		}
		else
		{
			ccLog::Warning("[DRACO] Failed to export active scalar field");
		}
	}

#if 0 // DGM: it seems DRACO supports only one "Generic" field
	// create SF attributes (if any)
	if (ccCloud.isA(CC_TYPES::POINT_CLOUD) && ccCloud.hasScalarFields())
	{
		const ccPointCloud& cc = static_cast<const ccPointCloud&>(ccCloud);
		for (unsigned k = 0; k < cc.getNumberOfScalarFields(); ++k)
		{
			CCCoreLib::ScalarField* sf = cc.getScalarField(static_cast<int>(k));

			draco::GeometryAttribute ga;
			ga.Init(draco::GeometryAttribute::GENERIC, nullptr, 1, draco::DT_FLOAT32, false, DataTypeLength(draco::DT_FLOAT32), 0);
			const int sfAttributeID = dracoCloud.AddAttribute(ga, true, pointCount);
			// retrieve it
			draco::PointAttribute* sfAttribute = dracoCloud.attribute(sfAttributeID);
			if (nullptr != sfAttribute)
			{
				for (draco::PointIndex::ValueType i = 0; i < pointCount; ++i)
				{
					sfAttribute->SetAttributeValue(draco::AttributeValueIndex(i), &sf->getValue(i));
				}
			}
			else
			{
				ccLog::Warning("[DRACO] Failed to export scalar field " + QString(sf->getName()));
			}

		}
	}
#endif

	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR CCMeshToDraco(ccGenericMesh& ccMesh, draco::Mesh& dracoMesh)
{
	unsigned faceCount = ccMesh.size();
	dracoMesh.SetNumFaces(faceCount);

	// save triangles
	draco::FaceIndex faceIndex(0);
	for (unsigned i = 0; i < faceCount; ++i)
	{
		const auto tri = ccMesh.getTriangleVertIndexes(i);
		draco::Mesh::Face face;
		{
			face[0] = tri->i1;
			face[1] = tri->i2;
			face[2] = tri->i3;
		}
		dracoMesh.SetFace(faceIndex, face);
		faceIndex++;
	}

	ccGenericPointCloud* vertices = ccMesh.getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return CC_FILE_ERROR::CC_FERR_BAD_ARGUMENT;
	}

#if 0 // DGM: useless without a texture!
	// save texture coordinates
	TextureCoordsContainer* texCoords = ccMesh.getTexCoordinatesTable();
	unsigned vertexCount = vertices->size();
	if (texCoords && texCoords->size() == vertexCount)
	{
		//dracoMesh.SetAttribute
		draco::GeometryAttribute ga;
		ga.Init(draco::GeometryAttribute::TEX_COORD, nullptr, 2, draco::DT_FLOAT32, false, DataTypeLength(draco::DT_FLOAT32) * 2, 0);
		const int texCoordAttributeID = dracoMesh.AddAttribute(ga, true, vertexCount);
		// retrieve it
		draco::PointAttribute* texCoordAttribute = dracoMesh.attribute(texCoordAttributeID);
		if (nullptr != texCoordAttribute)
		{
			for (draco::PointIndex::ValueType i = 0; i < vertexCount; ++i)
			{
				texCoordAttribute->SetAttributeValue(draco::AttributeValueIndex(i), texCoords->at(i).t);
			}
		}
		else
		{
			ccLog::Warning("[DRACO] Failed to export texture coordinates");
		}
	}
#endif

	// save vertices
	CC_FILE_ERROR error = CCCloudToDraco(*vertices, dracoMesh);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR DRCFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (nullptr == entity)
	{
		assert(false);
		return CC_FERR_BAD_ARGUMENT;
	}
	
	draco::Encoder encoder;
	encoder.SetSpeedOptions(0, 0);

	int coordQuantization = 11;
	int texCoordQuantization = 10;
	int normalQuantization = 8;
	int sfQuantization = 8;

	// we always create the dialog, even if we don't display it, to retrieve the default values
	SaveDracoFileDlg drcDialog(parameters.parentWidget);
	if (parameters.parentWidget && parameters.alwaysDisplaySaveDialog)
	{
		if (drcDialog.exec() == 0)
		{
			return CC_FILE_ERROR::CC_FERR_CANCELED_BY_USER;
		}
	}
	
	coordQuantization = drcDialog.coordsQuantSpinBox->value();
	//texCoordQuantization = XXX; //not available yet since we don't know how to save the texture!
	normalQuantization = drcDialog.normQuantSpinBox->value();
	sfQuantization = drcDialog.sfQuantSpinBox->value();

	encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, coordQuantization);
	encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, texCoordQuantization);
	encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, normalQuantization);
	encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, sfQuantization);

	draco::EncoderBuffer buffer;
	if (entity->isKindOf(CC_TYPES::MESH))
	{
		ccGenericMesh* ccMesh = static_cast<ccGenericMesh*>(entity);

		// save mesh
		draco::Mesh dracoMesh;
		CC_FILE_ERROR error = CCMeshToDraco(*ccMesh, dracoMesh);
		if (error != CC_FERR_NO_ERROR)
		{
			return error;
		}

		if (!encoder.EncodeMeshToBuffer(dracoMesh, &buffer).ok())
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}
	}
	else if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* ccCloud = static_cast<ccGenericPointCloud*>(entity);

		// save cloud
		draco::PointCloud dracoCloud;
		CC_FILE_ERROR error = CCCloudToDraco(*ccCloud, dracoCloud);
		if (error != CC_FERR_NO_ERROR)
		{
			return error;
		}

		if (!encoder.EncodePointCloudToBuffer(dracoCloud, &buffer).ok())
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}
	}
	else
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	QFile file(filename);
	if (!file.open(QFile::WriteOnly))
	{
		return CC_FERR_WRITING;
	}
	file.write(buffer.data(), static_cast<qint64>(buffer.size()));

	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR LoadCloud(ccPointCloud& ccCloud, const draco::PointCloud& dracoCloud, FileIOFilter::LoadParameters& parameters)
{
	if (dracoCloud.num_points() == 0)
	{
		return CC_FERR_NO_LOAD;
	}
	
	unsigned pointCount = dracoCloud.num_points();
	if (!ccCloud.reserve(pointCount))
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	// load vertices
	const draco::PointAttribute* pointAttribute = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::POSITION);
	if (nullptr == pointAttribute)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}
	if (pointCount != pointAttribute->size())
	{
		return CC_FERR_MALFORMED_FILE;
	}
	draco::DataType dt = pointAttribute->data_type();
	if (dt == draco::DT_FLOAT32)
	{
		for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(pointAttribute->size()); ++i)
		{
			CCVector3f p;
			pointAttribute->GetValue(i, p.u);
			ccCloud.addPoint(p);
		}
	}
	else if (dt == draco::DT_FLOAT64)
	{
		CCVector3d Pshift(0, 0, 0);
		bool preserveCoordinateShift = true;
		for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(pointAttribute->size()); ++i)
		{
			CCVector3d P;
			pointAttribute->GetValue(i, P.u);

			//first point: check for large coordinates
			if (i == 0)
			{
				if (FileIOFilter::HandleGlobalShift(P, Pshift, preserveCoordinateShift, parameters))
				{
					if (preserveCoordinateShift)
					{
						ccCloud.setGlobalShift(Pshift);
					}
					ccLog::Warning("[DRACO] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}
			}

			ccCloud.addPoint((P + Pshift).toPC());
		}
	}

	// load normals?
	const draco::PointAttribute* normalAttribute = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::NORMAL);
	if (	(nullptr != normalAttribute)
		&&	(normalAttribute->data_type() == draco::DataType::DT_FLOAT32)
		&&	(pointCount == normalAttribute->size())
		)
	{
		if (ccCloud.reserveTheNormsTable())
		{
			for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(normalAttribute->size()); ++i)
			{
				draco::Vector3f n;
				normalAttribute->GetValue(i, &n[0]);
				ccCloud.addNorm(CCVector3f::fromArray(n.data()));
			}
			ccCloud.showNormals(true);
		}
		else
		{
			ccLog::Warning("Failed to load normals (not enough memory)");
		}
	}

	// load colors?
	const draco::PointAttribute* colorAttribute = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::COLOR);
	if (	(nullptr != colorAttribute)
		&&	(colorAttribute->data_type() == draco::DataType::DT_UINT8)
		&&	(pointCount == colorAttribute->size())
		)
	{
		bool rgb = (colorAttribute->num_components() == 3);
		bool rgba = (colorAttribute->num_components() == 4);
		if (rgb || rgba)
		{
			if (ccCloud.reserveTheRGBTable())
			{
				if (rgb)
				{
					std::array<uint8_t, 3> col;
					for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(colorAttribute->size()); ++i)
					{
						colorAttribute->GetValue(i, &col[0]);
						ccCloud.addColor(ccColor::Rgb(col.data()));
					}
				}
				else if (rgba)
				{
					std::array<uint8_t, 4> col;
					for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(colorAttribute->size()); ++i)
					{
						colorAttribute->GetValue(i, &col[0]);
						ccCloud.addColor(ccColor::Rgba(col.data()));
					}
				}
				ccCloud.showColors(true);
			}
			else
			{
				ccLog::Warning("Failed to load colors (not enough memory)");
			}
		}
	}

	// load SF?
	const draco::PointAttribute* sfAttribute = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::GENERIC);
	if (	(nullptr != sfAttribute)
		&&	(sfAttribute->data_type() == draco::DataType::DT_FLOAT32)
		&&	(sfAttribute->size() == pointCount)
		)
	{
		ccScalarField* sf = new ccScalarField();
		if (sf->reserveSafe(pointCount))
		{
			for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(sfAttribute->size()); ++i)
			{
				float sfValue = 0;
				sfAttribute->GetValue(i, &sfValue);
				sf->addElement(sfValue);
			}
			sf->computeMinAndMax();
			ccCloud.addScalarField(sf);
			ccCloud.setCurrentDisplayedScalarField(0);
			ccCloud.showSF(true);
		}
		else
		{
			ccLog::Warning("Failed to load generic field (not enough memory)");
		}
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR DRCFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	draco::DecoderBuffer buffer;

	QFile file(filename);
	if (!file.open(QFile::ReadOnly))
	{
		return CC_FERR_READING;
	}

	QByteArray byteArray = file.readAll();
	buffer.Init(byteArray.data(), byteArray.size());

	const auto result = draco::Decoder::GetEncodedGeometryType(&buffer);
	if (!result.ok())
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	const draco::EncodedGeometryType geomType = result.value();
	if (geomType == draco::TRIANGULAR_MESH)
	{
		const auto resultMesh = draco::Decoder().DecodeMeshFromBuffer(&buffer);
		if (!resultMesh.ok())
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}
		const std::unique_ptr<draco::Mesh>& meshDraco = resultMesh.value();
		const draco::PointAttribute *const pointAttribute = meshDraco->GetNamedAttribute(draco::GeometryAttribute::POSITION);
		if (!pointAttribute)
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}
		ccLog::Print("[DRACO] Mesh size: " + QString::number(meshDraco->num_faces()) + " / vertex count: " + QString::number(meshDraco->num_points()));

		ccPointCloud* vertices = new ccPointCloud("vertices");
		CC_FILE_ERROR verticesError = LoadCloud(*vertices, *meshDraco, parameters);
		if (CC_FERR_NO_ERROR != verticesError)
		{
			delete vertices;
			return verticesError;
		}

		ccMesh* meshCC = new ccMesh(vertices);
		meshCC->addChild(vertices);
		vertices->setVisible(false);
		if (vertices->hasNormals())
		{
			meshCC->showNormals(true);
		}
		if (vertices->hasColors())
		{
			meshCC->showColors(true);
		}

		if (!meshCC->reserve(meshDraco->num_faces()))
		{
			delete meshCC;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		// load faces
		for (draco::FaceIndex f(0); f < meshDraco->num_faces(); ++f)
		{
			const draco::Mesh::Face& face = meshDraco->face(f);
			assert(face.size() == 3);
			uint32_t i1 = pointAttribute->mapped_index(face[0]).value();
			uint32_t i2 = pointAttribute->mapped_index(face[1]).value();
			uint32_t i3 = pointAttribute->mapped_index(face[2]).value();
			meshCC->addTriangle(i1, i2, i3);
		}

		container.addChild(meshCC);
	}
	else if (geomType == draco::POINT_CLOUD)
	{
		const auto resultCloud = draco::Decoder().DecodePointCloudFromBuffer(&buffer);
		if (!resultCloud.ok())
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}
		const std::unique_ptr<draco::PointCloud>& cloudDraco = resultCloud.value();
		ccLog::Print("[DRACO] Cloud size: " + QString::number(cloudDraco->num_points()));

		ccPointCloud* cloudCC = new ccPointCloud("unnamed - Cloud");
		CC_FILE_ERROR cloudError = LoadCloud(*cloudCC, *cloudDraco, parameters);
		if (CC_FERR_NO_ERROR != cloudError)
		{
			delete cloudCC;
			return cloudError;
		}

		container.addChild(cloudCC);
	}

	return CC_FERR_NO_ERROR;
}
