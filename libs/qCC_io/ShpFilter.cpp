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

#ifdef CC_SHP_SUPPORT

#include "ShpFilter.h"

//Local
#include "ui_saveSHPFileDlg.h"
#include "ui_importDBFFieldDlg.h"
#include "ShpDBFFields.h"

//qCC_db
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>

//Qt
#include <QString>
#include <QFile>
#include <QFileInfo>
#include <QApplication>
#include <QtEndian>
#include <QDialog>
#include <QMap>

//CCLib
#include <Neighbourhood.h>
#include <MeshSamplingTools.h>

//System
#include <string.h>
#include <array>

//Specific value for NaN
static const double ESRI_NO_DATA = -1.0e38;

//! ESRI Shapefile's shape types
enum ESRI_SHAPE_TYPE {	SHP_NULL_SHAPE		= 0 ,
						//below this point are 2D types
						SHP_POINT			= 1	,
						SHP_POLYLINE		= 3 ,
						SHP_POLYGON			= 5 ,
						SHP_MULTI_POINT		= 8 ,
						//below this point are 3D types
						SHP_POINT_Z			= 11,
						SHP_POLYLINE_Z		= 13,
						SHP_POLYGON_Z		= 15,
						SHP_MULTI_POINT_Z	= 18,
						SHP_POINT_M			= 21,
						SHP_POLYLINE_M		= 23,
						SHP_POLYGON_M		= 25,
						SHP_MULTI_POINT_M	= 28,
						SHP_MULTI_PATCH		= 31
};

//DGM: by default qToLittleEndian and qFromLittleEndian only works for integer types!
double swapD(double in)
{
	//! Change the endianness (see https://stackoverflow.com/questions/41012414/convert-double-value-from-little-endian-to-big-endian)
	std::array<char, sizeof(double)> p;
	memcpy(&p[0], &in, sizeof(double));
	std::reverse(p.begin(), p.end());
	memcpy(&in, &p[0], sizeof(double));
	return in;
}

double qFromLittleEndianD(double in)
{
#if Q_BYTE_ORDER == Q_BIG_ENDIAN
	return swapD(in);
#endif
	return in;
}

double qToLittleEndianD(double in)
{
#if Q_BYTE_ORDER == Q_BIG_ENDIAN
	return swapD(in);
#endif
	return in;
}


//! Shape File Save dialog
class SaveSHPFileDialog : public QDialog, public Ui::SaveSHPFileDlg
{
public:
	//! Default constructor
	explicit SaveSHPFileDialog(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::SaveSHPFileDlg()
	{
		setupUi(this);
	}
};

//! Shape File Load dialog (to choose an 'altitude' field)
class ImportDBFFieldDialog : public QDialog, public Ui::ImportDBFFieldDlg
{
public:
	//! Default constructor
	explicit ImportDBFFieldDialog(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::ImportDBFFieldDlg()
	{
		setupUi(this);
	}
};

bool ShpFilter::canLoadExtension(const QString& upperCaseExt) const
{
	return (upperCaseExt == "SHP");
}

bool ShpFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POLY_LINE
		|| type == CC_TYPES::POINT_CLOUD
		|| type == CC_TYPES::HIERARCHY_OBJECT
		/*||	type == CC_TYPES::MESH*/)
	{
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

QString ToString(ESRI_SHAPE_TYPE type)
{
	switch (type)
	{
	case SHP_NULL_SHAPE:
		return "Unhandled";
	case SHP_POINT:
		return "2D point";
	case SHP_POLYLINE:
		return "2D polyline";
	case SHP_POLYGON:
		return "2D polygon";
	case SHP_MULTI_POINT:
		return "2D point cloud";
	case SHP_POINT_Z:
		return "3D point";
	case SHP_POLYLINE_Z:
		return "3D polyline";
	case SHP_POLYGON_Z:
		return "3D polygon";
	case SHP_MULTI_POINT_Z:
		return "3D point cloud";
	case SHP_POINT_M:
		return "2D point (+measure)";
	case SHP_POLYLINE_M:
		return "2D polyline (+measure)";
	case SHP_POLYGON_M:
		return "2D polygon (+measure)";
	case SHP_MULTI_POINT_M:
		return "2D point cloud (+measure)";
	case SHP_MULTI_PATCH:
		return "Multi patch";
	default:
		assert(false);
		break;
	}

	return QString("Unknown");
}

void GetSupportedShapes(ccHObject* baseEntity, ccHObject::Container& shapes, ESRI_SHAPE_TYPE& shapeType)
{
	shapeType = SHP_NULL_SHAPE;
	if (!baseEntity)
	{
		assert(false);
		shapes.clear();
		return;
	}

	switch (baseEntity->getClassID())
	{
	case CC_TYPES::POINT_CLOUD:
	{
		unsigned count = ccHObjectCaster::ToGenericPointCloud(baseEntity)->size();
		if (count != 0)
		{
			shapeType = SHP_MULTI_POINT_Z;
			shapes.push_back(baseEntity);
		}
	}
	break;
	//DGM: TODO
	//case CC_MESH:
	//case CC_SUB_MESH:
	//	{
	//		unsigned count = ccHObjectCaster::ToGenericMesh(baseEntity)->size();
	//		if (count != 0)
	//		{
	//			shapeType = SHP_MULTI_PATCH;
	//			shapes.push_back(baseEntity);
	//		}
	//	}
	//	break;
	case CC_TYPES::POLY_LINE:
	{
		ccPolyline* poly = static_cast<ccPolyline*>(baseEntity);
		shapeType = poly->is2DMode() ? SHP_POLYLINE : SHP_POLYLINE_Z;
		shapes.push_back(baseEntity);
		break;
	}
	case CC_TYPES::HIERARCHY_OBJECT:
		//we only allow groups with children of the same type!
		if (baseEntity->getChildrenNumber())
		{
			ccHObject* child = baseEntity->getChild(0);
			assert(child);
			if (!child)
				return;

			//first we check that all entities have the same type
			{
				for (unsigned i = 1; i < baseEntity->getChildrenNumber(); ++i)
				{
					if (baseEntity->getChild(i) && baseEntity->getChild(i)->getClassID() != child->getClassID())
					{
						//mixed shapes are not allowed in shape files (yet?)
						return;
					}
				}
			}

			//call the same method on the first child so as to get its type
			GetSupportedShapes(child, shapes, shapeType/*,closedPolylinesAsPolygons*/);
			if (shapeType == SHP_NULL_SHAPE)
				return;

			//then add the remaining children
			{
				for (unsigned i = 1; i < baseEntity->getChildrenNumber(); ++i)
				{
					ESRI_SHAPE_TYPE otherShapeType = SHP_NULL_SHAPE;
					ccHObject* child = baseEntity->getChild(i);
					if (child)
						GetSupportedShapes(child, shapes, otherShapeType);

					if (otherShapeType != shapeType)
					{
						if (child)
							ccLog::Warning(QString("[SHP] Entity %1 has not the same type (%2) as the others in the selection (%3)! Can't mix types...")
							.arg(child->getName())
							.arg(ToString(otherShapeType))
							.arg(ToString(shapeType)));
						//mixed shapes are not allowed in shape files (yet?)
						shapes.clear();
						return;
					}
				}
			}
		}
		break;
	default:
		//nothing to do
		break;
	}
}

CC_FILE_ERROR LoadPolyline(QFile& file,
	ccHObject& container,
	int32_t index,
	ESRI_SHAPE_TYPE shapeTypeInt,
	const CCVector3d& PShift,
	bool load2DPolyAs3DPoly = true)
{
	char header[40];
	file.read(header, 40);
	//check for errors
	if (file.error() != QFile::NoError)
		return CC_FERR_READING;

	//Byte 0: Box
	{
		//The Bounding Box for the PolyLine stored in the order Xmin, Ymin, Xmax, Ymax
		//DGM: ignored
		//double xMin = qFromLittleEndianD(*reinterpret_cast<double*>(header   ));
		//double xMax = qFromLittleEndianD(*reinterpret_cast<double*>(header+ 8));
		//double yMin = qFromLittleEndianD(*reinterpret_cast<double*>(header+16));
		//double yMax = qFromLittleEndianD(*reinterpret_cast<double*>(header+24));
	}

	//Byte 32: NumParts (The number of parts in the PolyLine)
	int32_t numParts = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header + 32));

	//Byte 36: NumPoints (The total number of points for all parts)
	int32_t numPoints = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header + 36));

	//Byte 40: Parts (An array of length NumParts)
	//for each part, the index of its first point in the points array
	std::vector<int32_t> startIndexes;
	{
		try
		{
			startIndexes.resize(numParts, 0);
		}
		catch (const std::bad_alloc&)
		{
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		for (int32_t i = 0; i != numParts; ++i)
		{
			file.read(header, 4);
			startIndexes[i] = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header));
		}
		//FIXME: we should use this information and create as many polylines as necessary!
	}

	//Points (An array of length NumPoints)
	std::vector<CCVector3> points;
	try
	{
		points.resize(numPoints);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}
	{
		for (int32_t i = 0; i < numPoints; ++i)
		{
			file.read(header, 16);
			//check for errors
			if (file.error() != QFile::NoError)
				return CC_FERR_READING;
			double x = qFromLittleEndianD(*reinterpret_cast<double*>(header));
			double y = qFromLittleEndianD(*reinterpret_cast<double*>(header + 8));
			points[i].x = static_cast<PointCoordinateType>(x + PShift.x);
			points[i].y = static_cast<PointCoordinateType>(y + PShift.y);
			points[i].z = 0;
		}
	}

	//3D polylines
	bool is3D = (shapeTypeInt > SHP_POINT_Z && shapeTypeInt < SHP_POINT_M);
	if (is3D)
	{
		//Z boundaries
		{
			file.read(header, 16);
			//DGM: ignored
			//double zMin = qFromLittleEndianD(*reinterpret_cast<double*>(header  ));
			//double zMax = qFromLittleEndianD(*reinterpret_cast<double*>(header+8));
		}

		//Z coordinates (an array of length NumPoints)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				file.read(header, 8);
				//check for errors
				if (file.error() != QFile::NoError)
					return CC_FERR_READING;
				double z = qFromLittleEndianD(*reinterpret_cast<double*>(header));
				points[i].z = static_cast<PointCoordinateType>(z + PShift.z);
			}
		}
	}

	//3D polylines or 2D polylines + measurement
	std::vector<ScalarType> scalarValues;
	if (shapeTypeInt > SHP_POINT_Z)
	{
		//M boundaries
		{
			file.read(header, 16);
			//check for errors
			if (file.error() != QFile::NoError)
				return CC_FERR_READING;
			double mMin = qFromLittleEndianD(*reinterpret_cast<double*>(header));
			double mMax = qFromLittleEndianD(*reinterpret_cast<double*>(header + 8));

			if (mMin != ESRI_NO_DATA && mMax != ESRI_NO_DATA)
			{
				try
				{
					scalarValues.resize(numPoints);
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory to load scalar values!
					ccLog::Warning(QString("[SHP] Polyline #%1: not enough memory to load scalar values!").arg(index));
				}
			}
		}

		//M values (an array of length NumPoints)
		if (!scalarValues.empty())
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				file.read(header, 8);
				//check for errors
				if (file.error() != QFile::NoError)
					return CC_FERR_READING;
				double m = qFromLittleEndianD(*reinterpret_cast<double*>(header));
				scalarValues[i] = (m == ESRI_NO_DATA ? NAN_VALUE : static_cast<ScalarType>(m));
			}
		}
	}

	//and of course the polyline(s)
	for (int32_t i = 0; i < numParts; ++i)
	{
		const int32_t& firstIndex = startIndexes[i];
		const int32_t& lastIndex = (i + 1 < numParts ? startIndexes[i + 1] : numPoints) - 1;
		int32_t vertCount = lastIndex - firstIndex + 1;

		//test if the polyline is closed
		bool isClosed = false;
		if (vertCount > 2 && (points[firstIndex] - points[lastIndex]).norm() < ZERO_TOLERANCE)
		{
			vertCount--;
			isClosed = true;
		}

		//vertices
		ccPointCloud* vertices = new ccPointCloud("vertices");
		if (!vertices->reserve(vertCount))
		{
			delete vertices;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		for (int32_t j = 0; j < vertCount; ++j)
		{
			vertices->addPoint(points[firstIndex + j]);
		}
		vertices->setEnabled(false);
		vertices->setGlobalShift(PShift);

		//polyline
		ccPolyline* poly = new ccPolyline(vertices);
		poly->addChild(vertices);
		poly->setGlobalShift(PShift); //shouldn't be necessary but who knows ;)

		if (!poly->reserve(vertCount))
		{
			delete poly;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		poly->addPointIndex(0, static_cast<unsigned>(vertCount));
		poly->showSF(vertices->sfShown());
		QString name = QString("Polyline #%1").arg(index);
		if (numParts != 1)
			name += QString(".%1").arg(i + 1);
		poly->setName(name);
		poly->setClosed(isClosed);
		poly->set2DMode(!is3D && !load2DPolyAs3DPoly);

		if (!scalarValues.empty())
		{
			ccScalarField* sf = new ccScalarField("Measures");
			if (!sf->reserve(vertCount))
			{
				ccLog::Warning(QString("[SHP] Polyline #%1.%2: not enough memory to load scalar values!").arg(index).arg(i + 1));
				sf->release();
				sf = 0;
			}
			for (int32_t j = 0; j < vertCount; ++j)
			{
				sf->addElement(scalarValues[j + firstIndex]);
			}
			sf->computeMinAndMax();
			int sfIdx = vertices->addScalarField(sf);
			vertices->setCurrentDisplayedScalarField(sfIdx);
			vertices->showSF(true);
		}

		container.addChild(poly);
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR SavePolyline(ccPolyline* poly, QFile& file, int32_t& bytesWritten, ESRI_SHAPE_TYPE outputShapeType, int vertDim = 2)
{
	assert(vertDim >= 0 && vertDim < 3);
	const unsigned char Z = static_cast<unsigned char>(vertDim);
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	bytesWritten = 0;

	if (!poly)
	{
		assert(false);
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	CCLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
	if (!vertices)
		return CC_FERR_BAD_ENTITY_TYPE;

	int32_t realNumPoints = poly->size();
	if (realNumPoints < 3)
		return CC_FERR_BAD_ENTITY_TYPE;

	bool is2D = poly->is2DMode();
	bool isClosed = poly->isClosed();

	CCVector3d bbMing, bbMaxg;
	poly->getGlobalBB(bbMing, bbMaxg);

	//Shape Type
	{
		//Byte 0: Shape Type
		int32_t shapeTypeInt = qToLittleEndian<int32_t>(outputShapeType);
		file.write((const char*)&shapeTypeInt, 4);
		bytesWritten += 4;
	}

	//Byte 4: Box
	{
		double xMin = qToLittleEndianD(bbMing.u[X]);
		double xMax = qToLittleEndianD(bbMaxg.u[X]);
		double yMin = qToLittleEndianD(bbMing.u[Y]);
		double yMax = qToLittleEndianD(bbMaxg.u[Y]);
		//The Bounding Box for the PolyLine stored in the order Xmin, Ymin, Xmax, Ymax
		/*Byte  4*/file.write((const char*)&xMin, 8);
		/*Byte 12*/file.write((const char*)&yMin, 8);
		/*Byte 20*/file.write((const char*)&xMax, 8);
		/*Byte 28*/file.write((const char*)&yMax, 8);
		bytesWritten += 32;
	}

	//Byte 36: NumParts (The number of parts in the PolyLine)
	{
		int32_t numParts = qToLittleEndian<int32_t>(1);
		file.write((const char*)&numParts, 4);
		bytesWritten += 4;
	}

	//Byte 40: NumPoints (The total number of points for all parts)
	int32_t numPoints = realNumPoints;
	if (isClosed)
		numPoints++;
	{
		int32_t numPointsLE = qToLittleEndian<int32_t>(numPoints);
		file.write((const char*)&numPointsLE, 4);
		bytesWritten += 4;
	}

	//Byte 44: Parts (An array of length NumParts)
	{
		//for each part, the index of its first point in the points array
		int32_t startIndex = qToLittleEndian<int32_t>(0);
		file.write((const char*)&startIndex, 4);
		bytesWritten += 4;
	}

	//for polygons we must list the vertices in the right order:
	//"The neighborhood to the right of an observer walking along
	//the ring in vertex order is the inside of the polygon"
	//== clockwise order
	bool inverseOrder = false;
	if (outputShapeType == SHP_POLYGON || outputShapeType == SHP_POLYGON_Z)
	{
		assert(isClosed);
		assert(numPoints > 2);

		unsigned char dim1 = X;
		unsigned char dim2 = Y;
		if (outputShapeType == SHP_POLYGON_Z)
		{
			CCVector3d diag = bbMaxg - bbMing;

			//in 3D we have to guess the 'flat' dimension
			unsigned char minDim = diag.u[1] < diag.u[0] ? 1 : 0;
			if (diag.u[2] < diag.u[minDim])
				minDim = 2;
			dim1 = minDim == 2 ? 0 : minDim + 1;
			dim2 = dim1 == 2 ? 0 : dim1 + 1;
		}

		//test if the polygon is clock-wise or not
		{
			double sum = 0.0;
			//http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
			//or http://en.wikipedia.org/wiki/Shoelace_formula
			for (int32_t i = 0; i + 1 < numPoints; ++i)
			{
				const CCVector3* P1 = vertices->getPoint(i);
				const CCVector3* P2 = vertices->getPoint((i + 1) % realNumPoints);
				sum += (P2->u[dim1] - P2->u[dim1]) * (P2->u[dim2] + P1->u[dim2]);
			}

			//negative sum = counter-clockwise
			inverseOrder = (sum < 0.0);
		}
	}

	//Points (An array of length NumPoints)
	{
		for (int32_t i = 0; i < numPoints; ++i)
		{
			int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
			const CCVector3* P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed
			CCVector3d Pg = poly->toGlobal3d(*P);

			double x = qToLittleEndianD(Pg.u[X]);
			double y = qToLittleEndianD(Pg.u[Y]);
			/*Byte 0*/file.write((const char*)&x, 8);
			/*Byte 8*/file.write((const char*)&y, 8);
			bytesWritten += 16;
		}
	}

	//3D polylines
	if (!is2D)
	{
		//Z boundaries
		{
			double zMin = qToLittleEndianD(bbMing.u[Z]);
			double zMax = qToLittleEndianD(bbMaxg.u[Z]);
			file.write((const char*)&zMin, 8);
			file.write((const char*)&zMax, 8);
			bytesWritten += 16;
		}

		//Z coordinates (for each part - just one here)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
				const CCVector3* P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed
				CCVector3d Pg = poly->toGlobal3d(*P);
				double z = qToLittleEndianD(Pg.u[Z]);
				file.write((const char*)&z, 8);
				bytesWritten += 8;
			}
		}

		//M boundaries
		bool hasSF = vertices->isScalarFieldEnabled();
		{
			double mMin = ESRI_NO_DATA;
			double mMax = ESRI_NO_DATA;
			if (hasSF)
			{
				for (int32_t i = 0; i < realNumPoints; ++i)
				{
					ScalarType scalar = vertices->getPointScalarValue(i);
					if (i != 0)
					{
						if (mMin > scalar)
							mMin = static_cast<double>(scalar);
						else if (mMax < scalar)
							mMax = static_cast<double>(scalar);
					}
					else
					{
						mMin = mMax = static_cast<double>(scalar);
					}
				}
			}
			mMin = qToLittleEndianD(mMin);
			mMax = qToLittleEndianD(mMax);
			file.write((const char*)&mMin, 8);
			file.write((const char*)&mMax, 8);
			bytesWritten += 16;
		}

		//M values (for each part - just one here)
		{
			double scalar = qToLittleEndianD(ESRI_NO_DATA);
			for (int32_t i = 0; i < numPoints; ++i)
			{
				if (hasSF)
				{
					scalar = static_cast<double>(vertices->getPointScalarValue(i % realNumPoints)); //warning: handle loop if polyline is closed
					scalar = qToLittleEndianD(scalar);
				}
				file.write((const char*)&scalar, 8);
				bytesWritten += 8;
			}
		}
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LoadCloud(QFile& file, ccHObject& container, int32_t index, ESRI_SHAPE_TYPE shapeTypeInt, const CCVector3d& PShift)
{
	char header[36];
	file.read(header, 36);

	//Byte 0: Box
	{
		//The Bounding Box for the Cloud stored in the order Xmin, Ymin, Xmax, Ymax
		//DGM: ignored
		//double xMin = qFromLittleEndianD(*reinterpret_cast<double*>(header   ));
		//double xMax = qFromLittleEndianD(*reinterpret_cast<double*>(header+ 8));
		//double yMin = qFromLittleEndianD(*reinterpret_cast<double*>(header+16));
		//double yMax = qFromLittleEndianD(*reinterpret_cast<double*>(header+24));
	}

	//Byte 32: NumPoints (The total number of points)
	int32_t numPoints = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header + 32));

	ccPointCloud* cloud = new ccPointCloud(QString("Cloud #%1").arg(index));
	if (!cloud->reserve(numPoints))
	{
		delete cloud;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}
	cloud->setGlobalShift(PShift);

	//Points (An array of length NumPoints)
	{
		for (int32_t i = 0; i < numPoints; ++i)
		{
			file.read(header, 16);
			double x = qFromLittleEndianD(*reinterpret_cast<double*>(header));
			double y = qFromLittleEndianD(*reinterpret_cast<double*>(header + 8));
			CCVector3 P(static_cast<PointCoordinateType>(x + PShift.x),
				static_cast<PointCoordinateType>(y + PShift.y),
				0);
			cloud->addPoint(P);
		}
	}

	//3D clouds
	if (shapeTypeInt == SHP_MULTI_POINT_Z)
	{
		//Z boundaries
		{
			file.read(header, 16);
			//DGM: ignored
			//double zMin = qFromLittleEndianD(*reinterpret_cast<double*>(header  ));
			//double zMax = qFromLittleEndianD(*reinterpret_cast<double*>(header+8));
		}

		//Z coordinates (an array of length NumPoints)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				file.read(header, 8);
				double z = qFromLittleEndianD(*reinterpret_cast<double*>(header));
				const CCVector3* P = cloud->getPoint(i);
				const_cast<CCVector3*>(P)->z = static_cast<PointCoordinateType>(z + PShift.z);
			}
			cloud->invalidateBoundingBox();
		}
	}

	//3D clouds or 2D clouds + measurement
	if (shapeTypeInt == SHP_MULTI_POINT_Z
		|| shapeTypeInt == SHP_MULTI_POINT_M)
	{
		//M boundaries
		ccScalarField* sf = 0;
		{
			file.read(header, 16);
			double mMin = qFromLittleEndianD(*reinterpret_cast<double*>(header));
			double mMax = qFromLittleEndianD(*reinterpret_cast<double*>(header + 8));

			if (mMin != ESRI_NO_DATA && mMax != ESRI_NO_DATA)
			{
				sf = new ccScalarField("Measures");
				if (!sf->reserve(numPoints))
				{
					ccLog::Warning("[SHP] Not enough memory to load scalar values!");
					sf->release();
					sf = 0;
				}
			}
		}

		//M values (an array of length NumPoints)
		if (sf)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				file.read(header, 8);
				double m = qFromLittleEndianD(*reinterpret_cast<double*>(header));
				ScalarType s = m == ESRI_NO_DATA ? NAN_VALUE : static_cast<ScalarType>(m);
				sf->addElement(s);
			}
			sf->computeMinAndMax();
			int sfIdx = cloud->addScalarField(sf);
			cloud->setCurrentDisplayedScalarField(sfIdx);
			cloud->showSF(true);
		}
	}

	container.addChild(cloud);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR SaveAsCloud(ccGenericPointCloud* cloud, QFile& file, int32_t& bytesWritten)
{
	bytesWritten = 0;

	if (!cloud)
	{
		assert(false);
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	CCVector3d bbMing, bbMaxg;
	cloud->getGlobalBB(bbMing, bbMaxg);

	//Shape Type
	{
		//Byte 0: Shape Type
		int32_t shapeTypeInt = qToLittleEndian<int32_t>(SHP_MULTI_POINT_Z);
		file.write((const char*)&shapeTypeInt, 4);
		bytesWritten += 4;
	}

	//Byte 4: Box
	{
		double xMin = qToLittleEndianD(bbMing.x);
		double xMax = qToLittleEndianD(bbMaxg.x);
		double yMin = qToLittleEndianD(bbMing.y);
		double yMax = qToLittleEndianD(bbMaxg.y);
		//The Bounding Box for the Cloud stored in the order Xmin, Ymin, Xmax, Ymax
		/*Byte  4*/file.write((const char*)&xMin, 8);
		/*Byte 12*/file.write((const char*)&yMin, 8);
		/*Byte 20*/file.write((const char*)&xMax, 8);
		/*Byte 28*/file.write((const char*)&yMax, 8);
		bytesWritten += 32;
	}

	//Byte 36: NumPoints (The total number of points)
	int32_t numPoints = static_cast<int32_t>(cloud->size());
	{
		int32_t numPointsLE = qToLittleEndian<int32_t>(numPoints);
		file.write((const char*)&numPointsLE, 4);
		bytesWritten += 4;
	}

	//Points (An array of length NumPoints)
	{
		for (int32_t i = 0; i < numPoints; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			CCVector3d Pg = cloud->toGlobal3d(*P);

			double x = qToLittleEndianD(Pg.x);
			double y = qToLittleEndianD(Pg.y);
			/*Byte 0*/file.write((const char*)&x, 8);
			/*Byte 8*/file.write((const char*)&y, 8);
			bytesWritten += 16;
		}
	}

	//Z boundaries
	{
		double zMin = qToLittleEndianD(bbMing.z);
		double zMax = qToLittleEndianD(bbMaxg.z);
		file.write((const char*)&zMin, 8);
		file.write((const char*)&zMax, 8);
		bytesWritten += 16;
	}

	//Z coordinates
	{
		for (int32_t i = 0; i < numPoints; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			CCVector3d Pg = cloud->toGlobal3d(*P);
			double z = qToLittleEndianD(Pg.z);
			file.write((const char*)&z, 8);
			bytesWritten += 8;
		}
	}

	//M boundaries
	bool hasSF = cloud->isScalarFieldEnabled();
	{
		double mMin = ESRI_NO_DATA;
		double mMax = ESRI_NO_DATA;
		if (hasSF)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				ScalarType scalar = cloud->getPointScalarValue(i);
				if (i != 0)
				{
					if (mMin > scalar)
						mMin = static_cast<double>(scalar);
					else if (mMax < scalar)
						mMax = static_cast<double>(scalar);
				}
				else
				{
					mMin = mMax = static_cast<double>(scalar);
				}
			}
		}
		mMin = qToLittleEndianD(mMin);
		mMax = qToLittleEndianD(mMax);
		file.write((const char*)&mMin, 8);
		file.write((const char*)&mMax, 8);
		bytesWritten += 16;
	}

	//M values
	{
		double scalar = qToLittleEndianD(ESRI_NO_DATA);
		for (int32_t i = 0; i < numPoints; ++i)
		{
			if (hasSF)
			{
				scalar = static_cast<double>(cloud->getPointScalarValue(i));
				scalar = qToLittleEndianD(scalar);
			}
			file.write((const char*)&scalar, 8);
			bytesWritten += 8;
		}
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LoadSinglePoint(QFile& file, ccPointCloud* &singlePoints, ESRI_SHAPE_TYPE shapeTypeInt, const CCVector3d& PShift)
{
	char buffer[16];
	file.read(buffer, 16);

	double x = qFromLittleEndianD(*reinterpret_cast<double*>(buffer));
	double y = qFromLittleEndianD(*reinterpret_cast<double*>(buffer + 8));
	CCVector3 P(static_cast<PointCoordinateType>(x + PShift.x),
		static_cast<PointCoordinateType>(y + PShift.y),
		0);

	//3D point
	if (shapeTypeInt == SHP_POINT_Z)
	{
		//Z coordinate
		{
			file.read(buffer, 8);
			double z = qFromLittleEndianD(*reinterpret_cast<double*>(buffer));
			P.z = static_cast<PointCoordinateType>(z + PShift.z);
		}
	}

	if (!singlePoints)
	{
		singlePoints = new ccPointCloud("Points");
		singlePoints->setGlobalShift(PShift);
	}
	if (!singlePoints->reserve(singlePoints->size() + 1))
	{
		if (singlePoints->size() == 0)
		{
			delete singlePoints;
			singlePoints = 0;
		}
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	ScalarType s = NAN_VALUE;
	if (shapeTypeInt == SHP_POINT_Z
		|| shapeTypeInt == SHP_POINT_M)
	{
		//Measure
		{
			file.read(buffer, 8);
			double m = qFromLittleEndianD(*reinterpret_cast<double*>(buffer));
			if (m != ESRI_NO_DATA)
			{
				s = static_cast<ScalarType>(m);
				//add a SF to the cloud if not done already
				if (!singlePoints->hasScalarFields())
				{
					int sfIdx = singlePoints->addScalarField("Measures");
					if (sfIdx >= 0)
					{
						//set the SF value for the previous points
						singlePoints->setCurrentScalarField(sfIdx);
						for (unsigned i = 0; i < singlePoints->size(); ++i)
						{
							singlePoints->setPointScalarValue(i, NAN_VALUE);
						}
					}
				}
			}
		}
	}

	singlePoints->addPoint(P);

	if (singlePoints->hasScalarFields())
		singlePoints->setPointScalarValue(singlePoints->size() - 1, s);

	return CC_FERR_NO_ERROR;
}


CC_FILE_ERROR ShpFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	std::vector<GenericDBFField*> fields;
	return saveToFile(entity, fields, filename, parameters);
}

//semi-persistent parameters
static bool s_save3DPolysAs2D = false;
static int  s_poly2DVertDim = 2;
static bool s_save3DPolyHeightInDBF = false;
CC_FILE_ERROR ShpFilter::saveToFile(ccHObject* entity, const std::vector<GenericDBFField*>& fields, const QString& filename, const SaveParameters& parameters)
{
	if (!entity)
		return CC_FERR_BAD_ENTITY_TYPE;

	//this filter only supports point clouds, meshes and polylines!
	ESRI_SHAPE_TYPE inputShapeType = SHP_NULL_SHAPE;
	ccHObject::Container toSave;
	GetSupportedShapes(entity, toSave, inputShapeType/*,m_closedPolylinesAsPolygons*/);

	if (inputShapeType == SHP_NULL_SHAPE || toSave.empty())
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	CCVector3d bbMinCorner, bbMaxCorner;
	{
		bool isValid = false;
		//get the global bounding-box
		if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
			{
				ccHObject* child = entity->getChild(i);
				CCVector3d minC, maxC;
				if (child->getGlobalBB(minC, maxC))
				{
					if (isValid)
					{
						bbMinCorner.x = std::min(bbMinCorner.x, minC.x);
						bbMinCorner.y = std::min(bbMinCorner.y, minC.y);
						bbMinCorner.z = std::min(bbMinCorner.z, minC.z);
						bbMaxCorner.x = std::max(bbMaxCorner.x, maxC.x);
						bbMaxCorner.y = std::max(bbMaxCorner.y, maxC.y);
						bbMaxCorner.z = std::max(bbMaxCorner.z, maxC.z);
					}
					else
					{
						bbMinCorner = minC;
						bbMaxCorner = maxC;
						isValid = true;
					}
				}
			}
		}
		else
		{
			isValid = entity->getGlobalBB(bbMinCorner, bbMaxCorner);
		}
		if (!isValid)
		{
			ccLog::Error("Entity(ies) has(ve) an invalid bounding box?!");
			return CC_FERR_BAD_ENTITY_TYPE;
		}
	}


	bool save3DPolysAs2D = false;
	int poly2DVertDim = 2;
	bool save3DPolyHeightInDBF = false;
	if (parameters.alwaysDisplaySaveDialog && inputShapeType == SHP_POLYLINE_Z)
	{
		//display SHP save dialog
		SaveSHPFileDialog ssfDlg(0);
		ssfDlg.save3DPolyAs2DCheckBox->setChecked(s_save3DPolysAs2D);
		ssfDlg.save3DPolyHeightInDBFCheckBox->setChecked(s_save3DPolyHeightInDBF);
		ssfDlg.dimComboBox->setCurrentIndex(s_poly2DVertDim);

		if (!ssfDlg.exec())
			return CC_FERR_CANCELED_BY_USER;

		save3DPolysAs2D = s_save3DPolysAs2D = ssfDlg.save3DPolyAs2DCheckBox->isChecked();
		poly2DVertDim = s_poly2DVertDim = ssfDlg.dimComboBox->currentIndex();
		save3DPolyHeightInDBF = s_save3DPolyHeightInDBF = ssfDlg.save3DPolyHeightInDBFCheckBox->isChecked();
	}
	assert(poly2DVertDim >= 0 && poly2DVertDim < 3);
	const unsigned char Z = static_cast<unsigned char>(poly2DVertDim);
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	ESRI_SHAPE_TYPE outputShapeType = inputShapeType;
	if (m_closedPolylinesAsPolygons && (inputShapeType == SHP_POLYLINE || inputShapeType == SHP_POLYLINE_Z))
	{
		//check if all the polylines are closed!
		bool allAreClosed = true;
		for (size_t i = 0; i < toSave.size() && allAreClosed; ++i)
		{
			if (!static_cast<ccPolyline*>(toSave[i])->isClosed())
				allAreClosed = false;
		}

		if (allAreClosed)
		{
			//promote 'polylines' to 'polygon'
			outputShapeType = SHP_POLYLINE ? SHP_POLYGON : SHP_POLYGON_Z;
		}
	}

	ccLog::Print("[SHP] Output type: " + ToString(outputShapeType));

	QFileInfo fi(filename);
	QString baseFileName = fi.path() + QString("/") + fi.completeBaseName();

	//the main file (suffix should be ".shp")
	QString shpFilename = baseFileName + QString(".shp");
	QFile file(shpFilename);
	if (!file.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	//index file (same base name + ".shx")
	QString indexFilename = baseFileName + QString(".shx");
	QFile indexFile(indexFilename);
	if (!indexFile.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	//build header (refer to ESRI Shapefile Technical Description)
	char header[100];
	memset(header, 0, 100);
	{
		/*** WARNING: the beginning of the header is written with big endianness! ***/
		char* _header = header;

		//Byte 0: SHP code
		const int32_t code = qToBigEndian<int32_t>(9994);
		memcpy(_header, (const char*)&code, 4);
		_header += 4;

		//Byte 4: unused (20 bytes)
		_header += 20;

		//Byte 24: file length (will be written... later ;)
		_header += 4;

		/*** WARNING: from now on, we only write data with little endianness! ***/

		//Byte 28: file verion
		const int32_t version = qToLittleEndian<int32_t>(1000);
		memcpy(_header, (const char*)&version, 4);
		_header += 4;

		//Byte 32: shape type
		int32_t shapeTypeInt = qToLittleEndian<int32_t>(outputShapeType);
		memcpy(_header, (const char*)&shapeTypeInt, 4);
		_header += 4;

		//X and Y bounaries
		double xMin = qToLittleEndianD(bbMinCorner.u[X]);
		double xMax = qToLittleEndianD(bbMaxCorner.u[X]);
		double yMin = qToLittleEndianD(bbMinCorner.u[Y]);
		double yMax = qToLittleEndianD(bbMaxCorner.u[Y]);
		//Byte 36: box X min
		memcpy(_header, (const char*)&xMin, 8);
		_header += 8;
		//Byte 44: box Y min
		memcpy(_header, (const char*)&yMin, 8);
		_header += 8;
		//Byte 52: box X max
		memcpy(_header, (const char*)&xMax, 8);
		_header += 8;
		//Byte 60: box Y max
		memcpy(_header, (const char*)&yMax, 8);
		_header += 8;

		//Z bounaries
		//Unused, with value 0.0, if not Measured or Z type
		double zMin = outputShapeType < SHP_POINT_Z ? 0.0 : qToLittleEndianD(bbMinCorner.u[Z]);
		double zMax = outputShapeType < SHP_POINT_Z ? 0.0 : qToLittleEndianD(bbMaxCorner.u[Z]);
		//Byte 68: box Z min
		memcpy(_header, (const char*)&zMin, 8);
		_header += 8;
		//Byte 76: box Z max
		memcpy(_header, (const char*)&zMax, 8);
		_header += 8;

		//M bounaries (M = measures)
		double mMin = ESRI_NO_DATA;
		double mMax = ESRI_NO_DATA;
		//Byte 84: M min
		memcpy(_header, (const char*)&mMin, 8);
		_header += 8;
		//Byte 92: M max
		memcpy(_header, (const char*)&mMax, 8);
		_header += 8;
	}

	//actually write the header
	if (!file.write(header, 100))
		return CC_FERR_WRITING;
	if (!indexFile.write(header, 100)) //exact same header for index file!
		return CC_FERR_WRITING;
	int32_t fileLength = 100;

	//save shapes
	unsigned shapeIndex = 0;
	for (unsigned i = 0; i < toSave.size(); ++i)
	{
		ccHObject* child = toSave[i];

		//check entity eligibility
		if (child->isA(CC_TYPES::POLY_LINE))
		{
			if (static_cast<ccPolyline*>(child)->size() < 3)
			{
				ccLog::Warning(QString("Polyline '%1' is too small! It won't be saved...").arg(child->getName()));
				continue;
			}
		}

		int32_t recordSize = 0;
		qint64 recordStart = file.pos();
		//write shape record in main SHP file
		{
			//Byte 0: Record Number
			const int32_t recordNumber = qToBigEndian<int32_t>(++shapeIndex); //Record numbers begin at 1
			file.write((const char*)&recordNumber, 4);
			//Byte 4: Content Length
			file.write((const char*)&recordSize, 4); //will be updated later
		}
		fileLength += 8;

		CC_FILE_ERROR error = CC_FERR_NO_ERROR;
		switch (inputShapeType)
		{
		case SHP_POLYLINE:
		case SHP_POLYLINE_Z:
			assert(child->isKindOf(CC_TYPES::POLY_LINE));
			error = SavePolyline(static_cast<ccPolyline*>(child), file, recordSize, save3DPolysAs2D ? SHP_POLYLINE : outputShapeType, poly2DVertDim);
			break;
		case SHP_MULTI_POINT_Z:
			assert(child->isKindOf(CC_TYPES::POINT_CLOUD));
			error = SaveAsCloud(ccHObjectCaster::ToGenericPointCloud(child), file, recordSize);
			break;
			//case SHP_MULTI_PATCH:
			//	assert(child->isKindOf(CC_TYPES::MESH));
			//	error = SaveAsMesh(ccHObjectCaster::ToMesh(child),file,recordSize);
			//	break;
		default:
			assert(false);
			break;
		}

		if (error != CC_FERR_NO_ERROR)
			return error;

		fileLength += recordSize;

		//update record size
		{
			//backup current pos
			qint64 currentPos = file.pos();
			assert((qint64)fileLength == currentPos);
			//go backward
			file.seek(recordStart + 4);
			int32_t recordSize16bits = qToBigEndian<int32_t>(recordSize / 2); //recordSize is measured in 16-bit words
			file.write((const char*)&recordSize16bits, 4);
			//restore last pos
			file.seek(currentPos);
		}

		//write corresponding entry in index SHX file
		{
			//Byte 0: Offset
			int32_t recordStart16bits = qToBigEndian<int32_t>(recordStart / 2); //recordStart is measured in 16-bit words
			indexFile.write((const char*)&recordStart16bits, 4);

			//Byte 4: Content Length
			int32_t recordSize16bits = qToBigEndian<int32_t>(recordSize / 2); //recordSize is measured in 16-bit words
			indexFile.write((const char*)&recordSize16bits, 4);
		}
	}

	//update main file length
	{
		file.seek(24);
		//Byte 24: file length (in 16-bit words)
		int32_t fileLength16bits = qToBigEndian<int32_t>(fileLength / 2);
		file.write((const char*)&fileLength16bits, 4);
	}
	file.close();

	//update idx file length
	{
		indexFile.seek(24);
		//Byte 24: file length (in 16-bit words)
		int32_t idxFileLength = (int32_t)indexFile.size();
		int32_t idxFileLength16bits = qToBigEndian<int32_t>(idxFileLength / 2);
		indexFile.write((const char*)&idxFileLength16bits, 4);
	}
	indexFile.close();

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//eventually, we create the DB file (suffix should be ".dbf")
	QString dbfFilename = baseFileName + QString(".dbf");
	DBFHandle dbfHandle = DBFCreate(qPrintable(dbfFilename));
	if (dbfHandle)
	{
		while (true) //trick: we use 'while' to be able to break anytime
		{
			//always write an 'index' table
			{
				int fieldIdx = DBFAddField(dbfHandle, "local_idx", FTInteger, 6, 0);
				if (fieldIdx >= 0)
				{
					for (size_t i = 0; i < toSave.size(); ++i)
						DBFWriteIntegerAttribute(dbfHandle, static_cast<int>(i), fieldIdx, static_cast<int>(i)+1);
				}
				else
				{
					ccLog::Warning(QString("[SHP] Failed to save field 'index' (default)"));
					result = CC_FERR_WRITING;
					break;
				}
			}

			//write the '3D polylines height' field if request
			if (save3DPolyHeightInDBF)
			{
				int fieldIdx = DBFAddField(dbfHandle, "height", FTDouble, 8, 8);
				if (fieldIdx >= 0)
				{
					for (size_t i = 0; i < toSave.size(); ++i)
					{
						ccPolyline* poly = static_cast<ccPolyline*>(toSave[i]);
						double height = 0.0;
						if (poly && poly->size() != 0)
						{
							const CCVector3* P0 = poly->getPoint(0);
							CCVector3d Pg0 = poly->toGlobal3d(*P0);
							height = Pg0.u[Z];
						}
						DBFWriteDoubleAttribute(dbfHandle, static_cast<int>(i), fieldIdx, height);
					}
				}
				else
				{
					ccLog::Warning(QString("[SHP] Failed to save field 'height' (3D polylines height)"));
					result = CC_FERR_WRITING;
					break;
				}
			}

			//and write the other tables (specified by the user)
			for (std::vector<GenericDBFField*>::const_iterator it = fields.begin(); it != fields.end(); ++it)
			{
				const GenericDBFField* field = *it;
				if (field->is3D()) //3D case
				{
					int xFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_x")), field->type(), field->width(), field->decimal());
					int yFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_y")), field->type(), field->width(), field->decimal());
					int zFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_z")), field->type(), field->width(), field->decimal());
					if (xFieldIdx >= 0 && yFieldIdx >= 0 && zFieldIdx >= 0)
					{
						if (!(*it)->save(dbfHandle, xFieldIdx, yFieldIdx, zFieldIdx))
							xFieldIdx = -1;
					}

					if (xFieldIdx < 0)
					{
						ccLog::Warning(QString("[SHP] Failed to save field '%1'").arg(field->name()));
						result = CC_FERR_WRITING;
						break;
					}
				}
				else //1D case
				{
					int fieldIdx = DBFAddField(dbfHandle, qPrintable(field->name()), field->type(), field->width(), field->decimal());
					if (fieldIdx >= 0)
					{
						if (!(*it)->save(dbfHandle, fieldIdx))
							fieldIdx = -1;
					}

					if (fieldIdx < 0)
					{
						ccLog::Warning(QString("[SHP] Failed to save field '%1'").arg(field->name()));
						result = CC_FERR_WRITING;
						break;
					}
				}
			}

			break;
		}

		DBFClose(dbfHandle);
	}
	else
	{
		result = CC_FERR_WRITING;
	}

	return result;
}

typedef QPair<int, QString> FieldIndexAndName;

//semi-persistent settings
static double s_dbfFielImportScale = 1.0;
CC_FILE_ERROR ShpFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	//global shift
	CCVector3d Pshift(0, 0, 0);

	//read header (refer to ESRI Shapefile Technical Description)
	if (file.size() < 100)
		return CC_FERR_MALFORMED_FILE;
	char header[100];
	file.read(header, 100);
	int32_t fileLength = 0;
	{
		/*** WARNING: the beginning of the header is written with big endianness! ***/
		const char* _header = header;

		//Byte 0: SHP code
		const int32_t code = qFromBigEndian<int32_t>(*reinterpret_cast<const int32_t*>(_header));
		if (code != 9994)
		{
			return CC_FERR_MALFORMED_FILE;
		}
		_header += 4;

		//Byte 4: unused (20 bytes)
		_header += 20;

		//Byte 24: file length (will be written... later ;)
		fileLength = qFromBigEndian<int32_t>(*reinterpret_cast<const int32_t*>(_header));
		fileLength *= 2; //fileLength is measured in 16-bit words

		_header += 4;

		/*** WARNING: from now on, we only read data with little endianness! ***/

		//Byte 28: file verion
		const int32_t version = qFromLittleEndian<int32_t>(*reinterpret_cast<const int32_t*>(_header));
		_header += 4;

		//Byte 32: shape type
		int32_t shapeTypeInt = qFromLittleEndian<int32_t>(*reinterpret_cast<const int32_t*>(_header));
		_header += 4;

		ccLog::Print(QString("[SHP] Version: %1 - type: %2").arg(version).arg(ToString(static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt))));

		//X and Y bounaries
		//Byte 36: box X min
		double xMin = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 44: box Y min
		double yMin = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 52: box X max
		//double xMax = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 60: box Y max
		//double yMax = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;

		//Z bounaries
		//Unused, with value 0.0, if not Measured or Z type
		//Byte 68: box Z min
		double zMin = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 76: box Z max
		//double zMax = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;

		if (std::isnan(zMin))
		{
			//for 2D entities, the zMin value might be NaN!!!
			zMin = 0;
		}

		CCVector3d Pmin(xMin, yMin, zMin);
		if (HandleGlobalShift(Pmin, Pshift, parameters))
		{
			ccLog::Warning("[SHP] Entities will be recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
		}

		//M bounaries (M = measures)
		//Byte 84: M min
		//double mMin = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 92: M max
		//double mMax = qFromLittleEndianD(*reinterpret_cast<const double*>(_header));
		_header += 8;
	}
	assert(fileLength >= 100);
	if (fileLength < 100)
	{
		assert(false);
		return CC_FERR_MALFORMED_FILE;
	}
	fileLength -= 100;

	if (fileLength == 0)
	{
		return CC_FERR_NO_LOAD;
	}

	//progress bar
	QScopedPointer<ccProgressDialog> pDlg(0);
	qint64 fileSize = file.size();
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMaximum(static_cast<int>(fileSize));
		pDlg->setMethodTitle(QObject::tr("Load SHP file"));
		pDlg->setInfo(QObject::tr("File size: %1").arg(fileSize));
		pDlg->start();
		QApplication::processEvents();
	}

	//load shapes
	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	ccPointCloud* singlePoints = 0;
	qint64 pos = file.pos();
	//we also keep track of the polylines 'record number' (if any)
	QMap<ccPolyline*, int32_t> polyIDs;
	int32_t maxPolyID = 0;
	int32_t maxPointID = 0;
	bool is3DShape = false;
	while (fileLength >= 12)
	{
		file.seek(pos);
		assert(pos + fileLength == fileSize);
		//load shape record in main SHP file
		{
			file.read(header, 8);
			//Byte 0: Record Number
			int32_t recordNumber = qFromBigEndian<int32_t>(*reinterpret_cast<const int32_t*>(header)); //Record numbers begin at 1
			//Byte 4: Content Length
			int32_t recordSize = qFromBigEndian<int32_t>(*reinterpret_cast<const int32_t*>(header + 4)); //Record numbers begin at 1
			recordSize *= 2; //recordSize is measured in 16-bit words
			fileLength -= 8;
			pos += 8;

			if (fileLength < recordSize)
			{
				assert(false);
				error = CC_FERR_MALFORMED_FILE;
				break;
			}
			fileLength -= recordSize;
			pos += recordSize;

			//Record start (byte 0): Shape Type
			if (recordSize < 4)
			{
				assert(false);
				error = CC_FERR_MALFORMED_FILE;
				break;
			}
			file.read(header, 4);
			recordSize -= 4;
			int32_t shapeTypeInt = qToLittleEndian<int32_t>(*reinterpret_cast<const int32_t*>(header));
			ccLog::Print(QString("[SHP] Record #%1 - type: %2 (%3 bytes)").arg(recordNumber).arg(ToString(static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt))).arg(recordSize));

			switch (shapeTypeInt)
			{
			case SHP_POLYLINE_Z:
			case SHP_POLYGON_Z:
				is3DShape = true;
			case SHP_POLYLINE:
			case SHP_POLYGON:
			{
				unsigned childCountBefore = container.getChildrenNumber();
				error = LoadPolyline(file, container, recordNumber, static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt), Pshift);
				if (error == CC_FERR_NO_ERROR && shapeTypeInt == SHP_POLYLINE)
				{
					unsigned childCountAfter = container.getChildrenNumber();
					//warning: we can load mutliple polylines for a single record!
					for (unsigned i = childCountBefore; i < childCountAfter; ++i)
					{
						ccHObject* child = container.getChild(i);
						assert(child && child->isA(CC_TYPES::POLY_LINE));
						polyIDs[static_cast<ccPolyline*>(child)] = recordNumber;
						if (recordNumber > maxPolyID)
							maxPolyID = recordNumber;
					}
				}
			}
			break;
			case SHP_MULTI_POINT_Z:
			case SHP_MULTI_POINT_M:
				is3DShape = true;
			case SHP_MULTI_POINT:
				error = LoadCloud(file, container, recordNumber, static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt), Pshift);
				break;
			case SHP_POINT_Z:
			case SHP_POINT_M:
				is3DShape = true;
			case SHP_POINT:
				error = LoadSinglePoint(file, singlePoints, static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt), Pshift);
				if (error == CC_FERR_NO_ERROR && recordNumber > maxPointID)
				{
					maxPointID = recordNumber;
				}
				break;
				//case SHP_MULTI_PATCH:
				//error = LoadMesh(file, recordSize);
				//break;
			case SHP_NULL_SHAPE:
				//ignored
				break;
			default:
				//unhandled entity
				ccLog::Warning("[SHP] Unhandled type!");
				break;
			}
		}

		if (error != CC_FERR_NO_ERROR)
		{
			break;
		}

		if (pDlg)
		{
			pDlg->setValue(fileSize - (pos + fileLength));
			if (pDlg->wasCanceled())
			{
				error = CC_FERR_CANCELED_BY_USER;
				break;
			}
		}
	}

	//try to load the DBF to see if there's a 'height' field or something similar for polylines
	bool hasPolylines = (!polyIDs.empty());
	bool hasPoints = (singlePoints && singlePoints->size() != 0 && maxPointID == static_cast<int32_t>(singlePoints->size()));
	if (!is3DShape
		&&	error == CC_FERR_NO_ERROR
		&& (hasPolylines || hasPoints)
		)
	{
		QFileInfo fi(filename);
		QString baseFileName = fi.path() + QString("/") + fi.completeBaseName();
		//try to load the DB file (suffix should be ".dbf")
		QString dbfFilename = baseFileName + QString(".dbf");
		DBFHandle dbfHandle = DBFOpen(qPrintable(dbfFilename), "rb");
		if (dbfHandle)
		{
			int fieldCount = DBFGetFieldCount(dbfHandle);
			int recordCount = DBFGetRecordCount(dbfHandle);
			if (fieldCount == 0)
			{
				ccLog::Warning("[SHP] No field in the associated DBF file!");
			}
			else if (hasPolylines && recordCount < static_cast<int>(maxPolyID))
			{
				ccLog::Warning("[SHP] No enough records in the associated DBF file!");
			}
			else if (hasPoints && recordCount < static_cast<int>(singlePoints->size()))
			{
				ccLog::Warning("[SHP] No enough records in the associated DBF file!");
			}
			else
			{
				QList<FieldIndexAndName> candidateFields;
				for (int i = 0; i < fieldCount; ++i)
				{
					char fieldName[256];
					DBFFieldType fieldType = DBFGetFieldInfo(dbfHandle, i, fieldName, 0, 0);
					if (fieldType == FTDouble || fieldType == FTInteger)
					{
						candidateFields.push_back(FieldIndexAndName(i, QString(fieldName)));
					}
				}

				if (!candidateFields.empty())
				{
					//create a list of available fields
					ImportDBFFieldDialog lsfDlg(0);
					for (QList<FieldIndexAndName>::const_iterator it = candidateFields.begin(); it != candidateFields.end(); ++it)
					{
						lsfDlg.listWidget->addItem(it->second);
					}
					lsfDlg.scaleDoubleSpinBox->setValue(s_dbfFielImportScale);
					lsfDlg.okPushButton->setVisible(false);


					if (lsfDlg.exec())
					{
						s_dbfFielImportScale = lsfDlg.scaleDoubleSpinBox->value();

						//look for the selected index
						int index = -1;
						for (int i = 0; i < candidateFields.size(); ++i)
						{
							if (lsfDlg.listWidget->isItemSelected(lsfDlg.listWidget->item(i)))
							{
								index = candidateFields[i].first;
								break;
							}
						}

						if (index >= 0)
						{
							double scale = s_dbfFielImportScale;
							//read values
							DBFFieldType fieldType = DBFGetFieldInfo(dbfHandle, index, 0, 0, 0);

							if (hasPolylines)
							{
								//for each poyline
								for (QMap<ccPolyline*, int32_t>::iterator it = polyIDs.begin(); it != polyIDs.end(); ++it)
								{
									//get the height
									double z = 0.0;
									if (fieldType == FTDouble)
										z = DBFReadDoubleAttribute(dbfHandle, it.value() - 1, index);
									else //if (fieldType == FTInteger)
										z = static_cast<double>(DBFReadIntegerAttribute(dbfHandle, it.value() - 1, index));
									z *= scale;

									//translate the polyline
									CCVector3 T(0, 0, static_cast<PointCoordinateType>(z));
									ccGLMatrix trans;
									trans.setTranslation(T);
									ccPolyline* poly = it.key();
									if (poly)
									{
										poly->applyGLTransformation_recursive(&trans);
										//this transformation is of no interest for the user
										poly->resetGLTransformationHistory_recursive();
										//add the 'const altitude' meta-data as well
										poly->setMetaData(ccPolyline::MetaKeyConstAltitude(), QVariant(z));
									}
								}
							}
							else if (hasPoints)
							{
								//for each point
								for (unsigned i = 0; i < singlePoints->size(); ++i)
								{
									//get the height
									double z = 0.0;
									if (fieldType == FTDouble)
										z = DBFReadDoubleAttribute(dbfHandle, static_cast<int>(i), index);
									else //if (fieldType == FTInteger)
										z = static_cast<double>(DBFReadIntegerAttribute(dbfHandle, static_cast<int>(i), index));
									z *= scale;

									//set the point height
									const_cast<CCVector3*>(singlePoints->getPoint(i))->z = z;
								}
								singlePoints->invalidateBoundingBox();
							}
							else
							{
								assert(false);
							}
						}
					}
				}
				else
				{
					ccLog::Warning("[SHP] No numerical field in the associated DBF file!");
				}
			}
		}
		else
		{
			ccLog::Warning(QString("[SHP] Failed to load associated DBF file ('%1')").arg(dbfFilename));
		}
	}

	if (singlePoints)
	{
		if (singlePoints->size() == 0)
		{
			delete singlePoints;
			singlePoints = 0;
		}
		else
		{
			CCLib::ScalarField* sf = singlePoints->getScalarField(0);
			if (sf)
			{
				sf->computeMinAndMax();
				singlePoints->showSF(true);
			}
			container.addChild(singlePoints);
		}
	}

	return error;
}

#endif //CC_SHP_SUPPORT
