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
#include <ccMesh.h>


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

using FieldIndexAndName = QPair<int, QString>;

// ESRI ShapeFile related constants
static const double ESRI_NO_DATA = -1.0e38;
static const uint8_t SHP_HEADER_SIZE = 100;
static const int32_t SHP_FILE_CODE = 9994;
static const int FILE_SIZE_OFFSET = 24;

//semi-persistent settings
static double s_dbfFieldImportScale = 1.0;

//semi-persistent parameters
static bool s_save3DPolysAs2D = false;
static int s_poly2DVertDim = 2;
static bool s_save3DPolyHeightInDBF = false;

//! Shape File Save dialog
class SaveSHPFileDialog : public QDialog, public Ui::SaveSHPFileDlg {
public:
	//! Default constructor
	explicit SaveSHPFileDialog(QWidget *parent = nullptr)
			: QDialog(parent), Ui::SaveSHPFileDlg() {
		setupUi(this);
	}
};

//! Shape File Load dialog (to choose an 'altitude' field)
class ImportDBFFieldDialog : public QDialog, public Ui::ImportDBFFieldDlg {
public:
	//! Default constructor
	explicit ImportDBFFieldDialog(QWidget *parent = nullptr)
			: QDialog(parent), Ui::ImportDBFFieldDlg() {
		setupUi(this);
	}
};

//! Returns true if the code corresponds to a valid ESRI Shape Type
/** \param code The code to check (typically read from a file)
**/
bool isValidESRIShapeCode(int32_t code)
{
	if (code < static_cast<int32_t >(ESRI_SHAPE_TYPE::NULL_SHAPE))
		return false;
	if (code > static_cast<int32_t >(ESRI_SHAPE_TYPE::MULTI_PATCH))
		return false;

	switch (static_cast<ESRI_SHAPE_TYPE >(code))
	{
		case ESRI_SHAPE_TYPE::NULL_SHAPE:
		case ESRI_SHAPE_TYPE::SHP_POINT:
		case ESRI_SHAPE_TYPE::POLYLINE:
		case ESRI_SHAPE_TYPE::POLYGON:
		case ESRI_SHAPE_TYPE::MULTI_POINT:
		case ESRI_SHAPE_TYPE::POINT_Z:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
		case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
		case ESRI_SHAPE_TYPE::POINT_M:
		case ESRI_SHAPE_TYPE::POLYLINE_M:
		case ESRI_SHAPE_TYPE::POLYGON_M:
		case ESRI_SHAPE_TYPE::MULTI_POINT_M:
		case ESRI_SHAPE_TYPE::MULTI_PATCH:
			return true;
		default:
			return false;
	}
}

//! Returns whether the shape type contains the 3rd dimensions Z
bool isESRIShape3D(ESRI_SHAPE_TYPE shapeType)
{
	switch (shapeType)
	{
		case ESRI_SHAPE_TYPE::POINT_Z:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
		case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
			return true;
		default:
			return false;
	}
}

//! Returns whether the shape type contains the additional measures dimension
bool hasMeasurements(ESRI_SHAPE_TYPE shapeType)
{
	switch (shapeType)
	{
		case ESRI_SHAPE_TYPE::POINT_Z:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
		case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
		case ESRI_SHAPE_TYPE::POINT_M:
		case ESRI_SHAPE_TYPE::POLYLINE_M:
		case ESRI_SHAPE_TYPE::POLYGON_M:
		case ESRI_SHAPE_TYPE::MULTI_POINT_M:
			return true;
		default:
			return false;
	}
}

//! Gives the bounding box containing all the elements in the hierarchy
/**
 *
 * @param entity The Hierarchy containing objects
 * @param[out] bbMinCorner Minimum point of the hierarchy
 * @param[out] bbMaxCorner Maximum point of the hierarchy
 * @return true if the min & max vectors are valid
 */
bool bbMinMaxOfHierarchy(const ccHObject *entity, CCVector3d &bbMinCorner, CCVector3d &bbMaxCorner)
{
	if (!entity->isA(CC_TYPES::HIERARCHY_OBJECT)) {
		return false;
	}

	bool isValid = false;
	for (unsigned i = 0; i < entity->getChildrenNumber(); ++i) {
		ccHObject *child = entity->getChild(i);
		CCVector3d minC, maxC;
		if (child->getGlobalBB(minC, maxC)) {
			if (isValid) {
				bbMinCorner.x = std::min(bbMinCorner.x, minC.x);
				bbMinCorner.y = std::min(bbMinCorner.y, minC.y);
				bbMinCorner.z = std::min(bbMinCorner.z, minC.z);
				bbMaxCorner.x = std::max(bbMaxCorner.x, maxC.x);
				bbMaxCorner.y = std::max(bbMaxCorner.y, maxC.y);
				bbMaxCorner.z = std::max(bbMaxCorner.z, maxC.z);
			} else {
				bbMinCorner = minC;
				bbMaxCorner = maxC;
				isValid = true;
			}
		}
	}
	return isValid;
}


//! Test if the vertices ar" counter clock-wise
//! http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
//! or http://en.wikipedia.org/wiki/Shoelace_formula
bool arePolygonVerticesCounterClockwise(
		CCLib::GenericIndexedCloudPersist *vertices,
		const CCVector3d &diag,
		unsigned char dim1,
		unsigned char dim2)
{
	unsigned realNumPoints = vertices->size();
	unsigned numPoints = realNumPoints + 1; // Polygon, so closed

	//or http://en.wikipedia.org/wiki/Shoelace_formula
	double sum = 0.0;
	for (int32_t i = 0; i + 1 < numPoints; ++i) {
		const CCVector3 *P1 = vertices->getPoint(i);
		const CCVector3 *P2 = vertices->getPoint((i + 1) % realNumPoints);
		sum += (P2->u[dim1] - P2->u[dim1]) * (P2->u[dim2] + P1->u[dim2]);
	}

	//negative sum = counter-clockwise
	return sum < 0.0;
}

void guessFlatDimension(const CCVector3d &diag, unsigned char &dim1, unsigned char &dim2) {
	auto minDim = static_cast<unsigned char>(diag.u[1] < diag.u[0] ? 1 : 0);
	if (diag.u[2] < diag.u[minDim])
			minDim = 2;
	dim1 = static_cast<unsigned char>(minDim == 2 ? 0 : minDim + 1);
	dim2 = static_cast<unsigned char>(dim1 == 2 ? 0 : dim1 + 1);
}


bool ShpFilter::canLoadExtension(const QString &upperCaseExt) const {
	return (upperCaseExt == "SHP");
}

bool ShpFilter::canSave(CC_CLASS_ENUM type, bool &multiple, bool &exclusive) const {
	if (type == CC_TYPES::POLY_LINE
		|| type == CC_TYPES::POINT_CLOUD
		|| type == CC_TYPES::HIERARCHY_OBJECT
		|| type == CC_TYPES::MESH) {
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

QString toString(ESRI_SHAPE_TYPE type) {
	switch (type) {
		case ESRI_SHAPE_TYPE::NULL_SHAPE:
			return "Null";
		case ESRI_SHAPE_TYPE::SHP_POINT:
			return "2D point";
		case ESRI_SHAPE_TYPE::POLYLINE:
			return "2D polyline";
		case ESRI_SHAPE_TYPE::POLYGON:
			return "2D polygon";
		case ESRI_SHAPE_TYPE::MULTI_POINT:
			return "2D point cloud";
		case ESRI_SHAPE_TYPE::POINT_Z:
			return "3D point";
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
			return "3D polyline";
		case ESRI_SHAPE_TYPE::POLYGON_Z:
			return "3D polygon";
		case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
			return "3D point cloud";
		case ESRI_SHAPE_TYPE::POINT_M:
			return "2D point (+measure)";
		case ESRI_SHAPE_TYPE::POLYLINE_M:
			return "2D polyline (+measure)";
		case ESRI_SHAPE_TYPE::POLYGON_M:
			return "2D polygon (+measure)";
		case ESRI_SHAPE_TYPE::MULTI_POINT_M:
			return "2D point cloud (+measure)";
		case ESRI_SHAPE_TYPE::MULTI_PATCH:
			return "Multi patch";
		default:
			break;
	}

	return QString("Unknown");
}

//! Returns the corresponding ESRI Shape type for the given entity
//! If the entity has no valid corresponding shape type, returns
//! the ESRI NUll_SHAPE type
ESRI_SHAPE_TYPE esriTypeOfccHobject(ccHObject *entity, bool closedPolylineAsPolygon = true, bool save3DPolyAs2D = false) {
	if (!entity) {
		return ESRI_SHAPE_TYPE::NULL_SHAPE;
	}

	switch (entity->getClassID())
	{
		case CC_TYPES::POINT_CLOUD:
			return ESRI_SHAPE_TYPE::MULTI_POINT_Z;
		case CC_TYPES::POLY_LINE:
		{
			auto *poly = static_cast<ccPolyline *>(entity);
			if (closedPolylineAsPolygon && poly->isClosed())
			{
			    if (!poly->is2DMode() && save3DPolyAs2D)
			    	return ESRI_SHAPE_TYPE::POLYGON;
			    else
					return ESRI_SHAPE_TYPE::POLYGON_Z;
			}
			else
			{
				if (!poly->is2DMode() && save3DPolyAs2D)
					return ESRI_SHAPE_TYPE::POLYLINE;
				else
					return ESRI_SHAPE_TYPE::POLYLINE_Z;
			}
		}
		case CC_TYPES::MESH:
			return ESRI_SHAPE_TYPE::MULTI_PATCH;
		default:
			return ESRI_SHAPE_TYPE::NULL_SHAPE;
	}
}

//! Returns true il all the elements in the container have the same corresponding ESRI shape type
bool haveSameCorrespondingEsriType(const ccHObject::Container& container, bool closedPolylineAsPolygon = true, bool save3DPolyAs2D = false) {
	ESRI_SHAPE_TYPE firstType = esriTypeOfccHobject(container.front(), closedPolylineAsPolygon, save3DPolyAs2D);
	auto haveSameTypeAsFirst = [firstType, closedPolylineAsPolygon, save3DPolyAs2D](ccHObject *obj) {
		return esriTypeOfccHobject(obj, closedPolylineAsPolygon, save3DPolyAs2D) == firstType;
	};
	return std::all_of(container.begin(), container.end(), haveSameTypeAsFirst);
}

bool haveSameCCType(const ccHObject::Container& container) {
	CC_CLASS_ENUM firstType = container.front()->getClassID();
	auto haveSameTypeAsFirst = [firstType](ccHObject *obj) {
		return obj->getClassID() == firstType;
	};
	return std::all_of(container.begin(), container.end(), haveSameTypeAsFirst);

}


//! Puts in the container all the shapes that have a corresponding ESRI shape type
void getSupportedShapes(ccHObject *baseEntity, ccHObject::Container &shapes)
{
	if (baseEntity->getClassID() == CC_TYPES::HIERARCHY_OBJECT)
	{
		for (unsigned i(0); i < baseEntity->getChildrenNumber(); ++i){
			getSupportedShapes(baseEntity->getChild(i), shapes);
		}
	}
	else if (esriTypeOfccHobject(baseEntity) != ESRI_SHAPE_TYPE::NULL_SHAPE) {
		shapes.push_back(baseEntity);
	}
}




//! Saves the polyline to the stream as specified in the Shapefile specification
CC_FILE_ERROR savePolyline(ccPolyline *poly, QDataStream &stream, int32_t recordNumber, ESRI_SHAPE_TYPE outputShapeType,
						   int vertDim = 2) {
	if (!poly) {
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	if (vertDim < 0 || vertDim >= 3) {
		throw std::invalid_argument("vertical dim must be >= 0 & <3");
	}


	CCLib::GenericIndexedCloudPersist *vertices = poly->getAssociatedCloud();
	if (!vertices)
		return CC_FERR_BAD_ENTITY_TYPE;

	int32_t realNumPoints = poly->size();
	if (realNumPoints < 2)
		return CC_FERR_BAD_ENTITY_TYPE;

	bool is2D = poly->is2DMode();
	bool isClosed = poly->isClosed();

	CCVector3d bbMing, bbMaxg;
	poly->getGlobalBB(bbMing, bbMaxg);
	int32_t numPoints = isClosed ? (realNumPoints + 1 ) : realNumPoints;
	int32_t numParts = 1;

	int32_t recordSize(0);
	recordSize += sizeof(int32_t); // ShapeType
	recordSize += (4* sizeof(double)); // MBR
	recordSize += sizeof(int32_t); // nbParts
	recordSize += sizeof(int32_t); // nbPoints
	recordSize += (numParts * sizeof(int32_t)); // Parts
	recordSize += (numPoints * 2 * sizeof(double)); //Points

	if (!is2D && outputShapeType == ESRI_SHAPE_TYPE::POLYLINE_Z)
	{
		recordSize += (2 * sizeof(double)); // zRange
		recordSize += (numPoints) * sizeof(double); // Zs
	}
	if (hasMeasurements(outputShapeType))
	{
		recordSize += (2 * sizeof(double)); // mRange
		recordSize += (numPoints) * sizeof(double); // Measures
	}
	recordSize /= 2; // 16bit words

	stream.setByteOrder(QDataStream::BigEndian);
	stream << recordNumber << recordSize;
	stream.setByteOrder(QDataStream::LittleEndian);
	qint64 oldPos = stream.device()->pos();
	stream << static_cast<int32_t >(outputShapeType);

	stream << bbMing.x << bbMing.y << bbMaxg.x << bbMaxg.y;
	stream << static_cast<int32_t >(numParts);
	stream << numPoints;

	stream << static_cast<int32_t >(0); // Parts startIndex

	//for polygons we must list the vertices in the right order:
	//"The neighborhood to the right of an observer walking along
	//the ring in vertex order is the inside of the polygon"
	//== clockwise order
	bool inverseOrder = false;
	const auto Z = static_cast<unsigned char>(vertDim);
	auto X = static_cast<unsigned char>(Z == 2 ? 0 : Z + 1);
	auto Y = static_cast<unsigned char>(X == 2 ? 0 : X + 1);
	if (outputShapeType == ESRI_SHAPE_TYPE::POLYGON || outputShapeType == ESRI_SHAPE_TYPE::POLYGON_Z)
	{

		CCVector3d diag = bbMaxg - bbMing;
		if (outputShapeType == ESRI_SHAPE_TYPE::POLYGON_Z)
			guessFlatDimension(diag, X, Y);
		inverseOrder = arePolygonVerticesCounterClockwise(vertices, diag, X,Y);
	}

	//Points (An array of length NumPoints)
	for (int32_t i = 0; i < numPoints; ++i)
	{
		int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
		const CCVector3 *P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed
		CCVector3d Pg = poly->toGlobal3d(*P);

		stream << Pg.u[X] << Pg.u[Y];
	}

	//3D polylines
	if (!is2D && isESRIShape3D(outputShapeType)) {
		stream << bbMing.u[Z] << bbMaxg.u[Z];
		//Z coordinates (for each part - just one here)
		for (int32_t i = 0; i < numPoints; ++i)
		{
			int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
			const CCVector3 *P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed
			CCVector3d Pg = poly->toGlobal3d(*P);
			stream << Pg.u[Z];
		}
	}

	if (hasMeasurements(outputShapeType)) {
		bool hasSF = vertices->isScalarFieldEnabled();
		double mMin = ESRI_NO_DATA;
		double mMax = ESRI_NO_DATA;
		if (hasSF) {
			mMin = std::numeric_limits<double>::max();
			mMax = std::numeric_limits<double>::min();
			for (unsigned i = 0; i < numPoints; ++i) {
				auto scalar = static_cast<double >(vertices->getPointScalarValue(i));
				mMin = std::min(mMin, scalar);
				mMax = std::max(mMax, scalar);
			}
		}
		stream << mMin << mMax;

		//M values (for each part - just one here)
		double scalar = ESRI_NO_DATA;
		for (int32_t i = 0; i < numPoints; ++i) {
			if (hasSF) {
				//warning: handle loop if polyline is closed
				scalar = static_cast<double>(vertices->getPointScalarValue(i % realNumPoints));
			}
			stream << scalar;
		}
	}
	qint64 newPos = stream.device()->pos();
	ccLog::Print(QString("Expected: %1, Wrote: %2").arg(recordSize).arg(newPos - oldPos));
	return CC_FERR_NO_ERROR;
}

//! Saves the cloud to the shape file according to the specification
/**
 *
 * @param stream Output stream of the shapefile
 * @param cloud The cloud to save (pointcloud or vertices)
 * @param bbMing Min point of the cloud
 * @param bbMaxg MAx point of the cloud
 */
void save3DCloud(QDataStream &stream, ccGenericPointCloud *cloud, const CCVector3d &bbMing, const CCVector3d &bbMaxg)
{
    unsigned numPoints = cloud->size();

	// Points (x ,y)
	for (unsigned i = 0; i < numPoints; ++i) {
		const CCVector3 *P = cloud->getPoint(i);
		CCVector3d Pg = cloud->toGlobal3d(*P);
		stream << Pg.x << Pg.y;
	}

	// Z Coordinates
	stream << bbMing.z << bbMaxg.z;
	for (unsigned i = 0; i < numPoints; ++i) {
		const CCVector3 *P = cloud->getPoint(i);
		CCVector3d Pg = cloud->toGlobal3d(*P);
		stream << Pg.z;
	}

	// Measures
	bool hasSF = cloud->isScalarFieldEnabled();
	double mMin = ESRI_NO_DATA;
	double mMax = ESRI_NO_DATA;
	if (hasSF) {
		mMin = std::numeric_limits<double>::max();
		mMax = std::numeric_limits<double>::min();
		for (unsigned i = 0; i < numPoints; ++i) {
			auto scalar = static_cast<double >(cloud->getPointScalarValue(i));
			mMin = std::min(mMin, scalar);
			mMax = std::max(mMax, scalar);
		}
	}
	stream << mMin << mMax;

	auto scalar = ESRI_NO_DATA;
	for (unsigned i = 0; i < numPoints; ++i) {
		if (hasSF) {
			scalar = static_cast<double>(cloud->getPointScalarValue(i));
		}
		stream << scalar;
	}
}


CC_FILE_ERROR saveAsCloud(ccGenericPointCloud *cloud, QDataStream &stream, int32_t recordNumber) {
	if (!cloud) {
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	unsigned numPoints = cloud->size();
	int32_t recordLength = 0;
	recordLength += sizeof(int32_t); // shapeType
	recordLength += (4 * sizeof(double)); // x,y bbox
	recordLength += sizeof(int32_t); // numPoints
	recordLength += (2 * numPoints * sizeof(double)); // xs, ys
	recordLength += (2 * sizeof(double)); // zRange
	recordLength += (numPoints * sizeof(double)); // zs
	recordLength += (2 * sizeof(double)); // mRange
	recordLength += (numPoints * sizeof(double)); // Measures
	recordLength /= 2; // 16bit words

	stream.setByteOrder(QDataStream::BigEndian);
	stream << recordNumber << recordLength;

	qint64 recordStart = stream.device()->pos();
	stream.setByteOrder(QDataStream::LittleEndian);
	stream << static_cast<int32_t >(ESRI_SHAPE_TYPE::MULTI_POINT_Z);

	CCVector3d bbMing, bbMaxg;
	cloud->getGlobalBB(bbMing, bbMaxg);
	stream << bbMing.x << bbMing.y << bbMaxg.x << bbMaxg.y;
	stream << static_cast<int32_t >(numPoints);

	save3DCloud(stream, cloud, bbMing, bbMaxg);

	qint64 recordEnd = stream.device()->pos();
	qint64 bytesWritten = recordEnd - recordStart;
	if (bytesWritten != 2 * recordLength) {
		ccLog::Error(QString("wrote %1 bytes, expected to write %2").arg(bytesWritten).arg(recordLength * 2));
		return CC_FERR_WRITING;
	}
	return CC_FERR_NO_ERROR;
}


void writeHeaderTo(const ShapeFileHeader &hdr, QDataStream &stream) {
	uint32_t skip = 0;

	stream.setByteOrder(QDataStream::BigEndian);
	stream << SHP_FILE_CODE;
	stream << skip << skip << skip << skip << skip;
	stream << hdr.fileLength;

	stream.setByteOrder(QDataStream::LittleEndian);
	stream << hdr.version;
	stream << hdr.shapeTypeInt;
	stream << hdr.pointMin.x << hdr.pointMin.y;
	stream << hdr.pointMax.x << hdr.pointMax.y;
	stream << hdr.pointMin.z << hdr.pointMax.z;
	stream << hdr.mRange.x << hdr.mRange.y;
}

bool isTriangleStrip(const CCLib::VerticesIndexes *idx)
{
	return (idx->i3 - 1) == idx->i2 && (idx->i3 - 2) == idx->i1;
}

bool isTriangleFan(const CCLib::VerticesIndexes *idx)
{
	return idx->i1 == 0 && idx->i2 == (idx->i3 - 1);
}

CC_FILE_ERROR findTriangleOrganisation(ccMesh *mesh, ESRI_PART_TYPE &type)
{

	CCLib::VerticesIndexes *firstVert = mesh->getNextTriangleVertIndexes();
	if (!isTriangleFan(firstVert) && !isTriangleStrip(firstVert))
		return CC_FERR_BAD_ENTITY_TYPE;


	CCLib::VerticesIndexes *secondVert = mesh->getNextTriangleVertIndexes();
	if (isTriangleStrip(secondVert))
	{
		for (unsigned i(2); i < mesh->size(); ++i)
		{
			CCLib::VerticesIndexes *idx = mesh->getNextTriangleVertIndexes();
			if (!isTriangleStrip(idx))
				return CC_FERR_BAD_ENTITY_TYPE;
		}
		type = ESRI_PART_TYPE::TRIANGLE_STRIP;
		return CC_FERR_NO_ERROR;

	}
	else if (isTriangleFan(secondVert))
	{
		for (unsigned i(2); i < mesh->size(); ++i)
		{
			CCLib::VerticesIndexes *idx = mesh->getNextTriangleVertIndexes();
			if (!isTriangleFan(idx))
				return CC_FERR_BAD_ENTITY_TYPE;
		}
		type = ESRI_PART_TYPE::TRIANGLE_FAN;
		return CC_FERR_NO_ERROR;

	}
	else
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}
}

CC_FILE_ERROR saveMesh(QDataStream &stream, ccMesh *mesh, int32_t recordNumber)
{
    ESRI_PART_TYPE triangleType;
    if (findTriangleOrganisation(mesh, triangleType) == CC_FERR_BAD_ENTITY_TYPE)
		return CC_FERR_BAD_ENTITY_TYPE;

    ccLog::Print(QString("[SHP] Triangle type: %1").arg(static_cast<int32_t >(triangleType)));

    ccGenericPointCloud *vertices = mesh->getAssociatedCloud();
    int32_t numParts = 1;
    unsigned numPoints = vertices->size();

	int32_t recordSize(0);
	recordSize += sizeof(int32_t); // ShapeType
	recordSize += (4* sizeof(double)); // MBR
	recordSize += sizeof(int32_t); // nbParts
	recordSize += sizeof(int32_t); // nbPoints
	recordSize += (numParts * sizeof(int32_t)); // Parts
	recordSize += (numParts * sizeof(int32_t)); // Parts Type
	recordSize += (numPoints * 2 * sizeof(double)); //Points
    recordSize += (2 * sizeof(double)); // zRange
    recordSize += (numPoints) * sizeof(double); // Zs
    recordSize += (2 * sizeof(double)); // mRange
    recordSize += (numPoints) * sizeof(double); // Measures
	recordSize /= 2; // 16bit words

	// Record Header
	stream.setByteOrder(QDataStream::BigEndian);
	stream << recordNumber << recordSize;

	qint64 recordStart = stream.device()->pos();
	stream.setByteOrder(QDataStream::LittleEndian);
	stream << static_cast<int32_t >(ESRI_SHAPE_TYPE::MULTI_PATCH);

	CCVector3d bbMing, bbMaxg;
	mesh->getGlobalBB(bbMing, bbMaxg);

	stream << bbMing.x << bbMing.y << bbMaxg.x << bbMaxg.y;
	stream << numParts << numPoints;
	stream << static_cast<int32_t>(0); // Parts
	stream << static_cast<int32_t>(triangleType);

	save3DCloud(stream, vertices, bbMing, bbMaxg);

	qint64 recordEnd = stream.device()->pos();
	qint64 bytesWritten = recordEnd - recordStart;
	if (bytesWritten != 2 * recordSize) {
		ccLog::Error(QString("wrote %1 bytes, expected to write %2").arg(bytesWritten).arg(recordSize * 2));
		return CC_FERR_WRITING;
	}
	return CC_FERR_NO_ERROR;

}

//! Update the FileSize header field to the given value
/**
 *
 * @param stream stream of a .shp or .shx file
 * @param fileSize16bit new file length in 16bits words
 */
void updateFileSize(QDataStream &stream, int32_t fileSize16bit)
{
	qint64 oldPos = stream.device()->pos();
	stream.device()->seek(FILE_SIZE_OFFSET);
	stream.setByteOrder(QDataStream::BigEndian);
	stream << fileSize16bit;
	stream.device()->seek(oldPos);
}

CC_FILE_ERROR ShpFilter::saveToFile(ccHObject *entity, const QString &filename, const SaveParameters &parameters) {
    std::vector<GenericDBFField *> fields;
    return saveToFile(entity, fields, filename, parameters);
}

//! Saves the entity(ies) to a Shapefile
//! Clouds are savedd as ESRI_SHAPE_TYPE::MULTI_POINT_Z
//! Meshes are savec as ESRI_SHAPE_TYPE::MULTIPATCH
//! Polylines are stored as POLYGON, POLYGON_2, POLYLINE, POLYLINE_2
//! depending on a few options
//! \param entity
//! \param fields
//! \param filename
//! \param parameters
//! \return
CC_FILE_ERROR
ShpFilter::saveToFile(ccHObject *entity, const std::vector<GenericDBFField *> &fields, const QString &filename,
					  const SaveParameters &parameters) {
	if (!entity)
		return CC_FERR_BAD_ENTITY_TYPE;

	ccHObject::Container toSave;
	getSupportedShapes(entity, toSave);

	if (toSave.empty()) {
		ccLog::Warning("[SHP] No valid entity to save");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	if (!haveSameCCType(toSave)) {
		ccLog::Error("[SHP] All entities in a Shapefile must have the same type");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	int poly2DVertDim = 2;
	bool save3DPolysAs2D = false;
	bool save3DPolyHeightInDBF = false;
	if (toSave.front()->getClassID() == CC_TYPES::POLY_LINE)
	{
	    if (parameters.alwaysDisplaySaveDialog)
		{
			//display SHP save dialog
			SaveSHPFileDialog ssfDlg(nullptr);
			ssfDlg.save3DPolyAs2DCheckBox->setChecked(s_save3DPolysAs2D);
			ssfDlg.save3DPolyHeightInDBFCheckBox->setChecked(s_save3DPolyHeightInDBF);
			ssfDlg.dimComboBox->setCurrentIndex(s_poly2DVertDim);

			if (!ssfDlg.exec())
				return CC_FERR_CANCELED_BY_USER;

			save3DPolysAs2D = s_save3DPolysAs2D = ssfDlg.save3DPolyAs2DCheckBox->isChecked();
			poly2DVertDim = s_poly2DVertDim = ssfDlg.dimComboBox->currentIndex();
			save3DPolyHeightInDBF = s_save3DPolyHeightInDBF = ssfDlg.save3DPolyHeightInDBFCheckBox->isChecked();

			if (poly2DVertDim >= 3 || poly2DVertDim < 0)
				throw std::runtime_error("Invalid vertical dimension");
		}

		if (!haveSameCorrespondingEsriType(toSave, m_closedPolylinesAsPolygons, save3DPolysAs2D)) {
			ccLog::Error("[SHP] All entities in a Shapefile must have the same type");
			return CC_FERR_BAD_ENTITY_TYPE;
		}

	}
	ESRI_SHAPE_TYPE outputShapeType = esriTypeOfccHobject(toSave.front(), m_closedPolylinesAsPolygons, save3DPolysAs2D);

	if (outputShapeType == ESRI_SHAPE_TYPE::POLYGON ||outputShapeType == ESRI_SHAPE_TYPE::POLYLINE)
	{
		auto hasMeasures = [](const ccHObject *obj) {
			return static_cast<const ccPolyline*>(obj)->getAssociatedCloud()->isScalarFieldEnabled();
		};
		if (std::any_of(toSave.begin(), toSave.end(), hasMeasures))
		{
			outputShapeType = (outputShapeType == ESRI_SHAPE_TYPE::POLYLINE) ? ESRI_SHAPE_TYPE::POLYLINE_M : ESRI_SHAPE_TYPE::POLYGON_M;
		}
	}

	CCVector3d bbMinCorner, bbMaxCorner;
	bool isValid;
	if (entity->isA(CC_TYPES::HIERARCHY_OBJECT)) {
		isValid = bbMinMaxOfHierarchy(entity, bbMinCorner, bbMaxCorner);
	} else {
		isValid = entity->getGlobalBB(bbMinCorner, bbMaxCorner);
	}

	if (!isValid) {
		ccLog::Error("Entity(ies) has(ve) an invalid bounding box?!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	const auto Z = static_cast<unsigned char>(poly2DVertDim);

	ccLog::Print("[SHP] Output type: " + toString(outputShapeType));

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

	if (!isESRIShape3D(outputShapeType)) {
		bbMinCorner.z = 0.0;
		bbMaxCorner.z = 0.0;
	}

	ShapeFileHeader hdr;
	hdr.shapeTypeInt = static_cast<int32_t>(outputShapeType);
	hdr.pointMin = bbMinCorner;
	hdr.pointMax = bbMaxCorner;
	hdr.mRange = CCVector2d(ESRI_NO_DATA, ESRI_NO_DATA); //FIXME

	QDataStream shpStream(&file);
	QDataStream indexStream(&indexFile);

	writeHeaderTo(hdr, shpStream);
	writeHeaderTo(hdr, indexStream);

	unsigned shapeIndex = 0;
	for (auto child : toSave) {
		//check entity eligibility
		if (child->isA(CC_TYPES::POLY_LINE)) {
			if (static_cast<ccPolyline *>(child)->size() < 2) {
				ccLog::Warning(QString("Polyline '%1' is too small! It won't be saved...").arg(child->getName()));
				continue;
			}
		}

		qint64 recordStart = shpStream.device()->pos();

		CC_FILE_ERROR error = CC_FERR_NO_ERROR;
		int32_t recordNumber = ++shapeIndex;
		switch (outputShapeType) {
			case ESRI_SHAPE_TYPE::POLYLINE:
			case ESRI_SHAPE_TYPE::POLYGON:
			case ESRI_SHAPE_TYPE::POLYLINE_M:
			case ESRI_SHAPE_TYPE::POLYGON_M:
			case ESRI_SHAPE_TYPE::POLYGON_Z:
			case ESRI_SHAPE_TYPE::POLYLINE_Z:
			{
				auto *polyLine = static_cast<ccPolyline*>(child);
				error = savePolyline(polyLine, shpStream, recordNumber, outputShapeType, poly2DVertDim);
				break;
			}
			case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
			{
				error = saveAsCloud(ccHObjectCaster::ToGenericPointCloud(child), shpStream, recordNumber);
				break;
			}
			case ESRI_SHAPE_TYPE::MULTI_PATCH:
			{
                auto *mesh = dynamic_cast<ccMesh*>(child);
                if (!mesh){
                	ccLog::Warning("Not a mesh");
					return CC_FERR_BAD_ENTITY_TYPE;
                }
                error = saveMesh(shpStream, mesh, recordNumber);
                break;
			}
			default:
			{
				ccLog::Warning(QString("[SHP] Don't know how to save %1").arg(toString(outputShapeType)));
				break;
			}
		}

		if (error != CC_FERR_NO_ERROR)
			return error;


		//write corresponding entry in index SHX file
		qint64 recordEnd = shpStream.device()->pos();
		indexStream.setByteOrder(QDataStream::BigEndian);
		indexStream << static_cast<int32_t >(recordStart / 2);
		indexStream << static_cast<int32_t >((recordEnd - recordStart) / 2);
	}

	ccLog::Print(QString("FileSize: %1").arg(shpStream.device()->pos()));
	auto fileLength16bit = static_cast<int32_t >(shpStream.device()->pos() / 2);
	updateFileSize(shpStream, fileLength16bit);
	updateFileSize(indexStream, fileLength16bit);

	//eventually, we create the DB file (suffix should be ".dbf")
	QString dbfFilename = baseFileName + QString(".dbf");
	CC_FILE_ERROR result = writeDBF(fields, toSave, save3DPolyHeightInDBF, Z, dbfFilename);

	return result;
}



CC_FILE_ERROR ShpFilter::writeDBF(const std::vector<GenericDBFField *> &fields, const ccHObject::Container &toSave,
								  bool save3DPolyHeightInDBF, const unsigned char Z, const QString &dbfFilename) const {
	DBFHandle dbfHandle = DBFCreate(qPrintable(dbfFilename));
	if (!dbfHandle)
	{
		return CC_FERR_WRITING;
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	//always write an 'index' table
	int fieldIdx = DBFAddField(dbfHandle, "local_idx", FTInteger, 6, 0);
	if (fieldIdx >= 0) {
		for (size_t i = 0; i < toSave.size(); ++i)
			DBFWriteIntegerAttribute(dbfHandle, static_cast<int>(i), fieldIdx, static_cast<int>(i) + 1);
	} else {
		ccLog::Warning(QString("[SHP] Failed to save field 'index' (default)"));
		return CC_FERR_WRITING;
	}

	//write the '3D polylines height' field if request
	if (save3DPolyHeightInDBF) {
		fieldIdx = DBFAddField(dbfHandle, "height", FTDouble, 8, 8);
		if (fieldIdx >= 0) {
			for (size_t i = 0; i < toSave.size(); ++i) {
				auto *poly = static_cast<ccPolyline *>(toSave[i]);
				double height = 0.0;
				if (poly && poly->size() != 0) {
					const CCVector3 *P0 = poly->getPoint(0);
					CCVector3d Pg0 = poly->toGlobal3d(*P0);
					height = Pg0.u[Z];
				}
				DBFWriteDoubleAttribute(dbfHandle, static_cast<int>(i), fieldIdx, height);
			}
		} else {
			ccLog::Warning(QString("[SHP] Failed to save field 'height' (3D polylines height)"));
			return CC_FERR_WRITING;
		}
	}

	//and write the other tables (specified by the user)
	for (auto it : fields) {
		const GenericDBFField *field = it;
		if (field->is3D()) //3D case
		{
			int xFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_x")), field->type(),
										field->width(), field->decimal());
			int yFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_y")), field->type(),
										field->width(), field->decimal());
			int zFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_z")), field->type(),
										field->width(), field->decimal());
			if (xFieldIdx >= 0 && yFieldIdx >= 0 && zFieldIdx >= 0) {
				if (!it->save(dbfHandle, xFieldIdx, yFieldIdx, zFieldIdx))
					xFieldIdx = -1;
			}

			if (xFieldIdx < 0) {
				ccLog::Warning(QString("[SHP] Failed to save field '%1'").arg(field->name()));
				return CC_FERR_WRITING;
			}
		} else //1D case
		{
			fieldIdx = DBFAddField(dbfHandle, qPrintable(field->name()), field->type(), field->width(),
									   field->decimal());
			if (fieldIdx >= 0) {
				if (!it->save(dbfHandle, fieldIdx))
					fieldIdx = -1;
			}

			if (fieldIdx < 0) {
				ccLog::Warning(QString("[SHP] Failed to save field '%1'").arg(field->name()));
				return CC_FERR_WRITING;
			}
		}
	}

	DBFClose(dbfHandle);

	return result;
}


CC_FILE_ERROR ShpFilter::readHeaderInto(QDataStream &stream, ShapeFileHeader &hdr) {
	stream.setByteOrder(QDataStream::BigEndian);

	int32_t fileCode;
	stream >> fileCode;
	if (fileCode != SHP_FILE_CODE) {
		ccLog::Warning("Wrong file code, is it a Shapefile ?");
		return CC_FERR_MALFORMED_FILE;
	}

	stream.skipRawData(5 * sizeof(int32_t));

	stream >> hdr.fileLength;
	hdr.fileLength *= 2; //fileLength is measured in 16-bit words

	ccLog::Print(QString("Filelength: %1").arg(hdr.fileLength));

	/*** WARNING: from now on, we only read data with little endianness! ***/
	stream.setByteOrder(QDataStream::LittleEndian);

	stream >> hdr.version;
	stream >> hdr.shapeTypeInt;

	ccLog::Print(QString("[SHP] Version: %1 - type: %2").arg(hdr.version).arg(
			toString(static_cast<ESRI_SHAPE_TYPE>(hdr.shapeTypeInt))));

	double xMin, yMin, zMin, mMin;
	double xMax, yMax, zMax, mMax;

	stream >> xMin >> yMin;
	stream >> xMax >> yMax;
	stream >> zMin >> zMax;
	stream >> mMin >> mMax;

	if (std::isnan(zMin)) {
		//for 2D entities, the zMin value might be NaN!!!
		zMin = 0;
	}

	hdr.pointMin = CCVector3d(xMin, yMin, zMin);
	hdr.pointMax = CCVector3d(xMax, yMax, zMax);
	hdr.mRange = CCVector2d(mMin, mMax);

	return CC_FERR_NO_ERROR;
}

template <typename T>
CC_FILE_ERROR readElements(QDataStream& stream, std::vector<T>& vector, size_t numElements)
{
	try
	{
		vector.resize(numElements);
	}
	catch (const std::bad_alloc &)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	for (size_t i(0); i < numElements; ++i)
	{
		stream >> vector[i];
	}

	return CC_FERR_NO_ERROR;
}

void readZCoordinates(QDataStream& stream, int32_t numPoints, std::vector<CCVector3>& points, const CCVector3d& Pshift)
{
	double zMin, zMax, z;
	stream >> zMin >> zMax;
	for (int32_t i = 0; i < numPoints; ++i)
	{
		stream >> z;
		points[i].z = static_cast<PointCoordinateType>(z) + Pshift.z;
	}
}

CC_FILE_ERROR readPoints(QDataStream& stream, int32_t numPoints, std::vector<CCVector3>& points, const CCVector3d& Pshift)
{
	try {
		points.resize(numPoints);
	}
	catch (const std::bad_alloc &)
	{
		stream.skipRawData(sizeof(double) * 2 * numPoints);
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	double x, y;
	for (int32_t i = 0; i < numPoints; ++i)
	{
		stream >> x >> y;
		points[i].x = static_cast<PointCoordinateType>(x + Pshift.x);
		points[i].y = static_cast<PointCoordinateType>(y + Pshift.y);
		points[i].z = 0;
	}
	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR readMeasures(QDataStream& stream, int32_t numPoints, std::vector<ScalarType>& scalarValues)
{
	double mMin, mMax, m;
	stream >> mMin >> mMax;

	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	if (mMin != ESRI_NO_DATA && mMax != ESRI_NO_DATA) {
		try {
			scalarValues.reserve(numPoints);
		}
		catch (const std::bad_alloc &) {
			error = CC_FERR_NOT_ENOUGH_MEMORY;
		}
	}

	if (error == CC_FERR_NO_ERROR) {
		for (int32_t i = 0; i < numPoints; ++i) {
			stream >> m;
			scalarValues.push_back(m == ESRI_NO_DATA ? NAN_VALUE : static_cast<ScalarType>(m));
		}
	}
	else
	{
		stream.skipRawData(numPoints * sizeof(m));
	}
	return error;
}

CC_FILE_ERROR buildPolylinesInto(
		ccHObject &container,
		const std::vector<int32_t>& startIndexes,
		const std::vector<CCVector3>& points,
		const std::vector<ScalarType>& scalarValues,
		bool preserveCoordinateShift,
		CCVector3d Pshift,
		bool is3D,
		int index,
		bool load2DPolyAs3DPoly
	)
{
	size_t numParts = startIndexes.size();
	size_t numPoints = points.size();
	//and of course the polyline(s)
	for (int32_t i = 0; i < numParts; ++i) {
		const int32_t &firstIndex = startIndexes[i];
		const int32_t &lastIndex = (i + 1 < numParts ? startIndexes[i + 1] : numPoints) - 1;
		int32_t vertCount = lastIndex - firstIndex + 1;

		//test if the polyline is closed
		bool isClosed = false;
		if (vertCount > 2 && (points[firstIndex] - points[lastIndex]).norm() < ZERO_TOLERANCE) {
			vertCount--;
			isClosed = true;
		}

		//vertices
		ccPointCloud *vertices = new ccPointCloud("vertices");
		if (!vertices->reserve(vertCount)) {
			delete vertices;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		for (int32_t j = 0; j < vertCount; ++j) {
			vertices->addPoint(points[firstIndex + j]);
		}
		vertices->setEnabled(false);
		if (preserveCoordinateShift) {
			vertices->setGlobalShift(Pshift);
		}

		//polyline
		auto *poly = new ccPolyline(vertices);
		poly->addChild(vertices);
		if (preserveCoordinateShift) {
			poly->setGlobalShift(Pshift); //shouldn't be necessary but who knows ;)
		}

		if (!poly->reserve(vertCount)) {
			delete poly;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		poly->addPointIndex(0, static_cast<unsigned>(vertCount));
		QString name = QString("Polyline #%1").arg(index);
		if (numParts != 1)
			name += QString(".%1").arg(i + 1);
		poly->setName(name);
		poly->setClosed(isClosed);
		poly->set2DMode(!is3D && !load2DPolyAs3DPoly);

		if (!scalarValues.empty()) {
		    ccLog::Print("LOL");
			auto *sf = new ccScalarField("Measures");
			if (!sf->reserveSafe(vertCount)) {
				ccLog::Warning(
						QString("[SHP] Polyline #%1.%2: not enough memory to load scalar values!").arg(index).arg(
								i + 1));
				sf->release();
			}
			else
			{
				for (int32_t j = 0; j < vertCount; ++j) {
					sf->addElement(scalarValues[j + firstIndex]);
				}
				sf->computeMinAndMax();
				int sfIdx = vertices->addScalarField(sf);
				vertices->setCurrentDisplayedScalarField(sfIdx);
				vertices->showSF(true);
				vertices->enableScalarField();
			}
		}
		poly->showSF(vertices->sfShown());
		poly->showVertices(true);

		container.addChild(poly);
	}
	return CC_FERR_NO_ERROR;
}


//! Reads and build the Polylines OR Polygons of the record
//! \param stream Intput stream of the Shapefile
//! \param container where tjhe buit polylines or polygons will be stored
//! \param index of the current record
//! \param shapeType of the record
//! \param pShift Coordinate shift to apply
//! \param preserveCoordinateShift
//! \param load2DPolyAs3DPoly
//! \return any errors that could happen
CC_FILE_ERROR loadPolyline(QDataStream &stream,
						   ccHObject &container,
						   int32_t index,
						   ESRI_SHAPE_TYPE shapeType,
						   const CCVector3d &pShift,
						   bool preserveCoordinateShift,
						   bool load2DPolyAs3DPoly = true)
{
	CC_FILE_ERROR error;
	stream.setByteOrder(QDataStream::LittleEndian);
	double xMin, yMin, xMax, yMax;
	stream >> xMin >> yMin >> xMax >> yMax;

	int32_t numParts, numPoints;
	stream >> numParts >> numPoints;

	//for each part, the index of its first point in the points array
	std::vector<int32_t> startIndexes;
	error = readElements(stream, startIndexes, static_cast<size_t>(numParts));
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	std::vector<CCVector3> points;
	error = readPoints(stream, numPoints, points, pShift);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	//3D polylines
	bool is3D = isESRIShape3D(shapeType);
	if (is3D)
	{
		readZCoordinates(stream, numPoints, points, pShift);
	}

	//3D polylines or 2D polylines + measurement
	std::vector<ScalarType> scalarValues;
	if (hasMeasurements(shapeType))
	{
		if (readMeasures(stream, numPoints, scalarValues) == CC_FERR_NOT_ENOUGH_MEMORY)
		{
			ccLog::Warning(QString("[SHP] Not enough memory to read measures for shape n° %1").arg(index));
		}
	}

	error = buildPolylinesInto(
		container,
		startIndexes,
		points,
		scalarValues,
		preserveCoordinateShift,
		pShift,
		is3D,
		index,
		load2DPolyAs3DPoly
	);

	return error;
}

//! Reads numPoints z coordinates from the stream, applies the given shift
//! and modify the point cloud points's z
void readZCoordinates(QDataStream &stream, int32_t numPoints, ccPointCloud *cloud, double zShift) {
	double zMin, zMax;
	stream >> zMin >> zMax;

	double z;
	for (int32_t i = 0; i < numPoints; ++i) {
		stream >> z;
		const CCVector3 *P = cloud->getPoint(static_cast<unsigned int>(i));
		const_cast<CCVector3 *>(P)->z = static_cast<PointCoordinateType>(z + zShift);
	}
	cloud->invalidateBoundingBox();
}

//! Reads numPoints measures from the stream.
//! add them to the cloud if the range is not null
void readMeasures(QDataStream &stream, int32_t numPoints, ccPointCloud *cloud) {
	ccScalarField *sf = nullptr;
	double mMin, mMax;
	stream >> mMin >> mMax;

	if (mMin != ESRI_NO_DATA && mMax != ESRI_NO_DATA) {
		sf = new ccScalarField("Measures");
		if (!sf->reserveSafe(static_cast<size_t>(numPoints))) {
			ccLog::Warning("[SHP] Not enough memory to load scalar values!");
			sf->release();
		} else {
			double m;
			for (int32_t i = 0; i < numPoints; ++i) {
				stream >> m;
				ScalarType s = m == ESRI_NO_DATA ? NAN_VALUE : static_cast<ScalarType>(m);
				sf->addElement(s);
			}
			sf->computeMinAndMax();
			int sfIdx = cloud->addScalarField(sf);
			cloud->setCurrentDisplayedScalarField(sfIdx);
			cloud->showSF(true);
		}
	} else {
		stream.skipRawData(sizeof(double) * numPoints);
	}
}

//! Reads the content of a MultiPoint record type and builds its point cloud
//! \param stream Intput stream of the Shapefile
//! \param container where the built cloud will be stored
//! \param index of the record
//! \param shapeType shape type of the record
//! \param pShift Coordinate shift to apply to the read coordinates
//! \param preserveCoordinateShift
//! \return errors that happened during the process
CC_FILE_ERROR loadCloud(QDataStream &stream,
						ccHObject &container,
						int32_t index,
						ESRI_SHAPE_TYPE shapeType,
						const CCVector3d &pShift,
						bool preserveCoordinateShift) {
	double xMin, yMin, xMax, yMax;
	stream >> xMin >> yMin >> xMax >> yMax;

	int32_t numPoints;
	stream >> numPoints;

	auto *cloud = new ccPointCloud(QString("Cloud #%1").arg(index));
	if (!cloud->reserve(static_cast<unsigned int>(numPoints)))
	{
		delete cloud;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	if (preserveCoordinateShift)
	{
		cloud->setGlobalShift(pShift);
	}

	double x, y;
	for (int32_t i = 0; i < numPoints; ++i)
	{
		stream >> x >> y;
		CCVector3 P(static_cast<PointCoordinateType>(x + pShift.x),
					static_cast<PointCoordinateType>(y + pShift.y),
					0);
		cloud->addPoint(P);
	}

	//3D clouds
	if (shapeType == ESRI_SHAPE_TYPE::MULTI_POINT_Z)
	{
		readZCoordinates(stream, numPoints, cloud, pShift.z);
	}

	//3D clouds or 2D clouds + measurement
	if (hasMeasurements(shapeType))
	{
		readMeasures(stream, numPoints, cloud);
	}

	container.addChild(cloud);

	return CC_FERR_NO_ERROR;
}

//! Reads the measure for a single point shape type
//! This functions does not allocate a scalar field until it sees a value
//! different than ESRI_NO_DATA
//! \param stream Input stream of the Shapefile
//! \param cloud where the measure will be stored
void readSingleMeasure(QDataStream &stream, ccPointCloud *cloud) {
	double m;
	stream >> m;
	if (m != ESRI_NO_DATA) {
		if (!cloud->hasScalarFields())
		{
			int sfIdx = cloud->addScalarField("Measures");
			cloud->setCurrentScalarField(sfIdx);
		} else
		{
			int sfIdx = cloud->getScalarFieldIndexByName("Measures");
			auto *sf = cloud->getScalarField(sfIdx);
			sf->resize(cloud->size() + 1);
		}

		auto s = (m == ESRI_NO_DATA) ? NAN_VALUE : static_cast<ScalarType >(m);
		for (unsigned j(0); j < cloud->size() - 1; ++j)
		{
			cloud->setPointScalarValue(j, NAN_VALUE);
		}
		cloud->setPointScalarValue(cloud->size() - 1, s);
	}
}

//! Reads a single point record and append the data the cloud
//! \param stream Input stream of the Shapefile
//! \param shapeType POINT or POINT_M or PONT_Z
//! \param cloud where the point read will be appended
void readSinglePoint(QDataStream &stream, ESRI_SHAPE_TYPE shapeType, ccPointCloud *cloud) {
	stream.setByteOrder(QDataStream::LittleEndian);

	double x, y, z = 0.0;
	const CCVector3d pShift = cloud->getGlobalShift();

	stream >> x >> y;
	x += pShift.x;
	y += pShift.y;

	if (isESRIShape3D(shapeType)) {
		stream >> z;
		z += pShift.z;
	}

	CCVector3 point(
			static_cast<PointCoordinateType >(x),
			static_cast<PointCoordinateType >(y),
			static_cast<PointCoordinateType >(z));
	cloud->addPoint(point);

	if (hasMeasurements(shapeType)) {
		readSingleMeasure(stream, cloud);
	}
	ccLog::Print(QString("Single Points: %1").arg(cloud->size()));
}

//! Builds the ccPointCloud of vertices
//! Returns nullptr if out of memory
ccPointCloud* buildVertices(const std::vector<CCVector3 >& points, int32_t firstIndex, int32_t lastIndex)
{
	int32_t vertCount = lastIndex - firstIndex + 1;

	auto *vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertCount)) {
		delete vertices;
		return nullptr;
	}
	for (int32_t j = 0; j < vertCount; ++j) {
		vertices->addPoint(points[firstIndex + j]);
	}
	vertices->setEnabled(false);
	ccLog::Print(QString("Vertices: %1").arg(vertCount));
	return vertices;
}

//! Creates the ccMesh
ccMesh *createMesh(
		const std::vector<CCVector3>& points,
		const std::vector<ScalarType>& scalarValues,
		int32_t firstIndex,
		int32_t lastIndex
		)
{
	int32_t vertCount = lastIndex - firstIndex + 1;
	if (vertCount < 3) {
		return nullptr;
	}

	ccPointCloud *vertices = buildVertices(points, firstIndex, lastIndex);
	if (!vertices)
	{
		return nullptr;
	}

	if (!scalarValues.empty())
	{
		auto *sf = new ccScalarField("Measures");
		if (!sf->reserveSafe(vertCount)) {
			ccLog::Warning(
					QString("[SHP] Mesh: not enough memory to load scalar values!"));
			sf->release();
		}
		else
		{
			for (int32_t j = 0; j < vertCount; ++j) {
				sf->addElement(scalarValues[j + firstIndex]);
			}
			sf->computeMinAndMax();
			int sfIdx = vertices->addScalarField(sf);
			vertices->setCurrentDisplayedScalarField(sfIdx);
			vertices->showSF(true);
			vertices->enableScalarField();
		}
	}

	return new ccMesh(vertices);
}

//! Builds the patches that where read from a MultiPatch record of a Shapefile
//! patches are build as meshes, each part is its own mesh
//! The only supported Patches/Part types are Triangle_Fan & Triangle_Strip
CC_FILE_ERROR buildPatches(
		ccHObject &container,
		const std::vector<int32_t>& startIndexes,
		const std::vector<int32_t>& partTypes,
		const std::vector<CCVector3>& points,
		const std::vector<ScalarType>& scalarValues
		)
{
	size_t numParts = startIndexes.size();
	size_t numPoints = points.size();


	for (int32_t i = 0; i < numParts; ++i) {
		ccLog::Print(QString("Part %1, firstIdx: %2").arg(i).arg(startIndexes[i]));
	}
	ccLog::Print(QString("NumPts: %1, numParts: %2").arg(numPoints).arg(numParts));
	for (int32_t i = 0; i < numParts; ++i) {
		ccLog::Print(QString("PART TYPE: %1").arg(partTypes[i]));
		auto type = static_cast<ESRI_PART_TYPE>(partTypes[i]);

		const int32_t &firstIndex = startIndexes[i];
		const int32_t &lastIndex = (i + 1 < numParts ? startIndexes[i + 1] : numPoints) - 1;
		const int32_t vertCount = lastIndex - firstIndex + 1;

		switch (type)
		{
			case ESRI_PART_TYPE::TRIANGLE_STRIP:
			{
				ccMesh *mesh = createMesh(points, scalarValues, firstIndex, lastIndex);
				for (int32_t j(2); j < vertCount; ++j) {
					mesh->addTriangle(j - 2, j - 1, j);
				}
				container.addChild(mesh);
				return CC_FERR_NO_ERROR;
			}
			case ESRI_PART_TYPE::TRIANGLE_FAN:
			{
				ccMesh *mesh = createMesh(points, scalarValues, firstIndex, lastIndex);
				for (int32_t j(2); j < vertCount; ++j) {
					mesh->addTriangle(0, j - 1, j);
				}
				container.addChild(mesh);
				return CC_FERR_NO_ERROR;
			}
			default:
				ccLog::Print(QString("[SHP] Cannot handle Patch of type: %1").arg(partTypes[i]));
            	return CC_FERR_BAD_ENTITY_TYPE;
		}
	}
}

//! Reads the Multi Patch record ands builds the meshes if it can
//! \param stream Input stream of the Shapefile
//! \param container where the build meshes will be stored
//! \param Pshift Coordinate shift to apply
//! \return Any error that could happen
CC_FILE_ERROR readMultiPatch(QDataStream &stream, ccHObject &container, CCVector3d Pshift)
{
	CC_FILE_ERROR error;
	CCVector2d bbMin, bbMax;
	stream >> bbMin.x >> bbMin.y >> bbMax.x >> bbMax.y;

	int32_t numParts, numPoints;
	stream >> numParts >> numPoints;

	std::vector<int32_t> parts;
	error = readElements(stream, parts, static_cast<size_t>(numParts));
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	std::vector<int32_t> partTypes;
	error = readElements(stream, partTypes, numParts);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	std::vector<CCVector3 > points;
	error = readPoints(stream, numPoints, points, Pshift);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	readZCoordinates(stream, numPoints, points, Pshift);

	std::vector<ScalarType> measures;
	error = readMeasures(stream, numPoints, measures);
	if (error != CC_FERR_NO_ERROR)
	{
		ccLog::Warning(QString("Out of memory when reading measurements"));
	}

	error = buildPatches(container, parts, partTypes, points, measures);

	return error;
}

void ShpFilter::loadDBF(const QString &filename, ccPointCloud *singlePoints, QMap<ccPolyline *, int32_t> &polyIDs,
						int32_t maxPolyID, bool hasPolylines, bool hasPoints) const {
	QFileInfo fi(filename);
	QString baseFileName = fi.path() + QString("/") + fi.completeBaseName();
	//try to load the DB file (suffix should be ".dbf")
	QString dbfFilename = baseFileName + QString(".dbf");
	DBFHandle dbfHandle = DBFOpen(qPrintable(dbfFilename), "rb");
	if (dbfHandle) {
		int fieldCount = DBFGetFieldCount(dbfHandle);
		int recordCount = DBFGetRecordCount(dbfHandle);
		if (fieldCount == 0) {
			ccLog::Warning("[SHP] No field in the associated DBF file!");
		} else if (hasPolylines && recordCount < static_cast<int>(maxPolyID)) {
			ccLog::Warning("[SHP] No enough records in the associated DBF file!");
		} else if (hasPoints && recordCount < static_cast<int>(singlePoints->size())) {
			ccLog::Warning("[SHP] No enough records in the associated DBF file!");
		} else {
			QList<FieldIndexAndName> candidateFields;
			for (int i = 0; i < fieldCount; ++i) {
				char fieldName[256];
				DBFFieldType fieldType = DBFGetFieldInfo(dbfHandle, i, fieldName, nullptr, nullptr);
				if (fieldType == FTDouble || fieldType == FTInteger) {
					candidateFields.push_back(FieldIndexAndName(i, QString(fieldName)));
				}
			}

			if (!candidateFields.empty()) {
				//create a list of available fields
				ImportDBFFieldDialog lsfDlg(nullptr);
				for (auto it = candidateFields.begin(); it != candidateFields.end(); ++it) {
					lsfDlg.listWidget->addItem(it->second);
				}
				lsfDlg.scaleDoubleSpinBox->setValue(s_dbfFieldImportScale);
				lsfDlg.okPushButton->setVisible(false);


				if (lsfDlg.exec()) {
					s_dbfFieldImportScale = lsfDlg.scaleDoubleSpinBox->value();

					//look for the selected index
					int index = -1;
					for (int i = 0; i < candidateFields.size(); ++i) {
						if (lsfDlg.listWidget->isItemSelected(lsfDlg.listWidget->item(i))) {
							index = candidateFields[i].first;
							break;
						}
					}

					if (index >= 0) {
						double scale = s_dbfFieldImportScale;
						//read values
						DBFFieldType fieldType = DBFGetFieldInfo(dbfHandle, index, nullptr, nullptr, nullptr);

						if (hasPolylines) {
							//for each poyline
							for (auto it = polyIDs.begin(); it != polyIDs.end(); ++it) {
								//get the height
								double z;
								if (fieldType == FTDouble)
									z = DBFReadDoubleAttribute(dbfHandle, it.value() - 1, index);
								else //if (fieldType == FTInteger)
									z = static_cast<double>(DBFReadIntegerAttribute(dbfHandle, it.value() - 1,
																					index));
								z *= scale;

								//translate the polyline
								CCVector3 T(0, 0, static_cast<PointCoordinateType>(z));
								ccGLMatrix trans;
								trans.setTranslation(T);
								ccPolyline *poly = it.key();
								if (poly) {
									poly->applyGLTransformation_recursive(&trans);
									//this transformation is of no interest for the user
									poly->resetGLTransformationHistory_recursive();
									//add the 'const altitude' meta-data as well
									poly->setMetaData(ccPolyline::MetaKeyConstAltitude(), QVariant(z));
								}
							}
						} else if (hasPoints) {
							//for each point
							for (unsigned i = 0; i < singlePoints->size(); ++i) {
								//get the height
								double z;
								if (fieldType == FTDouble)
									z = DBFReadDoubleAttribute(dbfHandle, static_cast<int>(i), index);
								else //if (fieldType == FTInteger)
									z = static_cast<double>(DBFReadIntegerAttribute(dbfHandle, static_cast<int>(i),
																					index));
								z *= scale;

								//set the point height
								const_cast<CCVector3 *>(singlePoints->getPoint(
										i))->z = static_cast<PointCoordinateType >(z);
							}
							singlePoints->invalidateBoundingBox();
						} else {
							assert(false);
						}
					}
				}
			} else {
				ccLog::Warning("[SHP] No numerical field in the associated DBF file!");
			}
		}
	}
	else
	{
		ccLog::Warning(QString("[SHP] Failed to load associated DBF file ('%1')").arg(dbfFilename));
	}
}

CC_FILE_ERROR ShpFilter::loadFile(const QString &filename, ccHObject &container, LoadParameters &parameters) {
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	if (file.size() < SHP_HEADER_SIZE)
		return CC_FERR_MALFORMED_FILE;

	QDataStream stream(&file);
	ShapeFileHeader hdr;

	CC_FILE_ERROR err = readHeaderInto(stream, hdr);
	if (err != CC_FERR_NO_ERROR) {
		return err;
	}

	CCVector3d pShift(0, 0, 0);
	bool preserveCoordinateShift = true;
	if (HandleGlobalShift(hdr.pointMin, pShift, preserveCoordinateShift, parameters)) {
		ccLog::Warning("[SHP] Entities will be recentered! Translation: (%.2f ; %.2f ; %.2f)", pShift.x, pShift.y,
					   pShift.z);
	}

	//progress bar
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget) {
		qint64 fileSize = file.size();
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMaximum(static_cast<int>(fileSize));
		pDlg->setMethodTitle(QObject::tr("Load SHP file"));
		pDlg->setInfo(QObject::tr("File size: %1").arg(fileSize));
		pDlg->start();
		QApplication::processEvents();
	}

	//load shapes
	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	ccPointCloud *singlePoints = nullptr;
	//we also keep track of the polyline's 'record number' (if any)
	QMap<ccPolyline *, int32_t> polyIDs;
	int32_t maxPolyID = 0;
	int32_t maxPointID = 0;
	while (hdr.fileLength - stream.device()->pos() > 0) {
		if (stream.status() != QDataStream::Status::Ok) {
			error = CC_FERR_MALFORMED_FILE;
			break;
		}

		int32_t recordNumber, recordSize, shapeTypeInt;
		stream.setByteOrder(QDataStream::BigEndian);
		stream >> recordNumber >> recordSize;
		qint64 recordStart = stream.device()->pos();
		stream.setByteOrder(QDataStream::LittleEndian);
		stream >> shapeTypeInt;
		recordSize *= 2; //16 bit words

		if (!isValidESRIShapeCode(shapeTypeInt))
		{
			ccLog::Warning(QString("[SHP] Shape %1 does not have a valid type (%2)").arg(recordNumber).arg(shapeTypeInt));
			stream.skipRawData((recordSize) - sizeof(shapeTypeInt));
			continue;
		}
		auto shapeType = static_cast<ESRI_SHAPE_TYPE >(shapeTypeInt);
		ccLog::Print(QString("[SHP] Record #%1 - type: %2").arg(recordNumber).arg(toString(shapeType)));

		switch (shapeType) {
			case ESRI_SHAPE_TYPE::POLYGON_M:
			case ESRI_SHAPE_TYPE::POLYGON_Z:
			case ESRI_SHAPE_TYPE::POLYGON:
			{
				error = loadPolyline(stream, container, recordNumber, shapeType, pShift, preserveCoordinateShift);
				break;
			}
			case ESRI_SHAPE_TYPE::POLYLINE_M:
			case ESRI_SHAPE_TYPE::POLYLINE_Z:
			case ESRI_SHAPE_TYPE::POLYLINE:
			{
				unsigned childCountBefore = container.getChildrenNumber();
				error = loadPolyline(stream, container, recordNumber, shapeType, pShift, preserveCoordinateShift);
				if (error == CC_FERR_NO_ERROR) {
					unsigned childCountAfter = container.getChildrenNumber();
					//warning: we can load mutliple polylines for a single record!
					for (unsigned i = childCountBefore; i < childCountAfter; ++i) {
						ccHObject *child = container.getChild(i);
						polyIDs[static_cast<ccPolyline *>(child)] = recordNumber;
						maxPolyID = std::max(recordNumber, maxPolyID);
					}
				}
				break;
			}
			case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
			case ESRI_SHAPE_TYPE::MULTI_POINT_M:
			case ESRI_SHAPE_TYPE::MULTI_POINT:
			{
				error = loadCloud(stream, container, recordNumber, shapeType, pShift, preserveCoordinateShift);
				break;
			}
			case ESRI_SHAPE_TYPE::POINT_Z:
			case ESRI_SHAPE_TYPE::POINT_M:
			case ESRI_SHAPE_TYPE::SHP_POINT:
			{
				if (!singlePoints) {
					singlePoints = new ccPointCloud("Points");
					if (preserveCoordinateShift) {
						singlePoints->setGlobalShift(pShift);
					}
				}
				readSinglePoint(stream, shapeType, singlePoints);
				maxPointID = std::max(recordNumber, maxPointID);
				break;
			}
			case ESRI_SHAPE_TYPE::MULTI_PATCH:
				error = readMultiPatch(stream, container, pShift);
				break;
			case ESRI_SHAPE_TYPE::NULL_SHAPE:
				break;
			default:
				ccLog::Warning("[SHP] Unhandled type!");
				stream.skipRawData(recordSize - sizeof(shapeTypeInt));
				break;
		}


		if (error != CC_FERR_NO_ERROR) {
			return error;
		}

		if (pDlg) {
			pDlg->setValue(static_cast<int>(stream.device()->pos()));
			if (pDlg->wasCanceled()) {
				error = CC_FERR_CANCELED_BY_USER;
				break;
			}
		}

		qint64 newPos = stream.device()->pos();
		if (recordSize != newPos - recordStart)
		{
			ccLog::Error(QString("Expected to read: %1 bytes, actually read: %2").arg(recordSize).arg(newPos - recordStart));
			return CC_FERR_READING;
		}
	}

	//try to load the DBF to see if there's a 'height' field or something similar for polylines
	bool is3DShape = isESRIShape3D(static_cast<ESRI_SHAPE_TYPE >(hdr.shapeTypeInt));
	bool hasPolylines = (!polyIDs.empty());
	bool hasPoints = (singlePoints && singlePoints->size() != 0 &&
					  maxPointID == static_cast<int32_t>(singlePoints->size()));
	if (!is3DShape && (hasPolylines || hasPoints))
	{
		loadDBF(filename, singlePoints, polyIDs, maxPolyID, hasPolylines, hasPoints);
	}

	if (singlePoints) {
		if (singlePoints->size() == 0) {
			delete singlePoints;
		} else {
			CCLib::ScalarField *sf = singlePoints->getScalarField(0);
			if (sf) {
				sf->computeMinAndMax();
				singlePoints->showSF(true);
			}
			container.addChild(singlePoints);
		}
	}

	return error;
}



#endif //CC_SHP_SUPPORT
      