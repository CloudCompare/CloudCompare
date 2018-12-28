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
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>

//Qt
#include <QFileInfo>

//CCLib
#include <MeshSamplingTools.h>

//System
#include <array>


using FieldIndexAndName = QPair<int, QString>;

//Specific value for NaN
static const double ESRI_NO_DATA = -1.0e38;

static const int32_t ESRI_SHAPE_FILE_CODE = 9994;
static const size_t ESRI_HEADER_SIZE = 100;
static const size_t ESRI_FILE_LENGTH_OFFSET = 24;

//semi-persistent settings
static double s_dbfFielImportScale = 1.0;

//semi-persistent parameters
static bool s_save3DPolysAs2D = false;
static int  s_poly2DVertDim = 2;
static bool s_save3DPolyHeightInDBF = false;


//! ESRI Shapefile's shape types
enum class ESRI_SHAPE_TYPE : int32_t
{
	NULL_SHAPE		= 0,
	//below this point are 2D types
	POINT			= 1,
	POLYLINE		= 3,
	POLYGON			= 5,
	MULTI_POINT		= 8,
	//below this point are 3D types
	POINT_Z			= 11,
	POLYLINE_Z		= 13,
	POLYGON_Z		= 15,
	MULTI_POINT_Z	= 18,
	POINT_M			= 21,
	POLYLINE_M		= 23,
	POLYGON_M		= 25,
	MULTI_POINT_M	= 28,
	MULTI_PATCH		= 31
};

//! Returns true if the code corresponds to a valid ESRI Shape Type
/**
 * \param code The code to check (typically read from a file)
**/
static inline bool isValidESRIShapeCode(int32_t code)
{
	if (code < static_cast<int32_t >(ESRI_SHAPE_TYPE::NULL_SHAPE))
		return false;
	if (code > static_cast<int32_t >(ESRI_SHAPE_TYPE::MULTI_PATCH))
		return false;

	switch (static_cast<ESRI_SHAPE_TYPE >(code))
	{
		case ESRI_SHAPE_TYPE::NULL_SHAPE:
		case ESRI_SHAPE_TYPE::POINT:
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
static inline bool isESRIShape3D(ESRI_SHAPE_TYPE shapeType)
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
static inline bool hasMeasurements(ESRI_SHAPE_TYPE shapeType)
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

static inline bool isESRINoData(double m)
{
	return m  <= ESRI_NO_DATA;
}

static int32_t sizeofMultiPointZ(size_t numPoints) noexcept
{
	size_t recordLength = 0;
	recordLength += sizeof(int32_t); // shapeType
	recordLength += (4 * sizeof(double)); // x,y bbox
	recordLength += sizeof(int32_t); // numPoints
	recordLength += (2 * numPoints * sizeof(double)); // xs, ys
	recordLength += (2 * sizeof(double)); // zRange
	recordLength += (numPoints * sizeof(double)); // zs
	recordLength += (2 * sizeof(double)); // mRange
	recordLength += (numPoints * sizeof(double)); // Measures
	recordLength /= 2; // 16bit words

	return static_cast<int32_t>(recordLength);
}

static int32_t sizeofPolyLine(ESRI_SHAPE_TYPE polylineType, size_t numPoints, size_t numParts = 1)
{
	switch (polylineType)
	{
		case ESRI_SHAPE_TYPE::POLYLINE:
		case ESRI_SHAPE_TYPE::POLYLINE_M:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
		case ESRI_SHAPE_TYPE::POLYGON:
		case ESRI_SHAPE_TYPE::POLYGON_M:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
			break;
		default:
			throw std::invalid_argument("Type is not a polygon or polyline");
	}

	size_t recordLength = 0;
	recordLength += sizeof(int32_t); // ShapeType
	recordLength += (4 * sizeof(double)); // MBR
	recordLength += sizeof(int32_t); // nbParts
	recordLength += sizeof(int32_t); // nbPoints
	recordLength += (numParts * sizeof(int32_t)); // Parts
	recordLength += (numPoints * 2 * sizeof(double)); //Points

	if (isESRIShape3D(polylineType))
	{
		recordLength += 2 * sizeof(double);
		recordLength += numPoints * sizeof(double);
	}

	if (hasMeasurements(polylineType))
	{
		recordLength += 2 * sizeof(double);
		recordLength += numPoints * sizeof(double);
	}
	return static_cast<int32_t>(recordLength / 2);
}

static bool areVerticesCounterClockwise(const CCLib::GenericIndexedCloudPersist *vertices,
                                        int32_t numPoints,
                                        unsigned char dim1,
                                        unsigned char dim2)
{
	//http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
	//or http://en.wikipedia.org/wiki/Shoelace_formula
	double sum = 0.0;
	unsigned realNumPoints = vertices->size();
	for (int32_t i = 0; i + 1 < numPoints; ++i)
	{
		CCVector3 P1;
		CCVector3 P2;
		vertices->getPoint(i, P1);
		vertices->getPoint((i + 1) % realNumPoints, P2);
		sum += (P2.u[dim1] - P2.u[dim1]) * (P2.u[dim2] + P1.u[dim2]);
	}

	//negative sum = counter-clockwise
	return sum < 0.0;
}

static CCVector2d minMaxOfEnabledScalarField(const CCLib::GenericIndexedCloudPersist *cloud)
{
	CCVector2d minMax(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());
	if (cloud->isScalarFieldEnabled())
	{
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			auto scalar = static_cast<double>(cloud->getPointScalarValue(i));
			minMax.x = std::min(minMax.x, scalar);
			minMax.y = std::max(minMax.y, scalar);
		}
	}

	if (minMax.x == std::numeric_limits<double>::max())
	{
		minMax.x = minMax.y = ESRI_NO_DATA;
	}
	return minMax;
}

static bool bBoxOfHObjectContainer(const ccHObject::Container& objects, CCVector3d &bbMinCorner, CCVector3d bbMaxCorner)
{
	bool isValid = false;
	for (ccHObject *obj : objects)
	{
		CCVector3d minC;
		CCVector3d maxC;
		if (obj->getGlobalBB(minC, maxC))
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
	return isValid;
}


//! Computes the range (min & max) of the enabled scalar field
//! for each object in the container
/**
 * @param objects containter of objects to compute the range on
 * @return 2D vector, x is the min, y in the max
 */
CCVector2d mRangeOfContainer(ccHObject::Container &objects)
{
	CCVector2d range(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());

	auto updateRange = [&range](const CCLib::GenericIndexedCloudPersist *cloud)
	{
		if (!cloud->isScalarFieldEnabled())
			return;
		unsigned numPoints = cloud->size();
		for (unsigned i = 0; i < numPoints; ++i)
		{
			ScalarType val = cloud->getPointScalarValue(i);
			if (val != NAN_VALUE)
			{
				auto m = static_cast<double>(val);
				range.x = std::min(range.x, m);
				range.y = std::max(range.y, m);
			}
		}
	};

	//call the same method on the first child so as to get its type
	for (ccHObject *obj : objects)
	{
		switch (obj->getClassID())
		{
			case CC_TYPES::POINT_CLOUD:
			{
				const ccGenericPointCloud *cloud = ccHObjectCaster::ToGenericPointCloud(obj);
				updateRange(cloud);
				break;
			}
			case CC_TYPES::POLY_LINE:
			{
				const ccPolyline *poly = ccHObjectCaster::ToPolyline(obj);
				const CCLib::GenericIndexedCloudPersist *vertices = poly->getAssociatedCloud();
				updateRange(vertices);
				break;
			}
			default:
				break;
		}
	}

	if (range.x == std::numeric_limits<double>::max())
	{
		range.x = range.y = ESRI_NO_DATA;
	}
	return range;
}


static void updateFileLength(QDataStream& out, int32_t newFileLentgh)
{
	qint64 oldPos = out.device()->pos();
	QDataStream::ByteOrder oldOrder = out.byteOrder();

	out.device()->seek(ESRI_FILE_LENGTH_OFFSET);
	out.setByteOrder(QDataStream::BigEndian);
	out << newFileLentgh;

	out.device()->seek(oldPos);
	out.setByteOrder(oldOrder);
}

struct ShapeFileHeader
{
	int32_t fileLength = ESRI_HEADER_SIZE;
	int32_t version = 1000;
	int32_t shapeTypeInt = static_cast<int32_t >(ESRI_SHAPE_TYPE::NULL_SHAPE);
	CCVector3d pointMin;
	CCVector3d pointMax;
	CCVector2d mRange; // x is the min, y is the max

	CC_FILE_ERROR readFrom(QDataStream& sin);
	CC_FILE_ERROR writeTo(QDataStream& out);
};

CC_FILE_ERROR ShapeFileHeader::readFrom(QDataStream &sin)
{
	sin.setByteOrder(QDataStream::BigEndian);

	int32_t fileCode;
	sin >> fileCode;
	if (fileCode != ESRI_SHAPE_FILE_CODE)
	{
		ccLog::Warning("[SHP] wrong file code (%d), is this a shape file?", fileCode);
		return CC_FERR_MALFORMED_FILE;
	}

	sin.skipRawData(5 * sizeof(int32_t));

	sin >> fileLength;
	fileLength *= 2;  //fileLength is measured in 16-bit words

	sin.setByteOrder(QDataStream::LittleEndian);

	sin >> version >> shapeTypeInt;

	if (!isValidESRIShapeCode(shapeTypeInt))
	{
		ccLog::Warning("[SHP] invalid shape type code in header (%d)", shapeTypeInt);
		return CC_FERR_MALFORMED_FILE;
	}

	sin >> pointMin.x >> pointMin.y >> pointMax.x >> pointMax.y;
	sin >> pointMin.z >> pointMax.z;

	pointMin.z = std::isnan(pointMin.z) ? 0 : pointMin.z;
	pointMax.z = std::isnan(pointMax.z) ? 0 : pointMax.z;

	sin >> mRange.x >> mRange.y;

	if (sin.status() != QDataStream::Ok)
	{
		ccLog::Warning("[SHP] Something went wrong reading the shp header");
		return CC_FERR_READING;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR ShapeFileHeader::writeTo(QDataStream& out)
{
	out.setByteOrder(QDataStream::BigEndian);

	out << ESRI_SHAPE_FILE_CODE;

	constexpr size_t numBytesToSkip = 5 * sizeof(int32_t);
	constexpr char skipValues[numBytesToSkip] = {0};
	out.writeRawData(skipValues, numBytesToSkip);

	out << fileLength;

	out.setByteOrder(QDataStream::LittleEndian);
	out << version;
	out << shapeTypeInt;
	out << pointMin.x << pointMin.y << pointMax.x << pointMax.y;
	out << pointMin.z << pointMax.z;

	out << mRange.x << mRange.y;

	assert(out.device()->pos() == ESRI_HEADER_SIZE);
	return CC_FERR_NO_ERROR;
}


//! Shape File Save dialog
class SaveSHPFileDialog : public QDialog, public Ui::SaveSHPFileDlg
{
public:
	//! Default constructor
	explicit SaveSHPFileDialog(QWidget* parent = nullptr)
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
	explicit ImportDBFFieldDialog(QWidget* parent = nullptr)
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

static QString ToString(ESRI_SHAPE_TYPE type)
{
	switch (type)
	{
	case ESRI_SHAPE_TYPE::NULL_SHAPE:
		return "Unhandled";
	case ESRI_SHAPE_TYPE::POINT:
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
		return "Unknown";
	}

	return QString("Unknown");
}

void GetSupportedShapes(ccHObject* baseEntity, ccHObject::Container& shapes, ESRI_SHAPE_TYPE& shapeType)
{
	shapeType = ESRI_SHAPE_TYPE::NULL_SHAPE;
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
				shapeType = ESRI_SHAPE_TYPE::MULTI_POINT_Z;
				shapes.push_back(baseEntity);
			}
			break;
		}
		case CC_TYPES::POLY_LINE:
		{
			auto * poly = static_cast<ccPolyline*>(baseEntity);
			shapeType = ESRI_SHAPE_TYPE::POLYLINE_Z;
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
				for (unsigned i = 1; i < baseEntity->getChildrenNumber(); ++i)
				{
					if (baseEntity->getChild(i) && baseEntity->getChild(i)->getClassID() != child->getClassID())
					{
						//mixed shapes are not allowed in shape files
						return;
					}
				}

				//call the same method on the first child so as to get its type
				GetSupportedShapes(child, shapes, shapeType);
				if (shapeType == ESRI_SHAPE_TYPE::NULL_SHAPE)
					return;

				//then add the remaining children
				for (unsigned i = 1; i < baseEntity->getChildrenNumber(); ++i)
				{
					ESRI_SHAPE_TYPE otherShapeType = ESRI_SHAPE_TYPE::NULL_SHAPE;
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
						//mixed shapes are not allowed in shape files
						shapes.clear();
						return;
					}
				}
			}
			break;
		default:
			//nothing to do
			break;
	}
}

static CC_FILE_ERROR LoadPolyline(QDataStream &shpStream,
                                  ccHObject &container,
                                  int32_t index,
                                  ESRI_SHAPE_TYPE shapeType,
                                  const CCVector3d &Pshift,
                                  bool preserveCoordinateShift,
                                  bool load2DPolyAs3DPoly = true)
{
	// skip record bbox
	shpStream.skipRawData(4 * sizeof(double));

	int32_t numParts;
	int32_t numPoints;
	shpStream >> numParts >> numPoints;


	//for each part, the index of its first point in the points array
	std::vector<int32_t> startIndexes;
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
		shpStream >> startIndexes[i];
	}
	//FIXME: we should use this information and create as many polylines as necessary!

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

	for (int32_t i = 0; i < numPoints; ++i)
	{
		double x;
		double y;
		shpStream >> x >> y;
		points[i].x = static_cast<PointCoordinateType>(x + Pshift.x);
		points[i].y = static_cast<PointCoordinateType>(y + Pshift.y);
		points[i].z = 0;
	}

	//3D polylines
	bool is3D = isESRIShape3D(shapeType);
	if (is3D)
	{
		//Z boundaries
		shpStream.skipRawData(2 * sizeof(double));

		//Z coordinates (an array of length NumPoints)
		for (int32_t i = 0; i < numPoints; ++i)
		{
			double z;
			shpStream >> z;
			points[i].z = static_cast<PointCoordinateType>(z + Pshift.z);
		}
	}

	//3D polylines or 2D polylines + measurement
	std::vector<ScalarType> scalarValues;
	if (hasMeasurements(shapeType))
	{
		//M boundaries
		double mMin;
		double mMax;
		shpStream >> mMin >> mMax;

		if (!isESRINoData(mMin) && !isESRINoData(mMax))
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

		//M values (an array of length NumPoints)
		if (!scalarValues.empty())
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				double m;
				shpStream >> m;
				scalarValues[i] = (isESRINoData(m)  ? NAN_VALUE : static_cast<ScalarType>(m));
			}
		}
		else
		{
			shpStream.skipRawData(numPoints * sizeof(double));
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
		if (preserveCoordinateShift)
		{
			vertices->setGlobalShift(Pshift);
		}

		//polyline
		ccPolyline* poly = new ccPolyline(vertices);
		poly->addChild(vertices);
		if (preserveCoordinateShift)
		{
			poly->setGlobalShift(Pshift); //shouldn't be necessary but who knows ;)
		}

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
			bool allNans = std::all_of(
					scalarValues.begin() + firstIndex,
					scalarValues.begin() + lastIndex,
					[](ScalarType v) {return std::isnan(v);}
			);
			if (!allNans)
			{
				auto* sf = new ccScalarField("Measures");
				if (!sf->reserveSafe(vertCount))
				{
					ccLog::Warning(QString("[SHP] Polyline #%1.%2: not enough memory to load scalar values!").arg(index).arg(i + 1));
					sf->release();
					sf = nullptr;
				}
				for (int32_t j = 0; j < vertCount; ++j)
				{
					sf->addElement(scalarValues[j + firstIndex]);
				}
				sf->computeMinAndMax();
				int sfIdx = vertices->addScalarField(sf);
				vertices->setCurrentDisplayedScalarField(sfIdx);
				vertices->showSF(true);
				vertices->enableScalarField();
			}
		}
		container.addChild(poly);
	}
	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR SavePolyline(ccPolyline *poly,
                                  QDataStream &out,
                                  int32_t &recordSize,
                                  int32_t recordNumber,
                                  ESRI_SHAPE_TYPE outputShapeType,
                                  int vertDim = 2)
{
	assert(vertDim >= 0 && vertDim < 3);
	const auto Z = static_cast<unsigned char>(vertDim);
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	if (!poly)
	{
		assert(false);
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	CCLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
	if (!vertices)
		return CC_FERR_BAD_ENTITY_TYPE;

	int32_t realNumPoints = poly->size();
	switch (outputShapeType)
	{
		case ESRI_SHAPE_TYPE::POLYGON:
		case ESRI_SHAPE_TYPE::POLYGON_M:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
			if (realNumPoints < 3)
			{
				ccLog::Warning("[SHP] Polyline doest not have enough points to be saved as polygon");
				return CC_FERR_BAD_ENTITY_TYPE;
			}
			break;
		case ESRI_SHAPE_TYPE::POLYLINE:
		case ESRI_SHAPE_TYPE::POLYLINE_M:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
			if (realNumPoints < 2)
			{
				ccLog::Warning("[SHP] Polyline does not have enough points to be saved");
				return CC_FERR_BAD_ENTITY_TYPE;
			}
			break;
		default:
			assert(false);
			return CC_FERR_BAD_ENTITY_TYPE;
	}

	bool isClosed = poly->isClosed();

	int32_t numPoints = realNumPoints;
	if (isClosed)
		numPoints++;

	if (numPoints > std::numeric_limits<int32_t>::max())
	{
		ccLog::Warning("[SHP] Polyline has to many points to be saved");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	int32_t numParts = 1;

	recordSize = sizeofPolyLine(outputShapeType, numPoints, numParts);
	out.setByteOrder(QDataStream::BigEndian);
	out << recordNumber << recordSize;

	qint64 recordStart = out.device()->pos();
	out.setByteOrder(QDataStream::LittleEndian);
	out << static_cast<int32_t>(outputShapeType);

	CCVector3d bbMing;
	CCVector3d bbMaxg;
	poly->getGlobalBB(bbMing, bbMaxg);
	out << bbMing.u[X] << bbMing.u[Y] << bbMaxg.u[X] << bbMaxg.u[Y];

	out << numParts;
	out << numPoints;

	//for each part, the index of its first point in the points array
	out << static_cast<int32_t>(0);

	//for polygons we must list the vertices in the right order:
	//"The neighborhood to the right of an observer walking along
	//the ring in vertex order is the inside of the polygon"
	//== clockwise order
	bool inverseOrder = false;
	if (outputShapeType == ESRI_SHAPE_TYPE::POLYGON || outputShapeType == ESRI_SHAPE_TYPE::POLYGON_Z)
	{
		assert(isClosed);
		assert(numPoints > 2);

		unsigned char dim1 = X;
		unsigned char dim2 = Y;
		if (outputShapeType == ESRI_SHAPE_TYPE::POLYGON_Z)
		{
			CCVector3d diag = bbMaxg - bbMing;

			//in 3D we have to guess the 'flat' dimension
			unsigned char minDim = diag.u[1] < diag.u[0] ? 1 : 0;
			if (diag.u[2] < diag.u[minDim])
				minDim = 2;
			dim1 = minDim == 2 ? 0 : minDim + 1;
			dim2 = dim1 == 2 ? 0 : dim1 + 1;
		}

		inverseOrder = areVerticesCounterClockwise(vertices, numPoints, dim1, dim2);
	}

	//Points (An array of length NumPoints)
	for (int32_t i = 0; i < numPoints; ++i)
	{
		int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
		const CCVector3* P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed
		CCVector3d Pg = poly->toGlobal3d(*P);
		out << Pg.u[X] << Pg.u[Y];
	}

	//3D polylines
	if (isESRIShape3D(outputShapeType))
	{
		//Z boundaries
		out << bbMing.u[Z] << bbMaxg.u[Z];

		//Z coordinates (for each part - just one here)
		for (int32_t i = 0; i < numPoints; ++i)
		{
			int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
			const CCVector3 *P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed
			CCVector3d Pg = poly->toGlobal3d(*P);
			out << Pg.u[Z];
		}
	}

	if (hasMeasurements(outputShapeType))
	{
		//M boundaries
		bool hasSF = vertices->isScalarFieldEnabled();
		CCVector2d minMax = minMaxOfEnabledScalarField(vertices);
		out << minMax.x << minMax.y;

		//M values (for each part - just one here)
		if (hasSF)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				ScalarType scalar = vertices->getPointScalarValue(i % realNumPoints);
				out << (scalar == NAN_VALUE ? ESRI_NO_DATA : static_cast<double>(scalar));
			}
		}
		else
		{
			for (int32_t i = 0; i < numPoints; ++i)
				out << ESRI_NO_DATA;
		}
	}

	assert(out.device()->pos() - recordStart == recordSize * 2);
	return CC_FERR_NO_ERROR;
}


static CC_FILE_ERROR LoadCloud(QDataStream &shpStream,
                               ccHObject &container,
                               int32_t index,
                               ESRI_SHAPE_TYPE shapeType,
                               const CCVector3d &Pshift,
                               bool preserveCoordinateShift)
{
	// Skip record bbox
	shpStream.skipRawData(4 * sizeof(double));

	int32_t numPoints;
	shpStream >> numPoints;

	auto* cloud = new ccPointCloud(QString("Cloud #%1").arg(index));
	if (!cloud->reserve(numPoints))
	{
		delete cloud;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}
	if (preserveCoordinateShift)
	{
		cloud->setGlobalShift(Pshift);
	}

	//Points (An array of length NumPoints)
	for (int32_t i = 0; i < numPoints; ++i)
	{
		double x;
		double y;
		shpStream >> x >> y;
		CCVector3 P(static_cast<PointCoordinateType>(x + Pshift.x),
					static_cast<PointCoordinateType>(y + Pshift.y),
					0);
		cloud->addPoint(P);
	}

	//3D clouds
	if (isESRIShape3D(shapeType))
	{
		//Z boundaries
		shpStream.skipRawData(2 * sizeof(double));

		//Z coordinates (an array of length NumPoints)
		for (int32_t i = 0; i < numPoints; ++i)
		{
			double z;
			shpStream >> z;
			const CCVector3* P = cloud->getPoint(i);
			const_cast<CCVector3*>(P)->z = static_cast<PointCoordinateType>(z + Pshift.z);
		}
		cloud->invalidateBoundingBox();
	}

	//3D clouds or 2D clouds + measurement
	if (hasMeasurements(shapeType))
	{
		//M boundaries
		ccScalarField* sf = nullptr;
		double mMin;
		double mMax;
		shpStream >> mMin >> mMax;

		if (mMin != ESRI_NO_DATA && mMax != ESRI_NO_DATA)
		{
			sf = new ccScalarField("Measures");
			if (!sf->reserveSafe(numPoints))
			{
				ccLog::Warning("[SHP] Not enough memory to load scalar values!");
				sf->release();
				sf = nullptr;
			}
		}

		//M values (an array of length NumPoints)
		if (sf)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				double m;
				shpStream >> m;
				ScalarType s = isESRINoData(m) ? NAN_VALUE : static_cast<ScalarType>(m);
				sf->addElement(s);
			}
			bool allNans = std::all_of(sf->begin(), sf->end(), [](ScalarType s) { return std::isnan(s); });
			if (!allNans)
			{
				sf->computeMinAndMax();
				int sfIdx = cloud->addScalarField(sf);
				cloud->setCurrentDisplayedScalarField(sfIdx);
				cloud->showSF(true);
			}
			else
			{
				sf->release();
			}
		}
		else
		{
			shpStream.skipRawData(numPoints * sizeof(double));
		}
	}
	container.addChild(cloud);
	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR SaveAsCloud(ccGenericPointCloud* cloud, QDataStream& out, int32_t recordNumber, int32_t& recordSize)
{
	if (!cloud)
	{
		assert(false);
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	if (cloud->size() > std::numeric_limits<int32_t>::max())
	{
		ccLog::Print("[SHP] Cloud is to big to be saved");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	recordSize = sizeofMultiPointZ(cloud->size());
	out.setByteOrder(QDataStream::BigEndian);
	out << recordNumber << recordSize;

	CCVector3d bbMing;
	CCVector3d bbMaxg;
	cloud->getGlobalBB(bbMing, bbMaxg);

	int64_t recordStart = out.device()->pos();
	out.setByteOrder(QDataStream::LittleEndian);
	out << static_cast<int32_t>(ESRI_SHAPE_TYPE::MULTI_POINT_Z);

	out << bbMing.x << bbMing.y << bbMaxg.x << bbMaxg.y;
	out << static_cast<int32_t >(cloud->size());


	// X & Y
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		CCVector3d Pg = cloud->toGlobal3d(*P);
		out << Pg.x << Pg.y;
	}

	// Z Coordinates
	out << bbMing.z << bbMaxg.z;
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		CCVector3d Pg = cloud->toGlobal3d(*P);
		out << Pg.z;
	}

	//M boundaries
	bool hasSF = cloud->isScalarFieldEnabled();
	CCVector2d minMax = minMaxOfEnabledScalarField(cloud);
	out << minMax.x << minMax.y;

	//M values
	double scalar = ESRI_NO_DATA;
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		if (hasSF)
		{
			ScalarType value = cloud->getPointScalarValue(i);
			if (value != NAN_VALUE)
			{
				scalar = static_cast<double>(cloud->getPointScalarValue(i));
			}
		}
		out << scalar;
	}

	assert(out.device()->pos() - recordStart  == recordSize * 2);
	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR LoadSinglePoint(QDataStream &shpStream,
                                     ccPointCloud *&singlePoints,
                                     ESRI_SHAPE_TYPE shapeType,
                                     const CCVector3d &Pshift,
                                     bool preserveCoordinateShift)
{
	if (!singlePoints)
	{
		singlePoints = new ccPointCloud("Points");
		if (preserveCoordinateShift)
		{
			singlePoints->setGlobalShift(Pshift);
		}
	}

	double x;
	double y;
	shpStream >> x >> y;
	CCVector3 P(static_cast<PointCoordinateType>(x + Pshift.x),
	            static_cast<PointCoordinateType>(y + Pshift.y),
	            0);

	if (isESRIShape3D(shapeType))
	{
		double z;
		shpStream >> z;
		P.z = static_cast<PointCoordinateType>(z + Pshift.z);
	}
	singlePoints->addPoint(P);

	ScalarType s = NAN_VALUE;
	if (hasMeasurements(shapeType))
	{
		double m;
		shpStream >> m;
		if (!isESRINoData(m))
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

	//make sure to reserve the point cloud memory AFTER declaring the scalar field
	//(otherwise the SF won't be reserved...)
	if (singlePoints->size() == singlePoints->capacity() && !singlePoints->reserve(singlePoints->size() + 256)) //256 each time because it appears some SHP files have many isolated points...
	{
		delete singlePoints;
		singlePoints = nullptr;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	singlePoints->addPoint(P);

	if (singlePoints->getCurrentOutScalarField())
		singlePoints->getCurrentOutScalarField()->addElement(s);

	return CC_FERR_NO_ERROR;
}


CC_FILE_ERROR ShpFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	std::vector<GenericDBFField*> fields;
	return saveToFile(entity, fields, filename, parameters);
}


CC_FILE_ERROR ShpFilter::saveToFile(ccHObject* entity, const std::vector<GenericDBFField*>& fields, const QString& filename, const SaveParameters& parameters)
{
	if (!entity)
		return CC_FERR_BAD_ENTITY_TYPE;

	//this filter only supports point clouds, meshes and polylines!
	ESRI_SHAPE_TYPE inputShapeType = ESRI_SHAPE_TYPE::NULL_SHAPE;
	ccHObject::Container toSave;
	GetSupportedShapes(entity, toSave, inputShapeType);

	if (inputShapeType == ESRI_SHAPE_TYPE::NULL_SHAPE || toSave.empty())
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	CCVector3d bbMinCorner, bbMaxCorner;
	if (!bBoxOfHObjectContainer(toSave, bbMinCorner, bbMaxCorner))
	{
		ccLog::Error("Entity(ies) has(ve) an invalid bounding box?!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	bool save3DPolysAs2D = false;
	int poly2DVertDim = 2;
	bool save3DPolyHeightInDBF = false;
	if (parameters.alwaysDisplaySaveDialog && inputShapeType == ESRI_SHAPE_TYPE::POLYLINE_Z)
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
	}
	assert(poly2DVertDim >= 0 && poly2DVertDim < 3);
	const auto Z = static_cast<unsigned char>(poly2DVertDim);
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	ESRI_SHAPE_TYPE outputShapeType = inputShapeType;

	// Promote to polygon
	if (m_closedPolylinesAsPolygons &&  outputShapeType == ESRI_SHAPE_TYPE::POLYLINE_Z)
	{
		auto isClosed = [](const ccHObject *obj) {return static_cast<const ccPolyline*>(obj)->isClosed();};
		bool allClosed = std::all_of(toSave.begin(), toSave.end(), isClosed);
		if (allClosed)
		{
			outputShapeType = ESRI_SHAPE_TYPE::POLYGON_Z;
		}
	}

	// Demote to 2D polyline/polygon and remove "measure" dimension if not needed
	if (save3DPolysAs2D)
	{
		auto hasSF = [](const ccHObject *obj){ return static_cast<const ccPolyline*>(obj)->isScalarFieldEnabled(); };
		bool anyHasSF = std::any_of(toSave.begin(), toSave.end(), hasSF);
		if (anyHasSF)
		{
			if (outputShapeType == ESRI_SHAPE_TYPE::POLYLINE_Z)
				outputShapeType = ESRI_SHAPE_TYPE::POLYLINE_M;
			else
				outputShapeType = ESRI_SHAPE_TYPE::POLYGON_M;
		}
		else
		{
			if (outputShapeType == ESRI_SHAPE_TYPE::POLYLINE_Z)
				outputShapeType = ESRI_SHAPE_TYPE::POLYLINE;
			else
				outputShapeType = ESRI_SHAPE_TYPE::POLYGON;
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

	QDataStream shpStream(&file);
	QDataStream idxStream(&indexFile);

	ShapeFileHeader hdr;
	hdr.pointMin = CCVector3d(bbMinCorner.u[X], bbMinCorner.u[Y], bbMinCorner.u[Z]);
	hdr.pointMax = CCVector3d(bbMaxCorner.u[X], bbMaxCorner.u[Y], bbMaxCorner.u[Z]);
	hdr.shapeTypeInt = static_cast<int32_t>(outputShapeType);
	hdr.mRange = mRangeOfContainer(toSave);

	hdr.writeTo(shpStream);
	hdr.writeTo(idxStream);

	//save shapes
	unsigned shapeIndex = 0;
	for (ccHObject *child : toSave)
	{
		//check entity eligibility
		if (child->isA(CC_TYPES::POLY_LINE))
		{
			if (static_cast<ccPolyline*>(child)->size() < 2)
			{
				ccLog::Warning(QString("Polyline '%1' is too small! It won't be saved...").arg(child->getName()));
				continue;
			}
		}

		int32_t recordSize = 0;
		qint64 recordStart = shpStream.device()->pos();
		CC_FILE_ERROR error = CC_FERR_NO_ERROR;

		switch (outputShapeType)
		{
		case ESRI_SHAPE_TYPE::POLYLINE:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
		case ESRI_SHAPE_TYPE::POLYLINE_M:
		case ESRI_SHAPE_TYPE::POLYGON:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
		case ESRI_SHAPE_TYPE::POLYGON_M:
			assert(child->isKindOf(CC_TYPES::POLY_LINE));
			error = SavePolyline(static_cast<ccPolyline*>(child), shpStream, recordSize, shapeIndex, outputShapeType, poly2DVertDim);
			break;
		case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
			assert(child->isKindOf(CC_TYPES::POINT_CLOUD));
			error = SaveAsCloud(ccHObjectCaster::ToGenericPointCloud(child), shpStream, shapeIndex, recordSize);
			break;
		default:
			assert(false);
			break;
		}

		if (error != CC_FERR_NO_ERROR)
			return error;

		//write corresponding entry in index SHX file
		idxStream << static_cast<int32_t>(recordStart / 2);
		idxStream << static_cast<int32_t>(recordSize);

		ccLog::PrintDebug("[SHP] Saved shape #%d (%d bytes)", shapeIndex, recordSize * 2);
		shapeIndex++;
	}

	//update file lengths
	updateFileLength(shpStream, shpStream.device()->pos() / 2);
	updateFileLength(idxStream, idxStream.device()->pos() / 2);

	file.close();
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
			for (GenericDBFField* field: fields)
			{
				if (field->is3D()) //3D case
				{
					int xFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_x")), field->type(), field->width(), field->decimal());
					int yFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_y")), field->type(), field->width(), field->decimal());
					int zFieldIdx = DBFAddField(dbfHandle, qPrintable(field->name() + QString("_z")), field->type(), field->width(), field->decimal());
					if (xFieldIdx >= 0 && yFieldIdx >= 0 && zFieldIdx >= 0)
					{
						if (!field->save(dbfHandle, xFieldIdx, yFieldIdx, zFieldIdx))
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
						if (!field->save(dbfHandle, fieldIdx))
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


CC_FILE_ERROR ShpFilter::loadFile(const QString &filename, ccHObject &container, LoadParameters &parameters)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	if (file.size() < ESRI_HEADER_SIZE)
	{
		ccLog::Warning("[SHP] File is too small to be valid");
		return CC_FERR_MALFORMED_FILE;
	}

	QDataStream shpStream(&file);

	ShapeFileHeader hdr;
	CC_FILE_ERROR error = hdr.readFrom(shpStream);
	if (error != CC_FERR_NO_ERROR)
		return error;

	//global shift
	CCVector3d Pshift(0, 0, 0);
	bool preserveCoordinateShift = true;
	CCVector3d Pmin = hdr.pointMin;
	if (HandleGlobalShift(Pmin, Pshift, preserveCoordinateShift, parameters))
	{
		ccLog::Warning("[SHP] Entities will be recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
	}

	//progress bar
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
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
	error = CC_FERR_NO_ERROR;
	ccPointCloud* singlePoints = nullptr;
	//we also keep track of the polylines 'record number' (if any)
	QMap<ccPolyline*, int32_t> polyIDs;
	int32_t maxPolyID = 0;
	int32_t maxPointID = 0;
	bool is3DShape = false;
	while (hdr.fileLength - shpStream.device()->pos() > 0)
	{
		if (shpStream.status() != QDataStream::Ok )
		{
			ccLog::Warning("[SHP] Something went wrong reading the file");
			return CC_FERR_READING;
		}
		int32_t recordNumber;
		int32_t recordSize;
		int32_t shapeTypeInt;
		shpStream.setByteOrder(QDataStream::BigEndian);
		shpStream >> recordNumber >> recordSize;
		recordSize *= 2; //recordSize is measured in 16-bit words
		shpStream.setByteOrder(QDataStream::LittleEndian);
		shpStream >> shapeTypeInt;

		if (!isValidESRIShapeCode(shapeTypeInt))
		{
			ccLog::Warning("[SHP] Shape %d has an invalid shape code (%d)", recordNumber, shapeTypeInt);
			return CC_FERR_READING;
		}
		auto shapeType = static_cast<ESRI_SHAPE_TYPE >(shapeTypeInt);

		if (recordNumber < 64)
			ccLog::Print(QString("[SHP] Record #%1 - type: %2 (%3 bytes)").arg(recordNumber).arg(ToString(shapeType)).arg(recordSize));
		else if (recordNumber == 64)
			ccLog::Print("[SHP] Records won't be displayed in the Console anymore to avoid flooding it...");

		switch (shapeType)
		{
			case ESRI_SHAPE_TYPE::POLYLINE_Z:
			case ESRI_SHAPE_TYPE::POLYGON_Z:
				is3DShape = true;
			case ESRI_SHAPE_TYPE::POLYLINE:
			case ESRI_SHAPE_TYPE::POLYGON:
			case ESRI_SHAPE_TYPE::POLYLINE_M:
			case ESRI_SHAPE_TYPE::POLYGON_M:
			{
				unsigned childCountBefore = container.getChildrenNumber();
				error = LoadPolyline(shpStream, container, recordNumber, shapeType, Pshift, preserveCoordinateShift);
				if (error == CC_FERR_NO_ERROR && shapeType == ESRI_SHAPE_TYPE::POLYLINE)
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
			case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
			case ESRI_SHAPE_TYPE::MULTI_POINT_M:
				is3DShape = true;
			case ESRI_SHAPE_TYPE::MULTI_POINT:
				error = LoadCloud(shpStream, container, recordNumber, shapeType, Pshift, preserveCoordinateShift);
				break;
			case ESRI_SHAPE_TYPE::POINT_Z:
			case ESRI_SHAPE_TYPE::POINT_M:
				is3DShape = true;
			case ESRI_SHAPE_TYPE::POINT:
				error = LoadSinglePoint(shpStream, singlePoints, shapeType, Pshift, preserveCoordinateShift);
				if (error == CC_FERR_NO_ERROR && recordNumber > maxPointID)
				{
					maxPointID = recordNumber;
				}
				break;
			case ESRI_SHAPE_TYPE::NULL_SHAPE:
				//ignored
				break;
			default:
				//unhandled entity
				shpStream.skipRawData(recordSize - sizeof(shapeTypeInt));
				ccLog::Warning("[SHP] Unhandled type!");
				break;
		}

		if (error != CC_FERR_NO_ERROR)
		{
			break;
		}

		if (pDlg)
		{
			pDlg->setValue(static_cast<int>(shpStream.device()->pos()));
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
					DBFFieldType fieldType = DBFGetFieldInfo(dbfHandle, i, fieldName, nullptr, nullptr);
					if (fieldType == FTDouble || fieldType == FTInteger)
					{
						candidateFields.push_back(FieldIndexAndName(i, QString(fieldName)));
					}
				}

				if (!candidateFields.empty())
				{
					//create a list of available fields
					ImportDBFFieldDialog lsfDlg(nullptr);
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
							DBFFieldType fieldType = DBFGetFieldInfo(dbfHandle, index, nullptr, nullptr, nullptr);

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
			singlePoints = nullptr;
		}
		else
		{
			CCLib::ScalarField* sf = singlePoints->getScalarField(0);
			if (sf)
			{
				sf->computeMinAndMax();
				singlePoints->showSF(true);
			}
			singlePoints->shrinkToFit();
			container.addChild(singlePoints);
		}
	}

	return error;
}

#endif //CC_SHP_SUPPORT
