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
#include "ShpDBFFields.h"
#include "ui_importDBFFieldDlg.h"
#include "ui_saveSHPFileDlg.h"

//qCC_db
#include <ccGenericMesh.h>
#include <ccGenericPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

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
static bool IsValidESRIShapeCode(int32_t code)
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
static bool IsESRIShape3D(ESRI_SHAPE_TYPE shapeType)
{
	switch (shapeType)
	{
		case ESRI_SHAPE_TYPE::POINT_Z:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
		case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
		case ESRI_SHAPE_TYPE::MULTI_PATCH:
			return true;
		default:
			return false;
	}
}


//! Returns whether the shape type contains the additional measures dimension
static bool HasMeasurements(ESRI_SHAPE_TYPE shapeType)
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
		case ESRI_SHAPE_TYPE::MULTI_PATCH:
			return true;
		default:
			return false;
	}
}

enum class ESRI_PART_TYPE : int32_t
{
	TRIANGLE_STRIP = 0,
	TRIANGLE_FAN = 1,
	OUTER_RING = 2,
	INNER_RING = 3,
	FIRST_RING = 4,
	RING = 5,
};

static bool IsValidEsriPartType(int32_t code)
{
	if (code < static_cast<int32_t >(ESRI_PART_TYPE::TRIANGLE_STRIP))
		return false;
	if (code > static_cast<int32_t >(ESRI_PART_TYPE::RING))
		return false;

	switch (static_cast<ESRI_PART_TYPE >(code))
	{
		case ESRI_PART_TYPE::TRIANGLE_STRIP:
		case ESRI_PART_TYPE::TRIANGLE_FAN:
		case ESRI_PART_TYPE::INNER_RING:
		case ESRI_PART_TYPE::OUTER_RING:
		case ESRI_PART_TYPE::FIRST_RING:
		case ESRI_PART_TYPE::RING:
			return true;
		default:
			return false;
	}
}

static inline bool IsESRINoData(double m)
{
	return m <= ESRI_NO_DATA;
}

static int32_t SizeofMultiPointZ(unsigned numPoints) noexcept
{
	size_t recordSize = 0;
	recordSize += 4; // shapeType
	recordSize += 4 * 8; // x,y bbox
	recordSize += 4; // numPoints
	recordSize += static_cast<size_t>(numPoints) * 2 * 8; // xs, ys
	recordSize += 2 * 8; // zRange
	recordSize += static_cast<size_t>(numPoints) * 8; // zs
	recordSize += 2 * 8; // mRange
	recordSize += static_cast<size_t>(numPoints) * 8; // Measures
	recordSize /= 2; // 16bit words

	return static_cast<int32_t>(recordSize);
}

static int32_t SizeofPolyLine(ESRI_SHAPE_TYPE polylineType, int32_t numPoints, int32_t numParts = 1)
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

	size_t recordSize = 0;
	recordSize += 4; // ShapeType
	recordSize += 4 * 8; // 2D Bounding Box
	recordSize += 4; // numParts
	recordSize += 4; // numPoints
	recordSize += static_cast<size_t>(numParts) * 4; // Parts
	recordSize += static_cast<size_t>(numPoints) * 16; //Points

	if (IsESRIShape3D(polylineType))
	{
		recordSize += 2 * 8;
		recordSize += static_cast<size_t>(numPoints) * 8;
	}

	if (HasMeasurements(polylineType))
	{
		recordSize += 2 * 8;
		recordSize += static_cast<size_t>(numPoints) * 8;
	}
	
	recordSize /= 2; // 16bit words

	return static_cast<int32_t>(recordSize);
}

static int32_t SizeofMultipatch(unsigned numPoints, int32_t numParts = 1)
{
	size_t recordSize = 0;
	recordSize += 4; // ShapeType
	recordSize += 4 * 8; // MBR
	recordSize += 4; // nbParts
	recordSize += 4; // nbPoints
	recordSize += static_cast<size_t>(numParts) * 4; // Parts
	recordSize += static_cast<size_t>(numParts) * 4; // Parts Type
	recordSize += static_cast<size_t>(numPoints) * 2 * 8; //Points
	recordSize += 2 * 8; // zRange
	recordSize += static_cast<size_t>(numPoints) * 8; // Zs
	recordSize += 2 * 8; // mRange
	recordSize += static_cast<size_t>(numPoints) * 8; // Measures
	recordSize /= 2; // 16bit words

	return static_cast<int32_t>(recordSize);
}

static bool AreVerticesCounterClockwise(const CCLib::GenericIndexedCloudPersist *vertices,
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
		sum += (P2.u[dim1] - P1.u[dim1]) * (P2.u[dim2] + P1.u[dim2]);
	}

	//negative sum = counter-clockwise
	return sum < 0.0;
}

static CCVector2d MinMaxOfEnabledScalarField(const CCLib::GenericIndexedCloudPersist *cloud)
{
	CCVector2d minMax(ESRI_NO_DATA, ESRI_NO_DATA);
	if (cloud->isScalarFieldEnabled() && cloud->size() != 0)
	{
		minMax.x = minMax.y = cloud->getPointScalarValue(0);
		for (unsigned i = 1; i < cloud->size(); ++i)
		{
			double scalar = cloud->getPointScalarValue(i);
			minMax.x = std::min(minMax.x, scalar);
			minMax.y = std::max(minMax.y, scalar);
		}
	}

	return minMax;
}

static ccBBox BBoxOfHObjectContainer(const ccHObject::Container& objects)
{
	ccBBox globalBB;
	for (ccHObject *obj : objects)
	{
		ccBBox box = obj->getBB_recursive();
		if (box.isValid())
		{
			globalBB += box;
		}
	}
	return globalBB;
}

//! Computes the range (min & max) of the enabled scalar field
//! for each object in the container
/**
 * @param objects containter of objects to compute the range on
 * @return 2D vector, x is the min, y in the max
 */
static CCVector2d MRangeOfContainer(ccHObject::Container &objects)
{
	CCVector2d range(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());

	auto updateRange = [&range](const CCLib::GenericIndexedCloudPersist *cloud)
	{
		if (!cloud || !cloud->isScalarFieldEnabled())
			return;
		unsigned numPoints = cloud->size();
		for (unsigned i = 0; i < numPoints; ++i)
		{
			ScalarType val = cloud->getPointScalarValue(i);
			if (ccScalarField::ValidValue(val))
			{
				range.x = std::min(range.x, static_cast<double>(val));
				range.y = std::max(range.y, static_cast<double>(val));
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
		range.x = 0.0;
		range.y = 0.0;
	}
	return range;
}


static void UpdateFileLength(QDataStream& out, int32_t newFileLentgh)
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

	sin.skipRawData(5 * 4);

	sin >> fileLength;
	fileLength *= 2;  //fileLength is measured in 16-bit words

	sin.setByteOrder(QDataStream::LittleEndian);

	sin >> version >> shapeTypeInt;

	if (!IsValidESRIShapeCode(shapeTypeInt))
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

	constexpr size_t numBytesToSkip = 5 * 4;
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

ShpFilter::ShpFilter()
	: FileIOFilter( {
					"_Shape Filter",
					14.0f,	// priority
					QStringList{ "shp" },
					"shp",
					QStringList{ "SHP entity (*.shp)" },
					QStringList{ "SHP entity (*.shp)" },
					Import | Export | BuiltIn
					} )
{
}

bool ShpFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POLY_LINE ||
	    type == CC_TYPES::POINT_CLOUD ||
	    type == CC_TYPES::HIERARCHY_OBJECT ||
	    type == CC_TYPES::MESH)
	{
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

static QString ToString(ESRI_PART_TYPE type)
{
	switch (type)
	{
		case ESRI_PART_TYPE::TRIANGLE_STRIP:
			return "Triangle Strip";
		case ESRI_PART_TYPE::TRIANGLE_FAN:
			return "Triangle Fan";
		case ESRI_PART_TYPE::INNER_RING:
			return "Inner Ring";
		case ESRI_PART_TYPE::OUTER_RING:
			return "Outer Ring";
		case ESRI_PART_TYPE::FIRST_RING:
			return "First Ring";
		case ESRI_PART_TYPE::RING:
			return "Ring";
		default:
			return "Unknown";
	}
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

static void GetSupportedShapes(ccHObject* baseEntity, ccHObject::Container& shapes, ESRI_SHAPE_TYPE& shapeType)
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
			shapeType = ESRI_SHAPE_TYPE::POLYLINE_Z;
			shapes.push_back(baseEntity);
			break;
		}
		case CC_TYPES::MESH:
		{
			shapeType = ESRI_SHAPE_TYPE::MULTI_PATCH;
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

static CC_FILE_ERROR ReadParts(QDataStream& shpStream, int32_t numParts, std::vector<int32_t>& startIndexes)
{
	try
	{
		startIndexes.resize(numParts, 0);
	}
	catch (const std::bad_alloc&)
	{
		shpStream.skipRawData(4 * numParts);
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	for (int32_t i = 0; i != numParts; ++i)
	{
		shpStream >> startIndexes[i];
	}

	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR ReadPoints(QDataStream& shpStream, int32_t numPoints, const CCVector3d& Pshift, std::vector<CCVector3>& points)
{
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

	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR ReadMeasures(QDataStream& shpStream, int32_t numPoints, std::vector<ScalarType>& scalarValues)
{
	//M boundaries
	double mMin;
	double mMax;
	shpStream >> mMin >> mMax;

	if (IsESRINoData(mMin) && IsESRINoData(mMax))
	{
		shpStream.skipRawData(8 * numPoints);
		return CC_FERR_NO_ERROR;
	}

	try
	{
		scalarValues.resize(numPoints, NAN_VALUE);
	}
	catch (const std::bad_alloc&)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	//M values (an array of length NumPoints)
	for (int32_t i = 0; i < numPoints; ++i)
	{
		double m;
		shpStream >> m;
		if (!IsESRINoData(m) && std::isfinite(m))
		{
			scalarValues[i] = static_cast<ScalarType>(m);
		}
	}

	return CC_FERR_NO_ERROR;
}

//! Builds the ccPointCloud of vertices
/** \return nullptr if out of memory
**/
static ccPointCloud* BuildVertices(const std::vector<CCVector3> &points, int32_t firstIndex, int32_t lastIndex)
{
	int32_t vertCount = lastIndex - firstIndex + 1;
	if (vertCount <= 0)
	{
		assert(false);
		return nullptr;
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertCount))
	{
		delete vertices;
		return nullptr;
	}
	
	for (int32_t j = 0; j < vertCount; ++j)
	{
		vertices->addPoint(points[firstIndex + j]);
	}
	
	vertices->setEnabled(false);
	
	return vertices;
}

//! Creates the ccMesh
static ccMesh* CreateMesh(
		const std::vector<CCVector3> &points,
		const std::vector<ScalarType> &scalarValues,
		int32_t firstIndex,
		int32_t lastIndex)
{
	int32_t vertCount = lastIndex - firstIndex + 1;
	if (vertCount < 3)
	{
		return nullptr;
	}

	ccPointCloud *vertices = BuildVertices(points, firstIndex, lastIndex);
	if (!vertices)
	{
		return nullptr;
	}

	if (!scalarValues.empty())
	{
		bool areAllValuesOfPartsNaNs = true;
		if (firstIndex < scalarValues.size() && lastIndex < scalarValues.size())
		{
			areAllValuesOfPartsNaNs = std::all_of(scalarValues.begin() + firstIndex,
			                                      scalarValues.begin() + lastIndex,
			                                      [](ScalarType val)
			                                      { return std::isnan(val); });
		}
		if (!areAllValuesOfPartsNaNs)
		{
			ccScalarField* sf = new ccScalarField("Measures");
			if (!sf->reserveSafe(vertCount))
			{
				ccLog::Warning(QString("[SHP] Mesh: not enough memory to load scalar values!"));
				sf->release();
			}
			else
			{
				for (int32_t j = 0; j < vertCount; ++j)
				{
					sf->addElement(scalarValues[j + firstIndex]);
				}
				sf->computeMinAndMax();
				int sfIdx = vertices->addScalarField(sf);
				vertices->setCurrentDisplayedScalarField(sfIdx);
				vertices->showSF(true);
			}
		}
	}

	return new ccMesh(vertices);
}

//! Builds the patches that where read from a MultiPatch record of a Shapefile
/** Patches are buit as meshes, each part is its own mesh.
	The only supported Patches/Part types are Triangle_Fan & Triangle_Strip
**/
static CC_FILE_ERROR BuildPatches(
		ccHObject &container,
		const std::vector<int32_t> &startIndexes,
		const std::vector<int32_t> &partTypes,
		const std::vector<CCVector3> &points,
		const std::vector<ScalarType> &scalarValues)
{
	size_t numParts = startIndexes.size();
	size_t numPoints = points.size();

	for (int32_t i = 0; i < numParts; ++i)
	{
		if (!IsValidEsriPartType(partTypes[i]))
		{
			ccLog::Warning("[SHP] Multipatch part %d has an invalid part type (%d)", i, partTypes[i]);
			continue;
		}
		ESRI_PART_TYPE type = static_cast<ESRI_PART_TYPE>(partTypes[i]);

		const int32_t &firstIndex = startIndexes[i];
		const int32_t &lastIndex = static_cast<const int32_t &>((i + 1 < numParts ? startIndexes[i + 1] : numPoints) -1);
		const int32_t vertCount = lastIndex - firstIndex + 1;

		switch (type)
		{
			case ESRI_PART_TYPE::TRIANGLE_STRIP:
			{
				ccMesh *mesh = CreateMesh(points, scalarValues, firstIndex, lastIndex);
				for (int32_t j = 2; j < vertCount; ++j)
				{
					mesh->addTriangle(j - 2, j - 1, j);
				}
				container.addChild(mesh);
				break;
			}
			case ESRI_PART_TYPE::TRIANGLE_FAN:
			{
				ccMesh *mesh = CreateMesh(points, scalarValues, firstIndex, lastIndex);
				for (int32_t j = 2; j < vertCount; ++j)
				{
					mesh->addTriangle(0, j - 1, j);
				}
				container.addChild(mesh);
				break;
			}
			default:
			{
				ccLog::Print(QString("[SHP] Cannot handle Patch of type: %1").arg(ToString(type)));
				return CC_FERR_BAD_ENTITY_TYPE;
			}
		}
	}
	return CC_FERR_NO_ERROR;
}


static CC_FILE_ERROR LoadMultiPatch(QDataStream &shpStream,
                                    ccHObject &container,
                                    CCVector3d Pshift)
{
	// skip record bbox
	shpStream.skipRawData(4 * 8);

	int32_t numParts;
	int32_t numPoints;
	shpStream >> numParts >> numPoints;

	std::vector<int32_t> startIndexes;
	CC_FILE_ERROR error = ReadParts(shpStream, numParts, startIndexes);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	std::vector<int32_t> partTypes;
	error = ReadParts(shpStream, numParts, partTypes);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	std::vector<CCVector3> points;
	error = ReadPoints(shpStream, numPoints, Pshift, points);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	//Z boundaries
	shpStream.skipRawData(2 * 8);

	//Z coordinates (an array of length NumPoints)
	for (int32_t i = 0; i < numPoints; ++i)
	{
		double z;
		shpStream >> z;
		points[i].z = static_cast<PointCoordinateType>(z + Pshift.z);
	}

	std::vector<ScalarType> scalarValues;
	error = ReadMeasures(shpStream, numPoints, scalarValues);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	return BuildPatches(container, startIndexes, partTypes, points, scalarValues);
}

//! Saves the cloud to the shape file according to the specification
/**
 *
 * @param stream Output stream of the shapefile
 * @param cloud The cloud to save (pointcloud or vertices)
 * @param bbMing Min point of the cloud
 * @param bbMaxg Mxx point of the cloud
 */
static void Save3DCloud(QDataStream &stream, const ccGenericPointCloud *cloud, const CCVector3d &bbMing, const CCVector3d &bbMaxg)
{
	const unsigned numPoints = cloud->size();
	CCVector3 P;

	// Points (x ,y)
	for (unsigned i = 0; i < numPoints; ++i)
	{
		cloud->getPoint(i, P);
		CCVector3d Pg = cloud->toGlobal3d(P);
		stream << Pg.x << Pg.y;
	}

	// Z Coordinates
	stream << bbMing.z << bbMaxg.z;
	for (unsigned i = 0; i < numPoints; ++i)
	{
		cloud->getPoint(i, P);
		CCVector3d Pg = cloud->toGlobal3d(P);
		stream << Pg.z;
	}

	// Measures
	bool hasSF = cloud->isScalarFieldEnabled();
	double mMin = ESRI_NO_DATA;
	double mMax = ESRI_NO_DATA;
	if (hasSF)
	{
		mMin = std::numeric_limits<double>::max();
		mMax = std::numeric_limits<double>::min();
		for (unsigned i = 0; i < numPoints; ++i)
		{
			auto scalar = static_cast<double >(cloud->getPointScalarValue(i));
			mMin = std::min(mMin, scalar);
			mMax = std::max(mMax, scalar);
		}
	}
	stream << mMin << mMax;

	auto scalar = ESRI_NO_DATA;
	for (unsigned i = 0; i < numPoints; ++i)
	{
		if (hasSF)
		{
			scalar = static_cast<double>(cloud->getPointScalarValue(i));
		}
		stream << scalar;
	}
}

static inline bool IsTriangleStrip(const CCLib::VerticesIndexes *idx)
{
	return		idx != nullptr
			&&	(idx->i3 - 1) == idx->i2
			&&	(idx->i3 - 2) == idx->i1;
}

static inline bool IsTriangleFan(const CCLib::VerticesIndexes *idx)
{
	return		idx != nullptr
			&&	idx->i1 == 0
			&&	idx->i2 == (idx->i3 - 1);
}

/**
 * Tells wether the mesh's vertices are organised in triangle fan or triangle strip fashion.
 * @param mesh
 * @param[out] type
 * @return CC_FERR_BAD_ENTITY_TYPE if the mesh is neither a triangle fan or triangle strip
 */
static CC_FILE_ERROR FindTriangleOrganisation(ccMesh *mesh, ESRI_PART_TYPE &type)
{
	const CCLib::VerticesIndexes *firstVert = mesh->getNextTriangleVertIndexes();
	if (!IsTriangleFan(firstVert) && !IsTriangleStrip(firstVert))
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	const CCLib::VerticesIndexes *secondVert = mesh->getNextTriangleVertIndexes();
	if (IsTriangleStrip(secondVert))
	{
		for (unsigned i = 2; i < mesh->size(); ++i)
		{
			CCLib::VerticesIndexes *idx = mesh->getNextTriangleVertIndexes();
			if (!IsTriangleStrip(idx))
			{
				return CC_FERR_BAD_ENTITY_TYPE;
			}
		}
		type = ESRI_PART_TYPE::TRIANGLE_STRIP;
		return CC_FERR_NO_ERROR;

	}
	else if (IsTriangleFan(secondVert))
	{
		for (unsigned i = 2; i < mesh->size(); ++i)
		{
			CCLib::VerticesIndexes *idx = mesh->getNextTriangleVertIndexes();
			if (!IsTriangleFan(idx))
			{
				return CC_FERR_BAD_ENTITY_TYPE;
			}
		}
		type = ESRI_PART_TYPE::TRIANGLE_FAN;
		return CC_FERR_NO_ERROR;
	}
	else
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}
}

static CC_FILE_ERROR SaveMesh(ccMesh *mesh, QDataStream &stream, int32_t recordNumber, int32_t &recordSize)
{
	ESRI_PART_TYPE triangleType;
	if (FindTriangleOrganisation(mesh, triangleType) == CC_FERR_BAD_ENTITY_TYPE)
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	ccLog::Print(QString("[SHP] Triangle type: %1").arg(ToString(triangleType)));

	ccGenericPointCloud *vertices = mesh->getAssociatedCloud();
	int32_t numParts = 1;
	unsigned numPoints = vertices->size();
	recordSize = SizeofMultipatch(numPoints, numParts);

	// Record Header
	stream.setByteOrder(QDataStream::BigEndian);
	stream << recordNumber << recordSize;

	qint64 recordStart = stream.device()->pos();
	stream.setByteOrder(QDataStream::LittleEndian);
	stream << static_cast<int32_t >(ESRI_SHAPE_TYPE::MULTI_PATCH);

	CCVector3d bbMing;
	CCVector3d bbMaxg;
	mesh->getGlobalBB(bbMing, bbMaxg);

	stream << bbMing.x << bbMing.y << bbMaxg.x << bbMaxg.y;
	stream << numParts << numPoints;
	stream << static_cast<int32_t>(0); // Parts
	stream << static_cast<int32_t>(triangleType); // Parts Type

	Save3DCloud(stream, vertices, bbMing, bbMaxg);

	qint64 recordEnd = stream.device()->pos();
	qint64 bytesWritten = recordEnd - recordStart;
	assert(bytesWritten == 2 * recordSize);
	
	return CC_FERR_NO_ERROR;
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
	shpStream.skipRawData(4 * 8);

	int32_t numParts;
	int32_t numPoints;
	shpStream >> numParts >> numPoints;

	std::vector<int32_t> startIndexes;
	CC_FILE_ERROR error = ReadParts(shpStream, numParts, startIndexes);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}
	//for each part, the index of its first point in the points array
	//FIXME: we should use this information and create as many polylines as necessary!

	//Points (An array of length NumPoints)
	std::vector<CCVector3> points;
	error = ReadPoints(shpStream, numPoints, Pshift, points);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	//3D polylines
	bool is3D = IsESRIShape3D(shapeType);
	if (is3D)
	{
		//Z boundaries
		shpStream.skipRawData(2 * 8);

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
	if (HasMeasurements(shapeType))
	{
		error = ReadMeasures(shpStream, numPoints, scalarValues);
		if (error != CC_FERR_NO_ERROR)
		{
			return error;
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
		{
			name += QString(".%1").arg(i + 1);
		}
		poly->setName(name);
		poly->setClosed(isClosed);
		poly->set2DMode(!is3D && !load2DPolyAs3DPoly);

		if (!scalarValues.empty())
		{
			bool allNaNs = std::all_of(
					scalarValues.begin() + firstIndex,
					scalarValues.begin() + lastIndex,
					[](ScalarType v) {return std::isnan(v);}
			);
			if (!allNaNs)
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
			}
		}
		container.addChild(poly);
	}
	return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR SavePolyline(ccPolyline* poly,
                                  QDataStream& out,
                                  int32_t& recordSize,
                                  int32_t recordNumber,
                                  ESRI_SHAPE_TYPE outputShapeType,
                                  unsigned char vertDim = 2)
{
	assert(vertDim >= 0 && vertDim < 3);

	if (!poly)
	{
		assert(false);
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	const unsigned char Z = static_cast<unsigned char>(vertDim);
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	CCLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
	if (!vertices)
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	unsigned realVertexCount = poly->size();
	switch (outputShapeType)
	{
		case ESRI_SHAPE_TYPE::POLYGON:
		case ESRI_SHAPE_TYPE::POLYGON_M:
		case ESRI_SHAPE_TYPE::POLYGON_Z:
			if (realVertexCount < 3)
			{
				ccLog::Warning(QObject::tr("[SHP] Polyline %1 does not have enough vertices to be saved as polygon entity").arg(poly->getName()));
				return CC_FERR_BAD_ENTITY_TYPE;
			}
			break;
		case ESRI_SHAPE_TYPE::POLYLINE:
		case ESRI_SHAPE_TYPE::POLYLINE_M:
		case ESRI_SHAPE_TYPE::POLYLINE_Z:
			if (realVertexCount < 2)
			{
				ccLog::Warning(QObject::tr("[SHP] Polyline %1 does not have enough vertices to be saved as polyline entity").arg(poly->getName()));
				return CC_FERR_BAD_ENTITY_TYPE;
			}
			break;
		default:
			assert(false);
			return CC_FERR_BAD_ENTITY_TYPE;
	}

	bool isClosed = poly->isClosed();

	if (static_cast<int64_t>(realVertexCount) + 1 > std::numeric_limits<int32_t>::max())
	{
		ccLog::Warning(QObject::tr("[SHP] Polyline %1 has too many points to be saved").arg(poly->getName()));
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	int32_t iRealVertexCount = static_cast<int32_t>(realVertexCount);
	int32_t numPoints = iRealVertexCount + (isClosed ? 1 : 0);
	const int32_t numParts = 1;

	recordSize = SizeofPolyLine(outputShapeType, numPoints, numParts);

	//write shape record in main SHP file
	{
		out.setByteOrder(QDataStream::BigEndian);
		//Byte 0: Record Number
		assert(recordNumber > 0); //Record numbers begin at 1
		out << recordNumber;
		//Byte 4: Content Length
		out << recordSize;
	}

	qint64 recordStart = out.device()->pos();
	out.setByteOrder(QDataStream::LittleEndian);

	//Byte 0: Shape Type
	out << static_cast<int32_t>(outputShapeType);

	//Byte 4: Box
	CCVector3d bbMing;
	CCVector3d bbMaxg;
	poly->getGlobalBB(bbMing, bbMaxg);
	//The Bounding Box for the PolyLine stored in the order Xmin, Ymin, Xmax, Ymax (24 bytes)
	out << bbMing.u[X] << bbMing.u[Y] << bbMaxg.u[X] << bbMaxg.u[Y];

	//Byte 36: NumParts (The number of parts in the PolyLine)
	out << numParts;
	//Byte 40: NumPoints (The total number of points for all parts)
	out << numPoints;

	//Byte 44: Parts (An array of length NumParts)
	//(for each part, the index of its first point in the points array)
	assert(numParts == 1);
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

		inverseOrder = AreVerticesCounterClockwise(vertices, numPoints, dim1, dim2);
	}

	//Points (An array of length NumPoints)
	for (int32_t i = 0; i < numPoints; ++i)
	{
		int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
		const CCVector3* P = vertices->getPoint(ii % iRealVertexCount); //warning: handle loop if polyline is closed
		CCVector3d Pg = poly->toGlobal3d(*P);
		//2D point (16 bytes)
		out << Pg.u[X] << Pg.u[Y];
	}

	//3D polylines
	if (IsESRIShape3D(outputShapeType))
	{
		//Z boundaries (16 bytes)
		out << bbMing.u[Z] << bbMaxg.u[Z];

		//Z coordinates (for each part - just one here)
		for (int32_t i = 0; i < numPoints; ++i)
		{
			int32_t ii = (inverseOrder ? numPoints - 1 - i : i);
			const CCVector3 *P = vertices->getPoint(ii % iRealVertexCount); //warning: handle loop if polyline is closed
			CCVector3d Pg = poly->toGlobal3d(*P);
			out << Pg.u[Z];
		}
	}

	if (HasMeasurements(outputShapeType))
	{
		//M boundaries (16 bytes)
		bool hasSF = vertices->isScalarFieldEnabled();
		CCVector2d minMax = MinMaxOfEnabledScalarField(vertices);
		out << minMax.x << minMax.y;

		//M values (for each part - just one here)
		if (hasSF)
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				ScalarType scalar = vertices->getPointScalarValue(i % iRealVertexCount);
				out << (ccScalarField::ValidValue(scalar) ? ESRI_NO_DATA : static_cast<double>(scalar));
			}
		}
		else
		{
			for (int32_t i = 0; i < numPoints; ++i)
			{
				out << ESRI_NO_DATA;
			}
		}
	}

	assert(out.device()->pos() == recordStart + recordSize * 2);
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
	shpStream.skipRawData(4 * 8);

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
	if (IsESRIShape3D(shapeType))
	{
		//Z boundaries
		shpStream.skipRawData(2 * 8);

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
	if (HasMeasurements(shapeType))
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
				ScalarType s = IsESRINoData(m) ? NAN_VALUE : static_cast<ScalarType>(m);
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
			shpStream.skipRawData(numPoints * 8);
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

	if (cloud->size() > static_cast<unsigned >(std::numeric_limits<int32_t>::max()))
	{
		ccLog::Print("[SHP] Cloud is to big to be saved");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	recordSize = SizeofMultiPointZ(cloud->size());
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

	Save3DCloud(out, cloud, bbMing, bbMaxg);
	assert(out.device()->pos() - recordStart == recordSize * 2);
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

	if (IsESRIShape3D(shapeType))
	{
		double z;
		shpStream >> z;
		P.z = static_cast<PointCoordinateType>(z + Pshift.z);
	}

	ScalarType s = NAN_VALUE;
	if (HasMeasurements(shapeType))
	{
		double m;
		shpStream >> m;
		if (!IsESRINoData(m))
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

	ccBBox bb = BBoxOfHObjectContainer(toSave);
	if (!bb.isValid())
	{
		ccLog::Error("Entity(ies) has(ve) an invalid bounding box?!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	bool save3DPolysAs2D = false;
	static unsigned char s_poly2DVertDim = 2;
	bool save3DPolyHeightInDBF = false;
	if (parameters.alwaysDisplaySaveDialog && inputShapeType == ESRI_SHAPE_TYPE::POLYLINE_Z)
	{
		//display SHP save dialog
		SaveSHPFileDialog ssfDlg(nullptr);
		ssfDlg.save3DPolyAs2DCheckBox->setChecked(m_save3DPolyAs2D);
		ssfDlg.save3DPolyHeightInDBFCheckBox->setChecked(m_save3DPolyHeightInDBF);
		ssfDlg.dimComboBox->setCurrentIndex(s_poly2DVertDim);

		if (!ssfDlg.exec())
			return CC_FERR_CANCELED_BY_USER;

		save3DPolysAs2D = ssfDlg.save3DPolyAs2DCheckBox->isChecked();
		int iPoly2DVertDim = ssfDlg.dimComboBox->currentIndex();
		if (iPoly2DVertDim < 0 || iPoly2DVertDim > 2)
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		s_poly2DVertDim = static_cast<unsigned char>(iPoly2DVertDim);

		save3DPolyHeightInDBF  = ssfDlg.save3DPolyHeightInDBFCheckBox->isChecked();
	}
	const unsigned char Z = s_poly2DVertDim;
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	ESRI_SHAPE_TYPE outputShapeType = inputShapeType;

	// Promote to polygon
	if (m_closedPolylinesAsPolygons && outputShapeType == ESRI_SHAPE_TYPE::POLYLINE_Z)
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

	CCVector2d mRange(0.0, 0.0);
	if (HasMeasurements(outputShapeType))
	{
		mRange = MRangeOfContainer(toSave);
	}

	if (!IsESRIShape3D(outputShapeType))
	{
		bb.minCorner().u[Z] = 0;
		bb.maxCorner().u[Z] = 0;
	}

	ShapeFileHeader hdr;
	hdr.pointMin = CCVector3d(bb.minCorner().u[X], bb.minCorner().u[Y], bb.minCorner().u[Z]);
	hdr.pointMax = CCVector3d(bb.maxCorner().u[X], bb.maxCorner().u[Y], bb.maxCorner().u[Z]);
	hdr.shapeTypeInt = static_cast<int32_t>(outputShapeType);
	hdr.mRange = mRange;

	hdr.writeTo(shpStream);
	hdr.writeTo(idxStream);

	//save shapes
	unsigned shapeIndex = 1;
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
				error = SavePolyline(static_cast<ccPolyline *>(child), shpStream, recordSize, shapeIndex,
				                     outputShapeType, s_poly2DVertDim);
				break;
			case ESRI_SHAPE_TYPE::MULTI_POINT_Z:
				assert(child->isKindOf(CC_TYPES::POINT_CLOUD));
				error = SaveAsCloud(ccHObjectCaster::ToGenericPointCloud(child), shpStream, shapeIndex, recordSize);
				break;
			case ESRI_SHAPE_TYPE::MULTI_PATCH:
				error = SaveMesh(ccHObjectCaster::ToMesh(child), shpStream, shapeIndex, recordSize);
				break;
			default:
				assert(false);
				break;
		}

		if (error != CC_FERR_NO_ERROR)
			return error;

		//write corresponding entry in index SHX file
		idxStream.setByteOrder(QDataStream::BigEndian);
		idxStream << static_cast<int32_t>(recordStart / 2); //recordStart must be converted to a number of 16-bit words
		idxStream << recordSize; //recordSize should already be expressed as a number of 16-bit words

		ccLog::PrintDebug("[SHP] Saved shape #%d (%d bytes)", shapeIndex, recordSize * 2);
		shapeIndex++;
	}

	//update file lengths
	UpdateFileLength(shpStream, static_cast<int32_t>(shpStream.device()->pos() / 2));
	UpdateFileLength(idxStream, static_cast<int32_t>(idxStream.device()->pos() / 2));

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
		shpStream.setByteOrder(QDataStream::BigEndian);
		int32_t recordNumber;
		shpStream >> recordNumber;
		int32_t recordSize;
		shpStream >> recordSize;
		
		shpStream.setByteOrder(QDataStream::LittleEndian);
		int64_t recordStart = shpStream.device()->pos();

		int32_t shapeTypeInt;
		shpStream >> shapeTypeInt;

		if (!IsValidESRIShapeCode(shapeTypeInt))
		{
			ccLog::Warning("[SHP] Shape %d has an invalid shape code (%d)", recordNumber, shapeTypeInt);
			return CC_FERR_READING;
		}
		auto shapeType = static_cast<ESRI_SHAPE_TYPE >(shapeTypeInt);

		if (recordNumber < 64)
			ccLog::Print(QString("[SHP] Record #%1 - type: %2 (%3 bytes)").arg(recordNumber).arg(ToString(shapeType)).arg(recordSize * 2)); //recordSize is measured in 16-bit words
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
			case ESRI_SHAPE_TYPE::MULTI_PATCH:
				error = LoadMultiPatch(shpStream, container, Pshift);
			case ESRI_SHAPE_TYPE::NULL_SHAPE:
				//ignored
				break;
			default:
				//unhandled entity
				shpStream.skipRawData(recordSize * 2 - sizeof(shapeTypeInt)); //recordSize is measured in 16-bit words
				ccLog::Warning("[SHP] Unhandled type!");
				break;
		}

		if (error != CC_FERR_NO_ERROR)
		{
			break;
		}

		qint64 filePos = shpStream.device()->pos();
		ccLog::PrintDebug(
			QString("[SHP] File position = %1 / record start = %2 / record size = %3 (x2) / position shift = %4")
			.arg(filePos)
			.arg(recordStart)
			.arg(recordSize)
			.arg(filePos - (recordStart + recordSize * 2)));

		assert(shpStream.device()->pos() == recordStart + recordSize * 2); //recordSize is measured in 16-bit words

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
	bool hasPoints = (singlePoints && singlePoints->size() != 0 &&
	                  maxPointID == static_cast<int32_t>(singlePoints->size()));
	if (!is3DShape && error == CC_FERR_NO_ERROR && (hasPolylines || hasPoints))
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
					if (parameters.parentWidget)
					{
						//create a list of available fields
						ImportDBFFieldDialog lsfDlg(nullptr);
						for (QList<FieldIndexAndName>::const_iterator it = candidateFields.begin(); it != candidateFields.end(); ++it)
						{
							lsfDlg.listWidget->addItem(it->second);
						}
						static double s_dbfFieldImportScale = 1.0;
						lsfDlg.scaleDoubleSpinBox->setValue(s_dbfFieldImportScale);
						lsfDlg.okPushButton->setVisible(false);

						if (lsfDlg.exec())
						{
							s_dbfFieldImportScale = lsfDlg.scaleDoubleSpinBox->value();

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
								double scale = s_dbfFieldImportScale;
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
							else
							{
								//the user didn't select any field?
							}
						}
						else
						{
							//ignored/cancelled by the uer
						}
					}
					else
					{
						ccLog::Warning("[SHP] Silent mode: no field will be used as Z coordinate!");
					}
				}
				else
				{
					ccLog::Warning("[SHP] No numerical field in the associated DBF file!");
				}
			}
			DBFClose(dbfHandle);
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
