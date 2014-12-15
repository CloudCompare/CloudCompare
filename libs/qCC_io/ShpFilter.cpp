//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qBRGM                        #
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
//#                           COPYRIGHT: BRGM                              #
//#                                                                        #
//##########################################################################

#ifdef CC_SHP_SUPPORT

#include "ShpFilter.h"

//qCC_db
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QString>
#include <QFile>
#include <QFileInfo>
#include <QApplication>
#include <QtEndian>

//CCLib
#include <Neighbourhood.h>
#include <MeshSamplingTools.h>

//System
#include <string.h>

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

bool ShpFilter::canLoadExtension(QString upperCaseExt) const
{
	return (upperCaseExt == "SHP");
}

bool ShpFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (	type == CC_TYPES::POLY_LINE
		||	type == CC_TYPES::POINT_CLOUD
		||	type == CC_TYPES::HIERARCHY_OBJECT
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
				for (unsigned i=1; i<baseEntity->getChildrenNumber(); ++i)
				{
					if (baseEntity->getChild(i) && baseEntity->getChild(i)->getClassID() != child->getClassID())
					{
						//mixed shapes are not allowed in shape files (yet?)
						return;
					}
				}
			}

			//call the same method on the first child so as to get its type
			GetSupportedShapes(child,shapes,shapeType/*,closedPolylinesAsPolygons*/);
			if (shapeType == SHP_NULL_SHAPE)
				return;

			//then add the remaining children
			{
				for (unsigned i=1; i<baseEntity->getChildrenNumber(); ++i)
				{
					ESRI_SHAPE_TYPE otherShapeType = SHP_NULL_SHAPE;
					ccHObject* child = baseEntity->getChild(i);
					if (child)
						GetSupportedShapes(child,shapes,otherShapeType);
					
					if (otherShapeType != shapeType)
					{
						if (child)
							ccLog::Warning(QString("[SHP] Entity %1 has not the same type (%1) as the others in the selection (%2)! Can't mix types...")
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

CC_FILE_ERROR LoadPolyline(QFile& file, ccHObject& container, int32_t index, ESRI_SHAPE_TYPE shapeTypeInt, const CCVector3d& PShift)
{
	char header[40];
	file.read(header,40);
	//check for errors
	if (file.error() != QFile::NoError)
		return CC_FERR_READING;

	//Byte 0: Box
	{
		//The Bounding Box for the PolyLine stored in the order Xmin, Ymin, Xmax, Ymax
		//DGM: ignored
		//double xMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header   ));
		//double xMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+ 8));
		//double yMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+16));
		//double yMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+24));
	}

	//Byte 32: NumParts (The number of parts in the PolyLine)
	int32_t numParts = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header+32));

	//Byte 36: NumPoints (The total number of points for all parts)
	int32_t numPoints = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header+36));

	//Byte 40: Parts (An array of length NumParts)
	//for each part, the index of its first point in the points array
	std::vector<int32_t> startIndexes;
	{
		try
		{
			startIndexes.resize(numParts,0);
		}
		catch(std::bad_alloc)
		{
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		for (int32_t i=0; i!=numParts; ++i)
		{
			file.read(header,4);
			startIndexes[i] = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header));
		}
		//FIXME: we should use this information and create as many polylines as necessary!
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(numPoints))
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}
	vertices->setEnabled(false);
	
	//Points (An array of length NumPoints)
	{
		for (int32_t i=0; i<numPoints; ++i)
		{
			file.read(header,16);
			//check for errors
			if (file.error() != QFile::NoError)
				return CC_FERR_READING;
			double x = qFromLittleEndian<double>(*reinterpret_cast<double*>(header  ));
			double y = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+8));
			CCVector3 P(static_cast<PointCoordinateType>(x + PShift.x),
						static_cast<PointCoordinateType>(y + PShift.y),
						0);
			vertices->addPoint(P);
		}
	}

	//3D polylines
	bool is3D = (shapeTypeInt > SHP_POINT_Z && shapeTypeInt < SHP_POINT_M);
	if (is3D)
	{
		//Z boundaries
		{
			file.read(header,16);
			//DGM: ignored
			//double zMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header  ));
			//double zMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+8));
		}

		//Z coordinates (an array of length NumPoints)
		{
			for (int32_t i=0; i<numPoints; ++i)
			{
				file.read(header,8);
				//check for errors
				if (file.error() != QFile::NoError)
					return CC_FERR_READING;
				double z = qFromLittleEndian<double>(*reinterpret_cast<double*>(header));
				const CCVector3* P = vertices->getPoint(i);
				const_cast<CCVector3*>(P)->z = static_cast<PointCoordinateType>(z + PShift.z);
			}
		}
	}

	//3D polylines or 2D polylines + measurement
	if (shapeTypeInt > SHP_POINT_Z)
	{
		//M boundaries
		ccScalarField* sf = 0;
		{
			file.read(header,16);
			//check for errors
			if (file.error() != QFile::NoError)
				return CC_FERR_READING;
			double mMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header  ));
			double mMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+8));

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
			double scalar = qToLittleEndian<double>(ESRI_NO_DATA);
			for (int32_t i=0; i<numPoints; ++i)
			{
				file.read(header,8);
				//check for errors
				if (file.error() != QFile::NoError)
					return CC_FERR_READING;
				double m = qFromLittleEndian<double>(*reinterpret_cast<double*>(header));
				ScalarType s = m == ESRI_NO_DATA ? NAN_VALUE : static_cast<ScalarType>(m);
				sf->addElement(s);
			}
			sf->computeMinAndMax();
			int sfIdx = vertices->addScalarField(sf);
			vertices->setCurrentDisplayedScalarField(sfIdx);
			vertices->showSF(true);
		}
	}

	//and of course the polyline!
	{
		ccPolyline* poly = new ccPolyline(vertices);
		poly->addChild(vertices);

		//test if the polyline is closed
		if (numPoints > 2 && (*vertices->getPoint(0) - *vertices->getPoint(numPoints-1)).norm() < ZERO_TOLERANCE)
		{
			numPoints--;
			vertices->resize(numPoints);
			poly->setClosed(true);
		}

		if (!poly->reserve(numPoints))
		{
			delete poly;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		poly->addPointIndex(0,numPoints);
		poly->showSF(vertices->sfShown());
		poly->setName(QString("Polyline #%1").arg(index));
		poly->set2DMode(!is3D); //FIXME DGM: maybe we should ask the user?
		container.addChild(poly);
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR SavePolyline(ccPolyline* poly, QFile& file, int32_t& bytesWritten, ESRI_SHAPE_TYPE outputShapeType)
{
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

	ccBBox box = poly->getBB();
	assert(box.isValid());

	//Shape Type
	{
		//Byte 0: Shape Type
		int32_t shapeTypeInt = qToLittleEndian<int32_t>(outputShapeType);
		file.write((const char*)&shapeTypeInt,4);
		bytesWritten += 4;
	}

	//Byte 4: Box
	{
		double xMin = qToLittleEndian<double>(box.minCorner().x);
		double xMax = qToLittleEndian<double>(box.maxCorner().x);
		double yMin = qToLittleEndian<double>(box.minCorner().y);
		double yMax = qToLittleEndian<double>(box.maxCorner().y);
		//The Bounding Box for the PolyLine stored in the order Xmin, Ymin, Xmax, Ymax
		/*Byte  4*/file.write((const char*)&xMin,8);
		/*Byte 12*/file.write((const char*)&yMin,8);
		/*Byte 20*/file.write((const char*)&xMax,8);
		/*Byte 28*/file.write((const char*)&yMax,8);
		bytesWritten += 32;
	}

	//Byte 36: NumParts (The number of parts in the PolyLine)
	{
		int32_t numParts = qToLittleEndian<int32_t>(1);
		file.write((const char*)&numParts,4);
		bytesWritten += 4;
	}

	//Byte 40: NumPoints (The total number of points for all parts)
	int32_t numPoints = realNumPoints;
	if (isClosed)
		numPoints++;
	{
		int32_t numPointsLE = qToLittleEndian<int32_t>(numPoints);
		file.write((const char*)&numPointsLE,4);
		bytesWritten += 4;
	}

	//Byte 44: Parts (An array of length NumParts)
	{
		//for each part, the index of its first point in the points array
		int32_t startIndex = qToLittleEndian<int32_t>(0);
		file.write((const char*)&startIndex,4);
		bytesWritten += 4;
	}

	//for polygons we must list the vertices in the right order:
	//"The neighborhood to the right of an observer walking along
	//the ring in vertex order is the inside of the polygon"
	bool inverseOrder = false;
	if (outputShapeType == SHP_POLYGON || outputShapeType == SHP_POLYGON_Z)
	{
		assert(isClosed);
		assert(numPoints > 2);
		//get bounding box
		ccBBox box = poly->getBB();
		assert(box.isValid());
		//get the two largest (ordered) dimensions (dim1, dim2)
		CCVector3 diag = box.getDiagVec();
		unsigned char minDim = diag.y < diag.x ? 1 : 0;
		if (diag.z < diag.u[minDim])
			minDim = 2;
		unsigned char dim1 = ((minDim+1) % 3);
		unsigned char dim2 = ((minDim+2) % 3);

		if (diag.u[dim1] > 0) //if the polyline is flat, no need to bother ;)
		{
			//look for the top-left-most point in this 'plane'
			int32_t leftMostPointIndex = 0;
			{
				const CCVector3* leftMostPoint = vertices->getPoint(0);
				for (int32_t i=1; i<realNumPoints; ++i)
				{
					const CCVector3* P = vertices->getPoint(i);
					if (P->u[dim1] < leftMostPoint->u[dim1] || (P->u[dim1] == leftMostPoint->u[dim1] && P->u[dim2] < leftMostPoint->u[dim2]))
					{
						leftMostPoint = P;
						leftMostPointIndex = i;
					}
				}
			}

			//we simply compare the angles betwween the two edges that have the top-left-most vertex in common
			{
				const CCVector3* B = vertices->getPoint(leftMostPointIndex > 0 ? leftMostPointIndex-1 : realNumPoints-1);
				const CCVector3* P = vertices->getPoint(leftMostPointIndex);
				const CCVector3* A = vertices->getPoint(leftMostPointIndex+1 < realNumPoints ? leftMostPointIndex+1 : 0);
			
				CCVector3 PA = *A-*P;
				CCVector3 PB = *B-*P;
				PointCoordinateType anglePA = atan2(PA.u[dim2],PA.u[dim1]); //forward
				PointCoordinateType anglePB = atan2(PB.u[dim2],PB.u[dim1]); //backward
				//angles should all be in [-PI/2;0]
				if (anglePA < anglePB)
					inverseOrder = true;
			}
		}
	}

	//Points (An array of length NumPoints)
	{
		for (int32_t i=0; i<numPoints; ++i)
		{
			int32_t ii = (inverseOrder ? numPoints-1-i : i);
			const CCVector3* P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed

			double x = qToLittleEndian<double>(P->x);
			double y = qToLittleEndian<double>(P->y);
			/*Byte 0*/file.write((const char*)&x,8);
			/*Byte 8*/file.write((const char*)&y,8);
			bytesWritten += 16;
		}
	}

	//3D polylines
	if (!is2D)
	{
		//Z boundaries
		{
			double zMin = qToLittleEndian<double>(box.minCorner().z);
			double zMax = qToLittleEndian<double>(box.maxCorner().z);
			file.write((const char*)&zMin,8);
			file.write((const char*)&zMax,8);
			bytesWritten += 16;
		}

		//Z coordinates (for each part - just one here)
		{
			for (int32_t i=0; i<numPoints; ++i)
			{
				int32_t ii = (inverseOrder ? numPoints-1-i : i);
				const CCVector3* P = vertices->getPoint(ii % realNumPoints); //warning: handle loop if polyline is closed
				double z = qToLittleEndian<double>(P->z);
				file.write((const char*)&z,8);
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
				for (int32_t i=0; i<realNumPoints; ++i)
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
			mMin = qToLittleEndian<double>(mMin);
			mMax = qToLittleEndian<double>(mMax);
			file.write((const char*)&mMin,8);
			file.write((const char*)&mMax,8);
			bytesWritten += 16;
		}

		//M values (for each part - just one here)
		{
			double scalar = qToLittleEndian<double>(ESRI_NO_DATA);
			for (int32_t i=0; i<numPoints; ++i)
			{
				if (hasSF)
				{
					scalar = static_cast<double>(vertices->getPointScalarValue(i % realNumPoints)); //warning: handle loop if polyline is closed
					scalar = qToLittleEndian<double>(scalar);
				}
				file.write((const char*)&scalar,8);
				bytesWritten += 8;
			}
		}
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LoadCloud(QFile& file, ccHObject& container, int32_t index, ESRI_SHAPE_TYPE shapeTypeInt, const CCVector3d& PShift)
{
	char header[36];
	file.read(header,36);

	//Byte 0: Box
	{
		//The Bounding Box for the Cloud stored in the order Xmin, Ymin, Xmax, Ymax
		//DGM: ignored
		//double xMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header   ));
		//double xMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+ 8));
		//double yMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+16));
		//double yMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+24));
	}

	//Byte 32: NumPoints (The total number of points)
	int32_t numPoints = qFromLittleEndian<int32_t>(*reinterpret_cast<int32_t*>(header+32));

	ccPointCloud* cloud = new ccPointCloud(QString("Cloud #%1").arg(index));
	if (!cloud->reserve(numPoints))
	{
		delete cloud;
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}
	
	//Points (An array of length NumPoints)
	{
		for (int32_t i=0; i<numPoints; ++i)
		{
			file.read(header,16);
			double x = qFromLittleEndian<double>(*reinterpret_cast<double*>(header  ));
			double y = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+8));
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
			file.read(header,16);
			//DGM: ignored
			//double zMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header  ));
			//double zMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+8));
		}

		//Z coordinates (an array of length NumPoints)
		{
			for (int32_t i=0; i<numPoints; ++i)
			{
				file.read(header,8);
				double z = qFromLittleEndian<double>(*reinterpret_cast<double*>(header));
				const CCVector3* P = cloud->getPoint(i);
				const_cast<CCVector3*>(P)->z = static_cast<PointCoordinateType>(z + PShift.z);
			}
			cloud->invalidateBoundingBox();
		}
	}

	//3D clouds or 2D clouds + measurement
	if (	shapeTypeInt == SHP_MULTI_POINT_Z
		||	shapeTypeInt == SHP_MULTI_POINT_M )
	{
		//M boundaries
		ccScalarField* sf = 0;
		{
			file.read(header,16);
			double mMin = qFromLittleEndian<double>(*reinterpret_cast<double*>(header  ));
			double mMax = qFromLittleEndian<double>(*reinterpret_cast<double*>(header+8));

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
			double scalar = qToLittleEndian<double>(ESRI_NO_DATA);
			for (int32_t i=0; i<numPoints; ++i)
			{
				file.read(header,8);
				double m = qFromLittleEndian<double>(*reinterpret_cast<double*>(header));
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

	ccBBox box = cloud->getBB();
	assert(box.isValid());

	//Shape Type
	{
		//Byte 0: Shape Type
		int32_t shapeTypeInt = qToLittleEndian<int32_t>(SHP_MULTI_POINT_Z);
		file.write((const char*)&shapeTypeInt,4);
		bytesWritten += 4;
	}

	//Byte 4: Box
	{
		double xMin = qToLittleEndian<double>(box.minCorner().x);
		double xMax = qToLittleEndian<double>(box.maxCorner().x);
		double yMin = qToLittleEndian<double>(box.minCorner().y);
		double yMax = qToLittleEndian<double>(box.maxCorner().y);
		//The Bounding Box for the Cloud stored in the order Xmin, Ymin, Xmax, Ymax
		/*Byte  4*/file.write((const char*)&xMin,8);
		/*Byte 12*/file.write((const char*)&yMin,8);
		/*Byte 20*/file.write((const char*)&xMax,8);
		/*Byte 28*/file.write((const char*)&yMax,8);
		bytesWritten += 32;
	}

	//Byte 36: NumPoints (The total number of points)
	int32_t numPoints = static_cast<int32_t>(cloud->size());
	{
		int32_t numPointsLE = qToLittleEndian<int32_t>(numPoints);
		file.write((const char*)&numPointsLE,4);
		bytesWritten += 4;
	}

	//Points (An array of length NumPoints)
	{
		for (int32_t i=0; i<numPoints; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);

			double x = qToLittleEndian<double>(P->x);
			double y = qToLittleEndian<double>(P->y);
			/*Byte 0*/file.write((const char*)&x,8);
			/*Byte 8*/file.write((const char*)&y,8);
			bytesWritten += 16;
		}
	}

	//Z boundaries
	{
		double zMin = qToLittleEndian<double>(box.minCorner().z);
		double zMax = qToLittleEndian<double>(box.maxCorner().z);
		file.write((const char*)&zMin,8);
		file.write((const char*)&zMax,8);
		bytesWritten += 16;
	}

	//Z coordinates
	{
		for (int32_t i=0; i<numPoints; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			double z = qToLittleEndian<double>(P->z);
			file.write((const char*)&z,8);
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
			for (int32_t i=0; i<numPoints; ++i)
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
		mMin = qToLittleEndian<double>(mMin);
		mMax = qToLittleEndian<double>(mMax);
		file.write((const char*)&mMin,8);
		file.write((const char*)&mMax,8);
		bytesWritten += 16;
	}

	//M values
	{
		double scalar = qToLittleEndian<double>(ESRI_NO_DATA);
		for (int32_t i=0; i<numPoints; ++i)
		{
			if (hasSF)
			{
				scalar = static_cast<double>(cloud->getPointScalarValue(i));
				scalar = qToLittleEndian<double>(scalar);
			}
			file.write((const char*)&scalar,8);
			bytesWritten += 8;
		}
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LoadSinglePoint(QFile& file, ccPointCloud* &singlePoints, ESRI_SHAPE_TYPE shapeTypeInt, const CCVector3d& PShift)
{
	char buffer[16];
	file.read(buffer,16);

	double x = qFromLittleEndian<double>(*reinterpret_cast<double*>(buffer  ));
	double y = qFromLittleEndian<double>(*reinterpret_cast<double*>(buffer+8));
	CCVector3 P(static_cast<PointCoordinateType>(x + PShift.x),
				static_cast<PointCoordinateType>(y + PShift.y),
				0);

	//3D point
	if (shapeTypeInt == SHP_POINT_Z)
	{
		//Z coordinate
		{
			file.read(buffer,8);
			double z = qFromLittleEndian<double>(*reinterpret_cast<double*>(buffer));
			P.z = static_cast<PointCoordinateType>(z + PShift.z);
		}
	}

	ScalarType s = NAN_VALUE;
	if (	shapeTypeInt == SHP_POINT_Z
		||	shapeTypeInt == SHP_POINT_M )
	{
		//Measure
		{
			file.read(buffer,8);
			double m = qFromLittleEndian<double>(*reinterpret_cast<double*>(buffer));
			if (m != ESRI_NO_DATA)
			{
				s = static_cast<ScalarType>(m);
				//add a SF to the cloud if not done already
				if (!singlePoints->hasScalarFields())
				{
					int sfIdx = singlePoints->addScalarField("Measures");
					singlePoints->setCurrentScalarField(sfIdx);
					//set the SF value for the previous points
					if (singlePoints)
					{
						for (unsigned i=0; i<singlePoints->size(); ++i)
							singlePoints->setPointScalarValue(i,NAN_VALUE);
					}
				}
				else
				{
					ccLog::Warning("[SHP] Not enough memory to load scalar values on points");
				}
			}
		}
	}

	if (!singlePoints)
		singlePoints = new ccPointCloud("Points");
	if (!singlePoints->reserve(singlePoints->size()+1))
		return CC_FERR_NOT_ENOUGH_MEMORY;
	
	singlePoints->addPoint(P);

	if (singlePoints->hasScalarFields())
		singlePoints->setPointScalarValue(singlePoints->size()-1,s);

	return CC_FERR_NO_ERROR;
}


CC_FILE_ERROR ShpFilter::saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters)
{
	std::vector<GenericField*> fields;
	return saveToFile(entity, fields, filename);
}

bool ShpFilter::IntegerField::save(DBFHandle handle, int fieldIndex) const
{
	if (!handle || fieldIndex < 0)
	{
		assert(false);
		return false;
	}
	
	for (size_t i=0; i<values.size(); ++i)
		DBFWriteIntegerAttribute(handle,static_cast<int>(i),fieldIndex,values[i]);

	return true;
}

bool ShpFilter::DoubleField::save(DBFHandle handle, int fieldIndex) const
{
	if (!handle || fieldIndex < 0)
	{
		assert(false);
		return false;
	}
	
	for (size_t i=0; i<values.size(); ++i)
		DBFWriteDoubleAttribute(handle,static_cast<int>(i),fieldIndex,values[i]);

	return true;
}

bool ShpFilter::DoubleField3D::save(DBFHandle handle, int xFieldIndex, int yFieldIndex, int zFieldIndex) const
{
	if (!handle || xFieldIndex < 0 || yFieldIndex < 0 || zFieldIndex < 0)
	{
		assert(false);
		return false;
	}
	
	for (size_t i=0; i<values.size(); ++i)
	{
		DBFWriteDoubleAttribute(handle,static_cast<int>(i),xFieldIndex,values[i].x);
		DBFWriteDoubleAttribute(handle,static_cast<int>(i),yFieldIndex,values[i].y);
		DBFWriteDoubleAttribute(handle,static_cast<int>(i),zFieldIndex,values[i].z);
	}

	return true;
}

CC_FILE_ERROR ShpFilter::saveToFile(ccHObject* entity, const std::vector<GenericField*>& fields, QString filename)
{
	if (!entity)
		return CC_FERR_BAD_ENTITY_TYPE;
	
	//this filter only supports point clouds, meshes and polylines!
	ESRI_SHAPE_TYPE inputShapeType = SHP_NULL_SHAPE;
	ccHObject::Container toSave;
	GetSupportedShapes(entity,toSave,inputShapeType/*,m_closedPolylinesAsPolygons*/);

	if (inputShapeType == SHP_NULL_SHAPE || toSave.empty())
		return CC_FERR_BAD_ENTITY_TYPE;

	ESRI_SHAPE_TYPE outputShapeType = inputShapeType;
	if (m_closedPolylinesAsPolygons && (inputShapeType == SHP_POLYLINE || inputShapeType == SHP_POLYLINE_Z))
	{
		//check if all the polylines are closed!
		bool allAreClosed = true;
		for (size_t i=0; i<toSave.size() && allAreClosed; ++i)
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
	QString shpFilename = baseFileName+QString(".shp");
	QFile file(shpFilename);
	if (!file.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	//index file (same base name + ".shx")
	QString indexFilename = baseFileName+QString(".shx");
	QFile indexFile(indexFilename);
	if (!indexFile.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	//build header (refer to ESRI Shapefile Technical Description)
	char header[100];
	memset(header,0,100);
	{
		/*** WARNING: the beginning of the header is written with big endianness! ***/
		char* _header = header;
		
		//Byte 0: SHP code
		const int32_t code = qToBigEndian<int32_t>(9994);
		memcpy(_header,(const char*)&code,4);
		_header += 4;

		//Byte 4: unused (20 bytes)
		_header += 20;

		//Byte 24: file length (will be written... later ;)
		_header += 4;

		/*** WARNING: from now on, we only write data with little endianness! ***/

		//Byte 28: file verion
		const int32_t version = qToLittleEndian<int32_t>(1000);
		memcpy(_header,(const char*)&version,4);
		_header += 4;

		//Byte 32: shape type
		int32_t shapeTypeInt = qToLittleEndian<int32_t>(outputShapeType);
		memcpy(_header,(const char*)&shapeTypeInt,4);
		_header += 4;

		ccBBox box = entity->getBB();
		assert(box.isValid());

		//X and Y bounaries
		double xMin = qToLittleEndian<double>(box.minCorner().x);
		double xMax = qToLittleEndian<double>(box.maxCorner().x);
		double yMin = qToLittleEndian<double>(box.minCorner().y);
		double yMax = qToLittleEndian<double>(box.maxCorner().y);
		//Byte 36: box X min
		memcpy(_header,(const char*)&xMin,8);
		_header += 8;
		//Byte 44: box Y min
		memcpy(_header,(const char*)&yMin,8);
		_header += 8;
		//Byte 52: box X max
		memcpy(_header,(const char*)&xMax,8);
		_header += 8;
		//Byte 60: box Y max
		memcpy(_header,(const char*)&yMax,8);
		_header += 8;

		//Z bounaries
		//Unused, with value 0.0, if not Measured or Z type
		double zMin = outputShapeType < SHP_POINT_Z ? 0.0 : qToLittleEndian<double>(box.minCorner().z);
		double zMax = outputShapeType < SHP_POINT_Z ? 0.0 : qToLittleEndian<double>(box.maxCorner().z);
		//Byte 68: box Z min
		memcpy(_header,(const char*)&zMin,8);
		_header += 8;
		//Byte 76: box Z max
		memcpy(_header,(const char*)&zMax,8);
		_header += 8;

		//M bounaries (M = measures)
		double mMin = ESRI_NO_DATA;
		double mMax = ESRI_NO_DATA;
		//Byte 84: M min
		memcpy(_header,(const char*)&mMin,8);
		_header += 8;
		//Byte 92: M max
		memcpy(_header,(const char*)&mMax,8);
		_header += 8;
	}

	//actually write the header
	if (!file.write(header,100))
		return CC_FERR_WRITING;
	if (!indexFile.write(header,100)) //exact same header for index file!
		return CC_FERR_WRITING;
	int32_t fileLength = 100;

	//save shapes
	unsigned shapeIndex = 0;
	for (unsigned i=0; i<toSave.size(); ++i)
	{
		ccHObject* child = toSave[i];
		
		//check entity elligibility
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
			file.write((const char*)&recordNumber,4);
			//Byte 4: Content Length
			file.write((const char*)&recordSize,4); //will be updated later
		}
		fileLength += 8;

		CC_FILE_ERROR error = CC_FERR_NO_ERROR;
		switch (inputShapeType)
		{
		case SHP_POLYLINE:
		case SHP_POLYLINE_Z:
			assert(child->isKindOf(CC_TYPES::POLY_LINE));
			error = SavePolyline(static_cast<ccPolyline*>(child),file,recordSize,outputShapeType);
			break;
		case SHP_MULTI_POINT_Z:
			assert(child->isKindOf(CC_TYPES::POINT_CLOUD));
			error = SaveAsCloud(ccHObjectCaster::ToGenericPointCloud(child),file,recordSize);
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
			file.seek(recordStart+4);
			int32_t recordSize16bits = qToBigEndian<int32_t>(recordSize/2); //recordSize is measured in 16-bit words
			file.write((const char*)&recordSize16bits,4);
			//restore last pos
			file.seek(currentPos);
		}

		//write corresponding entry in index SHX file
		{
			//Byte 0: Offset
			int32_t recordStart16bits = qToBigEndian<int32_t>(recordStart/2); //recordStart is measured in 16-bit words
			indexFile.write((const char*)&recordStart16bits,4);
			
			//Byte 4: Content Length
			int32_t recordSize16bits= qToBigEndian<int32_t>(recordSize/2); //recordSize is measured in 16-bit words
			indexFile.write((const char*)&recordSize16bits,4);
		}
	}

	//update main file length
	{
		file.seek(24);
		//Byte 24: file length (in 16-bit words)
		int32_t fileLength16bits = qToBigEndian<int32_t>(fileLength/2);
		file.write((const char*)&fileLength16bits,4);
	}
	file.close();

	//update idx file length
	{
		indexFile.seek(24);
		//Byte 24: file length (in 16-bit words)
		int32_t idxFileLength = (int32_t)indexFile.size();
		int32_t idxFileLength16bits = qToBigEndian<int32_t>(idxFileLength/2);
		indexFile.write((const char*)&idxFileLength16bits,4);
	}
	indexFile.close();

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//eventually, we create the DB file (suffix should be ".dbf")
	QString dbfFilename = baseFileName+QString(".dbf");
	DBFHandle dbfHandle = DBFCreate(qPrintable(dbfFilename));
	if (dbfHandle)
	{
		while (true) //trick: we use 'while' to be able to break anytime
		{
			//always write an 'index' table
			{
				int fieldIdx = DBFAddField(dbfHandle,"local_idx",FTInteger,6,0);
				if (fieldIdx >= 0)
				{
					for (unsigned i=0; i<toSave.size(); ++i)
						DBFWriteIntegerAttribute(dbfHandle,static_cast<int>(i),fieldIdx,static_cast<int>(i)+1);
				}
				else
				{
					ccLog::Warning(QString("[ShpFilter::saveToFile] Failed to save field 'index' (default)"));
					result = CC_FERR_WRITING;
					break;
				}
			}

			//and write the other tables (specified by the user)
			for (std::vector<GenericField*>::const_iterator it = fields.begin(); it != fields.end(); ++it)
			{
				const GenericField* field = *it;
				if (field->is3D()) //3D case
				{
					int xFieldIdx = DBFAddField(dbfHandle,qPrintable(field->name()+QString("_x")),field->type(),field->width(),field->decimal());
					int yFieldIdx = DBFAddField(dbfHandle,qPrintable(field->name()+QString("_y")),field->type(),field->width(),field->decimal());
					int zFieldIdx = DBFAddField(dbfHandle,qPrintable(field->name()+QString("_z")),field->type(),field->width(),field->decimal());
					if (xFieldIdx >= 0 && yFieldIdx >= 0 && zFieldIdx >= 0)
					{
						if (!(*it)->save(dbfHandle,xFieldIdx,yFieldIdx,zFieldIdx))
							xFieldIdx = -1;
					}

					if (xFieldIdx < 0)
					{
						ccLog::Warning(QString("[ShpFilter::saveToFile] Failed to save field '%1'").arg(field->name()));
						result = CC_FERR_WRITING;
						break;
					}
				}
				else //1D case
				{
					int fieldIdx = DBFAddField(dbfHandle,qPrintable(field->name()),field->type(),field->width(),field->decimal());
					if (fieldIdx >= 0)
					{
						if (!(*it)->save(dbfHandle,fieldIdx))
							fieldIdx = -1;
					}

					if (fieldIdx < 0)
					{
						ccLog::Warning(QString("[ShpFilter::saveToFile] Failed to save field '%1'").arg(field->name()));
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

CC_FILE_ERROR ShpFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	//global shift
	CCVector3d Pshift(0,0,0);

	//read header (refer to ESRI Shapefile Technical Description)
	if (file.size() < 100)
		return CC_FERR_MALFORMED_FILE;
	char header[100];
	file.read(header,100);
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
		double xMin = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 44: box Y min
		double xMax = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 52: box X max
		double yMin = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 60: box Y max
		double yMax = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
		_header += 8;

		//Z bounaries
		//Unused, with value 0.0, if not Measured or Z type
		//Byte 68: box Z min
		double zMin = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 76: box Z max
		double zMax = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
		_header += 8;

		CCVector3d Pmin(xMin,yMin,zMin);
		if (HandleGlobalShift(Pmin,Pshift,parameters))
		{
			ccLog::Warning("[SHP] Entities will be recentered! Translation: (%.2f,%.2f,%.2f)",Pshift.x,Pshift.y,Pshift.z);
		}

		//M bounaries (M = measures)
		//Byte 84: M min
		double mMin = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
		_header += 8;
		//Byte 92: M max
		double mMax = qFromLittleEndian<double>(*reinterpret_cast<const double*>(_header));
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

	//load shapes
	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	ccPointCloud* singlePoints = 0;
	qint64 pos = file.pos();
	while (fileLength >= 12)
	{
		file.seek(pos);
		assert(pos + fileLength == file.size());
		//load shape record in main SHP file
		{
			file.read(header,8);
			//Byte 0: Record Number
			int32_t recordNumber = qFromBigEndian<int32_t>(*reinterpret_cast<const int32_t*>(header)); //Record numbers begin at 1
			//Byte 4: Content Length
			int32_t recordSize = qFromBigEndian<int32_t>(*reinterpret_cast<const int32_t*>(header+4)); //Record numbers begin at 1
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
			file.read(header,4);
			recordSize -= 4;
			int32_t shapeTypeInt = qToLittleEndian<int32_t>(*reinterpret_cast<const int32_t*>(header));
			ccLog::Print(QString("[SHP] Record #%1 - type: %2 (%3 bytes)").arg(recordNumber).arg(ToString(static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt))).arg(recordSize));

			switch (shapeTypeInt)
			{
			case SHP_POLYLINE:
			case SHP_POLYLINE_Z:
			case SHP_POLYGON:
			case SHP_POLYGON_Z:
				error = LoadPolyline(file,container,recordNumber,static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt),Pshift);
				break;
			case SHP_MULTI_POINT:
			case SHP_MULTI_POINT_Z:
			case SHP_MULTI_POINT_M:
				error = LoadCloud(file,container,recordNumber,static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt),Pshift);
				break;
			case SHP_POINT:
			case SHP_POINT_Z:
			case SHP_POINT_M:
				error = LoadSinglePoint(file,singlePoints,static_cast<ESRI_SHAPE_TYPE>(shapeTypeInt),Pshift);
				break;
			//case SHP_MULTI_PATCH:
			//	error = LoadMesh(file,recordSize);
			//	break;
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
			break;
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
