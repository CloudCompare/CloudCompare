//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "DepthMapFileFilter.h"

//qCC_db
#include <ccGBLSensor.h>
#include <ccPointCloud.h>
#include <ccLog.h>

//Qt
#include <QFileInfo>
#include <QString>

//system
#include <assert.h>

CC_FILE_ERROR DepthMapFileFilter::saveToFile(ccHObject* entity, QString filename)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	ccHObject::Container sensors;
	if (entity->isKindOf(CC_TYPES::GBL_SENSOR))
		sensors.push_back(entity);
	else
		entity->filterChildren(sensors, true, CC_TYPES::GBL_SENSOR);

	if (sensors.empty())
	{
		ccLog::Error("No sensor in input selection!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//multiple filenames handling
	QFileInfo fi(filename);
	QString baseName = fi.baseName();
	baseName.append("_");
	QString extension = fi.suffix();
	if (!extension.isNull())
		extension.prepend("_");

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	size_t sensorCount = sensors.size();
	for (size_t i=0; i < sensorCount && result == CC_FERR_NO_ERROR; ++i)
	{
		//more than one sensor? we must generate auto filename
		QString thisFilename = (sensorCount < 2 ? filename : baseName + QString::number(i) + extension);

		//opening file
		FILE* fp = fopen(qPrintable(thisFilename),"wt");
		if (!fp)
		{
			ccLog::Error(QString("[ccGBLSensor::saveASCII] Can't open file '%1' for writing!").arg(thisFilename));
			result = CC_FERR_WRITING;
		}
		else
		{
			ccGBLSensor* sensor = static_cast<ccGBLSensor*>(sensors[i]);
			result = saveToOpenedFile(fp,sensor);
			fclose(fp);
		}
	}

	return result;
}

CC_FILE_ERROR DepthMapFileFilter::saveToOpenedFile(FILE* fp, ccGBLSensor* sensor)
{
	assert(fp && sensor);

	if (!sensor->getParent()->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Warning(QString("[DepthMap] sensor '%1' is not associated to a point cloud!").arg(sensor->getName()));
		return CC_FERR_NO_ERROR; //this is not a severe error (the process can go on)
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(sensor->getParent());

	//the depth map associated to this sensor
	const ccGBLSensor::DepthBuffer& db = sensor->getDepthBuffer();

	fprintf(fp,"// CLOUDCOMPARE DEPTH MAP\n");
	fprintf(fp,"// Associated cloud: %s\n",qPrintable(cloud->getName()));
	fprintf(fp,"// dPhi   = %f [ %f : %f ]\n",
		sensor->getDeltaPhi(),
		sensor->getPhiMin(),
		sensor->getPhiMax());
	fprintf(fp,"// dTheta = %f [ %f : %f ]\n",
		sensor->getDeltaTheta(),
		sensor->getThetaMin(),
		sensor->getThetaMax());
	fprintf(fp,"// pMax   = %f\n",sensor->getSensorRange());
	fprintf(fp,"// L      = %i\n",db.width);
	fprintf(fp,"// H      = %i\n",db.height);
	fprintf(fp,"/////////////////////////\n");

	//an array of projected normals (same size a depth map)
	PointCoordinateType* theNorms = NULL;
	//an array of projected colors (same size a depth map)
	colorType* theColors = NULL;

	//if the sensor is associated to a "ccPointCloud", we may also extract
	//normals and color!
	if (cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

		unsigned nbPoints = cloud->size();
		if (nbPoints == 0)
		{
			ccLog::Warning(QString("[DepthMap] sensor '%1' is associated to an empty cloud!").arg(sensor->getName()));
			return CC_FERR_NO_ERROR; //this is not a severe error (the process can go on)
		}
		else
		{
			//if possible, we create the array of projected normals
			if (pc->hasNormals())
			{
				NormsTableType* decodedNorms = new NormsTableType;
				if (decodedNorms->reserve(nbPoints))
				{
					for (unsigned i=0; i<nbPoints; ++i)
						decodedNorms->addElement(pc->getPointNormal(i).u);

					theNorms = sensor->projectNormals(pc,*decodedNorms);
					decodedNorms->clear();
				}
				else
				{
					ccLog::Warning(QString("[DepthMap] not enough memory to load normals on sensor '%1'!").arg(sensor->getName()));
				}
				decodedNorms->release();
				decodedNorms = 0;
			}

			//if possible, we create the array of projected colors
			if (pc->hasColors())
			{
				GenericChunkedArray<3,colorType>* rgbColors = new GenericChunkedArray<3,colorType>();
				rgbColors->reserve(nbPoints);

				for (unsigned i=0; i<nbPoints; ++i)
				{
					//conversion from colorType[3] to unsigned char[3]
					const colorType* col = pc->getPointColor(i);
					rgbColors->addElement(col);
				}

				theColors = sensor->projectColors(pc,*rgbColors);
				rgbColors->clear();
				rgbColors->release();
				rgbColors = 0;
			}
		}
	}

	PointCoordinateType* _theNorms = theNorms;
	colorType* _theColors = theColors;
	ScalarType* _zBuff = db.zBuff;

	for (unsigned k=0; k<db.height; ++k)
	{
		for (unsigned j=0; j<db.width; ++j)
		{
			//grid index and depth
			fprintf(fp,"%i %i %.12f",j,k,*_zBuff++);

			//color
			if (_theColors)
			{
				fprintf(fp," %i %i %i",_theColors[0],_theColors[1],_theColors[2]);
				_theColors += 3;
			}

			//normal
			if (_theNorms)
			{
				fprintf(fp," %f %f %f",_theNorms[0],_theNorms[1],_theNorms[2]);
				_theNorms += 3;
			}

			fprintf(fp,"\n");
		}
	}

	if (theNorms)
		delete[] theNorms;
	if (theColors)
		delete[] theColors;

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR DepthMapFileFilter::loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
	ccLog::Error("Not available yet!\n");

	return CC_FERR_NO_ERROR;
}
