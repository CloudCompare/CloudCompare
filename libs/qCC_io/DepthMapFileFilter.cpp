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

#include "DepthMapFileFilter.h"
#include "FileIO.h"

//qCC_db
#include <ccGBLSensor.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>

//Qt
#include <QFileInfo>
#include <QString>

//system
#include <cassert>


DepthMapFileFilter::DepthMapFileFilter()
	: FileIOFilter( {
					"_Depth Map Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList(),
					"txt",
					QStringList(),
					QStringList{ GetFileFilter() },
					Export | BuiltIn
					} )
{
}

bool DepthMapFileFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::GBL_SENSOR)
	{
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR DepthMapFileFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
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
	QString extension = fi.suffix();

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	size_t sensorCount = sensors.size();
	for (size_t i=0; i < sensorCount && result == CC_FERR_NO_ERROR; ++i)
	{
		//more than one sensor? we must generate auto filename
		QString sensorFilename = (sensorCount > 1 ? QString("%1_%2.%3").arg(baseName).arg(i).arg(extension) : filename);

		ccGBLSensor* sensor = static_cast<ccGBLSensor*>(sensors[i]);
		if (sensor)
		{
			result = saveToFile(sensorFilename,sensor);
			//we stop at the first severe error
			if (result != CC_FERR_NO_SAVE && result != CC_FERR_NO_ERROR)
				break;
		}
	}

	return result;
}

CC_FILE_ERROR DepthMapFileFilter::saveToFile(const QString& filename, ccGBLSensor* sensor)
{
	assert(sensor);
	if (!sensor)
	{
		return CC_FERR_BAD_ARGUMENT;
	}

	//the depth map associated to this sensor
	const ccDepthBuffer& db = sensor->getDepthBuffer();
	if (db.zBuff.empty())
	{
		ccLog::Warning(QString("[DepthMap] sensor '%1' has no associated depth map (you must compute it first)").arg(sensor->getName()));
		return CC_FERR_NO_SAVE; //this is not a severe error (the process can go on)
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(sensor->getParent());
	if (!cloud)
	{
		ccLog::Warning(QString("[DepthMap] sensor '%1' is not associated to a point cloud!").arg(sensor->getName()));
		//this is not a severe error (the process can go on)
	}

	if (CheckForSpecialChars(filename))
	{
		ccLog::Warning(QString("[DepthMap] Output filename contains special characters. It might be scrambled or rejected by the I/O filter..."));
	}

	//opening file
	FILE* fp = fopen(qPrintable(filename), "wt");
	if (!fp)
	{
		ccLog::Error(QString("[DepthMap] Can't open file '%1' for writing!").arg(filename));
		return CC_FERR_WRITING;
	}

	fprintf(fp, "// SENSOR DEPTH MAP\n");
	fprintf(fp, "// %s\n", qPrintable(FileIO::createdBy()));
	fprintf(fp, "// %s\n", qPrintable(FileIO::createdDateTime()));
	fprintf(fp, "// Associated cloud: %s\n", qPrintable(cloud ? cloud->getName() : "none"));
	fprintf(fp, "// Pitch  = %f [ %f : %f ]\n",
		sensor->getPitchStep(),
		sensor->getMinPitch(),
		sensor->getMaxPitch());
	fprintf(fp, "// Yaw   = %f [ %f : %f ]\n",
		sensor->getYawStep(),
		sensor->getMinYaw(),
		sensor->getMaxYaw());
	fprintf(fp, "// Range  = %f\n", sensor->getSensorRange());
	fprintf(fp, "// L      = %i\n", db.width);
	fprintf(fp, "// H      = %i\n", db.height);
	fprintf(fp, "/////////////////////////\n");

	//an array of projected normals (same size a depth map)
	ccGBLSensor::NormalGrid* theNorms = nullptr;
	//an array of projected colors (same size a depth map)
	ccGBLSensor::ColorGrid* theColors = nullptr;

	//if the sensor is associated to a "ccPointCloud", we may also extract
	//normals and color!
	if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

		unsigned nbPoints = cloud->size();
		if (nbPoints == 0)
		{
			ccLog::Warning(QString("[DepthMap] sensor '%1' is associated to an empty cloud?!").arg(sensor->getName()));
			//this is not a severe error (the process can go on)
		}
		else
		{
			//if possible, we create the array of projected normals
			if (pc->hasNormals())
			{
				std::vector<CCVector3> decodedNorms;
				try
				{
					decodedNorms.reserve(nbPoints);
					for (unsigned i = 0; i < nbPoints; ++i)
					{
						decodedNorms.emplace_back(pc->getPointNormal(i));
					}

					theNorms = sensor->projectNormals(pc, decodedNorms);
				}
				catch (const std::bad_alloc&)
				{
					ccLog::Warning(QString("[DepthMap] not enough memory to load normals on sensor '%1'!").arg(sensor->getName()));
				}
			}

			//if possible, we create the array of projected colors
			if (pc->hasColors())
			{
				try
				{
					std::vector<ccColor::Rgb> rgbColors;
					rgbColors.reserve(nbPoints);
					for (unsigned i = 0; i < nbPoints; ++i)
					{
						//conversion from ColorCompType[3] to unsigned char[3]
						rgbColors.emplace_back(pc->getPointColor(i));
					}
					theColors = sensor->projectColors(pc, rgbColors);
				}
				catch (const std::bad_alloc&)
				{
					ccLog::Warning(QString("[DepthMap] not enough memory to load colors on sensor '%1'!").arg(sensor->getName()));
				}
			}
		}
	}

	const PointCoordinateType* _zBuff = db.zBuff.data();
	unsigned index = 0;
	for (unsigned k = 0; k < db.height; ++k)
	{
		for (unsigned j = 0; j < db.width; ++j, ++_zBuff, ++index)
		{
			//grid index and depth
			fprintf(fp, "%u %u %.12f", j, k, *_zBuff);

			//color
			if (theColors)
			{
				const ccColor::Rgb& C = theColors->at(index);
				fprintf(fp, " %i %i %i", C.r, C.g, C.b);
			}

			//normal
			if (theNorms)
			{
				const CCVector3& N = theNorms->at(index);
				fprintf(fp, " %f %f %f", N.x, N.y, N.z);
			}

			fprintf(fp, "\n");
		}
	}

	fclose(fp);
	fp = nullptr;

	if (theNorms)
	{
		delete theNorms;
		theNorms = nullptr;
	}
	if (theColors)
	{
		delete theColors;
		theColors = nullptr;
	}

	return CC_FERR_NO_ERROR;
}
