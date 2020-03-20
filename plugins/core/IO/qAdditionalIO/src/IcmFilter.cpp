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

#include "IcmFilter.h"

//qCC_db
#include <ccCameraSensor.h>
#include <ccImage.h>
#include <ccLog.h>

//Qt
#include <QFileInfo>

//system
#include <cassert>
#include <cstdio>

//Max number of characters per line in an ASCII file
//TODO: use QFile instead!
const int MAX_ASCII_FILE_LINE_LENGTH = 4096;


IcmFilter::IcmFilter()
	: FileIOFilter( {
					"_ICM Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "icm" },
					"icm",
					QStringList{ "Clouds + calibrated images [meta][ascii] (*.icm)" },
					QStringList(),
					Import
					} )
{
}

CC_FILE_ERROR IcmFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	if (CheckForSpecialChars(filename))
	{
		ccLog::Warning(QString("[ICM] Input filename contains special characters. It might be rejected by the I/O filter..."));
	}

	//ouverture du fichier
	FILE *fp = fopen(qPrintable(filename), "rt");
	if (!fp)
	{
		return CC_FERR_READING;
	}

	//buffer
	char line[MAX_ASCII_FILE_LINE_LENGTH];

	//lecture du header
	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
	{
		fclose(fp);
		return CC_FERR_READING;
	}

	if (strncmp(line,"#CC_ICM_FILE",12)!=0)
	{
		fclose(fp);
		return CC_FERR_WRONG_FILE_TYPE;
	}

	//on extrait le chemin relatif
	QString path = QFileInfo(filename).absolutePath();

	char cloudFileName[MAX_ASCII_FILE_LINE_LENGTH];
	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
	{
		fclose(fp);
		return CC_FERR_READING;
	}
	if (strncmp(line,"FILE_NAME=",10)!=0)
	{
		fclose(fp);
		return CC_FERR_WRONG_FILE_TYPE;
	}
	sscanf(line,"FILE_NAME=%s",cloudFileName);

	char subFileType[12];
	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
	{
		fclose(fp);
		return CC_FERR_READING;
	}
	if (strncmp(line,"FILE_TYPE=",10)!=0)
	{
		fclose(fp);
		return CC_FERR_WRONG_FILE_TYPE;
	}
	sscanf(line,"FILE_TYPE=%s",subFileType);

	FileIOFilter::Shared filter = FileIOFilter::FindBestFilterForExtension(subFileType);
	if (!filter)
	{
		ccLog::Warning(QString("[ICM] No I/O filter found for loading file '%1' (type = '%2')").arg(cloudFileName,subFileType));
		fclose(fp);
		return CC_FERR_UNKNOWN_FILE;
	}

	//load the corresponding file (potentially containing several clouds)
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	ccHObject* entities = FileIOFilter::LoadFromFile(QString("%1/%2").arg(path,cloudFileName), parameters, filter, result);
	if (!entities)
	{
		fclose(fp);
		return CC_FERR_READING;
	}

	container.addChild(entities);

	//chargement des images
	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
	{
		ccLog::Error("[ICM] Read error (IMAGES_DESCRIPTOR)! No image loaded");
		fclose(fp);
		return CC_FERR_READING;
	}
	else
	{
		if (strncmp(line,"IMAGES_DESCRIPTOR=",18)!=0)
		{
			fclose(fp);
			return CC_FERR_WRONG_FILE_TYPE;
		}
		char imagesDescriptorFileName[MAX_ASCII_FILE_LINE_LENGTH];
		sscanf(line,"IMAGES_DESCRIPTOR=%s",imagesDescriptorFileName);
		
		int n = LoadCalibratedImages(entities,path,imagesDescriptorFileName,entities->getBB_recursive());
		ccLog::Print("[ICM] %i image(s) loaded ...",n);
	}

	fclose(fp);
	return CC_FERR_NO_ERROR;
}

int IcmFilter::LoadCalibratedImages(ccHObject* entities, const QString& path, const QString& imageDescFilename, const ccBBox& globalBBox)
{
	assert(entities);

	//ouverture du fichier
	QString completeImageDescFilename = QString("%1/%2").arg(path,imageDescFilename);

	if (CheckForSpecialChars(completeImageDescFilename))
	{
		ccLog::Warning(QString("[ICM] File '%1' contains special characters. It might be rejected by the I/O filter...").arg(completeImageDescFilename));
	}

	FILE* fp = fopen(qPrintable(completeImageDescFilename), "rt");
	if (fp == nullptr)
	{
		ccLog::Error(QString("[IcmFilter::loadCalibratedImages] Error opening file %1!").arg(completeImageDescFilename));
		return -1;
	}

	//buffers
	char line[MAX_ASCII_FILE_LINE_LENGTH];
#ifdef INCLUDE_PHOTOS
	char totalFileName[256];
#endif
	int loadedImages = 0;

	//IL FAUDRAIT ETRE PLUS SOUPLE QUE CA !!!
	while (fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp) != nullptr)
	{
		if (line[0] == 'D' && line[1] == 'E' && line[2] == 'F')
		{
			char imageFileName[256];
			sscanf(line,"DEF %s Viewpoint {",imageFileName);

			//add absolute path
			ccImage* CI = new ccImage();
			QString errorStr;
			if (!CI->load(QString("%1/%2").arg(path,imageFileName),errorStr))
			{
				ccLog::Warning(QString("[IcmFilter] Failed to load image %1 (%2)! Process stopped...").arg(imageFileName,errorStr));
				delete CI;
				fclose(fp);
				return loadedImages;
			}

			ccLog::Print("[IcmFilter] Image '%s' loaded",imageFileName);
			CI->setEnabled(false);
			CI->setName(imageFileName);
#ifdef INCLUDE_PHOTOS
			CI->setCompleteFileName(totalFileName);
#endif

			//FOV
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccLog::Print("[IcmFilter] Read error (fieldOfView)!");
				delete CI;
				fclose(fp);
				return loadedImages;
			}
			
			float fov_rad = 0;
			sscanf(line,"\t fieldOfView %f\n",&fov_rad);

			float fov_deg = fov_rad*static_cast<float>(CC_RAD_TO_DEG);
			ccLog::Print("\t FOV=%f (degrees)",fov_deg);

			//Position
			float t[3];
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccLog::Error("[IcmFilter] Read error (position)!");
				delete CI;
				fclose(fp);
				return loadedImages;
			}
			sscanf(line,"\t position %f %f %f\n",t,t+1,t+2);

			ccLog::Print("\t Camera pos=(%f,%f,%f)",t[0],t[1],t[2]);

			//Description
			char desc[MAX_ASCII_FILE_LINE_LENGTH];
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccLog::Error("[IcmFilter] Read error (description)!");
				delete CI;
				fclose(fp);
				return loadedImages;
			}
			sscanf(line,"\t description \"%s\"\n",desc);

			//CI->setDescription(desc);
			ccLog::Print("\t Description: '%s'",desc);

			//Orientation
			float axis[3]{ 0.0f, 0.0f, 0.0f };
			float angle_rad = 0.0f;
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccLog::Error("[IcmFilter] Read error (orientation)!");
				fclose(fp);
				return loadedImages;
			}
			sscanf(line,"\t orientation %f %f %f %f\n",axis,axis+1,axis+2,&angle_rad);

			ccLog::Print("\t Camera orientation=(%f,%f,%f)+[%f]",axis[0],axis[1],axis[2],angle_rad);
			
			ccCameraSensor::IntrinsicParameters params;
			params.vFOV_rad = fov_rad;
			params.arrayWidth = CI->getW();
			params.arrayHeight = CI->getH();
			params.principal_point[0] = params.arrayWidth / 2.0f;
			params.principal_point[1] = params.arrayHeight / 2.0f;
			params.vertFocal_pix = 1.0f; //default focal (for the 3D symbol)
			params.pixelSize_mm[0] = params.pixelSize_mm[1] = 1.0f;
			ccCameraSensor* sensor = new ccCameraSensor(params);

			ccGLMatrix mat;
			mat.initFromParameters(angle_rad,CCVector3::fromArray(axis),CCVector3::fromArray(t));
			sensor->setRigidTransformation(mat);

			sensor->setGraphicScale(globalBBox.getDiagNorm() / 20);
			sensor->setVisible(true);
			sensor->setEnabled(false);
			CI->addChild(sensor);
			CI->setAssociatedSensor(sensor);

			entities->addChild(CI);
			++loadedImages;
		}
	}

	fclose(fp);

	return loadedImages;
}
//*/
