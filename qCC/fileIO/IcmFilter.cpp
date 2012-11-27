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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2224                                                              $
//$LastChangedDate:: 2012-07-25 19:13:23 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//
#include "IcmFilter.h"

//CCLib
#include <CCMiscTools.h>
#include <CCConst.h>

//qCC_db
#include <ccCalibratedImage.h>

//Qt
#include <QFileInfo>

//system
#include <stdio.h>
#include <assert.h>

#include "../ccConsole.h"

CC_FILE_ERROR IcmFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
    //ouverture du fichier
    FILE *fp = fopen(filename, "rt");
    if (!fp)
        return CC_FERR_READING;

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

	CC_FILE_TYPES fType = FileIOFilter::StringToFileFormat(subFileType);

	//chargement du fichier (potentiellement plusieurs listes) correspondant
	ccHObject* entities = FileIOFilter::LoadFromFile(qPrintable(QString("%0/%1").arg(path).arg(cloudFileName)),fType);
    if (!entities)
	{
		fclose(fp);
		return CC_FERR_READING;
	}

	container.addChild(entities);

	//chargement des images
	char imagesDescriptorFileName[MAX_ASCII_FILE_LINE_LENGTH];
	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
	{
		ccConsole::Error("[IcmFilter::loadModelFromIcmFile] Read error (IMAGES_DESCRIPTOR)! No image loaded");
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
		sscanf(line,"IMAGES_DESCRIPTOR=%s",imagesDescriptorFileName);
		int n = loadCalibratedImages(entities,path,imagesDescriptorFileName);
		ccConsole::Print("[IcmFilter::loadModelFromIcmFile] %i image(s) loaded ...",n);
	}

	fclose(fp);
	return CC_FERR_NO_ERROR;
}

int IcmFilter::loadCalibratedImages(ccHObject* entities, const QString& path, const QString& imageDescFilename)
{
    assert(entities);

    //ouverture du fichier
	QString completeImageDescFilename = QString("%0/%1").arg(path).arg(imageDescFilename);
    FILE *fp;
    if ((fp = fopen(qPrintable(completeImageDescFilename), "rt"))==NULL)
    {
		ccConsole::Error(QString("[IcmFilter::loadCalibratedImages] Error opening file %1!").arg(completeImageDescFilename));
        return -1;
    }

	//buffers
	char line[MAX_ASCII_FILE_LINE_LENGTH];
    #ifdef INCLUDE_PHOTOS
	char totalFileName[256];
	#endif
	int loadedImages = 0;

	//IL FAUDRAIT ETRE PLUS SOUPLE QUE CA !!!
	while (fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp)!=NULL)
	{
		if ((line[0]=='D')&&(line[1]=='E')&&(line[2]=='F'))
		{
			char imageFileName[256];
			sscanf(line,"DEF %s Viewpoint {",imageFileName);

			//on rajoute le chemin
			ccCalibratedImage* CI = new ccCalibratedImage();
			QString errorStr;
			if (!CI->load(QString("%0/%1").arg(path).arg(imageFileName),errorStr))
			{
				delete CI;
				ccConsole::Warning(QString("[IcmFilter] Failed to load image %1 (%2)! Process stopped ...").arg(imageFileName).arg(errorStr));
				fclose(fp);
				return loadedImages;
			}

			ccConsole::Print("[IcmFilter] Image '%s' loaded",imageFileName);
            CI->setEnabled(false);
			CI->setName(imageFileName);
            #ifdef INCLUDE_PHOTOS
			CI->setCompleteFileName(totalFileName);
            #endif

			//FOV
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccConsole::Print("[IcmFilter] Read error (fieldOfView)!");
				fclose(fp);
				return loadedImages;
			}
			float fov_rad=0.0;
			sscanf(line,"\t fieldOfView %f\n",&fov_rad);
			//fov *= CC_RAD_TO_DEG*float(CI->getH())/float(CI->getW());
			float fov_deg = fov_rad*(float)CC_RAD_TO_DEG;
			CI->setFov(fov_deg);
			ccConsole::Print("\t FOV=%f (degrees)",fov_deg);

			//Position
			float t[3];
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccConsole::Error("[IcmFilter] Read error (position)!");
				fclose(fp);
				return loadedImages;
			}
			sscanf(line,"\t position %f %f %f\n",t,t+1,t+2);

			ccConsole::Print("\t Camera pos=(%f,%f,%f)",t[0],t[1],t[2]);

			//Description
			char desc[MAX_ASCII_FILE_LINE_LENGTH];
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccConsole::Error("[IcmFilter] Read error (description)!");
				fclose(fp);
				return loadedImages;
			}
			sscanf(line,"\t description \"%s\"\n",desc);

			//CI->setDescription(desc);
			ccConsole::Print("\t Description: '%s'",desc);

			//Orientation
			float axis[3], angle_rad;
			if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
			{
				ccConsole::Error("[IcmFilter] Read error (orientation)!");
				fclose(fp);
				return loadedImages;
			}
			sscanf(line,"\t orientation %f %f %f %f\n",axis,axis+1,axis+2,&angle_rad);

			ccConsole::Print("\t Camera orientation=(%f,%f,%f)+[%f]",axis[0],axis[1],axis[2],angle_rad);
			CI->setCameraMatrix(CCVector3(axis),angle_rad,CCVector3(t));

			entities->addChild(CI);
			++loadedImages;
		}
	}

	fclose(fp);

	return loadedImages;
}
//*/

CC_FILE_ERROR IcmFilter::saveToFile(ccHObject* entity, const char* filename)
{
    ccConsole::Error("Not available yet!");

	return CC_FERR_NO_ERROR;
}
