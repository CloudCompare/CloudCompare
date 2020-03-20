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

#include "PovFilter.h"

//Local
#include "BinFilter.h"

//qCC_db
#include <ccGBLSensor.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>

//Qt
#include <QFileInfo>

//System
#include <cassert>

//Max number of characters per line in an ASCII file
//TODO: use QFile instead!
const int MAX_ASCII_FILE_LINE_LENGTH = 4096;

//Ground based LiDAR sensor mirror and body rotation order
//Refer to ccGBLSensor::ROTATION_ORDER
const char CC_SENSOR_ROTATION_ORDER_NAMES[][15] = {	"YAW_THEN_PITCH",		//Rotation: body then mirror
													"PITCH_THEN_YAW"			//Rotation: mirror then body
};

//same as CC_SENSOR_ROTATION_ORDER_NAMES but with the old names (used in versions prior to 2.5.6)
const char CC_SENSOR_ROTATION_ORDER_OLD_NAMES[][10] = {	"THETA_PHI",		//Rotation: body then mirror
														"PHI_THETA"			//Rotation: mirror then body
};


PovFilter::PovFilter()
	: FileIOFilter( {
					"_POV Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "pov" },
					"pov",
					QStringList{ "Clouds + sensor info. [meta][ascii] (*.pov)" },
					QStringList{ "Clouds + sensor info. [meta][ascii] (*.pov)" },
					Import | Export
					} )
{
}

bool PovFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POINT_CLOUD)
	{
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR PovFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	Q_UNUSED( parameters );
	
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	ccHObject::Container hClouds;
	entity->filterChildren(hClouds,false,CC_TYPES::POINT_CLOUD);

	if (hClouds.empty())
		return CC_FERR_NO_SAVE;

	std::vector<ccGBLSensor*> sensors;
	std::vector<ccGenericPointCloud*> clouds;
	{
		for (unsigned i=0; i<hClouds.size(); ++i)
		{
			ccHObject::Container cloudSensors;
			hClouds[i]->filterChildren(cloudSensors,false,CC_TYPES::GBL_SENSOR);
			if (!cloudSensors.empty())
			{
				clouds.push_back(ccHObjectCaster::ToGenericPointCloud(hClouds[i]));
				if (cloudSensors.size() > 1)
					ccLog::Warning(QString("Found more than one GBL sensor associated to entity '%1'. Only the first will be saved!").arg(hClouds[i]->getName()));

				sensors.push_back(static_cast<ccGBLSensor*>(cloudSensors[0]));
			}
		}
	}
	assert(sensors.size() == clouds.size());

	if (sensors.empty())
		return CC_FERR_NO_SAVE;

	//FIXME
	//the first GLS sensor will be used as reference! (ugly)
	ccGBLSensor* firstGls = sensors.front();
	if (sensors.size() > 1)
		ccLog::Warning("Assuming all sensors are equivalent...");

	//we extract the body of the filename (without extension)
	QString fullBaseName = QFileInfo(filename).completeBaseName();

	//main file (.POV)
	FILE* mainFile = fopen(qPrintable(filename),"wt");
	if (!mainFile)
		return CC_FERR_WRITING;

	if (	fprintf(mainFile,"#CC_POVS_FILE\n") < 0
		||	fprintf(mainFile,"SENSOR_TYPE = %s\n",CC_SENSOR_ROTATION_ORDER_NAMES[firstGls->getRotationOrder()]) < 0
		||	fprintf(mainFile,"SENSOR_BASE = 0\n") < 0 //DGM: sensor base is deprecated
		||	fprintf(mainFile,"UNITS = IGNORED\n") < 0
		||	fprintf(mainFile,"#END_HEADER\n") < 0 )
	{
		fclose(mainFile);
		return CC_FERR_WRITING;
	}

	//save sensor(s) info
	{
		for (unsigned i=0; i<clouds.size(); ++i)
		{
			QString thisFilename = fullBaseName + QString("_%1.bin").arg(i);

			BinFilter::SaveParameters parameters;
			{
				parameters.alwaysDisplaySaveDialog = false;
			}
			CC_FILE_ERROR error = FileIOFilter::SaveToFile(clouds[i],thisFilename,parameters,BinFilter::GetFileFilter());
			if (error != CC_FERR_NO_ERROR)
			{
				fclose(mainFile);
				return error;
			}

			//il faut ecrire le nom du fichier relatif et non absolu !
			int result = fprintf(mainFile,"\n#POV %u\nF %s\nT ASC\n",i,qPrintable(QFileInfo(thisFilename).fileName()));

			if (result > 0)
			{
				ccGBLSensor* gls = sensors[i];
				const float* C = gls->getRigidTransformation().getTranslation();
				result = fprintf(mainFile,"C %f %f %f\n",C[0],C[1],C[2]);

				if (result > 0)
				{
					const float* mat = gls->getRigidTransformation().data();
					result = fprintf(mainFile,"X %f %f %f\n",mat[0],mat[1],mat[2]);
					result = fprintf(mainFile,"Y %f %f %f\n",mat[4],mat[5],mat[6]);
					result = fprintf(mainFile,"Z %f %f %f\n",mat[8],mat[9],mat[10]);
				}

				if (result > 0)
					result = fprintf(mainFile,"A %f %f\n",gls->getYawStep(),gls->getPitchStep());

				if (result > 0)
					result = fprintf(mainFile,"#END_POV\n");
			}

			//if (++n == palier)
			//{
			//	//cancel requested
			//	if (pwin->isCancelRequested())
			//		result = -1;

			//	percent += 1.0;
			//	pwin->update(percent);
			//	n = 0;
			//}
		}
	}

	//delete pwin;

	fclose(mainFile);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR PovFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	assert(!filename.isEmpty());

	//opening file
	FILE* fp = fopen(qPrintable(filename), "rt");
	if (!fp)
		return CC_FERR_READING;

	//read buffer
	char line[MAX_ASCII_FILE_LINE_LENGTH];

	//header
	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH, fp))
	{
		fclose(fp);
		return CC_FERR_READING;
	}

	if (strcmp(line,"#CC_POVS_FILE\n")!=0)
	{
		fclose(fp);
		return CC_FERR_READING;
	}

	char sensorType[256];
	if (fscanf(fp,"SENSOR_TYPE = %s\n",sensorType) < 0)
	{
		fclose(fp);
		return CC_FERR_READING;
	}

	ccGBLSensor::ROTATION_ORDER rotationOrder;
	if (	strcmp(sensorType,CC_SENSOR_ROTATION_ORDER_NAMES[ccGBLSensor::YAW_THEN_PITCH]) == 0
			||	strcmp(sensorType,CC_SENSOR_ROTATION_ORDER_OLD_NAMES[ccGBLSensor::YAW_THEN_PITCH]) == 0)
	{
		rotationOrder = ccGBLSensor::YAW_THEN_PITCH;
	}
	else if (	strcmp(sensorType,CC_SENSOR_ROTATION_ORDER_NAMES[ccGBLSensor::PITCH_THEN_YAW]) == 0
		||	strcmp(sensorType,CC_SENSOR_ROTATION_ORDER_OLD_NAMES[ccGBLSensor::PITCH_THEN_YAW]) == 0)
	{
		rotationOrder = ccGBLSensor::PITCH_THEN_YAW;
	}
	else
	{
		ccLog::Warning("[PovFilter::loadFile] Unhandled rotation order description! (%s)",sensorType);
		fclose(fp);
		return CC_FERR_READING;
	}

	float base = 0.0f;
	char unitsType[3]; //units: ignored in this version
	if (	fscanf(fp,"SENSOR_BASE = %f\n",&base) < 0
		||	fscanf(fp,"UNITS = %s\n",unitsType) < 0
		||	!fgets(line, MAX_ASCII_FILE_LINE_LENGTH, fp)
		||	strcmp(line,"#END_HEADER\n") != 0 )
	{
		fclose(fp);
		return CC_FERR_READING;
	}

	ccLog::Print("[PovFilter::loadFile] POV FILE [Type %s - base=%f - unit: %s]",sensorType,base,unitsType);

	//on extrait le chemin relatif
	QString path = QFileInfo(filename).absolutePath();

	char subFileName[256];
	char subFileType[12];

	while (fgets(line, MAX_ASCII_FILE_LINE_LENGTH, fp))
	{
		if ((line[0] == '#') && (line[1] == 'P'))
		{
			ccLog::Print(QString(line).trimmed());
			if (fscanf(fp, "F %s\n", subFileName) < 0)
			{
				ccLog::PrintDebug("[PovFilter::loadFile] Read error (F) !");
				fclose(fp);
				return CC_FERR_READING;
			}
			if (fscanf(fp, "T %s\n", subFileType) < 0)
			{
				ccLog::PrintDebug("[PovFilter::loadFile] Read error (T) !");
				fclose(fp);
				return CC_FERR_READING;
			}

			//chargement du fichier (potentiellement plusieurs listes) correspondant au point de vue en cours
			FileIOFilter::Shared filter = FileIOFilter::FindBestFilterForExtension(subFileType);
			if (!filter)
			{
				ccLog::Warning(QString("[POV] No I/O filter found for loading file '%1' (type = '%2')").arg(subFileName, subFileType));
				fclose(fp);
				return CC_FERR_UNKNOWN_FILE;
			}

			CC_FILE_ERROR result = CC_FERR_NO_ERROR;
			ccHObject* entities = FileIOFilter::LoadFromFile(QString("%1/%2").arg(path, subFileName), parameters, filter, result);
			if (entities)
			{
				ccGLMatrix rot;
				rot.toIdentity();
				CCVector3 sensorCenter(0, 0, 0);
				float dPhi = 1.0f;
				float dTheta = 1.0f;

				while (fgets(line, MAX_ASCII_FILE_LINE_LENGTH, fp))
				{
					if (line[0] == '#')
						break;
					else if (line[0] == 'C')
					{
						float C[3];
						sscanf(line, "C %f %f %f\n", C, C + 1, C + 2);
						sensorCenter = CCVector3::fromArray(C);
					}
					else if (line[0] == 'X' || line[0] == 'Y' || line[0] == 'Z')
					{
						float V[3];
						sscanf(line + 2, "%f %f %f\n", V, V + 1, V + 2);

						unsigned char col = static_cast<unsigned char>(line[0]) - 88;
						float* mat = rot.data();
						mat[col + 0] = V[0];
						mat[col + 4] = V[1];
						mat[col + 8] = V[2];
					}
					else if (line[0] == 'A')
					{
						sscanf(line, "A %f %f\n", &dTheta, &dPhi);
					}
				}

				ccHObject::Container clouds;
				if (entities->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					clouds.push_back(entities);
				}
				else
				{
					entities->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
					entities->detatchAllChildren();
					delete entities;
				}
				entities = nullptr;

				for (size_t i = 0; i < clouds.size(); ++i)
				{
					ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(clouds[i]);

					ccGBLSensor* gls = new ccGBLSensor(rotationOrder);
					//DGM: the base simply corresponds to a shift of the center along the X axis!
					sensorCenter.x -= base;
					//DGM: sensor center is now integrated in rigid transformation (= inverse of former rotation matrix + center as translation)
					ccGLMatrix trans = rot.inverse();
					trans.setTranslation(sensorCenter);
					gls->setRigidTransformation(trans);
					gls->setYawStep(dTheta);
					gls->setPitchStep(dPhi);
					gls->setVisible(true);
					gls->setEnabled(false);

					if (gls->computeAutoParameters(theCloud))
					{
						theCloud->addChild(gls);
					}
					else
					{
						ccLog::Warning(QString("[PovFilter::loadFile] failed to create sensor on cloud #%1 (%2)").arg(i).arg(theCloud->getName()));
						delete gls;
						gls = nullptr;
					}

					//theCloud->setName(subFileName);
					container.addChild(theCloud);
				}
			}
			else
			{
				if (result == CC_FERR_CANCELED_BY_USER)
				{
					break;
				}
				else
				{
					ccLog::Print("[PovFilter::loadFile] File (%s) not found or empty!", subFileName);
				}
			}
		}
	}

	fclose(fp);

	return CC_FERR_NO_ERROR;
}
