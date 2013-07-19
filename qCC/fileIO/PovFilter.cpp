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

#include "PovFilter.h"

//CCLib
#include <SimpleCloud.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccGBLSensor.h>

//Qt
#include <QFileInfo>

//System
#include <assert.h>

#include "../ccConsole.h"

//Ground based LiDAR sensor mirror and body rotation order
//Refer to ccGBLSensor::ROTATION_ORDER
const char CC_SENSOR_ROTATION_ORDER_NAMES[][12] = {
	"THETA_PHI",		//Rotation: body then mirror
	"PHI_THETA"			//Rotation: mirror then body
};

CC_FILE_ERROR PovFilter::saveToFile(ccHObject* entity, const char* filename)
{
    if (!entity || !filename)
        return CC_FERR_BAD_ARGUMENT;

    ccHObject::Container hClouds;
    entity->filterChildren(hClouds,false,CC_POINT_CLOUD);

    if (hClouds.empty())
        return CC_FERR_NO_SAVE;

    unsigned i;

    std::vector<ccGBLSensor*> sensors;
    std::vector<ccGenericPointCloud*> clouds;
    for (i=0;hClouds.size();++i)
    {
        ccHObject::Container cloudSensors;
        hClouds[i]->filterChildren(cloudSensors,false,CC_GBL_SENSOR);
        if (!cloudSensors.empty())
        {
            clouds.push_back(ccHObjectCaster::ToGenericPointCloud(hClouds[i]));
            if (cloudSensors.size()>1)
                ccConsole::Warning(QString("Found more than one ground-based LIDAR sensor associated to entity '%1'. Only the first will be saved!").arg(hClouds[i]->getName()));

            sensors.push_back(static_cast<ccGBLSensor*>(cloudSensors[0]));
        }
    }
    assert(sensors.size() == clouds.size());

    if (sensors.empty())
        return CC_FERR_NO_SAVE;

    //FIXME
    //the first GLS sensor will be used as reference! (ugly)
    ccGBLSensor* firstGls = sensors.front();
    if (sensors.size()>1)
        ccConsole::Warning("Assuming all sensors are equivalent...");

    //we extract the body of the filename (without extension)
	QString fullBaseName = QFileInfo(filename).completeBaseName();

    //main file (.POV)
    FILE* mainFile = fopen(filename,"wt");
    if (!mainFile)
        return CC_FERR_WRITING;

    if (fprintf(mainFile,"#CC_POVS_FILE\n")<0)
    {
        fclose(mainFile);
        return CC_FERR_WRITING;
    };
    if (fprintf(mainFile,"SENSOR_TYPE = %s\n",CC_SENSOR_ROTATION_ORDER_NAMES[firstGls->getRotationOrder()])<0)
    {
        fclose(mainFile);
        return CC_FERR_WRITING;
    };
    if (fprintf(mainFile,"SENSOR_BASE = %f\n",firstGls->getSensorBase())<0)
    {
        fclose(mainFile);
        return CC_FERR_WRITING;
    };
    if (fprintf(mainFile,"UNITS = IGNORED\n")<0)
    {
        fclose(mainFile);
        return CC_FERR_WRITING;
    };
    if (fprintf(mainFile,"#END_HEADER\n")<0)
    {
        fclose(mainFile);
        return CC_FERR_WRITING;
    };

    for (i=0;i<clouds.size();++i)
    {
		QString thisFilename = fullBaseName + QString("_%1.bin").arg(i);

        CC_FILE_ERROR error = FileIOFilter::SaveToFile(clouds[i],qPrintable(thisFilename),BIN);
        if (error != CC_FERR_NO_ERROR)
        {
            fclose(mainFile);
            return error;
        }

        ccGBLSensor* gls = sensors[i];

        //il faut ecrire le nom du fichier relatif et non absolu !
		int result = fprintf(mainFile,"\n#POV %i\nF %s\nT ASC\n",i,qPrintable(QFileInfo(thisFilename).fileName()));

        if (result>0)
        {
            CCVector3 C = gls->getSensorCenter();
            result = fprintf(mainFile,"C %f %f %f\n",C[0],C[1],C[2]);

            if (result>0)
			{
				const float* mat = gls->getOrientationMatrix().data();
                result = fprintf(mainFile,"X %f %f %f\n",mat[0],mat[4],mat[8]);
                result = fprintf(mainFile,"Y %f %f %f\n",mat[1],mat[5],mat[9]);
                result = fprintf(mainFile,"Z %f %f %f\n",mat[2],mat[6],mat[10]);
            }

            if (result>0)
                result = fprintf(mainFile,"A %f %f\n",gls->getDeltaPhi(),gls->getDeltaTheta());

            if (result>0)
                result = fprintf(mainFile,"#END_POV\n");
        }

        /*if (++n==palier)
        {
            //cancel requested
            if (pwin->isCancelRequested())
                result=-1;

            percent += 1.0;
            pwin->update(percent);
            n = 0;
        }
        //*/
    }

    //delete pwin;


    fclose(mainFile);

    return CC_FERR_NO_ERROR;

}

CC_FILE_ERROR PovFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
    assert(!filename);

    //opening file
    FILE* fp = fopen(filename, "rt");
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

    char sensorType[12];
    if (fscanf(fp,"SENSOR_TYPE = %s\n",sensorType)<0)
    {
        fclose(fp);
        return CC_FERR_READING;
    }

	ccGBLSensor::ROTATION_ORDER rotationOrder;
    if (strcmp(sensorType,CC_SENSOR_ROTATION_ORDER_NAMES[ccGBLSensor::PHI_THETA])==0)
        rotationOrder = ccGBLSensor::PHI_THETA;
    else if (strcmp(sensorType,CC_SENSOR_ROTATION_ORDER_NAMES[ccGBLSensor::THETA_PHI])==0)
        rotationOrder = ccGBLSensor::THETA_PHI;
    else
    {
        fclose(fp);
        return CC_FERR_READING;
    }

    float base=0.0f;
    if (fscanf(fp,"SENSOR_BASE = %f\n",&base)<0)
    {
        fclose(fp);
        return CC_FERR_READING;
    }

    //units: ignored in this version
    char unitsType[3];
    if (fscanf(fp,"UNITS = %s\n",unitsType)<0)
    {
        fclose(fp);
        return CC_FERR_READING;
    }

    if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH, fp))
    {
        fclose(fp);
        return CC_FERR_READING;
    }

    if (strcmp(line,"#END_HEADER\n")!=0)
    {
        fclose(fp);
        return CC_FERR_READING;
    }

    ccConsole::Print("[PovFilter::loadFile] POV FILE [Type %s - base=%f - unit: %s}\n",sensorType,base,unitsType);

    //on extrait le chemin relatif
	QString path = QFileInfo(filename).absolutePath();

    char subFileName[256];
    char subFileType[12];

    while (fgets(line, MAX_ASCII_FILE_LINE_LENGTH, fp))
    {
        if ((line[0]=='#')&&(line[1]=='P'))
        {
            ccConsole::Print("%s",line);
            if (fscanf(fp,"F %s\n",subFileName)<0)
            {
                ccConsole::PrintDebug("[PovFilter::loadFile] Read error (F) !\n");
                fclose(fp);
                return CC_FERR_READING;
            }
            if (fscanf(fp,"T %s\n",subFileType)<0)
            {
                ccConsole::PrintDebug("[PovFilter::loadFile] Read error (T) !\n");
                fclose(fp);
                return CC_FERR_READING;
            }

            //chargement du fichier (potentiellement plusieurs listes) correspondant au point de vue en cours
            CC_FILE_TYPES fType = FileIOFilter::StringToFileFormat(subFileType);
            ccHObject* loadedLists = FileIOFilter::LoadFromFile(qPrintable(QString("%0/%1").arg(path).arg(subFileName)),fType);

            if (loadedLists)
            {
                ccGBLSensor* gls = new ccGBLSensor(rotationOrder);

                //ne pas oublier la base du scanner (SOISIC)
                gls->setSensorBase(base);

                ccGLMatrix rot;
				rot.toIdentity();

                while (fgets(line, MAX_ASCII_FILE_LINE_LENGTH, fp))
                {
                    if (line[0]=='#')
                        break;
                    else if (line[0]=='C')
                    {
                        float C[3];
                        sscanf(line,"C %f %f %f\n",C,C+1,C+2);
                        gls->setSensorCenter(C);
                    }
                    else if (line[0]=='X' || line[0]=='Y' || line[0]=='Z')
                    {
						float V[3];
                        sscanf(line+2,"%f %f %f\n",V,V+1,V+2);

						uchar col = uchar(line[0])-88;
						float* mat = rot.data();
						mat[col+0] = V[0];
						mat[col+4] = V[1];
						mat[col+8] = V[2];
                    }
                    else if (line[0]=='A')
                    {
                        float dPhi,dTheta;
                        sscanf(line,"A %f %f\n",&dPhi,&dTheta);
                        gls->setDeltaPhi(dPhi);
                        gls->setDeltaTheta(dTheta);
                    }
                }

				gls->setOrientationMatrix(rot);

                int errorCode;
                ccHObject::Container clouds;
                if (loadedLists->isKindOf(CC_POINT_CLOUD))
                    clouds.push_back(loadedLists);
                else
                    loadedLists->filterChildren(clouds,true,CC_POINT_CLOUD);

                for (unsigned i=0;i<clouds.size();++i)
                {
                    ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(clouds[i]);
                    CCLib::GenericIndexedCloud* projectedList = gls->project(theCloud,errorCode,true);

                    switch (errorCode)
                    {
                    case -1:
                        ccConsole::Print(QString("[PovFilter::loadFile] Error on cloud #%1 (%2): nothing to project?! Must be a bug, sorry ;)").arg(i).arg(theCloud->getName()));
                        break;
                    case -2:
                        ccConsole::Print(QString("[PovFilter::loadFile] Error on cloud #%1 (%2): the resulting depth map seems much too big! Check parameters, or reduce angular steps ...").arg(i).arg(theCloud->getName()));
                        break;
                    case -3:
                        ccConsole::Print(QString("[PovFilter::loadFile] Error on cloud #%1 (%2): the resulting depth map is void (too small)! Check parameters and input, or increase angular steps ...").arg(i).arg(theCloud->getName()));
                        break;
                    case -4:
                        ccConsole::Print(QString("[PovFilter::loadFile] Error on cloud #%1 (%2): not enough memory!").arg(i).arg(theCloud->getName()));
                        break;
                    }

                    if (projectedList)
                    {
                        delete projectedList;
                        projectedList=0;
                        theCloud->addChild(gls);
                    }
                    else
                    {
                        delete gls;
                        gls=0;
                    }

                    theCloud->setName(subFileName);
                    container.addChild(theCloud);
                }
            }
            else
			{
				ccConsole::Print("[PovFilter::loadFile] File (%s) not found or empty!\n",subFileName);
			}
        }
    }

    fclose(fp);

    return CC_FERR_NO_ERROR;
}
