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
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//
#include "PovFilter.h"

//CCLib
#include <CCMiscTools.h>
#include <SimpleCloud.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccGBLSensor.h>

//Qt
#include <QFileInfo>

//System
#include <assert.h>

#include "../ccConsole.h"

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
            clouds.push_back(static_cast<ccGenericPointCloud*>(hClouds[i]));
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
    int pointPos = CCLib::CCMiscTools::findCharLastOccurence('.',filename);
    if (pointPos<0)
        pointPos = strlen(filename);

    //main file (.POV)
    FILE* mainFile = fopen(filename,"wt");
    if (!mainFile)
        return CC_FERR_WRITING;

    if (fprintf(mainFile,"#CC_POVS_FILE\n")<0)
    {
        fclose(mainFile);
        return CC_FERR_WRITING;
    };
    if (fprintf(mainFile,"SENSOR_TYPE = %s\n",CCLib::CC_SENSOR_ROTATION_ORDER_NAMES[firstGls->getRotationOrder()])<0)
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

    //the files corresponding to the different points of view
    char baseFileName[1024];
    memcpy(baseFileName,filename,pointPos);
    baseFileName[pointPos]=0;

    for (i=0;i<clouds.size();++i)
    {
        char suffix[256];
        char baseNameCpy[256];
        strcpy(baseNameCpy,baseFileName);
        sprintf(suffix,"_%i.bin",i);
        strcat(baseNameCpy,suffix);
        /*FILE* f = fopen(baseNameCpy,"wt");

        if (!f)
        {
        	char buffer[1024];
        	ccConsole::Error("Couldn't create file (%s)!",baseNameCpy);
        	return CC_FERR_WRITING;
        }
        //*/

        //avancement du chargement
        /*char title[256];
        sprintf(title,"Saving list #%i POVs",aList->getListID());
        ProgressWindow* pwin = new ProgressWindow(title,true);
        float percent = 0.0;
        unsigned palier = unsigned(float(numberOfPoints) * 0.01); //1% du total
        unsigned n = 0;
        char buffer[256];
        sprintf(buffer,"Number of points = %i\nNumber of POV = %i",numberOfPoints,maxNumberOfScans);
        pwin->setMessage(buffer);
        scanIndexType theCurrentScanIndex=0;
        CCVector3* P;
        //*/

        CC_FILE_ERROR error = FileIOFilter::SaveToFile(clouds[i],baseNameCpy,BIN);
        if (error != CC_FERR_NO_ERROR)
        {
            fclose(mainFile);
            return error;
        }

        ccGBLSensor* gls = sensors[i];

        //il faut écrire le nom du fichier relatif et non absolu !
        int p = CCLib::CCMiscTools::findCharLastOccurence('/',baseNameCpy);
        int slashPos = ccMax(0,p);
        p = CCLib::CCMiscTools::length(baseNameCpy)-slashPos-1;
        int size = ccMax(0,p);
        char povFileName[1024];
        memcpy(povFileName,baseNameCpy+slashPos+1,size);
        povFileName[size]=0;
        int result = fprintf(mainFile,"\n#POV %i\nF %s\nT ASC\n",i,povFileName);

        if (result>0)
        {
            CCVector3 C = gls->getSensorCenter();
            result = fprintf(mainFile,"C %f %f %f\n",C[0],C[1],C[2]);

            CCLib::SquareMatrix* m = gls->getAxisMatrix();
            if (m && result>0)
            {
                result = fprintf(mainFile,"X %f %f %f\n",m->m_values[0][0],m->m_values[1][0],m->m_values[2][0]);
                result = fprintf(mainFile,"Y %f %f %f\n",m->m_values[0][1],m->m_values[1][1],m->m_values[2][1]);
                result = fprintf(mainFile,"Z %f %f %f\n",m->m_values[0][2],m->m_values[1][2],m->m_values[2][2]);
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

    CCLib::CC_SENSOR_ROTATION_ORDER rotationOrder;
    if (strcmp(sensorType,"PHI_THETA")==0)
        rotationOrder = CCLib::GBL_PHI_THETA;
    else if (strcmp(sensorType,"THETA_PHI")==0)
        rotationOrder = CCLib::GBL_THETA_PHI;
    else
    {
        fclose(fp);
        return CC_FERR_READING;
    }

    float base=0.0;
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

                CCLib::SquareMatrix* m = 0;

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
                    else if ((line[0]=='X')||(line[0]=='Y')||(line[0]=='Z'))
                    {
                        float X[3];
                        sscanf(line+2,"%f %f %f\n",X,X+1,X+2);
                        if (!m)
                        {
                            m = new CCLib::SquareMatrix(3);
                            m->toIdentity();
                        }
                        uchar col = uchar(line[0])-88;
                        m->setValue(0,col,X[0]);
                        m->setValue(1,col,X[1]);
                        m->setValue(2,col,X[2]);
                    }
                    else if (line[0]=='A')
                    {
                        float dPhi,dTheta;
                        sscanf(line,"A %f %f\n",&dPhi,&dTheta);
                        gls->setDeltaPhi(dPhi);
                        gls->setDeltaTheta(dTheta);
                    }
                }

                if (m) gls->setAxisMatrix(m);

                int errorCode;
                ccHObject::Container clouds;
                if (loadedLists->isKindOf(CC_POINT_CLOUD))
                    clouds.push_back(loadedLists);
                else
                    loadedLists->filterChildren(clouds,true,CC_POINT_CLOUD);

                for (unsigned i=0;i<clouds.size();++i)
                {
                    ccGenericPointCloud* theCloud = static_cast<ccGenericPointCloud*>(clouds[i]);
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
