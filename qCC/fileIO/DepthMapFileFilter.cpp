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
#include "DepthMapFileFilter.h"

//qCC_db
#include <ccGBLSensor.h>
#include <ccPointCloud.h>

//CCLib
#include <CCMiscTools.h>

#include <assert.h>

CC_FILE_ERROR DepthMapFileFilter::saveToFile(ccHObject* entity, const char* filename)
{
    if (!entity || !filename)
        return CC_FERR_BAD_ARGUMENT;

    ccHObject::Container sensors;
    if (entity->isKindOf(CC_GBL_SENSOR))
        sensors.push_back(entity);
    else
        entity->filterChildren(sensors, true, CC_GBL_SENSOR);

    if (sensors.empty())
    {
        ccConsole::Error("No sensor in input selection!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }

    //multiple filenames handling
    int ppos = CCLib::CCMiscTools::findCharLastOccurence('.',filename);
    char fName[1024];
    strcpy(fName,filename);

    unsigned i=0,nSens=sensors.size();
    CC_FILE_ERROR result = CC_FERR_NO_ERROR;

    while (result == CC_FERR_NO_ERROR && i<nSens)
    {
        //more than one sensor? we must generate auto filename
        if (nSens>1)
        {
            if (ppos>=0)
            {
                char fbody[256];
                strncpy(fbody,filename,ppos);
                fbody[ppos]=0;
                sprintf(fName,"%s_%i.%s",fbody,i,filename+ppos+1);
            }
            else
            {
                sprintf(fName,"%s_%i",filename,i);
            }
        }

        //opening file
        FILE* fp = fopen(fName,"wt");
        if (!fp)
        {
            ccConsole::Error("[ccGBLSensor::saveASCII] Can't open file '%s' for writing!",fName);
            result=CC_FERR_WRITING;
        }
        else
        {
            ccGBLSensor* sensor = static_cast<ccGBLSensor*>(sensors[i]);
            result = saveToOpenedFile(fp,sensor);
			fclose(fp);
        }

        ++i;
    }

    return result;
}

CC_FILE_ERROR DepthMapFileFilter::saveToOpenedFile(FILE* fp, ccGBLSensor* sensor)
{
    assert(fp && sensor);

    if (!sensor->getParent()->isKindOf(CC_POINT_CLOUD))
    {
        ccConsole::Warning(QString("Warning: sensor '%1' is not associated to a point cloud!").arg(sensor->getName()));
        return CC_FERR_NO_ERROR; //this is not a severe error (the process can go on)
    }

    ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(sensor->getParent());

    //the depth map associated to this sensor
    CCLib::GroundBasedLidarSensor::DepthBuffer db = sensor->getDepthBuffer();

    fprintf(fp,"// CLOUDCOMPARE DEPTH MAP\n");
    fprintf(fp,"// Associated cloud: %s\n",qPrintable(cloud->getName()));
    fprintf(fp,"// dPhi   = %f [ %f : %f ]\n",sensor->getDeltaPhi(),
            sensor->getPhiMin(),
            sensor->getPhiMax());
    fprintf(fp,"// dTheta = %f [ %f : %f ]\n",sensor->getDeltaTheta(),
            sensor->getThetaMin(),
            sensor->getThetaMax());
    fprintf(fp,"// pMax   = %f\n",sensor->getSensorRange());
    fprintf(fp,"// L      = %i\n",db.l_buff);
    fprintf(fp,"// H      = %i\n",db.h_buff);
    fprintf(fp,"/////////////////////////\n");

    //an array of projected normals (same size à depth map)
    PointCoordinateType* theNorms = NULL;
    //an array of projected colors (same size à depth map)
    uchar* theColors = NULL;

    //if the sensor is associated to a "ccPointCloud", we may also extract
    //normals and color!
    if (cloud->isA(CC_POINT_CLOUD))
    {
        ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

        unsigned nbPoints = cloud->size();
        if (nbPoints == 0)
        {
            ccConsole::Warning(QString("Warning: sensor '%1' is associated to an empty cloud!").arg(sensor->getName()));
            return CC_FERR_NO_ERROR; //this is not a severe error (the process can go on)
        }
        else
        {
            //if possible, we create the array of projected normals
            if (pc->hasNormals())
            {
                NormsTableType* decodedNorms = new NormsTableType;
                decodedNorms->reserve(nbPoints);

                for (unsigned i=0;i<nbPoints;++i)
                    decodedNorms->addElement(pc->getPointNormal(i));

                theNorms = sensor->projectNormals(pc,*decodedNorms);
                decodedNorms->clear();
				decodedNorms->release();
				decodedNorms=0;
            }

            //if possible, we create the array of projected colors
            if (pc->hasColors())
            {
                GenericChunkedArray<3,uchar>* rgbColors = new GenericChunkedArray<3,uchar>();
                uchar rgb[3];
                rgbColors->reserve(nbPoints);

                for (unsigned i=0;i<nbPoints;++i)
                {
                    //conversion from colorType[3] to uchar[3]
                    const colorType* col = pc->getPointColor(i);
                    rgb[0]=uchar(col[0]);
                    rgb[1]=uchar(col[1]);
                    rgb[2]=uchar(col[2]);
                    rgbColors->addElement(rgb);
                }

                theColors = sensor->projectColors(pc,*rgbColors);
                rgbColors->clear();
				rgbColors->release();
				rgbColors=0;
            }
        }
    }

    PointCoordinateType* _theNorms = theNorms;
    uchar* _theColors = theColors;
    DistanceType* _zBuff = db.zBuff;

    int j,k;
    for (j=0;j<db.h_buff;++j)
        for (k=0;k<db.l_buff;++k)
        {
            //grid index and depth
            fprintf(fp,"%f %f %f",float(k),float(j),*_zBuff++);

            //color
            if (_theColors)
            {
                fprintf(fp," %i %i %i",_theColors[0],_theColors[1],_theColors[2]);
                _theColors+=3;
            }

            //normal
            if (_theNorms)
            {
                fprintf(fp," %f %f %f",_theNorms[0],_theNorms[1],_theNorms[2]);
                _theNorms+=3;
            }

            fprintf(fp,"\n");
        }

    if (theNorms)
        delete[] theNorms;
    if (theColors)
        delete[] theColors;

    return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR DepthMapFileFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
    ccConsole::Error("Not available yet!\n");

    return CC_FERR_NO_ERROR;
}
