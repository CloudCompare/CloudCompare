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
#include "PNFilter.h"

//CCLib
#include <CCMiscTools.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

CC_FILE_ERROR PNFilter::saveToFile(ccHObject* entity, const char* filename)
{
	if (!entity || !filename)
        return CC_FERR_BAD_ARGUMENT;

	ccHObject::Container clouds;
	if (entity->isKindOf(CC_POINT_CLOUD))
        clouds.push_back(entity);
    else
        entity->filterChildren(clouds, true, CC_POINT_CLOUD);

    if (clouds.empty())
    {
        ccConsole::Error("No point cloud in input selection!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }
    else if (clouds.size()>1)
    {
        ccConsole::Error("Can't save more than one cloud per PN file!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }

    //the cloud to save
    ccGenericPointCloud* theCloud = static_cast<ccGenericPointCloud*>(clouds[0]);
	unsigned numberOfPoints = theCloud->size();

	if (numberOfPoints==0)
	{
        ccConsole::Error("Cloud is empty!");
        return CC_FERR_BAD_ENTITY_TYPE;
	}

    //open binary file for writing
	FILE* theFile = fopen(filename , "wb");

	if (!theFile)
        return CC_FERR_WRITING;

    //Has the cloud been recentered?
	const double* shift = theCloud->getOriginalShift();
	if (fabs(shift[0])+fabs(shift[0])+fabs(shift[0])>0.0)
        ccConsole::Warning(QString("[PNFilter::save] Can't recenter cloud %1 on PN file save!").arg(theCloud->getName()));

	bool hasNorms = theCloud->hasNormals();
	PointCoordinateType defaultNorm[3] = {0.0,0.0,1.0};

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfPoints);
	pdlg.setMethodTitle("Save PN file");
	char buffer[256];
	sprintf(buffer,"Points: %i",numberOfPoints);
	pdlg.setInfo(buffer);
	pdlg.start();

	float wBuff[3];

	for (unsigned i=0;i<numberOfPoints;i++)
	{
	    //conversion to float
	    const CCVector3* P = theCloud->getPoint(i);
	    wBuff[0]=(float)P->x;
	    wBuff[1]=(float)P->y;
	    wBuff[2]=(float)P->z;

		if (fwrite(wBuff,sizeof(float),3,theFile) < 0)
			{fclose(theFile);return CC_FERR_WRITING;}

		if (hasNorms)
		{
            //conversion to float
            const PointCoordinateType* N = theCloud->getPointNormal(i);
            wBuff[0]=float(N[0]);
            wBuff[1]=float(N[1]);
            wBuff[2]=float(N[2]);

			if (fwrite(wBuff,sizeof(float),3,theFile) < 0)
				{fclose(theFile);return CC_FERR_WRITING;}
		}
		else
		{
			if (fwrite(defaultNorm,sizeof(PointCoordinateType),3,theFile) < 0)
				{fclose(theFile);return CC_FERR_WRITING;}
		}

		if (!nprogress.oneStep())
			break;
	}

	fclose(theFile);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR PNFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	//opening file
	FILE *fp = fopen(filename, "rb");
    if (!fp)
        return CC_FERR_READING;

	CCLib::CCMiscTools::fseek64(fp,0,2);		                //we reach the end of the file
	__int64 fileSize = CCLib::CCMiscTools::ftell64(fp);		//file size query
	CCLib::CCMiscTools::fseek64(fp,0,0);						//back to the begining

    //we read the points number
	unsigned nbOfPoints = (unsigned) (fileSize  / (__int64)(6*sizeof(float)));

	//ccConsole::Print("[PNFilter::loadFile] Points: %i\n",nbOfPoints);

    //if the file is too big, it will be chuncked in multiple parts
	unsigned int fileChunkPos = 0;
	unsigned int fileChunkSize = ccMin(nbOfPoints,(unsigned)CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);

	//new cloud
    char name[64],k=0;
	sprintf(name,"unnamed - Cloud #%i",k);
	ccPointCloud* loadedCloud = new ccPointCloud(name);
    loadedCloud->reserveThePointsTable(fileChunkSize);
    loadedCloud->reserveTheNormsTable();
	loadedCloud->showNormals(true);

    float rBuff[3];
    CCVector3 P;
    PointCoordinateType N[3];

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,nbOfPoints);
	pdlg.setMethodTitle("Open PN file");
	char buffer[256];
	sprintf(buffer,"Points: %u",nbOfPoints);
	pdlg.setInfo(buffer);
	pdlg.start();

    //number of points read from the begining of the current cloud part
    //WARNING: different from lineCounter
	unsigned pointsRead=0;

	for (unsigned i=0;i<nbOfPoints;i++)
	{
        //if we reach the max. cloud size limit, we cerate a new chunk
		if (pointsRead == fileChunkPos+fileChunkSize)
		{
            container.addChild(loadedCloud);
			fileChunkPos = pointsRead;
			fileChunkSize = ccMin(nbOfPoints-pointsRead,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
            sprintf(name,"unnamed - Cloud #%i",++k);
			loadedCloud = new ccPointCloud(name);
            loadedCloud->reserveThePointsTable(fileChunkSize);
            loadedCloud->reserveTheNormsTable();
			loadedCloud->showNormals(true);
		}

        //we read the 3 coordinates of the point
		if (fread(rBuff,4,3,fp)>=0)
		{
		    //conversion to CCVector3
		    P = CCVector3(PointCoordinateType(rBuff[0]),
                          PointCoordinateType(rBuff[1]),
                          PointCoordinateType(rBuff[2]));
			loadedCloud->addPoint(P);
		}
		else
		{
            fclose(fp);
			return CC_FERR_READING;
		}

        //then the 3 composants of the normal vector
		if (fread(rBuff,4,3,fp)>=0)
		{
		    //conversion to PointCoordinateType[3]
		    N[0]=PointCoordinateType(rBuff[0]);
		    N[1]=PointCoordinateType(rBuff[1]);
		    N[2]=PointCoordinateType(rBuff[2]);
			loadedCloud->addNorm(N);
		}
		else
		{
            fclose(fp);
			return CC_FERR_READING;
		}

		++pointsRead;

		if (!nprogress.oneStep())
		{
			loadedCloud->resize(i+1-fileChunkPos);
			break;
		}
    }

    container.addChild(loadedCloud);

    fclose(fp);

	return CC_FERR_NO_ERROR;
}
