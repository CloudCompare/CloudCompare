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
#include "PVFilter.h"

//CCLib
#include <CCMiscTools.h>
#include <ScalarField.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

CC_FILE_ERROR PVFilter::saveToFile(ccHObject* entity, const char* filename)
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
        ccConsole::Error("Can't save more than one cloud per PV file!");
        return CC_FERR_BAD_ENTITY_TYPE;
    }

    //the cloud to save
    ccGenericPointCloud* theCloud = static_cast<ccGenericPointCloud*>(clouds[0]);
    //and its scalar field
	CCLib::ScalarField* sf = 0;
	if (theCloud->isA(CC_POINT_CLOUD))
	    sf = static_cast<ccPointCloud*>(theCloud)->getCurrentDisplayedScalarField();

    if (!sf)
        ccConsole::Warning("No displayed scalar field! Values will all be 0!\n");

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
        ccConsole::Warning(QString("[PVFilter::save] Can't recenter cloud %1 on PV file save!").arg(theCloud->getName()));

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfPoints);
	pdlg.setMethodTitle("Save PV file");
	char buffer[256];
	sprintf(buffer,"Points: %i",numberOfPoints);
	pdlg.setInfo(buffer);
	pdlg.start();

	float wBuff[3];
	float val=0.0;

	for (unsigned i=0;i<numberOfPoints;i++)
	{
	    //conversion to float
	    const CCVector3* P = theCloud->getPoint(i);
	    wBuff[0]=float(P->x);
	    wBuff[1]=float(P->y);
	    wBuff[2]=float(P->z);

		if (fwrite(wBuff,sizeof(float),3,theFile) < 0)
			{fclose(theFile);return CC_FERR_WRITING;}

		if (sf)
            val = (float)sf->getValue(i);

        if (fwrite(&val,sizeof(float),1,theFile) < 0)
            {fclose(theFile);return CC_FERR_WRITING;}

		if (!nprogress.oneStep())
			break;
	}

	fclose(theFile);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR PVFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	//ccConsole::Print("[PVFilter::loadFile] Opening binary file '%s'...\n",filename);

	//opening file
	FILE *fp = fopen(filename, "rb");
    if (!fp)
        return CC_FERR_NO_ERROR;

	CCLib::CCMiscTools::fseek64(fp,0,2);		                //we reach the end of the file
	__int64 fileSize = CCLib::CCMiscTools::ftell64(fp);		//file size query
	CCLib::CCMiscTools::fseek64(fp,0,0);						//back to the begining

    //we read the points number
	unsigned nbOfPoints = (unsigned) (fileSize  / (__int64)(4*sizeof(float)));

	//ccConsole::Print("[PVFilter::loadFile] Points: %i\n",nbOfPoints);

    //if the file is too big, it will be chuncked in multiple parts
	unsigned fileChunkPos = 0;
	unsigned fileChunkSize = ccMin(nbOfPoints,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);

	//new cloud
    char name[64],k=0;
	sprintf(name,"unnamed - Cloud #%i",k);
	ccPointCloud* loadedCloud = new ccPointCloud(name);
    loadedCloud->reserveThePointsTable(fileChunkSize);
	loadedCloud->enableScalarField();

    float rBuff[3];

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,nbOfPoints);
	pdlg.setMethodTitle("Open PV file");
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
			loadedCloud->getCurrentInScalarField()->computeMinAndMax();
			int sfIdx = loadedCloud->getCurrentInScalarFieldIndex();
			loadedCloud->setCurrentDisplayedScalarField(sfIdx);
			loadedCloud->showSF(sfIdx>=0);
            container.addChild(loadedCloud);
			fileChunkPos = pointsRead;
			fileChunkSize = ccMin(nbOfPoints-pointsRead,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
            sprintf(name,"unnamed - Cloud #%i",++k);
			loadedCloud = new ccPointCloud(name);
            loadedCloud->reserveThePointsTable(fileChunkSize);
			loadedCloud->enableScalarField();
		}

        //we read the 3 coordinates of the point
		if (fread(rBuff,4,3,fp)>=0)
		{
		    //conversion to CCVector3
		    CCVector3 P = CCVector3(PointCoordinateType(rBuff[0]),
                                    PointCoordinateType(rBuff[1]),
                                    PointCoordinateType(rBuff[2]));
			loadedCloud->addPoint(P);
		}
		else
		{
            fclose(fp);
			return CC_FERR_NO_ERROR;
		}

        //then the scalar value
		if (fread(rBuff,4,1,fp)>=0)
		{
		    //conversion to DistanceType
		    DistanceType d = DistanceType(rBuff[0]);
		    loadedCloud->setPointScalarValue(pointsRead,d);
		}
		else
		{
            fclose(fp);
			return CC_FERR_NO_ERROR;
		}

		++pointsRead;

		if (!nprogress.oneStep())
		{
			loadedCloud->resize(i+1-fileChunkPos);
			break;
		}
    }

    loadedCloud->getCurrentInScalarField()->computeMinAndMax();
	int sfIdx = loadedCloud->getCurrentInScalarFieldIndex();
	loadedCloud->setCurrentDisplayedScalarField(sfIdx);
	loadedCloud->showSF(sfIdx>=0);

    container.addChild(loadedCloud);

    fclose(fp);

	return CC_FERR_NO_ERROR;
}
