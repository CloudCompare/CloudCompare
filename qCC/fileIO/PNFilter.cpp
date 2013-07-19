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

#include "PNFilter.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Qt
#include <QFile>

//default normal value
static const PointCoordinateType s_defaultNorm[3] = {0,0,1};

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
    ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(clouds[0]);
	unsigned numberOfPoints = theCloud->size();

	if (numberOfPoints==0)
	{
        ccConsole::Error("Cloud is empty!");
        return CC_FERR_BAD_ENTITY_TYPE;
	}

    //open binary file for writing
	QFile out(filename);
	if (!out.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

    //Has the cloud been recentered?
	const double* shift = theCloud->getOriginalShift();
	if (fabs(shift[0])+fabs(shift[0])+fabs(shift[0])>0.0)
        ccConsole::Warning(QString("[PNFilter::save] Can't recenter cloud '%1' when saving it in a PN file!").arg(theCloud->getName()));

	bool hasNorms = theCloud->hasNormals();
	if (!hasNorms)
        ccConsole::Warning(QString("[PNFilter::save] Cloud '%1' has no normal (we will save points with a default normal)!").arg(theCloud->getName()));
	float norm[3] = {(float)s_defaultNorm[0], (float)s_defaultNorm[1], (float)s_defaultNorm[2]};

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfPoints);
	pdlg.setMethodTitle("Save PN file");
	pdlg.setInfo(qPrintable(QString("Points: %1").arg(numberOfPoints)));
	pdlg.start();

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	for (unsigned i=0;i<numberOfPoints;i++)
	{
		//write point
		{
			const CCVector3* P = theCloud->getPoint(i);
			
			//conversion to float
			float wBuff[3] = {(float)P->x, (float)P->y, (float)P->z};
			if (out.write((const char*)wBuff,3*sizeof(float))<0)
			{
				result = CC_FERR_WRITING;
				break;
			}
		}
			
		//write normal
		if (hasNorms)
		{
            const PointCoordinateType* N = theCloud->getPointNormal(i);
			//conversion to float
            norm[0] = (float)N[0];
            norm[1] = (float)N[1];
            norm[2] = (float)N[2];
		}
		if (out.write((const char*)norm,3*sizeof(float))<0)
		{
			result = CC_FERR_WRITING;
			break;
		}

		if (!nprogress.oneStep())
		{
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}
	}

	out.close();

	return result;
}

CC_FILE_ERROR PNFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	//opening file
	QFile in(filename);
	if (!in.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

    //we deduce the points number from the file size
	qint64 fileSize = in.size();
	qint64 singlePointSize = 6*sizeof(float);
	//check that size is ok
	if (fileSize == 0)
		return CC_FERR_NO_LOAD;
	if ((fileSize % singlePointSize) != 0)
		return CC_FERR_MALFORMED_FILE;
	unsigned numberOfPoints = (unsigned) (fileSize  / singlePointSize);

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfPoints);
	pdlg.setMethodTitle("Open PN file");
	pdlg.setInfo(qPrintable(QString("Points: %1").arg(numberOfPoints)));
	pdlg.start();

	ccPointCloud* loadedCloud = 0;
    //if the file is too big, it will be chuncked in multiple parts
	unsigned chunkIndex = 0;
	unsigned fileChunkPos = 0;
	unsigned fileChunkSize = 0;
    //number of points read for the current cloud part
	unsigned pointsRead = 0;
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	for (unsigned i=0;i<numberOfPoints;i++)
	{
        //if we reach the max. cloud size limit, we cerate a new chunk
		if (pointsRead == fileChunkPos+fileChunkSize)
		{
			if (loadedCloud)
				container.addChild(loadedCloud);
			fileChunkPos = pointsRead;
			fileChunkSize = std::min<unsigned>(numberOfPoints-pointsRead,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
			loadedCloud = new ccPointCloud(QString("unnamed - Cloud #%1").arg(++chunkIndex));
            if (!loadedCloud || !loadedCloud->reserveThePointsTable(fileChunkSize) || !loadedCloud->reserveTheNormsTable())
			{
				result = CC_FERR_NOT_ENOUGH_MEMORY;
				if (loadedCloud)
					delete loadedCloud;
				loadedCloud=0;
				break;
			}
			loadedCloud->showNormals(true);
		}

        //we read the 3 coordinates of the point
		float rBuff[3];
		if (in.read((char*)rBuff,3*sizeof(float))>=0)
		{
		    //conversion to CCVector3
		    CCVector3 P((PointCoordinateType)rBuff[0],
						(PointCoordinateType)rBuff[1],
						(PointCoordinateType)rBuff[2]);
			loadedCloud->addPoint(P);
		}
		else
		{
			result = CC_FERR_READING;
			break;
		}

        //then the 3 components of the normal vector
		if (in.read((char*)rBuff,3*sizeof(float))>=0)
		{
		    //conversion to PointCoordinateType[3]
			PointCoordinateType N[3] = {(PointCoordinateType)rBuff[0],
										(PointCoordinateType)rBuff[1],
										(PointCoordinateType)rBuff[2]};
			loadedCloud->addNorm(N);
		}
		else
		{
			//add fake normal for consistency then break
			loadedCloud->addNorm(s_defaultNorm);
			result = CC_FERR_READING;
			break;
		}

		++pointsRead;

		if (!nprogress.oneStep())
		{
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}
    }

	in.close();

	if (loadedCloud)
	{
		if (loadedCloud->size() < loadedCloud->capacity())
			loadedCloud->resize(loadedCloud->size());
		container.addChild(loadedCloud);
	}

	return result;
}
