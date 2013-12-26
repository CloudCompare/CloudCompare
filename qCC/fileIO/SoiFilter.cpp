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

#include "SoiFilter.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

CC_FILE_ERROR SoiFilter::saveToFile(ccHObject* entity, const char* filename)
{
	ccLog::Print("Function is not implemented yet !");
	return CC_FERR_NO_SAVE;
}

CC_FILE_ERROR SoiFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
    //open the file
	FILE *fp = fopen(filename, "rt");
    if (!fp)
        return CC_FERR_READING;

	std::string line;
	line.resize(MAX_ASCII_FILE_LINE_LENGTH);
	unsigned nbScansTotal = 0;
	unsigned nbPointsTotal = 0;

	//we read the first line
	char* eof = fgets ((char*)line.c_str(), MAX_ASCII_FILE_LINE_LENGTH , fp);
    char* pEnd;

    //header
	while ((strcmp((char*)line.substr(0,4).c_str(),"#CC#") != 0)&&(eof != NULL))
	{
		if (strcmp(line.substr(0,4).c_str(),"#NP#")==0)
		{
			std::string numPoints (line,4,line.size()-4);
			nbPointsTotal=strtol(numPoints.c_str(),&pEnd,0);
			//ccLog::Print("[SoiFilter::loadFile] Total number of points: %i",nbPointsTotal);
		}
		else if (strcmp(line.substr(0,4).c_str(),"#NS#")==0)
		{
			std::string numScans (line,4,line.size()-4);
			nbScansTotal=strtol(numScans.c_str(),&pEnd,0);
			//ccLog::Print("[SoiFilter::loadFile] Total number of scans: %i",nbScansTotal);
		}

		eof = fgets ((char*)line.c_str(), MAX_ASCII_FILE_LINE_LENGTH , fp);
	}

	if ((nbScansTotal == 0)||(nbPointsTotal == 0))
	{
		ccLog::Warning("[SoiFilter::loadFile] No points or scans defined in this file!");
		fclose(fp);
		return CC_FERR_NO_LOAD;
	}

	//Progress dialog
	ccProgressDialog pdlg(false); //cancel is not supported
	pdlg.setMethodTitle("Open SOI file");
	char buffer[256];
	sprintf(buffer,"%u scans / %u points\n",nbScansTotal,nbPointsTotal);
	CCLib::NormalizedProgress nprogress(&pdlg,nbPointsTotal);
	pdlg.setInfo(buffer);
	pdlg.start();

    //Scan by scan
	for (unsigned k=0; k<nbScansTotal; k++)
	{
		char* eof = fgets ((char*)line.c_str(), MAX_ASCII_FILE_LINE_LENGTH , fp);

        //we only look for points (we ignore the rest)
		while ((strcmp(line.substr(0,4).c_str(),"#pt#")!=0)&&(eof != NULL))
			eof = fgets ((char*)line.c_str(), MAX_ASCII_FILE_LINE_LENGTH , fp);

		unsigned nbOfPoints = 0;

		if (strcmp(line.substr(0,4).c_str(),"#pt#")==0)
		{
			std::string numPoints(line,4,line.size()-4);
			nbOfPoints = strtol(numPoints.c_str(),&pEnd,0);
			//ccLog::Print("[SoiFilter::loadFile] Scan %i - points: %i",k+1,nbOfPoints);
		}
		else
		{
			ccLog::Warning("[SoiFilter::loadFile] Can't find marker '#pt#'!");
			fclose(fp);
			return CC_FERR_WRONG_FILE_TYPE;
		}
		
		if (nbOfPoints == 0)
		{
			ccLog::Warning("[SoiFilter::loadFile] Scan #%i is empty!",k);
			continue;
		}

		//Creation de la liste de points
		QString name = QString("unnamed - Scan #%1").arg(k);

		ccPointCloud* loadedCloud = new ccPointCloud(name);
		if ( !loadedCloud->reserveThePointsTable(nbOfPoints) || !loadedCloud->reserveTheRGBTable() )
		{
			fclose(fp);
			delete loadedCloud;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		loadedCloud->showColors(true);

		//we can read points now
		for (unsigned i=0; i<nbOfPoints; i++)
		{
			float P[3];
			int c = 0;
			fscanf(fp,"%f %f %f %i",P,P+1,P+2,&c);

			loadedCloud->addPoint(CCVector3::fromArray(P));
			loadedCloud->addGreyColor(static_cast<colorType>(c<<3)); //<<2 ? <<3 ? we lack some info. here ...

			nprogress.oneStep();
		}

		container.addChild(loadedCloud);
	}

	fclose(fp);

    return CC_FERR_NO_ERROR;
}

