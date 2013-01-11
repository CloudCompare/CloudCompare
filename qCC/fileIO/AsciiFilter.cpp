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
//$Rev:: 2266                                                              $
//$LastChangedDate:: 2012-10-15 00:07:12 +0200 (lun., 15 oct. 2012)        $
//**************************************************************************
//
#include "AsciiFilter.h"
#include "../ccCoordinatesShiftManager.h"

//Qt
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

//CClib
#include <ScalarField.h>
#include <CCMiscTools.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccHObject.h>
#include <ccProgressDialog.h>
#include <ccLog.h>

#include <assert.h>

CC_FILE_ERROR AsciiFilter::saveToFile(ccHObject* entity, const char* filename)
{
    assert(entity && filename);

    if (!entity->isKindOf(CC_POINT_CLOUD))
	{
		if (entity->isA(CC_HIERARCHY_OBJECT)) //multiple clouds?
		{
			QFileInfo fi(filename);
			QString extension = fi.suffix();
			QString baseName = fi.completeBaseName();
			QString path = fi.path();

			unsigned i,count=entity->getChildrenNumber();
			unsigned counter=0;
			for (i=0;i<count;++i)
			{
				ccHObject* child = entity->getChild(i);
				if (child->isKindOf(CC_POINT_CLOUD))
				{
					QString subFilename = path+QString("/");
					subFilename += QString(baseName).replace("cloudname",child->getName(),Qt::CaseInsensitive);
					subFilename += QString("_%1").arg(counter++,6,10,QChar('0'));
					if (!extension.isEmpty())
						subFilename += QString(".")+extension;
					CC_FILE_ERROR result = saveToFile(entity->getChild(i),qPrintable(subFilename));
					if (result != CC_FERR_NO_ERROR)
						return result;
					else
						ccLog::Print(QString("[AsciiFilter::saveToFile] Cloud '%1' saved in: %2").arg(child->getName()).arg(subFilename));
				}
				else
				{
					ccLog::Warning(QString("[AsciiFilter::saveToFile] Entity '%1' can't be saved this way!").arg(child->getName()));
				}
			}
			
			return CC_FERR_NO_ERROR;
		}
		else
		{
			return CC_FERR_BAD_ARGUMENT;
		}
	}

	//hack: if the extension is 'pts', the color will be saved after the SFs
	bool swapColorAndSFs = (QFileInfo(filename).suffix().toUpper() == "PTS");

    QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Truncate))
        return CC_FERR_WRITING;
	QTextStream stream(&file);

    ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);

    unsigned numberOfPoints = cloud->size();
    bool writeColors = cloud->hasColors();
    bool writeNorms = cloud->hasNormals();
	std::vector<CCLib::ScalarField*> theScalarFields;
    if (cloud->isKindOf(CC_POINT_CLOUD))
	{
		ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
		for (unsigned i=0;i<ccCloud->getNumberOfScalarFields();++i)
			theScalarFields.push_back(ccCloud->getScalarField(i));
	}
    bool writeSF = (theScalarFields.size()!=0);

	if (swapColorAndSFs && writeColors && writeSF)
		ccLog::Warning("[AsciiFilter::saveToFile] PTS extension detected: color components will be saved after the scalar field(s)");

	//avancement du chargement
    ccProgressDialog pdlg(true);
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfPoints);
    pdlg.setMethodTitle(qPrintable(QString("Saving cloud [%1]").arg(cloud->getName())));
    pdlg.setInfo(qPrintable(QString("Number of points: %1").arg(numberOfPoints)));
    pdlg.start();

	//shift on load
	const double* shift = cloud->getOriginalShift();
	double shiftNorm = (shift ? shift[0]*shift[0]+shift[1]*shift[1]+shift[2]*shift[2] : 0.0);
	//default precision (6 for floats, 10 for doubles)
	const int s_coordPrecision = 2+(shiftNorm > 0 ? sizeof(double) : sizeof(PointCoordinateType));
	const int s_sfPrecision = 2+sizeof(DistanceType); 
	const int s_nPrecision = 2+sizeof(PointCoordinateType);

	QString line,color;

    for (unsigned i=0;i<numberOfPoints;++i)
    {
		//write current point coordinates
        const CCVector3* P = cloud->getPoint(i);
		line = QString("%1 %2 %3").arg(-shift[0]+(double)P->x,0,'f',s_coordPrecision).arg(-shift[1]+(double)P->y,0,'f',s_coordPrecision).arg(-shift[2]+(double)P->z,0,'f',s_coordPrecision);

		if (writeColors)
        {
			//add rgb color (if not a .pts file)
            const colorType* col = cloud->getPointColor(i);
			color = QString(" %1 %2 %3").arg(col[0]).arg(col[1]).arg(col[2]);

			if (!swapColorAndSFs)
				line.append(color);
        }

        if (writeSF)
        {
			//add each associated SF values
			for (std::vector<CCLib::ScalarField*>::const_iterator it = theScalarFields.begin(); it != theScalarFields.end(); ++it)
				line.append(QString(" %1").arg((*it)->getValue(i),0,'f',s_sfPrecision));
        }

        if (writeColors && swapColorAndSFs)
			line.append(color);

        if (writeNorms)
        {
			//add normal vector
            const PointCoordinateType* N = cloud->getPointNormal(i);
			line.append(QString(" %1 %2 %3").arg(N[0],0,'f',s_nPrecision).arg(N[1],0,'f',s_nPrecision).arg(N[2],0,'f',s_nPrecision));
        }

		stream << line << "\n";
		//if (stream.status() != QTextStream::Ok)
		//	return CC_FERR_WRITING;

		if (!nprogress.oneStep())
			return CC_FERR_CANCELED_BY_USER;
    }

    return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR AsciiFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
    //we get the size of the file to open
	QFile file(filename);
	if (!file.exists())
        return CC_FERR_READING;

	__int64 fileSize = file.size();
		
    if (fileSize == 0)
        return CC_FERR_NO_LOAD;

    //column attribution dialog
    AsciiOpenDlg aod(filename);

	QString dummyStr;
	if (alwaysDisplayLoadDialog || aod.getColumnsCount()!=3 || !AsciiOpenDlg::CheckOpenSequence(aod.getOpenSequence(),dummyStr))
	{
		if (!aod.exec())
			return CC_FERR_CANCELED_BY_USER;
	}

    //we compute the approximate line number
    double averageLineSize = aod.getAverageLineSize();
    unsigned approximateNumberOfLines = (unsigned)ceil((double)fileSize/averageLineSize);

	AsciiOpenDlg::Sequence openSequence = aod.getOpenSequence();

    return loadCloudFromFormatedAsciiFile(filename,
                                            container,
                                            openSequence,
                                            (char)aod.getSeparator(),
                                            approximateNumberOfLines,
                                            fileSize,
                                            aod.getSkippedLinesCount(),
											alwaysDisplayLoadDialog,
											coordinatesShiftEnabled,
											coordinatesShift);
}

struct cloudAttributesDescriptor
{
    ccPointCloud* cloud;
    union
    {
        struct{
            int xCoordIndex;
            int yCoordIndex;
            int zCoordIndex;
            int xNormIndex;
            int yNormIndex;
            int zNormIndex;
            int redIndex;
            int greenIndex;
            int blueIndex;
            int greyIndex;
        };
        int indexes[10];
    };
	std::vector<int> scalarIndexes;
	std::vector<CCLib::ScalarField*> scalarFields;
    bool hasNorms;
    bool hasRGBColors;

	cloudAttributesDescriptor()
	{
		reset();
	}

    void reset()
    {
        cloud=NULL;
        for (int i=0;i<10;++i)
            indexes[i]=-1;
        hasNorms=false;
        hasRGBColors=false;
		scalarIndexes.clear();
		scalarFields.clear();
    };

    void updateMaxIndex(int& maxIndex)
    {
		unsigned i;
        for (i=0;i<10;++i)
            if (indexes[i]>maxIndex)
                maxIndex=indexes[i];
        for (i=0;i<scalarIndexes.size();++i)
            if (scalarIndexes[i]>maxIndex)
                maxIndex=scalarIndexes[i];
    };

};

void clearStructure(cloudAttributesDescriptor &cloudDesc)
{
    if (cloudDesc.cloud)
		delete cloudDesc.cloud;
	cloudDesc.cloud=0;
	cloudDesc.reset();
}

cloudAttributesDescriptor prepareCloud(const AsciiOpenDlg::Sequence &openSequence,
                                       unsigned numberOfPoints,
                                       int& maxIndex,
									   unsigned step=1)
{
    unsigned seqSize = openSequence.size();

	char name[256];
	if (step==1)
		strcpy(name,"unnamed - Cloud");
	else
		sprintf(name,"unnamed - Cloud (part %i)",step);

	ccPointCloud* cloud = new ccPointCloud(name);

	if (!cloud || !cloud->reserveThePointsTable(numberOfPoints))
	{
		if (cloud)
			delete cloud;
        return cloudAttributesDescriptor();
	}

    cloudAttributesDescriptor cloudDesc;
	cloudDesc.cloud = cloud;

    unsigned i;
    for (i=0;i<seqSize;++i)
    {
		switch (openSequence[i].type)
		{
		case ASCII_OPEN_DLG_None:
			break;
		case ASCII_OPEN_DLG_X:
			cloudDesc.xCoordIndex = i;
			break;
		case ASCII_OPEN_DLG_Y:
			cloudDesc.yCoordIndex = i;
			break;
		case ASCII_OPEN_DLG_Z:
			cloudDesc.zCoordIndex = i;
			break;
		case ASCII_OPEN_DLG_NX:
			if (cloud->reserveTheNormsTable())
			{
				cloudDesc.xNormIndex = i;
				cloudDesc.hasNorms=true;
				cloud->showNormals(true);
			}
			else
			{
				ccConsole::Warning("Failed to allocate memory for normals! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_NY:
			if (cloud->reserveTheNormsTable())
			{
				cloudDesc.yNormIndex = i;
				cloudDesc.hasNorms=true;
				cloud->showNormals(true);
			}
			else
			{
				ccConsole::Warning("Failed to allocate memory for normals! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_NZ:
			if (cloud->reserveTheNormsTable())
			{
				cloudDesc.zNormIndex = i;
				cloudDesc.hasNorms=true;
				cloud->showNormals(true);
			}
			else
			{
				ccConsole::Warning("Failed to allocate memory for normals! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_Scalar:
		case ASCII_OPEN_DLG_Positive_Scalar:
			{
				int sfIndex = cloud->getNumberOfScalarFields()+1;
				bool isPositive = (openSequence[i].type==ASCII_OPEN_DLG_Positive_Scalar);
				QString sfName = openSequence[i].header;
				if (sfName.isEmpty())
				{
					sfName = "Scalar field";
					if (sfIndex > 1)
						sfName += QString(" #%1").arg(sfIndex);
					//if (isPositive)
					//	sfName += QString(" (positive)");
				}

				ccScalarField* sf = new ccScalarField(qPrintable(sfName),isPositive);
				sf->link();
				int sfIdx = cloud->addScalarField(sf);
				if (sfIdx>=0)
				{
					cloudDesc.scalarIndexes.push_back(i);
					cloudDesc.scalarFields.push_back(sf);
				}
				else
				{
					ccConsole::Warning("Failed to add scalar field #%i to cloud #%i! (skipped)",sfIndex);
				}
				sf->release();
			}
			break;
		case ASCII_OPEN_DLG_R:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.redIndex = i;
				cloudDesc.hasRGBColors=true;
				cloud->showColors(true);
			}
			else
			{
				ccConsole::Warning("Failed to allocate memory for colors! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_G:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.greenIndex = i;
				cloudDesc.hasRGBColors=true;
				cloud->showColors(true);
			}
			else
			{
				ccConsole::Warning("Failed to allocate memory for colors! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_B:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.blueIndex = i;
				cloudDesc.hasRGBColors=true;
				cloud->showColors(true);
			}
			else
			{
				ccConsole::Warning("Failed to allocate memory for colors! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_Grey:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.greyIndex = i;
				cloud->showColors(true);
			}
			else
			{
				ccConsole::Warning("Failed to allocate memory for colors! (skipped)");
			}
			break;
        }
    }

    //we compute the max index for each cloud descriptor
    maxIndex = -1;
	cloudDesc.updateMaxIndex(maxIndex);

    return cloudDesc;
}

CC_FILE_ERROR AsciiFilter::loadCloudFromFormatedAsciiFile(const char* filename,
                                                            ccHObject& container,
                                                            const AsciiOpenDlg::Sequence& openSequence,
                                                            char separator,
                                                            unsigned approximateNumberOfLines,
                                                            __int64 fileSize,
                                                            unsigned skipLines/*=0*/,
															bool alwaysDisplayLoadDialog/*=true*/,
															bool* coordinatesShiftEnabled/*=0*/,
															double* coordinatesShift/*=0*/)
{
    //we may have to "slice" clouds on opening if they are too big!
    unsigned cloudChunkSize = ccMin(CC_MAX_NUMBER_OF_POINTS_PER_CLOUD,approximateNumberOfLines);
    unsigned cloudChunkPos = 0;
    unsigned chunkRank = 1;
    unsigned i;

    //we initialize the loading accelerator structure and point cloud
    int maxPartIndex=-1;
    cloudAttributesDescriptor cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, chunkRank);

	if (!cloudDesc.cloud)
        return CC_FERR_NOT_ENOUGH_MEMORY;

    //we re-open the file (ASCII mode)
    FILE* fp = fopen(filename, "rt");
    if (!fp)
    {
        //we clear already initialized data
        clearStructure(cloudDesc);
        return CC_FERR_READING;
    }

	//we skip lines as defined on input
    char currentLine[MAX_ASCII_FILE_LINE_LENGTH];
    for (i=0;i<skipLines;++i)
	{
        if (!fgets(currentLine, MAX_ASCII_FILE_LINE_LENGTH, fp))
		{
			fclose(fp);
			//we clear already initialized data
			clearStructure(cloudDesc);
			return CC_FERR_READING;
		}
	}

    //progress indicator
    ccProgressDialog pdlg(true);
	char buffer[256];
    sprintf(buffer,"Open ASCII file [%s]",filename);
    pdlg.setMethodTitle(buffer);
    float percent = 0.0;
    unsigned palier = unsigned(float(approximateNumberOfLines) * 0.01); //1% of total
    unsigned lineCounter = 0;
    sprintf(buffer,"Approximate number of points: %i",approximateNumberOfLines);
    pdlg.setInfo(buffer);
    pdlg.start();

    //buffers
    DistanceType D=0.0;
	double P[3]={0.0,0.0,0.0};
    double Pshift[3]={0.0,0.0,0.0};
    float N[3]={0.0,0.0,0.0};
    colorType col[3]={0,0,0};

    //other useful variables
    unsigned linesRead = 0;
    unsigned pointsRead = 0;

    //main process
	unsigned nextLimit = cloudChunkPos+cloudChunkSize;
    while (fgets(currentLine, MAX_ASCII_FILE_LINE_LENGTH, fp))
    {
        ++linesRead;

        //comment
		if (currentLine[0]=='/' && currentLine[1]=='/')
            continue;

        if (currentLine[0]==0 || currentLine[0]==10)
        {
            ccConsole::Warning("[AsciiFilter::Load] Line %i is corrupted (empty)!",linesRead);
            continue;
        }

        //if we have attained max. number of points per cloud
        if (pointsRead == nextLimit)
        {
            ccConsole::PrintDebug("[ASCII] Point %i -> end of chunk (%i points)",pointsRead,cloudChunkSize);

        	//we re-evaluate the average line size
        	double averageLineSize = double(CCLib::CCMiscTools::ftell64(fp))/double(pointsRead+skipLines);
        	unsigned newNbOfLinesApproximation = (unsigned)(double(fileSize)/averageLineSize)-skipLines;

        	//if approximation is smaller than actual one, we add 2% by default
        	if (newNbOfLinesApproximation<=pointsRead)
        	{
        		newNbOfLinesApproximation = unsigned(ceil(float(pointsRead)*1.02));
        		newNbOfLinesApproximation = ccMax(cloudChunkSize+1,newNbOfLinesApproximation);
        	}

            ccConsole::PrintDebug("[ASCII] New approximate nb of lines: %i",newNbOfLinesApproximation);

        	//we try to resize actual clouds
        	if ((cloudChunkSize<CC_MAX_NUMBER_OF_POINTS_PER_CLOUD)||(newNbOfLinesApproximation-cloudChunkPos<=CC_MAX_NUMBER_OF_POINTS_PER_CLOUD))
        	{
                ccConsole::PrintDebug("[ASCII] We choose to enlarge existing clouds");

        		cloudChunkSize = ccMin(CC_MAX_NUMBER_OF_POINTS_PER_CLOUD,newNbOfLinesApproximation-cloudChunkPos);
       			if (!cloudDesc.cloud->reserve(cloudChunkSize))
        		{
        			ccConsole::Error("Not enough memory! Process stopped ...");
        			break;
        		}
        	}
        	else //otherwise we have to create new clouds
        	{
                ccConsole::PrintDebug("[ASCII] We choose to instantiate new clouds");

        		//we store (and resize) actual cloud
        		if (!cloudDesc.cloud->resize(cloudChunkSize))
        			ccConsole::Warning("Memory reallocation failed ... some memory may have been wasted ...");
				if (!cloudDesc.scalarFields.empty())
				{
					for (unsigned k=0;k<cloudDesc.scalarFields.size();++k)
						cloudDesc.scalarFields[k]->computeMinAndMax();
					cloudDesc.cloud->setCurrentDisplayedScalarField(0);
					cloudDesc.cloud->showSF(true);
				}
				//we add this cloud to the output container
				container.addChild(cloudDesc.cloud,true);
				cloudDesc.reset();

        		//and create new one
        		cloudChunkPos = pointsRead;
        		cloudChunkSize = ccMin(CC_MAX_NUMBER_OF_POINTS_PER_CLOUD,newNbOfLinesApproximation-cloudChunkPos);
        		cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, ++chunkRank);
        		if (!cloudDesc.cloud)
        		{
        			ccConsole::Error("Not enough memory! Process stopped ...");
        			break;
        		}
				cloudDesc.cloud->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
        	}

        	//on met à jour les informations sur la progression
        	percent = floor(float(pointsRead)*100.0/float(newNbOfLinesApproximation));
        	pdlg.update(percent);
        	lineCounter = 0;
        	palier = unsigned(float(newNbOfLinesApproximation-pointsRead) / (100.0-percent));
        	char buffer[256];
        	sprintf(buffer,"Approximate number of points: %i",newNbOfLinesApproximation);
        	pdlg.setInfo(buffer);
        	approximateNumberOfLines = newNbOfLinesApproximation;

			nextLimit = cloudChunkPos+cloudChunkSize;
        }

        //we split current line
        QStringList parts = QString(currentLine).split(separator,QString::SkipEmptyParts);

        int nParts = parts.size();
        if (nParts<=maxPartIndex)
        {
            ccConsole::Warning("[AsciiFilter::Load] Line %i is corrupted (found %i part(s) on %i attended)!",linesRead,nParts,maxPartIndex+1);
            continue;
        }

		//(X,Y,Z)
		if (cloudDesc.xCoordIndex>=0)
			P[0] = parts[cloudDesc.xCoordIndex].toDouble();
		if (cloudDesc.yCoordIndex>=0)
			P[1] = parts[cloudDesc.yCoordIndex].toDouble();
		if (cloudDesc.zCoordIndex>=0)
			P[2] = parts[cloudDesc.zCoordIndex].toDouble();

		//first point: check for 'big' coordinates
		if (pointsRead==0)
		{
			bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
			if (shiftAlreadyEnabled)
				memcpy(Pshift,coordinatesShift,sizeof(double)*3);
			bool applyAll=false;
			if (ccCoordinatesShiftManager::Handle(P,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,applyAll))
			{
				cloudDesc.cloud->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
				ccConsole::Warning("[ASCIIFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift[0],Pshift[1],Pshift[2]);

				//we save coordinates shift information
				if (applyAll && coordinatesShiftEnabled && coordinatesShift)
				{
					*coordinatesShiftEnabled = true;
					coordinatesShift[0] = Pshift[0];
					coordinatesShift[1] = Pshift[1];
					coordinatesShift[2] = Pshift[2];
				}
			}
		}

		//add point
		cloudDesc.cloud->addPoint(CCVector3(P[0]+Pshift[0],P[1]+Pshift[1],P[2]+Pshift[2]));

		//Normal vector
		if (cloudDesc.hasNorms)
		{
			if (cloudDesc.xNormIndex>=0)
				N[0] = parts[cloudDesc.xNormIndex].toFloat();
			if (cloudDesc.yNormIndex>=0)
				N[1] = parts[cloudDesc.yNormIndex].toFloat();
			if (cloudDesc.zNormIndex>=0)
				N[2] = parts[cloudDesc.zNormIndex].toFloat();
			cloudDesc.cloud->addNorm(N);
		}

		//Colors
		if (cloudDesc.hasRGBColors)
		{
			if (cloudDesc.redIndex>=0)
				col[0]=(colorType)parts[cloudDesc.redIndex].toInt();
			if (cloudDesc.greenIndex>=0)
				col[1]=(colorType)parts[cloudDesc.greenIndex].toInt();
			if (cloudDesc.blueIndex>=0)
				col[2]=(colorType)parts[cloudDesc.blueIndex].toInt();
			cloudDesc.cloud->addRGBColor(col);
		}
		else if (cloudDesc.greyIndex>=0)
		{
			col[0]=col[1]=col[2]=(colorType)parts[cloudDesc.greyIndex].toInt();
			cloudDesc.cloud->addRGBColor(col);
		}

		//Scalar distance
		if (!cloudDesc.scalarIndexes.empty())
		{
			for (unsigned j=0;j<cloudDesc.scalarIndexes.size();++j)
			{
				D=(DistanceType)parts[cloudDesc.scalarIndexes[j]].toDouble();
				cloudDesc.scalarFields[j]->setValue(pointsRead-cloudChunkPos,D);
			}
		}

        ++pointsRead;
        if (++lineCounter >= palier)
        {
            //cancel requested
            if (pdlg.isCancelRequested())
                break;

            lineCounter=0;
            percent += 1.0;
            pdlg.update(percent);
        }
    }

    fclose(fp);

    //if we have overestimated the number of points/lines
    if (pointsRead < approximateNumberOfLines)
    {
   		if (!cloudDesc.cloud->resize(pointsRead-cloudChunkPos))
    		ccConsole::Warning("Memory reallocation failed ... some memory may have been wasted ...");
    }

	//add cloud to output
	if (!cloudDesc.scalarFields.empty())
	{
		for (unsigned j=0;j<cloudDesc.scalarFields.size();++j)
			cloudDesc.scalarFields[j]->computeMinAndMax();
		cloudDesc.cloud->setCurrentDisplayedScalarField(0);
		cloudDesc.cloud->showSF(true);
	}
	container.addChild(cloudDesc.cloud,true);

    return CC_FERR_NO_ERROR;
}
