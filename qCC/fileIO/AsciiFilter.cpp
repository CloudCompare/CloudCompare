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

#include "AsciiFilter.h"
#include "AsciiSaveDlg.h"
#include "../ccCoordinatesShiftManager.h"

//Qt
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QSharedPointer>

//CClib
#include <ScalarField.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccHObject.h>
#include <ccProgressDialog.h>
#include <ccLog.h>

//System
#include <string.h>
#include <assert.h>

//! ASCII save dialog ('shared')
static QSharedPointer<AsciiSaveDlg> s_saveDialog(0);

CC_FILE_ERROR AsciiFilter::saveToFile(ccHObject* entity, const char* filename)
{
    assert(entity && filename);

	bool releaseDialog = false;
	if (!s_saveDialog)
	{
		s_saveDialog = QSharedPointer<AsciiSaveDlg>(new AsciiSaveDlg());

		if (!s_saveDialog->exec())
		{
			s_saveDialog.clear();
			return CC_FERR_CANCELED_BY_USER;
		}

		releaseDialog = true;
	}
	assert(s_saveDialog);

    if (!entity->isKindOf(CC_POINT_CLOUD))
	{
		if (entity->isA(CC_HIERARCHY_OBJECT)) //multiple clouds?
		{
			QFileInfo fi(filename);
			QString extension = fi.suffix();
			QString baseName = fi.completeBaseName();
			QString path = fi.path();

			unsigned count = entity->getChildrenNumber();
			//we count the number of clouds first
			unsigned cloudCount = 0;
			{
				for (unsigned i=0; i<count; ++i)
				{
					ccHObject* child = entity->getChild(i);
					if (child->isKindOf(CC_POINT_CLOUD))
						++cloudCount;
				}
			}
			//we can now create the corresponding file(s)
			{
				unsigned counter = 0;
				for (unsigned i=0; i<count; ++i)
				{
					ccHObject* child = entity->getChild(i);
					if (child->isKindOf(CC_POINT_CLOUD))
					{
						QString subFilename = path+QString("/");
						subFilename += QString(baseName).replace("cloudname",child->getName(),Qt::CaseInsensitive);
						counter++;
						assert(counter<=cloudCount);
						subFilename += QString("_%1").arg(cloudCount-counter,6,10,QChar('0'));
						if (!extension.isEmpty())
							subFilename += QString(".")+extension;
						CC_FILE_ERROR result = saveToFile(entity->getChild(i),qPrintable(subFilename));
						if (result != CC_FERR_NO_ERROR)
						{
							if (releaseDialog)
								s_saveDialog.clear();
							return result;
						}
						else
							ccLog::Print(QString("[AsciiFilter::saveToFile] Cloud '%1' saved in: %2").arg(child->getName()).arg(subFilename));
					}
					else
					{
						ccLog::Warning(QString("[AsciiFilter::saveToFile] Entity '%1' can't be saved this way!").arg(child->getName()));
					}
				}
			}
			
			if (releaseDialog)
				s_saveDialog.clear();
			return CC_FERR_NO_ERROR;
		}
		else
		{
			if (releaseDialog)
				s_saveDialog.clear();
			return CC_FERR_BAD_ARGUMENT;
		}
	}

    QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Truncate))
        return CC_FERR_WRITING;
	QTextStream stream(&file);

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

    unsigned numberOfPoints = cloud->size();
    bool writeColors = cloud->hasColors();
    bool writeNorms = cloud->hasNormals();
	std::vector<CCLib::ScalarField*> theScalarFields;
    if (cloud->isKindOf(CC_POINT_CLOUD))
	{
		ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
		for (unsigned i=0; i<ccCloud->getNumberOfScalarFields(); ++i)
			theScalarFields.push_back(ccCloud->getScalarField(i));
	}
    bool writeSF = (theScalarFields.size()!=0);

	//avancement du chargement
    ccProgressDialog pdlg(true);
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfPoints);
    pdlg.setMethodTitle(qPrintable(QString("Saving cloud [%1]").arg(cloud->getName())));
    pdlg.setInfo(qPrintable(QString("Number of points: %1").arg(numberOfPoints)));
    pdlg.start();

	//shift on load
	const double* shift = cloud->getOriginalShift();

	//output precision
	const int s_coordPrecision = s_saveDialog->coordsPrecision();
	const int s_sfPrecision = s_saveDialog->sfPrecision(); 
	const int s_nPrecision = 2+sizeof(PointCoordinateType);

	//other parameters
	bool saveColumnsHeader = s_saveDialog->saveColumnsNamesHeader();
	bool savePointCountHeader = s_saveDialog->savePointCountHeader();
	bool swapColorAndSFs = s_saveDialog->swapColorAndSF();
	QChar separator(s_saveDialog->getSeparator());

	if (saveColumnsHeader)
	{
		QString header("//");
		header.append("X");
		header.append(separator);
		header.append("Y");
		header.append(separator);
		header.append("Z");

		if (writeColors && !swapColorAndSFs)
		{
			header.append(separator);
			header.append("R");
			header.append(separator);
			header.append("G");
			header.append(separator);
			header.append("B");
		}

		if (writeSF)
		{
			//add each associated SF name
			for (std::vector<CCLib::ScalarField*>::const_iterator it = theScalarFields.begin(); it != theScalarFields.end(); ++it)
			{
				QString sfName((*it)->getName());
				sfName.replace(" ","_");
				header.append(separator);
				header.append(sfName);
			}
		}

		if (writeColors && swapColorAndSFs)
		{
			header.append(separator);
			header.append("R");
			header.append(separator);
			header.append("G");
			header.append(separator);
			header.append("B");
		}

        if (writeNorms)
		{
			header.append(separator);
			header.append("Nx");
			header.append(separator);
			header.append("Ny");
			header.append(separator);
			header.append("Nz");
		}
		
		stream << header << "\n";
	}

	if (savePointCountHeader)
	{
		stream << QString::number(numberOfPoints) << "\n";
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
    for (unsigned i=0;i<numberOfPoints;++i)
    {
		//line for the current point
		QString line;

		//write current point coordinates
        const CCVector3* P = cloud->getPoint(i);
		line.append(QString::number(-shift[0]+(double)P->x,'f',s_coordPrecision));
		line.append(separator);
		line.append(QString::number(-shift[1]+(double)P->y,'f',s_coordPrecision));
		line.append(separator);
		line.append(QString::number(-shift[2]+(double)P->z,'f',s_coordPrecision));

		QString color;
		if (writeColors)
        {
			//add rgb color
            const colorType* col = cloud->getPointColor(i);
			color.append(separator);
			color.append(QString::number(col[0]));
			color.append(separator);
			color.append(QString::number(col[1]));
			color.append(separator);
			color.append(QString::number(col[2]));

			if (!swapColorAndSFs)
				line.append(color);
        }

        if (writeSF)
        {
			//add each associated SF values
			for (std::vector<CCLib::ScalarField*>::const_iterator it = theScalarFields.begin(); it != theScalarFields.end(); ++it)
			{
				line.append(separator);
				line.append(QString::number((*it)->getValue(i),'f',s_sfPrecision));
			}
        }

        if (writeColors && swapColorAndSFs)
			line.append(color);

        if (writeNorms)
        {
			//add normal vector
            const PointCoordinateType* N = cloud->getPointNormal(i);
			line.append(separator);
			line.append(QString::number(N[0],'f',s_nPrecision));
			line.append(separator);
			line.append(QString::number(N[1],'f',s_nPrecision));
			line.append(separator);
			line.append(QString::number(N[2],'f',s_nPrecision));
        }

		stream << line << "\n";

		if (!nprogress.oneStep())
		{
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}
    }

	if (releaseDialog)
		s_saveDialog.clear();

    return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR AsciiFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
    //we get the size of the file to open
	QFile file(filename);
	if (!file.exists())
        return CC_FERR_READING;

	qint64 fileSize = file.size();
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
											aod.getMaxCloudSize(),
                                            aod.getSkippedLinesCount(),
											alwaysDisplayLoadDialog,
											coordinatesShiftEnabled,
											coordinatesShift);
}

struct cloudAttributesDescriptor
{
    ccPointCloud* cloud;
	static const unsigned c_attribCount = 12;
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
            int iRgbaIndex;
            int fRgbaIndex;
            int greyIndex;
        };
        int indexes[c_attribCount];
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
        for (unsigned int i=0;i<c_attribCount;++i)
            indexes[i]=-1;
        hasNorms=false;
        hasRGBColors=false;
		scalarIndexes.clear();
		scalarFields.clear();
    };

    void updateMaxIndex(int& maxIndex)
    {
		unsigned i;
        for (i=0;i<c_attribCount;++i)
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
	ccPointCloud* cloud = new ccPointCloud();
	if (!cloud || !cloud->reserveThePointsTable(numberOfPoints))
	{
		if (cloud)
			delete cloud;
        return cloudAttributesDescriptor();
	}

	if (step == 1)
		cloud->setName("unnamed - Cloud");
	else
		cloud->setName(QString("unnamed - Cloud (part %1)").arg(step));

    cloudAttributesDescriptor cloudDesc;
	cloudDesc.cloud = cloud;

    size_t seqSize = openSequence.size();
    for (int i=0;i<(int)seqSize;++i)
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
			{
				int sfIndex = cloud->getNumberOfScalarFields()+1;
				QString sfName = openSequence[i].header;
				if (sfName.isEmpty())
				{
					sfName = "Scalar field";
					if (sfIndex > 1)
						sfName += QString(" #%1").arg(sfIndex);
					//if (isPositive)
					//	sfName += QString(" (positive)");
				}

				ccScalarField* sf = new ccScalarField(qPrintable(sfName));
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
		case ASCII_OPEN_DLG_RGB32i:
		case ASCII_OPEN_DLG_RGB32f:
			if (cloud->reserveTheRGBTable())
			{
				if (openSequence[i].type == ASCII_OPEN_DLG_RGB32i)
					cloudDesc.iRgbaIndex = i;
				else
					cloudDesc.fRgbaIndex = i;
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
                                                            qint64 fileSize,
															unsigned maxCloudSize,
                                                            unsigned skipLines/*=0*/,
															bool alwaysDisplayLoadDialog/*=true*/,
															bool* coordinatesShiftEnabled/*=0*/,
															double* coordinatesShift/*=0*/)
{
    //we may have to "slice" clouds when opening them if they are too big!
	maxCloudSize = std::min(maxCloudSize,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
    unsigned cloudChunkSize = std::min(maxCloudSize,approximateNumberOfLines);
    unsigned cloudChunkPos = 0;
    unsigned chunkRank = 1;

    //we initialize the loading accelerator structure and point cloud
    int maxPartIndex=-1;
    cloudAttributesDescriptor cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, chunkRank);

	if (!cloudDesc.cloud)
        return CC_FERR_NOT_ENOUGH_MEMORY;

    //we re-open the file (ASCII mode)
	QFile file(filename);
	if (!file.open(QFile::ReadOnly))
	{
        //we clear already initialized data
        clearStructure(cloudDesc);
        return CC_FERR_READING;
	}

	//we skip lines as defined on input
    char currentLine[MAX_ASCII_FILE_LINE_LENGTH];
	{
		for (unsigned i=0;i<skipLines;++i)
		{
			if (file.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH)<0)
			{
				//we clear already initialized data
				clearStructure(cloudDesc);
				return CC_FERR_READING;
			}
		}
	}

    //progress indicator
    ccProgressDialog pdlg(true);
	CCLib::NormalizedProgress nprogress(&pdlg,approximateNumberOfLines);
    pdlg.setMethodTitle(qPrintable(QString("Open ASCII file [%1]").arg(filename)));
    pdlg.setInfo(qPrintable(QString("Approximate number of points: %1").arg(approximateNumberOfLines)));
    pdlg.start();

    //buffers
    ScalarType D = 0;
	double P[3] = {0.0,0.0,0.0};
    double Pshift[3] = {0.0,0.0,0.0};
    float N[3] = {0.0,0.0,0.0};
    colorType col[3] = {0,0,0};

    //other useful variables
    unsigned linesRead = 0;
    unsigned pointsRead = 0;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

    //main process
	unsigned nextLimit = /*cloudChunkPos+*/cloudChunkSize;
    while (file.readLine(currentLine,MAX_ASCII_FILE_LINE_LENGTH)>0)
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

        //if we have reached the max. number of points per cloud
        if (pointsRead == nextLimit)
        {
            ccConsole::PrintDebug("[ASCII] Point %i -> end of chunk (%i points)",pointsRead,cloudChunkSize);

        	//we re-evaluate the average line size
			{
        		double averageLineSize = (double)file.pos()/(double)(pointsRead+skipLines);
        		double newNbOfLinesApproximation = std::max(1.0, (double)fileSize/averageLineSize - (double)skipLines);

        		//if approximation is smaller than actual one, we add 2% by default
        		if (newNbOfLinesApproximation <= pointsRead)
        		{
        			newNbOfLinesApproximation = std::max((double)(cloudChunkPos+cloudChunkSize)+1.0,(double)pointsRead * 1.02);
        		}
				approximateNumberOfLines = (unsigned)ceil(newNbOfLinesApproximation);
				ccConsole::PrintDebug("[ASCII] New approximate nb of lines: %i",approximateNumberOfLines);
			}

        	//we try to resize actual clouds
        	if (cloudChunkSize < maxCloudSize || approximateNumberOfLines-cloudChunkPos <= maxCloudSize)
        	{
                ccConsole::PrintDebug("[ASCII] We choose to enlarge existing clouds");

        		cloudChunkSize = std::min(maxCloudSize,approximateNumberOfLines-cloudChunkPos);
       			if (!cloudDesc.cloud->reserve(cloudChunkSize))
        		{
        			ccConsole::Error("Not enough memory! Process stopped ...");
					result = CC_FERR_NOT_ENOUGH_MEMORY;
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
        		cloudChunkSize = std::min(maxCloudSize,approximateNumberOfLines-cloudChunkPos);
        		cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, ++chunkRank);
        		if (!cloudDesc.cloud)
        		{
        			ccConsole::Error("Not enough memory! Process stopped ...");
        			break;
        		}
				cloudDesc.cloud->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
        	}

        	//we update the progress info
			nprogress.scale(approximateNumberOfLines,100,true);
			pdlg.setInfo(qPrintable(QString("Approximate number of points: %1").arg(approximateNumberOfLines)));

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
			if (cloudDesc.iRgbaIndex>=0)
			{
				const uint32_t rgb = parts[cloudDesc.iRgbaIndex].toInt();
				col[0] = ((rgb >> 16)	& 0x0000ff);
				col[1] = ((rgb >> 8)	& 0x0000ff);
				col[2] = ((rgb)			& 0x0000ff);

			}
			else if (cloudDesc.fRgbaIndex>=0)
			{
				const float rgbf = parts[cloudDesc.fRgbaIndex].toFloat();
				const uint32_t rgb = (uint32_t)(*((uint32_t*)&rgbf));
				col[0] = ((rgb >> 16)	& 0x0000ff);
				col[1] = ((rgb >> 8)	& 0x0000ff);
				col[2] = ((rgb)			& 0x0000ff);
			}
			else
			{
				if (cloudDesc.redIndex>=0)
					col[0]=(colorType)parts[cloudDesc.redIndex].toInt();
				if (cloudDesc.greenIndex>=0)
					col[1]=(colorType)parts[cloudDesc.greenIndex].toInt();
				if (cloudDesc.blueIndex>=0)
					col[2]=(colorType)parts[cloudDesc.blueIndex].toInt();
			}
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
				D=(ScalarType)parts[cloudDesc.scalarIndexes[j]].toDouble();
				cloudDesc.scalarFields[j]->setValue(pointsRead-cloudChunkPos,D);
			}
		}

        ++pointsRead;
		if (!nprogress.oneStep())
        {
            //cancel requested
			result = CC_FERR_CANCELED_BY_USER;
			break;
        }
    }

    file.close();

	if (cloudDesc.cloud)
	{
		if (cloudDesc.cloud->size() < cloudDesc.cloud->capacity())
   			cloudDesc.cloud->resize(cloudDesc.cloud->size());

		//add cloud to output
		if (!cloudDesc.scalarFields.empty())
		{
			for (unsigned j=0;j<cloudDesc.scalarFields.size();++j)
				cloudDesc.scalarFields[j]->computeMinAndMax();
			cloudDesc.cloud->setCurrentDisplayedScalarField(0);
			cloudDesc.cloud->showSF(true);
		}

		container.addChild(cloudDesc.cloud,true);
    }

    return result;
}
