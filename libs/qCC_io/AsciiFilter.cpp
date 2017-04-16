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

#include "AsciiFilter.h"

//Qt
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QSharedPointer>

//CClib
#include <ScalarField.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccLog.h>
#include <ccScalarField.h>

//System
#include <string.h>
#include <assert.h>

//declaration of static members
AutoDeletePtr<AsciiSaveDlg> AsciiFilter::s_saveDialog(0);
AutoDeletePtr<AsciiOpenDlg> AsciiFilter::s_openDialog(0);

AsciiSaveDlg* AsciiFilter::GetSaveDialog(QWidget* parentWidget/*=0*/)
{
	if (!s_saveDialog.ptr)
	{
		s_saveDialog.ptr = new AsciiSaveDlg(parentWidget);
	}

	return s_saveDialog.ptr;
}

AsciiOpenDlg* AsciiFilter::GetOpenDialog(QWidget* parentWidget/*=0*/)
{
	if (!s_openDialog.ptr)
	{
		s_openDialog.ptr = new AsciiOpenDlg(parentWidget);
	}

	return s_openDialog.ptr;
}

bool AsciiFilter::canLoadExtension(QString upperCaseExt) const
{
	return (	upperCaseExt == "ASC"
			||	upperCaseExt == "TXT"
			||	upperCaseExt == "XYZ"
			||	upperCaseExt == "NEU"
			||	upperCaseExt == "PTS"
			||	upperCaseExt == "CSV");
}

bool AsciiFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (	type == CC_TYPES::POINT_CLOUD			//only one cloud per file
		||	type == CC_TYPES::HIERARCHY_OBJECT )	//but we can also save a group (each cloud inside will be saved as a separated file)
	{
		multiple = true;
		exclusive = true;
		return true;
	}

	return false;
}

CC_FILE_ERROR AsciiFilter::saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters)
{
	assert(entity && !filename.isEmpty());

	AsciiSaveDlg* saveDialog = GetSaveDialog(parameters.parentWidget);
	assert(saveDialog);

	//if the dialog shouldn't be shown, we'll simply take the default values!
	if (parameters.alwaysDisplaySaveDialog && saveDialog->autoShow() && !saveDialog->exec())
	{
		return CC_FERR_CANCELED_BY_USER;
	}

	if (!entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		if (entity->isA(CC_TYPES::HIERARCHY_OBJECT)) //multiple clouds?
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
					if (child->isKindOf(CC_TYPES::POINT_CLOUD))
						++cloudCount;
				}
			}
			
			//we can now create the corresponding file(s)
			if (cloudCount > 1)
			{
				unsigned counter = 0;
				//disable the save dialog so that it doesn't appear again!
				AsciiSaveDlg* saveDialog = GetSaveDialog();
				assert(saveDialog);

				bool autoShow = saveDialog->autoShow();
				saveDialog->setAutoShow(false);

				for (unsigned i=0; i<count; ++i)
				{
					ccHObject* child = entity->getChild(i);
					if (child->isKindOf(CC_TYPES::POINT_CLOUD))
					{
						QString subFilename = path+QString("/");
						subFilename += QString(baseName).replace("cloudname",child->getName(),Qt::CaseInsensitive);
						counter++;
						assert(counter <= cloudCount);
						subFilename += QString("_%1").arg(cloudCount-counter,6,10,QChar('0'));
						if (!extension.isEmpty())
							subFilename += QString(".") + extension;
						
						CC_FILE_ERROR result = saveToFile(entity->getChild(i),subFilename,parameters);
						if (result != CC_FERR_NO_ERROR)
						{
							return result;
						}
						else
						{
							ccLog::Print(QString("[ASCII] Cloud '%1' has been saved in: %2").arg(child->getName()).arg(subFilename));
						}
					}
					else
					{
						ccLog::Warning(QString("[ASCII] Entity '%1' can't be saved this way!").arg(child->getName()));
					}
				}

				//restore previous state
				saveDialog->setAutoShow(autoShow);

				return CC_FERR_NO_ERROR;
			}
		}
		else
		{
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
	std::vector<ccScalarField*> theScalarFields;
	if (cloud->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
		for (unsigned i = 0; i < ccCloud->getNumberOfScalarFields(); ++i)
			theScalarFields.push_back(static_cast<ccScalarField*>(ccCloud->getScalarField(i)));
	}
	bool writeSF = (theScalarFields.size() != 0);

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(0);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Saving cloud [%1]").arg(cloud->getName()));
		pDlg->setInfo(QObject::tr("Number of points: %1").arg(numberOfPoints));
		pDlg->start();
	}
	CCLib::NormalizedProgress nprogress(pDlg.data(), numberOfPoints);

	//output precision
	const int s_coordPrecision = saveDialog->coordsPrecision();
	const int s_sfPrecision = saveDialog->sfPrecision(); 
	const int s_nPrecision = 2+sizeof(PointCoordinateType);

	//other parameters
	bool saveColumnsHeader = saveDialog->saveColumnsNamesHeader();
	bool savePointCountHeader = saveDialog->savePointCountHeader();
	bool swapColorAndSFs = saveDialog->swapColorAndSF();
	QChar separator(saveDialog->getSeparator());
	bool saveFloatColors = saveDialog->saveFloatColors();

	if (saveColumnsHeader)
	{
		QString header("//");
		header.append(AsciiHeaderColumns::X());
		header.append(separator);
		header.append(AsciiHeaderColumns::Y());
		header.append(separator);
		header.append(AsciiHeaderColumns::Z());

		if (writeColors && !swapColorAndSFs)
		{
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Rf() : AsciiHeaderColumns::R());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Gf() : AsciiHeaderColumns::G());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Bf() : AsciiHeaderColumns::B());
		}

		if (writeSF)
		{
			//add each associated SF name
			for (std::vector<ccScalarField*>::const_iterator it = theScalarFields.begin(); it != theScalarFields.end(); ++it)
			{
				QString sfName((*it)->getName());
				sfName.replace(separator,'_');
				header.append(separator);
				header.append(sfName);
			}
		}

		if (writeColors && swapColorAndSFs)
		{
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Rf() : AsciiHeaderColumns::R());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Gf() : AsciiHeaderColumns::G());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Bf() : AsciiHeaderColumns::B());
		}

		if (writeNorms)
		{
			header.append(separator);
			header.append(AsciiHeaderColumns::Nx());
			header.append(separator);
			header.append(AsciiHeaderColumns::Ny());
			header.append(separator);
			header.append(AsciiHeaderColumns::Nz());
		}
		
		stream << header << "\n";
	}

	if (savePointCountHeader)
	{
		stream << QString::number(numberOfPoints) << "\n";
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	for (unsigned i=0; i<numberOfPoints; ++i)
	{
		//line for the current point
		QString line;

		//write current point coordinates
		const CCVector3* P = cloud->getPoint(i);
		CCVector3d Pglobal = cloud->toGlobal3d<PointCoordinateType>(*P);
		line.append(QString::number(Pglobal.x,'f',s_coordPrecision));
		line.append(separator);
		line.append(QString::number(Pglobal.y,'f',s_coordPrecision));
		line.append(separator);
		line.append(QString::number(Pglobal.z,'f',s_coordPrecision));

		QString colorLine;
		if (writeColors)
		{
			//add rgb color
			const ColorCompType* col = cloud->getPointColor(i);
			if (saveFloatColors)
			{
				colorLine.append(separator);
				colorLine.append(QString::number(static_cast<double>(col[0])/ccColor::MAX));
				colorLine.append(separator);
				colorLine.append(QString::number(static_cast<double>(col[1])/ccColor::MAX));
				colorLine.append(separator);
				colorLine.append(QString::number(static_cast<double>(col[2])/ccColor::MAX));
			}
			else
			{
				colorLine.append(separator);
				colorLine.append(QString::number(col[0]));
				colorLine.append(separator);
				colorLine.append(QString::number(col[1]));
				colorLine.append(separator);
				colorLine.append(QString::number(col[2]));
			}

			if (!swapColorAndSFs)
				line.append(colorLine);
		}

		if (writeSF)
		{
			//add each associated SF values
			for (std::vector<ccScalarField*>::const_iterator it = theScalarFields.begin(); it != theScalarFields.end(); ++it)
			{
				line.append(separator);
				double sfVal = (*it)->getGlobalShift() + (*it)->getValue(i);
				line.append(QString::number(sfVal,'f',s_sfPrecision));
			}
		}

		if (writeColors && swapColorAndSFs)
			line.append(colorLine);

		if (writeNorms)
		{
			//add normal vector
			const CCVector3& N = cloud->getPointNormal(i);
			line.append(separator);
			line.append(QString::number(N.x,'f',s_nPrecision));
			line.append(separator);
			line.append(QString::number(N.y,'f',s_nPrecision));
			line.append(separator);
			line.append(QString::number(N.z,'f',s_nPrecision));
		}

		stream << line << "\n";

		if (pDlg && !nprogress.oneStep())
		{
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}
	}

	return result;
}

CC_FILE_ERROR AsciiFilter::loadFile(QString filename,
									ccHObject& container,
									LoadParameters& parameters)
{
	//we get the size of the file to open
	QFile file(filename);
	if (!file.exists())
		return CC_FERR_READING;

	qint64 fileSize = file.size();
	if (fileSize == 0)
		return CC_FERR_NO_LOAD;

	//column attribution dialog
	//DGM: we ask for the semi-persistent dialog as it may have
	//been already initialized (by the command-line for instance)
	AsciiOpenDlg* openDialog = GetOpenDialog(parameters.parentWidget);
	assert(openDialog);
	openDialog->setFilename(filename);

	bool forceDialogDisplay = parameters.alwaysDisplayLoadDialog;
	//if we should try to avoid displaying the dialog
	//DGM: actually, we respect the wish of the caller by default ;)
	//if (!forceDialogDisplay)
	//{
	//	//we must check that the automatically guessed sequence is ok
	//	if (!openDialog->safeSequence())
	//	{
	//		forceDialogDisplay = true;
	//	}
	//}
	if (openDialog->restorePreviousContext())
	{
		//if we can/should use the previous sequence ('Apply all')
		forceDialogDisplay = false;
	}

	QString dummyStr;
	if (	forceDialogDisplay
		|| !AsciiOpenDlg::CheckOpenSequence(openDialog->getOpenSequence(), dummyStr))
	{
		//show the dialog
		if (!openDialog->exec())
		{
			s_openDialog.release(); //release the 'source' dialog (so as to be sure to reset it next time)

			//process was cancelled
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	//we compute the approximate line number
	double averageLineSize = openDialog->getAverageLineSize();
	unsigned approximateNumberOfLines = static_cast<unsigned>(ceil(static_cast<double>(fileSize)/averageLineSize));

	AsciiOpenDlg::Sequence openSequence = openDialog->getOpenSequence();
	char separator = static_cast<char>(openDialog->getSeparator());
	unsigned maxCloudSize = openDialog->getMaxCloudSize();
	unsigned skipLineCount = openDialog->getSkippedLinesCount();

	s_openDialog.release(); //release the 'source' dialog (so as to be sure to reset it next time)

	return loadCloudFromFormatedAsciiFile(	filename,
											container,
											openSequence,
											separator,
											approximateNumberOfLines,
											fileSize,
											maxCloudSize,
											skipLineCount,
											parameters);
}

struct cloudAttributesDescriptor
{
	ccPointCloud* cloud;
	static const unsigned c_attribCount = 12;
	union
	{
		struct{	int xCoordIndex;
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
	bool hasFloatRGBColors[3];

	cloudAttributesDescriptor()
	{
		reset();
	}

	void reset()
	{
		cloud = 0;
		for (unsigned i=0; i<c_attribCount; ++i)
			indexes[i] = -1;
		hasNorms = false;
		hasRGBColors = false;
		hasFloatRGBColors[0] = hasFloatRGBColors[1] = hasFloatRGBColors[2] = false;
		
		scalarIndexes.clear();
		scalarFields.clear();
	}

	void updateMaxIndex(int& maxIndex)
	{
		for (unsigned i=0; i<c_attribCount; ++i)
			if (indexes[i] > maxIndex)
				maxIndex = indexes[i];

		for (size_t j=0; j<scalarIndexes.size(); ++j)
			if (scalarIndexes[j] > maxIndex)
				maxIndex = scalarIndexes[j];
	}

};

void clearStructure(cloudAttributesDescriptor &cloudDesc)
{
	if (cloudDesc.cloud)
		delete cloudDesc.cloud;
	cloudDesc.cloud = 0;
	cloudDesc.reset();
}

cloudAttributesDescriptor prepareCloud(	const AsciiOpenDlg::Sequence &openSequence,
										unsigned numberOfPoints,
										int& maxIndex,
										char separator,
										unsigned step = 1)
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

	int seqSize = static_cast<int>(openSequence.size());
	for (int i=0; i<seqSize; ++i)
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
				cloudDesc.hasNorms = true;
				cloud->showNormals(true);
			}
			else
			{
				ccLog::Warning("Failed to allocate memory for normals! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_NY:
			if (cloud->reserveTheNormsTable())
			{
				cloudDesc.yNormIndex = i;
				cloudDesc.hasNorms = true;
				cloud->showNormals(true);
			}
			else
			{
				ccLog::Warning("Failed to allocate memory for normals! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_NZ:
			if (cloud->reserveTheNormsTable())
			{
				cloudDesc.zNormIndex = i;
				cloudDesc.hasNorms = true;
				cloud->showNormals(true);
			}
			else
			{
				ccLog::Warning("Failed to allocate memory for normals! (skipped)");
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
				else
				{
					sfName.replace('_',separator);
				}

				ccScalarField* sf = new ccScalarField(qPrintable(sfName));
				sf->link();
				int sfIdx = cloud->addScalarField(sf);
				if (sfIdx >= 0)
				{
					cloudDesc.scalarIndexes.push_back(i);
					cloudDesc.scalarFields.push_back(sf);
				}
				else
				{
					ccLog::Warning("Failed to add scalar field #%i to cloud #%i! (skipped)",sfIndex);
				}
				sf->release();
			}
			break;
		case ASCII_OPEN_DLG_Rf:
			cloudDesc.hasFloatRGBColors[0] = true;
		case ASCII_OPEN_DLG_R:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.redIndex = i;
				cloudDesc.hasRGBColors = true;
				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("Failed to allocate memory for colors! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_Gf:
			cloudDesc.hasFloatRGBColors[1] = true;
		case ASCII_OPEN_DLG_G:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.greenIndex = i;
				cloudDesc.hasRGBColors = true;
				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("Failed to allocate memory for colors! (skipped)");
			}
			break;
		case ASCII_OPEN_DLG_Bf:
			cloudDesc.hasFloatRGBColors[2] = true;
		case ASCII_OPEN_DLG_B:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.blueIndex = i;
				cloudDesc.hasRGBColors = true;
				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("Failed to allocate memory for colors! (skipped)");
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
				cloudDesc.hasRGBColors = true;
				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("Failed to allocate memory for colors! (skipped)");
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
				ccLog::Warning("Failed to allocate memory for colors! (skipped)");
			}
			break;
		}
	}

	//we compute the max index for each cloud descriptor
	maxIndex = -1;
	cloudDesc.updateMaxIndex(maxIndex);

	return cloudDesc;
}

CC_FILE_ERROR AsciiFilter::loadCloudFromFormatedAsciiFile(	const QString& filename,
															ccHObject& container,
															const AsciiOpenDlg::Sequence& openSequence,
															char separator,
															unsigned approximateNumberOfLines,
															qint64 fileSize,
															unsigned maxCloudSize,
															unsigned skipLines,
															LoadParameters& parameters)
{
	//we may have to "slice" clouds when opening them if they are too big!
	maxCloudSize = std::min(maxCloudSize,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
	unsigned cloudChunkSize = std::min(maxCloudSize,approximateNumberOfLines);
	unsigned cloudChunkPos = 0;
	unsigned chunkRank = 1;

	//we initialize the loading accelerator structure and point cloud
	int maxPartIndex = -1;
	cloudAttributesDescriptor cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, separator, chunkRank);

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
	QTextStream stream(&file);

	//we skip lines as defined on input
	{
		for (unsigned i = 0; i < skipLines; ++i)
		{
			stream.readLine();
		}
	}

	//progress indicator
	QScopedPointer<ccProgressDialog> pDlg(0);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Open ASCII file [%1]").arg(filename));
		pDlg->setInfo(QObject::tr("Approximate number of points: %1").arg(approximateNumberOfLines));
		pDlg->start();
	}
	CCLib::NormalizedProgress nprogress(pDlg.data(), approximateNumberOfLines);

	//buffers
	ScalarType D = 0;
	CCVector3d P(0, 0, 0);
	CCVector3d Pshift(0, 0, 0);
	CCVector3 N(0, 0, 0);
	ccColor::Rgb col;

	//other useful variables
	unsigned linesRead = 0;
	unsigned pointsRead = 0;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//main process
	unsigned nextLimit = /*cloudChunkPos+*/cloudChunkSize;
	QString currentLine = stream.readLine();
	while (!currentLine.isNull())
	{
		++linesRead;

		//comment
		if (currentLine.startsWith("//"))
		{
			currentLine = stream.readLine();
			continue;
		}

		if (currentLine.size() == 0)
		{
			ccLog::Warning("[AsciiFilter::Load] Line %i is corrupted (empty)!",linesRead);
			currentLine = stream.readLine();
			continue;
		}

		//if we have reached the max. number of points per cloud
		if (pointsRead == nextLimit)
		{
			ccLog::PrintDebug("[ASCII] Point %i -> end of chunk (%i points)",pointsRead,cloudChunkSize);

			//we re-evaluate the average line size
			{
				double averageLineSize = static_cast<double>(file.pos())/(pointsRead+skipLines);
				double newNbOfLinesApproximation = std::max(1.0, static_cast<double>(fileSize)/averageLineSize - static_cast<double>(skipLines));

				//if approximation is smaller than actual one, we add 2% by default
				if (newNbOfLinesApproximation <= pointsRead)
				{
					newNbOfLinesApproximation = std::max(static_cast<double>(cloudChunkPos+cloudChunkSize)+1.0,static_cast<double>(pointsRead) * 1.02);
				}
				approximateNumberOfLines = static_cast<unsigned>(ceil(newNbOfLinesApproximation));
				ccLog::PrintDebug("[ASCII] New approximate nb of lines: %i",approximateNumberOfLines);
			}

			//we try to resize actual clouds
			if (cloudChunkSize < maxCloudSize || approximateNumberOfLines-cloudChunkPos <= maxCloudSize)
			{
				ccLog::PrintDebug("[ASCII] We choose to enlarge existing clouds");

				cloudChunkSize = std::min(maxCloudSize,approximateNumberOfLines-cloudChunkPos);
				if (!cloudDesc.cloud->reserve(cloudChunkSize))
				{
					ccLog::Error("Not enough memory! Process stopped ...");
					result = CC_FERR_NOT_ENOUGH_MEMORY;
					break;
				}
			}
			else //otherwise we have to create new clouds
			{
				ccLog::PrintDebug("[ASCII] We choose to instantiate new clouds");

				//we store (and resize) actual cloud
				if (!cloudDesc.cloud->resize(cloudChunkSize))
					ccLog::Warning("Memory reallocation failed ... some memory may have been wasted ...");
				if (!cloudDesc.scalarFields.empty())
				{
					for (unsigned k=0; k<cloudDesc.scalarFields.size(); ++k)
						cloudDesc.scalarFields[k]->computeMinAndMax();
					cloudDesc.cloud->setCurrentDisplayedScalarField(0);
					cloudDesc.cloud->showSF(true);
				}
				//we add this cloud to the output container
				container.addChild(cloudDesc.cloud);
				cloudDesc.reset();

				//and create new one
				cloudChunkPos = pointsRead;
				cloudChunkSize = std::min(maxCloudSize,approximateNumberOfLines-cloudChunkPos);
				cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, separator, ++chunkRank);
				if (!cloudDesc.cloud)
				{
					ccLog::Error("Not enough memory! Process stopped ...");
					break;
				}
				cloudDesc.cloud->setGlobalShift(Pshift);
			}

			//we update the progress info
			if (pDlg)
			{
				nprogress.scale(approximateNumberOfLines, 100, true);
				pDlg->setInfo(QObject::tr("Approximate number of points: %1").arg(approximateNumberOfLines));
			}

			nextLimit = cloudChunkPos+cloudChunkSize;
		}

		//we split current line
		QStringList parts = currentLine.split(separator,QString::SkipEmptyParts);

		int nParts = parts.size();
		if (nParts > maxPartIndex)
		{
			//(X,Y,Z)
			if (cloudDesc.xCoordIndex >= 0)
				P.x = parts[cloudDesc.xCoordIndex].toDouble();
			if (cloudDesc.yCoordIndex >= 0)
				P.y = parts[cloudDesc.yCoordIndex].toDouble();
			if (cloudDesc.zCoordIndex >= 0)
				P.z = parts[cloudDesc.zCoordIndex].toDouble();

			//first point: check for 'big' coordinates
			if (pointsRead == 0)
			{
				if (HandleGlobalShift(P,Pshift,parameters))
				{
					cloudDesc.cloud->setGlobalShift(Pshift);
					ccLog::Warning("[ASCIIFilter::loadFile] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)",Pshift.x,Pshift.y,Pshift.z);
				}
			}

			//add point
			cloudDesc.cloud->addPoint(CCVector3::fromArray((P+Pshift).u));

			//Normal vector
			if (cloudDesc.hasNorms)
			{
				if (cloudDesc.xNormIndex >= 0)
					N.x = static_cast<PointCoordinateType>(parts[cloudDesc.xNormIndex].toDouble());
				if (cloudDesc.yNormIndex >= 0)
					N.y = static_cast<PointCoordinateType>(parts[cloudDesc.yNormIndex].toDouble());
				if (cloudDesc.zNormIndex >= 0)
					N.z = static_cast<PointCoordinateType>(parts[cloudDesc.zNormIndex].toDouble());
				cloudDesc.cloud->addNorm(N);
			}

			//Colors
			if (cloudDesc.hasRGBColors)
			{
				if (cloudDesc.iRgbaIndex >= 0)
				{
					const uint32_t rgb = parts[cloudDesc.iRgbaIndex].toInt();
					col.r = ((rgb >> 16) & 0x0000ff);
					col.g = ((rgb >> 8 ) & 0x0000ff);
					col.b = ((rgb      ) & 0x0000ff);

				}
				else if (cloudDesc.fRgbaIndex >= 0)
				{
					const float rgbf = parts[cloudDesc.fRgbaIndex].toFloat();
					const uint32_t rgb = (uint32_t)(*((uint32_t*)&rgbf));
					col.r = ((rgb >> 16) & 0x0000ff);
					col.g = ((rgb >> 8 ) & 0x0000ff);
					col.b = ((rgb      ) & 0x0000ff);
				}
				else
				{
					if (cloudDesc.redIndex >= 0)
					{
						float multiplier = cloudDesc.hasFloatRGBColors[0] ? static_cast<float>(ccColor::MAX) : 1.0f;
						col.r = static_cast<ColorCompType>(parts[cloudDesc.redIndex].toFloat() * multiplier);
					}
					if (cloudDesc.greenIndex >= 0)
					{
						float multiplier = cloudDesc.hasFloatRGBColors[1] ? static_cast<float>(ccColor::MAX) : 1.0f;
						col.g = static_cast<ColorCompType>(parts[cloudDesc.greenIndex].toFloat() * multiplier);
					}
					if (cloudDesc.blueIndex >= 0)
					{
						float multiplier = cloudDesc.hasFloatRGBColors[2] ? static_cast<float>(ccColor::MAX) : 1.0f;
						col.b = static_cast<ColorCompType>(parts[cloudDesc.blueIndex].toFloat() * multiplier);
					}
				}
				cloudDesc.cloud->addRGBColor(col.rgb);
			}
			else if (cloudDesc.greyIndex >= 0)
			{
				col.r = col.r = col.b = static_cast<ColorCompType>(parts[cloudDesc.greyIndex].toInt());
				cloudDesc.cloud->addRGBColor(col.rgb);
			}

			//Scalar distance
			if (!cloudDesc.scalarIndexes.empty())
			{
				for (size_t j=0; j<cloudDesc.scalarIndexes.size(); ++j)
				{
					D = static_cast<ScalarType>( parts[cloudDesc.scalarIndexes[j]].toDouble() );
					cloudDesc.scalarFields[j]->setValue(pointsRead-cloudChunkPos,D);
				}
			}

			++pointsRead;
		}
		else
		{
			ccLog::Warning("[AsciiFilter::Load] Line %i is corrupted (found %i part(s) on %i expected)!",linesRead,nParts,maxPartIndex+1);
		}

		if (pDlg && !nprogress.oneStep())
		{
			//cancel requested
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}

		//read next line
		currentLine = stream.readLine();
	}

	file.close();

	if (cloudDesc.cloud)
	{
		if (cloudDesc.cloud->size() < cloudDesc.cloud->capacity())
			cloudDesc.cloud->resize(cloudDesc.cloud->size());

		//add cloud to output
		if (!cloudDesc.scalarFields.empty())
		{
			for (size_t j = 0; j < cloudDesc.scalarFields.size(); ++j)
			{
				cloudDesc.scalarFields[j]->resize(cloudDesc.cloud->size(), true, NAN_VALUE);
				cloudDesc.scalarFields[j]->computeMinAndMax();
			}
			cloudDesc.cloud->setCurrentDisplayedScalarField(0);
			cloudDesc.cloud->showSF(true);
		}

		container.addChild(cloudDesc.cloud);
	}

	return result;
}
