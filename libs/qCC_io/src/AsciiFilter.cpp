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
#include <QSharedPointer>
#include <QTextStream>

//CClib
#include <ScalarField.h>

//qCC_db
#include <cc2DLabel.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccCoordinateSystem.h>

//System
#include <cassert>
#include <cstring>

//Qt
#include <QScopedPointer>

// Semi-persistent parameters
static int s_defaultSkippedLineCount = 0;
static int s_outputCoordPrecision = 8;
static int s_outputSFPrecision = 6;
static int s_outputSeparatorIndex = 0;
static bool s_saveSFBeforeColor = false;
static bool s_saveColumnsNamesHeader = false;
static bool s_savePointCountHeader = false;
static bool s_doNotCreateLabels = false;

void AsciiFilter::SetDefaultSkippedLineCount(int count)
{
	s_defaultSkippedLineCount = count;
}

void AsciiFilter::SetNoLabelCreated(bool state)
{
	s_doNotCreateLabels = state;
}

void AsciiFilter::SetOutputCoordsPrecision(int prec)
{
	s_outputCoordPrecision = prec;
}

void AsciiFilter::SetOutputSFPrecision(int prec)
{
	s_outputSFPrecision = prec;
}

void AsciiFilter::SetOutputSeparatorIndex(int separatorIndex)
{
	s_outputSeparatorIndex = separatorIndex;
}

void AsciiFilter::SaveSFBeforeColor(bool state)
{
	s_saveSFBeforeColor = state;
}

void AsciiFilter::SaveColumnsNamesHeader(bool state)
{
	s_saveColumnsNamesHeader = state;
}

void AsciiFilter::SavePointCountHeader(bool state)
{
	s_savePointCountHeader = state;
}

AsciiFilter::AsciiFilter()
	: FileIOFilter( {
					"_ASCII Filter",
					2.0f,	// priority
					QStringList{ "txt", "asc", "neu", "xyz", "pts", "csv" },
					"asc",
					QStringList{ GetFileFilter() },
					QStringList{ GetFileFilter() },
					Import | Export | BuiltIn
					} )
{
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

CC_FILE_ERROR AsciiFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	assert(entity && !filename.isEmpty());

	AsciiSaveDlg saveDialog(parameters.parentWidget);

	saveDialog.setCoordsPrecision(s_outputCoordPrecision);
	saveDialog.setSfPrecision(s_outputSFPrecision);
	saveDialog.setSeparatorIndex(s_outputSeparatorIndex);
	saveDialog.enableSwapColorAndSF(s_saveSFBeforeColor);
	saveDialog.enableSaveColumnsNamesHeader(s_saveColumnsNamesHeader);
	saveDialog.enableSavePointCountHeader(s_savePointCountHeader);

	//if the dialog shouldn't be shown, we'll simply take the default values!
	static bool s_showDialog = true;
	if (parameters.alwaysDisplaySaveDialog && s_showDialog)
	{
		if (!saveDialog.exec())
		{
			return CC_FERR_CANCELED_BY_USER;
		}
		s_outputCoordPrecision = saveDialog.coordsPrecision();
		s_outputSFPrecision = saveDialog.sfPrecision();
		s_outputSeparatorIndex = saveDialog.getSeparatorIndex();
		s_saveSFBeforeColor = saveDialog.swapColorAndSF();
		s_saveColumnsNamesHeader = saveDialog.saveColumnsNamesHeader();
		s_savePointCountHeader = saveDialog.savePointCountHeader();
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
				for (unsigned i = 0; i < count; ++i)
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

				bool autoShow = s_showDialog;
				s_showDialog = false;

				for (unsigned i = 0; i < count; ++i)
				{
					ccHObject* child = entity->getChild(i);
					if (child->isKindOf(CC_TYPES::POINT_CLOUD))
					{
						QString subFilename = path + QString("/");
						subFilename += QString(baseName).replace("cloudname", child->getName(), Qt::CaseInsensitive);
						counter++;
						assert(counter <= cloudCount);
						subFilename += QString("_%1").arg(cloudCount - counter, 6, 10, QChar('0'));
						if (!extension.isEmpty())
							subFilename += QString(".") + extension;

						CC_FILE_ERROR result = saveToFile(entity->getChild(i), subFilename, parameters);
						if (result != CC_FERR_NO_ERROR)
						{
							return result;
						}
						else
						{
							ccLog::Print(QString("[ASCII] Cloud '%1' has been saved in: %2").arg(child->getName(), subFilename));
						}
					}
					else
					{
						ccLog::Warning(QString("[ASCII] Entity '%1' can't be saved this way!").arg(child->getName()));
					}
				}

				//restore previous state
				s_showDialog = autoShow;

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
	bool writeSF = (!theScalarFields.empty());

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Saving cloud [%1]").arg(cloud->getName()));
		pDlg->setInfo(QObject::tr("Number of points: %1").arg(numberOfPoints));
		pDlg->start();
	}
	CCCoreLib::NormalizedProgress nprogress(pDlg.data(), numberOfPoints);

	//non static parameters
	int normalPrecision = 2 + sizeof(PointCoordinateType);
	QChar separator = saveDialog.getSeparator();
	bool saveFloatColors = saveDialog.saveFloatColors();
	bool saveAlphaChannel = saveDialog.saveAlphaChannel();

	if (s_saveColumnsNamesHeader)
	{
		QString header("//");
		header.append(AsciiHeaderColumns::X());
		header.append(separator);
		header.append(AsciiHeaderColumns::Y());
		header.append(separator);
		header.append(AsciiHeaderColumns::Z());

		if (writeColors && !s_saveSFBeforeColor)
		{
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Rf() : AsciiHeaderColumns::R());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Gf() : AsciiHeaderColumns::G());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Bf() : AsciiHeaderColumns::B());
			if (saveAlphaChannel)
			{
				header.append(separator);
				header.append(saveFloatColors ? AsciiHeaderColumns::Af() : AsciiHeaderColumns::A());
			}
		}

		if (writeSF)
		{
			//add each associated SF name
			for (std::vector<ccScalarField*>::const_iterator it = theScalarFields.begin(); it != theScalarFields.end(); ++it)
			{
				QString sfName(QString::fromStdString((*it)->getName()));
				sfName.replace(separator, '_');
				header.append(separator);
				header.append(sfName);
			}
		}

		if (writeColors && s_saveSFBeforeColor)
		{
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Rf() : AsciiHeaderColumns::R());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Gf() : AsciiHeaderColumns::G());
			header.append(separator);
			header.append(saveFloatColors ? AsciiHeaderColumns::Bf() : AsciiHeaderColumns::B());
			if (saveAlphaChannel)
			{
				header.append(separator);
				header.append(saveFloatColors ? AsciiHeaderColumns::Af() : AsciiHeaderColumns::A());
			}
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

	if (s_savePointCountHeader)
	{
		stream << QString::number(numberOfPoints) << "\n";
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	for (unsigned i = 0; i < numberOfPoints; ++i)
	{
		//line for the current point
		QString line;

		//write current point coordinates
		const CCVector3* P = cloud->getPoint(i);
		CCVector3d Pglobal = cloud->toGlobal3d<PointCoordinateType>(*P);
		line.append(QString::number(Pglobal.x, 'f', s_outputCoordPrecision));
		line.append(separator);
		line.append(QString::number(Pglobal.y, 'f', s_outputCoordPrecision));
		line.append(separator);
		line.append(QString::number(Pglobal.z, 'f', s_outputCoordPrecision));

		QString colorLine;
		if (writeColors)
		{
			//add rgb color
			const ccColor::Rgba& col = cloud->getPointColor(i);
			if (saveFloatColors)
			{
				colorLine.append(separator);
				colorLine.append(QString::number(static_cast<double>(col.r) / ccColor::MAX));
				colorLine.append(separator);
				colorLine.append(QString::number(static_cast<double>(col.g) / ccColor::MAX));
				colorLine.append(separator);
				colorLine.append(QString::number(static_cast<double>(col.b) / ccColor::MAX));
				if (saveAlphaChannel)
				{
					colorLine.append(separator);
					colorLine.append(QString::number(static_cast<double>(col.a) / ccColor::MAX));
				}
			}
			else
			{
				colorLine.append(separator);
				colorLine.append(QString::number(col.r));
				colorLine.append(separator);
				colorLine.append(QString::number(col.g));
				colorLine.append(separator);
				colorLine.append(QString::number(col.b));
				if (saveAlphaChannel)
				{
					colorLine.append(separator);
					colorLine.append(QString::number(col.a));
				}
			}

			if (!s_saveSFBeforeColor)
			{
				line.append(colorLine);
			}
		}

		if (writeSF)
		{
			//add each associated SF values
			for (std::vector<ccScalarField*>::const_iterator it = theScalarFields.begin(); it != theScalarFields.end(); ++it)
			{
				line.append(separator);
				ScalarType sfVal = (*it)->getValue(i);
				line.append(QString::number(sfVal, 'f', s_outputSFPrecision));
			}
		}

		if (writeColors && s_saveSFBeforeColor)
			line.append(colorLine);

		if (writeNorms)
		{
			//add normal vector
			const CCVector3& N = cloud->getPointNormal(i);
			line.append(separator);
			line.append(QString::number(N.x, 'f', normalPrecision));
			line.append(separator);
			line.append(QString::number(N.y, 'f', normalPrecision));
			line.append(separator);
			line.append(QString::number(N.z, 'f', normalPrecision));
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

CC_FILE_ERROR AsciiFilter::loadFile(const QString& filename,
									ccHObject& container,
									LoadParameters& parameters)
{
	QFile file(filename);
	if (!file.exists())
	{
		return CC_FERR_UNKNOWN_FILE;
	}
	if (!file.open(QFile::ReadOnly))
	{
		return CC_FERR_READING;
	}

	QTextStream stream(&file);

	return loadStream(stream, filename, file.size(), container, parameters);
}

CC_FILE_ERROR AsciiFilter::loadAsciiData(	const QByteArray& data,
											QString sourceName,
											ccHObject& container,
											LoadParameters& parameters)
{
	QTextStream stream(data);

	return loadStream(stream, sourceName, data.size(), container, parameters);
}

CC_FILE_ERROR AsciiFilter::loadStream(	QTextStream& stream,
										QString filenameOrTitle,
										qint64 dataSize,
										ccHObject& container,
										LoadParameters& parameters)
{
	if (dataSize == 0)
	{
		return CC_FERR_NO_LOAD;
	}

	bool forceDialogDisplay = parameters.alwaysDisplayLoadDialog;

	AsciiOpenDlg openDialog(parameters.parentWidget);
	if (openDialog.setInput(filenameOrTitle, &stream))
	{
		//if we should use the previous sequence ('Apply all')
		forceDialogDisplay = false;
	}

	QString dummyStr;
	if (	forceDialogDisplay
		|| !AsciiOpenDlg::CheckOpenSequence(openDialog.getOpenSequence(), dummyStr))
	{
		//show the dialog
		if (!openDialog.exec())
		{
			//process was cancelled
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	//we compute the approximate line number
	double averageLineSize = openDialog.getAverageLineSize();
	unsigned approximateNumberOfLines = static_cast<unsigned>(ceil(dataSize / averageLineSize));

	AsciiOpenDlg::Sequence openSequence = openDialog.getOpenSequence();
	char separator = static_cast<char>(openDialog.getSeparator());
	bool commaAsDecimal = openDialog.useCommaAsDecimal();
	unsigned maxCloudSize = openDialog.getMaxCloudSize();
	unsigned skipLineCount = openDialog.getSkippedLinesCount();
	bool showLabelsIn2D = openDialog.showLabelsIn2D();
	double quaternionScale = openDialog.getQuaternionScale();

	return loadCloudFromFormatedAsciiStream(stream,
											filenameOrTitle,
											container,
											openSequence,
											separator,
											commaAsDecimal,
											approximateNumberOfLines,
											dataSize,
											maxCloudSize,
											skipLineCount,
											quaternionScale,
											parameters,
											showLabelsIn2D);
}

struct cloudAttributesDescriptor
{
	ccPointCloud* cloud;
	static const unsigned c_attribCount = 18;
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
				int alphaIndex;
				int iRgbaIndex;
				int fRgbaIndex;
				int greyIndex;
				int qwIndex;
				int qxIndex;
				int qyIndex;
				int qzIndex;
				int labelIndex;
		};
		int indexes[c_attribCount];
	};
	std::vector<int> scalarIndexes;
	std::vector<CCCoreLib::ScalarField*> scalarFields;
	bool hasNorms;
	bool hasRGBColors;
	bool hasFloatRGBColors[4];
	bool hasQuaternion;

	cloudAttributesDescriptor()
	{
		reset();
	}

	void reset()
	{
		cloud = nullptr;
		for (unsigned i = 0; i < c_attribCount; ++i)
		{
			indexes[i] = -1;
		}
		hasNorms = false;
		hasRGBColors = false;
		hasFloatRGBColors[0] = hasFloatRGBColors[1] = hasFloatRGBColors[2] = hasFloatRGBColors[3] = false;
		hasQuaternion = false;
		
		scalarIndexes.clear();
		scalarFields.clear();
	}

	void updateMaxIndex(int& maxIndex)
	{
		for (int attribIndex : indexes)
			if (attribIndex > maxIndex)
				maxIndex = attribIndex;

		for (int sfIndex : scalarIndexes)
			if (sfIndex > maxIndex)
				maxIndex = sfIndex;
	}

};

void clearStructure(cloudAttributesDescriptor &cloudDesc)
{
	delete cloudDesc.cloud;
	cloudDesc.cloud = nullptr;
	cloudDesc.reset();
}

cloudAttributesDescriptor prepareCloud(	const AsciiOpenDlg::Sequence& openSequence,
										unsigned numberOfPoints,
										int& maxIndex,
										unsigned step = 1)
{
	ccPointCloud* cloud = new ccPointCloud();
	if (!cloud || !cloud->reserveThePointsTable(numberOfPoints))
	{
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
	unsigned char quaternionComponents = 0;
	for (int i = 0; i < seqSize; ++i)
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
				int sfIndex = cloud->getNumberOfScalarFields() + 1;
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
					sfName.replace('_', ' ');
				}

				ccScalarField* sf = new ccScalarField(sfName.toStdString());
				int sfIdx = cloud->addScalarField(sf);
				if (sfIdx >= 0)
				{
					cloudDesc.scalarIndexes.push_back(i);
					cloudDesc.scalarFields.push_back(sf);
				}
				else
				{
					ccLog::Warning("Failed to add scalar field #%i to cloud #%i! (skipped)", sfIndex);
					sf->release();
					sf = nullptr;
				}
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
		case ASCII_OPEN_DLG_Af:
			cloudDesc.hasFloatRGBColors[3] = true;
		case ASCII_OPEN_DLG_A:
			if (cloud->reserveTheRGBTable())
			{
				cloudDesc.alphaIndex = i;
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
		case ASCII_OPEN_DLG_QuatW:
			cloudDesc.qwIndex = i;
			cloudDesc.hasQuaternion = (++quaternionComponents == 4);
			break;
		case ASCII_OPEN_DLG_QuatX:
			cloudDesc.qxIndex = i;
			cloudDesc.hasQuaternion = (++quaternionComponents == 4);
			break;
		case ASCII_OPEN_DLG_QuatY:
			cloudDesc.qyIndex = i;
			cloudDesc.hasQuaternion = (++quaternionComponents == 4);
			break;
		case ASCII_OPEN_DLG_QuatZ:
			cloudDesc.qzIndex = i;
			cloudDesc.hasQuaternion = (++quaternionComponents == 4);
			break;
		case ASCII_OPEN_DLG_Label:
			assert(cloudDesc.labelIndex < 0); //There Can Be Only One
			cloudDesc.labelIndex = i;
			break;
		default:
			//unhandled case?
			assert(false);
			break;
		}
	}

	//we compute the max index for each cloud descriptor
	maxIndex = -1;
	cloudDesc.updateMaxIndex(maxIndex);

	return cloudDesc;
}

CC_FILE_ERROR AsciiFilter::loadCloudFromFormatedAsciiStream(QTextStream& stream,
															QString filenameOrTitle,
															ccHObject& container,
															const AsciiOpenDlg::Sequence& openSequence,
															char separator,
															bool commaAsDecimal,
															unsigned approximateNumberOfLines,
															qint64 fileSize,
															unsigned maxCloudSize,
															unsigned skipLines,
															double quaternionScale,
															LoadParameters& parameters,
															bool showLabelsIn2D/*=false*/)
{
	//we may have to "slice" clouds when opening them if they are too big!
	maxCloudSize = std::min(maxCloudSize, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
	unsigned cloudChunkSize = std::min(maxCloudSize, approximateNumberOfLines);
	unsigned cloudChunkPos = 0;
	unsigned chunkRank = 1;

	//we initialize the loading accelerator structure and point cloud
	int maxPartIndex = -1;
	cloudAttributesDescriptor cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, chunkRank);

	if (!cloudDesc.cloud)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	//just in case
	stream.seek(0);
	qint64 charactersRead = 0;

	//we skip lines as defined on input
	{
		for (unsigned i = 0; i < skipLines;)
		{
			QString currentLine = stream.readLine();
			if (currentLine.isEmpty())
			{
				//empty lines are ignored
				continue;
			}
			charactersRead += currentLine.size();
			++i;
		}
	}

	//progress indicator
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Open ASCII data [%1]").arg(filenameOrTitle));
		pDlg->setInfo(QObject::tr("Approximate number of points: %1").arg(approximateNumberOfLines));
		pDlg->start();
	}
	CCCoreLib::NormalizedProgress nprogress(pDlg.data(), approximateNumberOfLines);

	//buffers
	CCVector3d P(0, 0, 0);
	CCVector3d Pshift(0, 0, 0);
	CCVector3 N(0, 0, 0);
	ccColor::Rgba col(0, 0, 0, 255);
	bool preserveCoordinateShift = true;

	//other useful variables
	unsigned linesRead = 0;
	unsigned pointsRead = 0;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	QLocale locale(commaAsDecimal ? QLocale::French : QLocale::English);

	//main process
	unsigned nextLimit = /*cloudChunkPos+*/cloudChunkSize;
	while (true)
	{
		//read next line
		QString currentLine = stream.readLine();
		if (currentLine.isNull())
		{
			//end of file
			break;
		}
		charactersRead += currentLine.size();
		++linesRead;

		if (currentLine.isEmpty() || currentLine.startsWith("//"))
		{
			//empty lines and comments are ignored
			continue;
		}

		//if we have reached the max. number of points per cloud
		if (pointsRead == nextLimit)
		{
			ccLog::PrintDebug("[ASCII] Point %i -> end of chunk (%i points)", pointsRead, cloudChunkSize);

			//we re-evaluate the average line size
			{
				double averageLineSize = static_cast<double>(charactersRead) / (pointsRead + skipLines);
				double newNbOfLinesApproximation = std::max(1.0, static_cast<double>(fileSize) / averageLineSize - static_cast<double>(skipLines));

				//if approximation is smaller than actual one, we add 2% by default
				if (newNbOfLinesApproximation <= pointsRead)
				{
					newNbOfLinesApproximation = std::max(static_cast<double>(cloudChunkPos + cloudChunkSize) + 1.0, static_cast<double>(pointsRead)* 1.02);
				}
				approximateNumberOfLines = static_cast<unsigned>(ceil(newNbOfLinesApproximation));
				ccLog::PrintDebug("[ASCII] New approximate nb of lines: %i", approximateNumberOfLines);
			}

			//we try to resize actual clouds
			if (cloudChunkSize < maxCloudSize || approximateNumberOfLines - cloudChunkPos <= maxCloudSize)
			{
				ccLog::PrintDebug("[ASCII] We choose to enlarge existing clouds");

				cloudChunkSize = std::min(maxCloudSize, approximateNumberOfLines - cloudChunkPos);
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
					for (unsigned k = 0; k < cloudDesc.scalarFields.size(); ++k)
						cloudDesc.scalarFields[k]->computeMinAndMax();
					cloudDesc.cloud->setCurrentDisplayedScalarField(0);
					cloudDesc.cloud->showSF(true);
				}
				//we add this cloud to the output container
				container.addChild(cloudDesc.cloud);
				cloudDesc.reset();

				//and create new one
				cloudChunkPos = pointsRead;
				cloudChunkSize = std::min(maxCloudSize, approximateNumberOfLines - cloudChunkPos);
				cloudDesc = prepareCloud(openSequence, cloudChunkSize, maxPartIndex, ++chunkRank);
				if (!cloudDesc.cloud)
				{
					ccLog::Error("Not enough memory! Process stopped ...");
					break;
				}
				if (preserveCoordinateShift)
				{
					cloudDesc.cloud->setGlobalShift(Pshift);
				}
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
		QStringList parts = currentLine.simplified().split(separator, QString::SkipEmptyParts);

		int nParts = parts.size();
		if (nParts > maxPartIndex) //fake loop for easy break
		{
			//read the point coordinates
			bool lineIsCorrupted = true;
			for (int step = 0; step < 1; ++step) //fake loop for easy break
			{
				bool ok = true;
				if (cloudDesc.xCoordIndex >= 0)
				{
					P.x = locale.toDouble(parts[cloudDesc.xCoordIndex], &ok);
					if (!ok)
					{
						break;
					}
				}
				if (cloudDesc.yCoordIndex >= 0)
				{
					P.y = locale.toDouble(parts[cloudDesc.yCoordIndex], &ok);
					if (!ok)
					{
						break;
					}
				}
				if (cloudDesc.zCoordIndex >= 0)
				{
					P.z = locale.toDouble(parts[cloudDesc.zCoordIndex], &ok);
					if (!ok)
					{
						break;
					}
				}

				lineIsCorrupted = false;
			}

			if (lineIsCorrupted)
			{
				ccLog::Warning("[AsciiFilter::Load] Line %i is corrupted (non numerical value found)", linesRead);
				continue;
			}

			//first point: check for 'big' coordinates
			if (pointsRead == 0)
			{
				if (HandleGlobalShift(P, Pshift, preserveCoordinateShift, parameters))
				{
					if (preserveCoordinateShift)
					{
						cloudDesc.cloud->setGlobalShift(Pshift);
					}
					ccLog::Warning("[ASCIIFilter::loadFile] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}
			}

			//add point
			cloudDesc.cloud->addPoint((P + Pshift).toPC());

			//Normal vector
			if (cloudDesc.hasNorms)
			{
				if (cloudDesc.xNormIndex >= 0)
					N.x = static_cast<PointCoordinateType>(locale.toDouble(parts[cloudDesc.xNormIndex]));
				if (cloudDesc.yNormIndex >= 0)
					N.y = static_cast<PointCoordinateType>(locale.toDouble(parts[cloudDesc.yNormIndex]));
				if (cloudDesc.zNormIndex >= 0)
					N.z = static_cast<PointCoordinateType>(locale.toDouble(parts[cloudDesc.zNormIndex]));
				cloudDesc.cloud->addNorm(N);
			}

			//Colors
			if (cloudDesc.hasRGBColors)
			{
				if (cloudDesc.iRgbaIndex >= 0)
				{
					const uint32_t rgba = parts[cloudDesc.iRgbaIndex].toInt();
					col.a = ((rgba >> 24) & 0x0000ff);
					col.r = ((rgba >> 16) & 0x0000ff);
					col.g = ((rgba >>  8) & 0x0000ff);
					col.b = ((rgba      ) & 0x0000ff);

				}
				else if (cloudDesc.fRgbaIndex >= 0)
				{
					const float rgbaf = locale.toFloat(parts[cloudDesc.fRgbaIndex]);
					const uint32_t rgba = *(reinterpret_cast<const uint32_t *>(&rgbaf));
					col.a = ((rgba >> 24) & 0x0000ff);
					col.r = ((rgba >> 16) & 0x0000ff);
					col.g = ((rgba >>  8) & 0x0000ff);
					col.b = ((rgba      ) & 0x0000ff);
				}
				else
				{
					if (cloudDesc.redIndex >= 0)
					{
						float multiplier = cloudDesc.hasFloatRGBColors[0] ? static_cast<float>(ccColor::MAX) : 1.0f;
						col.r = static_cast<ColorCompType>(locale.toFloat(parts[cloudDesc.redIndex]) * multiplier);
					}
					if (cloudDesc.greenIndex >= 0)
					{
						float multiplier = cloudDesc.hasFloatRGBColors[1] ? static_cast<float>(ccColor::MAX) : 1.0f;
						col.g = static_cast<ColorCompType>(locale.toFloat(parts[cloudDesc.greenIndex]) * multiplier);
					}
					if (cloudDesc.blueIndex >= 0)
					{
						float multiplier = cloudDesc.hasFloatRGBColors[2] ? static_cast<float>(ccColor::MAX) : 1.0f;
						col.b = static_cast<ColorCompType>(locale.toFloat(parts[cloudDesc.blueIndex]) * multiplier);
					}
					if (cloudDesc.alphaIndex >= 0)
					{
						float multiplier = cloudDesc.hasFloatRGBColors[3] ? static_cast<float>(ccColor::MAX) : 1.0f;
						col.a = static_cast<ColorCompType>(locale.toFloat(parts[cloudDesc.alphaIndex]) * multiplier);
					}
				}
				cloudDesc.cloud->addColor(col);
			}
			else if (cloudDesc.greyIndex >= 0)
			{
				col.r = col.g = col.b = static_cast<ColorCompType>(parts[cloudDesc.greyIndex].toInt());
				col.a = ccColor::MAX;
				cloudDesc.cloud->addColor(col);
			}

			//Scalar distance
			if (!cloudDesc.scalarIndexes.empty())
			{
				for (size_t j = 0; j < cloudDesc.scalarIndexes.size(); ++j)
				{
					ScalarType sfValue = static_cast<ScalarType>(locale.toDouble(parts[cloudDesc.scalarIndexes[j]]));
					cloudDesc.scalarFields[j]->addElement(sfValue);
				}
			}

			//Quaternion
			if (cloudDesc.hasQuaternion)
			{
				double quat[4] { locale.toDouble(parts[cloudDesc.qwIndex]),
								 locale.toDouble(parts[cloudDesc.qxIndex]),
								 locale.toDouble(parts[cloudDesc.qyIndex]),
								 locale.toDouble(parts[cloudDesc.qzIndex])
				};

				ccGLMatrix mat = ccGLMatrix::FromQuaternion(quat);
				mat.setTranslation((P + Pshift).u);

				ccCoordinateSystem* cs = new ccCoordinateSystem(ccCoordinateSystem::DEFAULT_DISPLAY_SCALE,
																CCCoreLib::PC_ONE,
																&mat,
																QString("Quaternion #%1").arg(cloudDesc.cloud->size()));
				cs->setVisible(true);
				cs->showAxisPlanes(false);
				cs->showAxisLines(true);
				cs->setDisplayScale(static_cast<PointCoordinateType>(quaternionScale));
				cloudDesc.cloud->addChild(cs);
			}

			//Label
			if (cloudDesc.labelIndex >= 0 && !s_doNotCreateLabels)
			{
				cc2DLabel* label = new cc2DLabel();
				label->addPickedPoint(cloudDesc.cloud, cloudDesc.cloud->size() - 1);
				label->setName(parts[cloudDesc.labelIndex]);
				label->setDisplayedIn2D(showLabelsIn2D);
				label->displayPointLegend(!showLabelsIn2D);
				label->setVisible(true);
				cloudDesc.cloud->addChild(label);
			}

			++pointsRead;
		}
		else
		{
			ccLog::Warning("[AsciiFilter::Load] Line %i is corrupted (found %i part(s) on %i expected)!", linesRead, nParts, maxPartIndex + 1);
		}

		if (pDlg && !nprogress.oneStep())
		{
			//cancel requested
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}
	}

	if (cloudDesc.cloud)
	{
		if (cloudDesc.cloud->size() < cloudDesc.cloud->capacity())
		{
			cloudDesc.cloud->resize(cloudDesc.cloud->size());
		}

		//add cloud to output
		if (!cloudDesc.scalarFields.empty())
		{
			for (size_t j = 0; j < cloudDesc.scalarFields.size(); ++j)
			{
				if (cloudDesc.scalarFields[j]->resizeSafe(cloudDesc.cloud->size(), true, CCCoreLib::NAN_VALUE))
				{
					cloudDesc.scalarFields[j]->computeMinAndMax();
				}
				else
				{
					// nothing will happen, we just use too much memory...
				}
			}
			cloudDesc.cloud->setCurrentDisplayedScalarField(0);
			cloudDesc.cloud->showSF(true);
		}

		//position the labels
		if (cloudDesc.labelIndex >= 0 && !s_doNotCreateLabels)
		{
			ccHObject::Container labels;
			cloudDesc.cloud->filterChildren(labels, false, CC_TYPES::LABEL_2D, true);

			if (labels.size() > 1)
			{
				double angle_rad = (M_PI * 2) / labels.size();
				for (size_t i = 0; i < labels.size(); ++i)
				{
					//position the labels on a circle
					static_cast<cc2DLabel*>(labels[i])->setPosition(0.5 + 0.4 * cos(i * angle_rad), 0.5 + 0.4 * sin(i * angle_rad));
				}
			}
		}

		container.addChild(cloudDesc.cloud);
	}

	return result;
}
