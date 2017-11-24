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
//#                        COPYRIGHT: CNRS / OSUR                          #
//#                                                                        #
//##########################################################################

#include "SimpleBinFilter.h"

//Qt
#include <QFileInfo>
#include <QSettings>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//system
#include <assert.h>

//header: 32 first bytes
static const size_t c_headerSize = 32;
//header flag
static const uint16_t s_headerFlagSBF = (static_cast<uint16_t>(42) | static_cast<uint16_t>(42 << 8));

bool SimpleBinFilter::canLoadExtension(QString upperCaseExt) const
{
	return (upperCaseExt == "SBF" || upperCaseExt == "DATA");
}

bool SimpleBinFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	multiple = false;
	exclusive = true;
	return (type == CC_TYPES::POINT_CLOUD);
}

CC_FILE_ERROR SimpleBinFilter::saveToFile(ccHObject* root, QString filename, SaveParameters& parameters)
{
	if (!root || filename.isNull() || !root->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		return CC_FERR_BAD_ARGUMENT;
	}

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(root);
	if (!cloud)
	{
		return CC_FERR_BAD_ARGUMENT;
	}

	QString headerFilename = filename;
	QString dataFilename = filename + ".data";

	ccLog::Print(QString("[SBF] Saving file '%1'...").arg(headerFilename));

	//read the text file as an INI file
	if (QFileInfo(headerFilename).exists())
	{
		QSettings headerFile(headerFilename, QSettings::IniFormat);
		
		headerFile.setValue("Points", cloud->size());

		//save the global shift (if any)
		const CCVector3d& globalShift = cloud->getGlobalShift();
		if (globalShift.norm2() != 0)
		{
			QStringList strGlobalShift;
			strGlobalShift << QString::number(globalShift.x, 'f', 6);
			strGlobalShift << QString::number(globalShift.y, 'f', 6);
			strGlobalShift << QString::number(globalShift.z, 'f', 6);
			headerFile.setValue("GlobalShift", strGlobalShift);
		}

		//save the scalar field names (if any)
		if (cloud->hasScalarFields())
		{
			unsigned sfCount = cloud->getNumberOfScalarFields();
			headerFile.setValue("SFCount", sfCount);

			//try to load the description of each SF
			for (int i = 0; i < static_cast<int>(sfCount); ++i)
			{
				QString key = QString("SF%1").arg(i + 1);
				QString sfName = cloud->getScalarFieldName(i);

				QStringList tokens;
				tokens << sfName;

				QString precisionKey = QString("{%1}.precision").arg(sfName);
				if (cloud->hasMetaData(precisionKey))
				{
					bool ok = false;
					double precision = cloud->getMetaData(precisionKey).toDouble(&ok);
					if (ok)
					{
						tokens << QString::number(precision, 'f', 12);
					}
				}

				headerFile.setValue(key, tokens);
			}
		}
	}

	//we can now load the data file
	QFile dataFile(dataFilename);
	if (!dataFile.open(QFile::WriteOnly))
	{
		ccLog::Warning("[SBF] Failed to read the data file");
		return CC_FERR_READING;
	}

	QDataStream dataStream(&dataFile);
	size_t writtenBytes = 0;
	//header
	{
		//2 bytes = header flag
		{
			dataStream << s_headerFlagSBF;
		}
		writtenBytes += 2;

		//8 bytes = point count
		{
			uint64_t pointCount = cloud->size();
			dataStream << pointCount;
		}
		writtenBytes += 8;

		//2 bytes = sf count
		{
			uint16_t sfCount = static_cast<uint16_t>(cloud->getNumberOfScalarFields());
			dataStream << sfCount;
		}
		writtenBytes += 2;

		//remaining bytes (empty for now)
		for (; writtenBytes < c_headerSize; ++writtenBytes)
		{
			uint8_t byte = 0;
			dataStream >> byte;
		}
	}

	unsigned sfCount = cloud->getNumberOfScalarFields();
	unsigned pointCount = cloud->size();

	QScopedPointer<ccProgressDialog> pDlg(0);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Simple BIN file"));
		pDlg->setInfo(QObject::tr("Saving %1 points / %2 scalar field(s)").arg(pointCount).arg(sfCount));
		pDlg->setModal(true);
		pDlg->start();
	}

	CCLib::NormalizedProgress nProgress(pDlg.data(), pointCount);

	//we can eventually save the data
	for (unsigned i = 0; i < pointCount; ++i)
	{
		//save the point coordinates
		CCVector3d coords = cloud->toGlobal3d(*cloud->getPoint(i));
		dataStream << coords.x;
		dataStream << coords.y;
		dataStream << coords.z;

		//and now for the scalar values
		for (unsigned j = 0; j < sfCount; ++j)
		{
			ScalarType val = cloud->getScalarField(j)->getValue(i);
			float fVal = static_cast<float>(val);
			dataStream >> fVal;
		}

		if (!nProgress.oneStep())
		{
			dataFile.close();
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	return CC_FERR_NO_ERROR;
}

struct SFDescriptor
{
	QString name;
	double precision = std::numeric_limits<double>::quiet_NaN();
	ccScalarField* sf = nullptr;
};

struct GlobalDescriptor
{
	size_t pointCount;
	CCVector3d globalShift;
	std::vector<SFDescriptor> SFs;
};

CC_FILE_ERROR SimpleBinFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
	if (filename.isEmpty())
	{
		assert(false);
		return CC_FERR_BAD_ARGUMENT;
	}
	if (!QFileInfo(filename).exists())
	{
		return CC_FERR_READING;
	}

	QString headerFilename, dataFilename;
	if (filename.endsWith(".sbf.data", Qt::CaseInsensitive))
	{
		//we trim the '.data' and read the '.sbf' file instead
		headerFilename = filename.left(filename.size() - 5);
		dataFilename = filename;
	}
	else
	{
		headerFilename = filename;
		dataFilename = filename + ".data";
	}

	ccLog::Print(QString("[SBF] Loading file '%1'...").arg(headerFilename));
	GlobalDescriptor descriptor;

	//save the text file as an INI file
	{
		QSettings headerFile(headerFilename, QSettings::IniFormat);
		QVariant varPointCount = headerFile.value("Points");
		if (varPointCount.isValid())
		{
			//only indicative (we don't use it!)
			descriptor.pointCount = varPointCount.toLongLong();
		}

		//read the global shift (if any)
		if (headerFile.contains("GlobalShift"))
		{
			QStringList strGlobalShift = headerFile.value("GlobalShift").toStringList();
			if (strGlobalShift.size() != 3)
			{
				ccLog::Error("[SBF] Invalid global shift");
				return CC_FERR_MALFORMED_FILE;
			}
			else
			{
				bool ok[3] = { false, false, false };
				descriptor.globalShift.x = strGlobalShift[0].toDouble(ok);
				descriptor.globalShift.y = strGlobalShift[1].toDouble(ok + 1);
				descriptor.globalShift.z = strGlobalShift[2].toDouble(ok + 2);
				if (!ok[0] || !ok[1] || !ok[2])
				{
					ccLog::Error("[SBF] Invalid global shift");
					return CC_FERR_MALFORMED_FILE;
				}
			}
		}

		//read the scalar field names (if any)
		if (headerFile.contains("SFCount"))
		{
			bool ok = false;
			int sfCount = headerFile.value("SFCount").toInt(&ok);
			if (!ok || sfCount < 0)
			{
				ccLog::Error("[SBF] Invalid SF count");
				return CC_FERR_MALFORMED_FILE;
			}

			try
			{
				descriptor.SFs.resize(static_cast<size_t>(sfCount));
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}

			//try to load the description of each SF
			for (int i = 0; i < sfCount; ++i)
			{
				QString key = QString("SF%1").arg(i + 1);
				QStringList tokens = headerFile.value(key).toStringList();
				if (tokens.size() > 0)
				{
					descriptor.SFs[i].name = tokens[0];
					if (tokens.size() > 0)
					{
						descriptor.SFs[i].precision = tokens[1].toDouble(&ok);
						if (!ok)
						{
							ccLog::Error(QString("[SBF] Invalid %1 description (precision)").arg(key));
							return CC_FERR_MALFORMED_FILE;
						}
					}
				}
			}
		}
	}

	//we can now load the data file
	QFile dataFile(dataFilename);
	if (!dataFile.open(QFile::ReadOnly))
	{
		ccLog::Warning("[SBF] Failed to read the data file");
		return CC_FERR_READING;
	}

	QDataStream dataStream(&dataFile);
	size_t readBytes = 0;
	//header
	static const size_t c_headerSize = 32;
	{
		//2 bytes = header flag
		{
			uint16_t headerFlag = 0;
			dataStream >> headerFlag;
			if (headerFlag != s_headerFlagSBF)
			{
				return CC_FERR_MALFORMED_FILE;
			}
		}
		readBytes += 2;

		//8 bytes = point count
		{
			uint64_t pointCount = 0;
			dataStream >> pointCount;
			//check consistency
			if (descriptor.pointCount != pointCount)
			{
				ccLog::Warning("[SBF] Inconsistent number of points between the header and the data file!");
			}
			//anyway, the data file count supersedes the header's one
			descriptor.pointCount = static_cast<size_t>(pointCount);
		}
		readBytes += 8;

		//2 bytes = sf count
		{
			uint16_t sfCount = 0;
			dataStream >> sfCount;
			//check consistency
			if (sfCount != descriptor.SFs.size())
			{
				ccLog::Warning("[SBF] Inconsistent number of scalar fields between the header and the data file!");
			}
			//once again, the data file count supersedes the header's one
			try
			{
				descriptor.SFs.resize(sfCount);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}
		}
		readBytes += 2;

		//remaining bytes (empty for now)
		for (; readBytes < c_headerSize; ++readBytes)
		{
			uint8_t byte = 0;
			dataStream >> byte;
		}
	}

	//check data consistency
	size_t sizePerPoint = 3 * 8 + descriptor.SFs.size() * 4; //8 bytes (double) for coordinates + 4 bytes (float) for scalars
	size_t totalSize = sizePerPoint * descriptor.pointCount + c_headerSize;
	if (totalSize != dataFile.size())
	{
		return CC_FERR_MALFORMED_FILE;
	}

	//init structures
	QScopedPointer<ccPointCloud> cloud(new ccPointCloud("unnamed"));
	if (!cloud->reserve(static_cast<unsigned>(descriptor.pointCount)))
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	QScopedPointer<ccProgressDialog> pDlg(0);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Simple BIN file"));
		pDlg->setInfo(QObject::tr("Loading %1 points / %2 scalar field(s)").arg(descriptor.pointCount).arg(descriptor.SFs.size()));
		pDlg->setModal(true);
		pDlg->start();
	}
	CCLib::NormalizedProgress nProgress(pDlg.data(), static_cast<unsigned>(descriptor.pointCount));

	//reserve memory
	for (size_t i = 0; i < descriptor.SFs.size(); ++i)
	{
		SFDescriptor& sfDesc = descriptor.SFs[i];
		if (sfDesc.name.isEmpty())
		{
			sfDesc.name = QString("Scalar field #%1").arg(i + 1);
		}
		sfDesc.sf = new ccScalarField(qPrintable(sfDesc.name));
		if (!sfDesc.sf->reserve(static_cast<unsigned>(descriptor.pointCount)))
		{
			sfDesc.sf->release();
			sfDesc.sf = nullptr;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
		cloud->addScalarField(sfDesc.sf);

		//for now we save the 'precision' info as meta-data of the cloud
		if (!isnan(sfDesc.precision))
		{
			cloud->setMetaData(QString("{%1}.precision").arg(sfDesc.name), sfDesc.precision);
		}
	}

	//set global shift
	cloud->setGlobalShift(descriptor.globalShift);

	//we can eventually load the data
	for (size_t i = 0; i < descriptor.pointCount; ++i)
	{
		//read the point coordinates
		CCVector3d coords;
		dataStream >> coords.x;
		dataStream >> coords.y;
		dataStream >> coords.z;
		CCVector3 P = CCVector3::fromArray((coords - descriptor.globalShift).u);
		cloud->addPoint(P);

		//and now for the scalar values
		for (SFDescriptor& sfDesc : descriptor.SFs)
		{
			float val;
			dataStream >> val;
			sfDesc.sf->addElement(val);
		}

		if (!nProgress.oneStep())
		{
			break;
		}
	}

	dataFile.close();

	if (cloud->size() == 0)
	{
		return CC_FERR_CANCELED_BY_USER;
	}
	else if (cloud->size() < descriptor.pointCount)
	{
		cloud->shrinkToFit();
	}

	//update scalar fields
	if (!descriptor.SFs.empty())
	{
		for (size_t i = 0; i < descriptor.SFs.size(); ++i)
		{
			descriptor.SFs[i].sf->computeMinAndMax();
		}
		cloud->setCurrentDisplayedScalarField(0);
	}

	container.addChild(cloud.take());

	return CC_FERR_NO_ERROR;
}
