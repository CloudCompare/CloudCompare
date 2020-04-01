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

#include "PNFilter.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Qt
#include <QFile>

//default normal value
static const CCVector3 s_defaultNorm(0,0,1);


PNFilter::PNFilter()
	: FileIOFilter( {
					"_Point+Normal Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "pn" },
					"pn",
					QStringList{ "Point+Normal cloud (*.pn)" },
					QStringList{ "Point+Normal cloud (*.pn)" },
					Import | Export
					} )
{
}

bool PNFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POINT_CLOUD)
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR PNFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	//the cloud to save
	ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(entity);
	if (!theCloud)
	{
		ccLog::Warning("[PN] This filter can only save one cloud at a time!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}
	unsigned numberOfPoints = theCloud->size();

	if (numberOfPoints == 0)
	{
		ccLog::Warning("[PN] Input cloud is empty!");
		return CC_FERR_NO_SAVE;
	}

	//open binary file for writing
	QFile out(filename);
	if (!out.open(QIODevice::WriteOnly))
		return CC_FERR_WRITING;

	//Has the cloud been recentered?
	if (theCloud->isShifted())
		ccLog::Warning(QString("[PNFilter::save] Can't recenter or rescale cloud '%1' when saving it in a PN file!").arg(theCloud->getName()));

	bool hasNorms = theCloud->hasNormals();
	if (!hasNorms)
		ccLog::Warning(QString("[PNFilter::save] Cloud '%1' has no normal (we will save points with a default normal)!").arg(theCloud->getName()));
	float norm[3] = {	static_cast<float>(s_defaultNorm.x),
						static_cast<float>(s_defaultNorm.y),
						static_cast<float>(s_defaultNorm.z) };

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (pDlg)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
		pDlg->setMethodTitle(QObject::tr("Save PN file"));
		pDlg->setInfo(QObject::tr("Points: %L1").arg( numberOfPoints ));
		pDlg->start();
	}
	CCLib::NormalizedProgress nprogress(pDlg.data(), numberOfPoints);

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	for (unsigned i=0; i<numberOfPoints; i++)
	{
		//write point
		{
			const CCVector3* P = theCloud->getPoint(i);
			
			//conversion to float
			CCVector3f Pfloat = CCVector3f::fromArray(P->u);
			if (out.write(reinterpret_cast<const char*>(Pfloat.u),3*sizeof(float)) < 0)
			{
				result = CC_FERR_WRITING;
				break;
			}
		}
			
		//write normal
		if (hasNorms)
		{
			const CCVector3& N = theCloud->getPointNormal(i);
			//conversion to float
			norm[0] = static_cast<float>(N.x);
			norm[1] = static_cast<float>(N.y);
			norm[2] = static_cast<float>(N.z);
		}
		if (out.write(reinterpret_cast<const char*>(norm),3*sizeof(float)) < 0)
		{
			result = CC_FERR_WRITING;
			break;
		}

		if (pDlg && !nprogress.oneStep())
		{
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}
	}

	out.close();

	return result;
}

CC_FILE_ERROR PNFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
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
	unsigned numberOfPoints = static_cast<unsigned>(fileSize  / singlePointSize);

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
		pDlg->setMethodTitle(QObject::tr("Open PN file"));
		pDlg->setInfo(QObject::tr("Points: %L1").arg( numberOfPoints ));
		pDlg->start();
	}
	CCLib::NormalizedProgress nprogress(pDlg.data(), numberOfPoints);

	ccPointCloud* loadedCloud = nullptr;
	//if the file is too big, it will be chuncked in multiple parts
	unsigned chunkIndex = 0;
	unsigned fileChunkPos = 0;
	unsigned fileChunkSize = 0;
	//number of points read for the current cloud part
	unsigned pointsRead = 0;
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	for (unsigned i = 0; i < numberOfPoints; i++)
	{
		//if we reach the max. cloud size limit, we cerate a new chunk
		if (pointsRead == fileChunkPos + fileChunkSize)
		{
			if (loadedCloud)
				container.addChild(loadedCloud);
			fileChunkPos = pointsRead;
			fileChunkSize = std::min<unsigned>(numberOfPoints - pointsRead, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
			loadedCloud = new ccPointCloud(QString("unnamed - Cloud #%1").arg(++chunkIndex));
			if (!loadedCloud || !loadedCloud->reserveThePointsTable(fileChunkSize) || !loadedCloud->reserveTheNormsTable())
			{
				result = CC_FERR_NOT_ENOUGH_MEMORY;
				if (loadedCloud)
					delete loadedCloud;
				loadedCloud = nullptr;
				break;
			}
			loadedCloud->showNormals(true);
		}

		//we read the 3 coordinates of the point
		float rBuff[3];
		if (in.read((char*)rBuff, 3 * sizeof(float)) >= 0)
		{
			//conversion to CCVector3
			CCVector3 P = CCVector3::fromArray(rBuff);
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
			loadedCloud->addNorm(CCVector3::fromArray(rBuff));
		}
		else
		{
			//add fake normal for consistency then break
			loadedCloud->addNorm(s_defaultNorm);
			result = CC_FERR_READING;
			break;
		}

		++pointsRead;

		if (pDlg && !nprogress.oneStep())
		{
			result = CC_FERR_CANCELED_BY_USER;
			break;
		}
	}

	in.close();

	if (loadedCloud)
	{
		loadedCloud->shrinkToFit();
		container.addChild(loadedCloud);
	}

	return result;
}
