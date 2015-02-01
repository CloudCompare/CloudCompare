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

#include "PTXFilter.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccColorScalesManager.h>
#include <ccProgressDialog.h>
#include <ccGBLSensor.h>
#include <ccGriddedTools.h>

//Qt
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

//System
#include <assert.h>
#include <string.h>

const char CC_PTX_INTENSITY_FIELD_NAME[] = "Intensity";

bool PTXFilter::canLoadExtension(QString upperCaseExt) const
{
	return (upperCaseExt == "PTX");
}

bool PTXFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//not supported yet
	return false;
}

CC_FILE_ERROR PTXFilter::loadFile(	QString filename,
									ccHObject& container,
									LoadParameters& parameters)
{
	//open ASCII file for reading
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		return CC_FERR_READING;

	QTextStream inFile(&file);

	QString line;
	QStringList tokens;
	bool ok;

	//by default we don't compute normals without asking the user
	bool computeNormals = (parameters.autoComputeNormals == ccGriddedTools::ALWAYS);

	CCVector3d PshiftTrans(0,0,0);
	CCVector3d PshiftCloud(0,0,0);

	CC_FILE_ERROR result = CC_FERR_NO_LOAD;
	ScalarType minIntensity = 0;
	ScalarType maxIntensity = 0;
	for (unsigned cloudIndex = 0; result == CC_FERR_NO_ERROR || result == CC_FERR_NO_LOAD; cloudIndex++)
	{
		unsigned width = 0, height = 0;
		ccGLMatrix sensorTrans, cloudTrans;

		//read header
		{
			line = inFile.readLine();
			if (line.isNull() && container.getChildrenNumber() != 0) //end of file?
				break;
			height = line.toUInt(&ok);
			if (!ok)
				return CC_FERR_MALFORMED_FILE;
			line = inFile.readLine();
			width = line.toUInt(&ok);
			if (!ok)
				return CC_FERR_MALFORMED_FILE;

			ccLog::Print(QString("[PTX] Scan #%1 - grid size: %2 x %3").arg(cloudIndex+1).arg(width).arg(height));

			//read sensor transformation matrix
			for (int i=0; i<4; ++i)
			{
				line = inFile.readLine();
				tokens = line.split(" ",QString::SkipEmptyParts);
				if (tokens.size() != 3)
					return CC_FERR_MALFORMED_FILE;

				for (int j=0; j<3; ++j)
				{
					float* colDest = 0;
					if (i == 0)
						//Translation
						colDest = sensorTrans.getTranslation();
					else
						//X, Y and Z axis
						colDest = sensorTrans.getColumn(i-1);

					assert(colDest);
					colDest[j] = tokens[j].toFloat(&ok);
					if (!ok)
						return CC_FERR_MALFORMED_FILE;
				}
			}

			//read cloud transformation matrix
			ccGLMatrixd cloudTransD;
			for (int i=0; i<4; ++i)
			{
				line = inFile.readLine();
				tokens = line.split(" ",QString::SkipEmptyParts);
				if (tokens.size() != 4)
					return CC_FERR_MALFORMED_FILE;

				double* col = cloudTransD.getColumn(i);
				for (int j=0; j<4; ++j)
				{
					col[j] = tokens[j].toDouble(&ok);
					if (!ok)
						return CC_FERR_MALFORMED_FILE;
				}
			}
			//make the transform a little bit cleaner (necessary as it's read from ASCII!)
			{
				double* X = cloudTransD.getColumn(0);
				double* Y = cloudTransD.getColumn(1);
				double* Z = cloudTransD.getColumn(2);
				CCVector3d::vcross(X,Y,Z);
				CCVector3d::vcross(Y,Z,X);
				CCVector3d::vnormalize(X);
				CCVector3d::vnormalize(Y);
				CCVector3d::vnormalize(Z);
			}

			//handle Global Shift directly on the first cloud's translation!
			const CCVector3d& translation = cloudTransD.getTranslationAsVec3D();
			if (cloudIndex == 0)
			{
				if (HandleGlobalShift(translation,PshiftTrans,parameters))
				{
					ccLog::Warning("[PTXFilter::loadFile] Cloud has be recentered! Translation: (%.2f,%.2f,%.2f)",PshiftTrans.x,PshiftTrans.y,PshiftTrans.z);
				}
			}
			cloudTransD.setTranslation(translation + PshiftTrans);
			cloudTrans = ccGLMatrix(cloudTransD.data());
		}

		//now we can read the grid cells
		ccPointCloud* cloud = new ccPointCloud();
		if (container.getChildrenNumber() == 0)
		{
			cloud->setName("unnamed - Cloud");
		}
		else
		{
			if (container.getChildrenNumber() == 1)
				container.getChild(0)->setName("unnamed - Cloud 1"); //update previous cloud name!

			cloud->setName(QString("unnamed - Cloud %1").arg(container.getChildrenNumber()+1));
		}

		unsigned gridSize = width * height;
		if (!cloud->reserve(gridSize))
		{
			result = CC_FERR_NOT_ENOUGH_MEMORY;
			delete cloud;
			cloud = 0;
			break;
		}

		//set global shift
		cloud->setGlobalShift(PshiftTrans);

		//intensities
		ccScalarField* intensitySF = new ccScalarField(CC_PTX_INTENSITY_FIELD_NAME);
		if (!intensitySF->reserve(static_cast<unsigned>(gridSize)))
		{
			ccLog::Warning("[PTX] Not enough memory to load intensities!");
			intensitySF->release();
			intensitySF = 0;
		}

		//grid for computing normals
		std::vector<int> indexGrid;
		bool hasIndexGrid = true;
		try
		{
			indexGrid.resize(gridSize,-1);
		}
		catch(std::bad_alloc)
		{
			ccLog::Warning("[PTX] Not enough memory to save grid structure (required to compute normals!)");
			hasIndexGrid = false;
		}

		//read points
		{
			//progress dialog
			ccProgressDialog pdlg(true);
			CCLib::NormalizedProgress nprogress(&pdlg,gridSize);
			pdlg.setMethodTitle("Loading PTX file");
			pdlg.setInfo(qPrintable(QString("Number of cells: %1").arg(gridSize)));
			pdlg.start();

			bool firstPoint = true;
			bool hasColors = false;
			bool loadColors = false;
			int* _indexGrid = hasIndexGrid ? &(indexGrid[0]) : 0;

			for (unsigned j=0; j<height; ++j)
			{
				for (unsigned i=0; i<width; ++i, ++_indexGrid)
				{
					line = inFile.readLine();
					tokens = line.split(" ",QString::SkipEmptyParts);

					if (firstPoint)
					{
						hasColors = (tokens.size() == 7);
						if (hasColors)
						{
							loadColors = cloud->reserveTheRGBTable();
							if (!loadColors)
								ccLog::Warning("[PTX] Not enough memory to load RGB colors!");
						}
					}
					if ((hasColors && tokens.size() != 7) || (!hasColors && tokens.size() != 4))
					{
						result = CC_FERR_MALFORMED_FILE;
						//early stop
						j = height;
						break;
					}

					double values[4];
					for (int v=0; v<4; ++v)
					{
						values[v] = tokens[v].toDouble(&ok);
						if (!ok)
						{
							result = CC_FERR_MALFORMED_FILE;
							//early stop
							j = height;
							break;
						}
					}

					//we skip "empty" cells
					if (CCVector3d::fromArray(values).norm2() != 0)
					{
						const double* Pd = values;
						//first point: check for 'big' coordinates
						if (firstPoint)
						{
							if (cloudIndex == 0 && !cloud->isShifted()) //in case the trans. matrix was ok!
							{
								CCVector3d P(Pd);
								if (HandleGlobalShift(P,PshiftCloud,parameters))
								{
									cloud->setGlobalShift(PshiftCloud);
									ccLog::Warning("[PTXFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",PshiftCloud.x,PshiftCloud.y,PshiftCloud.z);
								}
							}
							firstPoint = false;
						}

						//update index grid
						if (hasIndexGrid)
							*_indexGrid = static_cast<int>(cloud->size()); // = index (default value = -1, means no point)

						//add point
						cloud->addPoint(CCVector3(	static_cast<PointCoordinateType>(Pd[0] + PshiftCloud.x),
													static_cast<PointCoordinateType>(Pd[1] + PshiftCloud.y),
													static_cast<PointCoordinateType>(Pd[2] + PshiftCloud.z)) );

						//add intensity
						if (intensitySF)
							intensitySF->addElement(static_cast<ScalarType>(values[3]));

						//color
						if (loadColors)
						{
							colorType rgb[3];
							for (int c=0; c<3; ++c)
							{
								unsigned temp = tokens[4+c].toUInt(&ok);
								ok &= (temp <= static_cast<unsigned>(ccColor::MAX));
								if (ok)
								{
									rgb[c] = static_cast<colorType>(temp);
								}
								else
								{
									result = CC_FERR_MALFORMED_FILE;
									//early stop
									j = height;
									break;
								}
							}
							cloud->addRGBColor(rgb);
						}
					}

					if (!nprogress.oneStep())
					{
						result = CC_FERR_CANCELED_BY_USER;
						break;
					}
				}
			}
		}

		//is there at least one valid point in this grid?
		if (cloud->size() == 0)
		{
			delete cloud;
			cloud = 0;
			if (intensitySF)
				intensitySF->release();

			ccLog::Warning(QString("[PTX] Scan #1 is empty?!").arg(cloudIndex+1));
		}
		else if (cloud->size() <= cloud->capacity())
		{
			if (result == CC_FERR_NO_LOAD)
				result = CC_FERR_NO_ERROR; //to make clear that we have loaded at least something!
			
			cloud->resize(cloud->size());
			if (intensitySF)
			{
				assert(intensitySF->currentSize() == cloud->size());
				intensitySF->resize(cloud->size());
				intensitySF->computeMinAndMax();
				intensitySF->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::ABS_NORM_GREY));
				int intensitySFIndex = cloud->addScalarField(intensitySF);

				//keep track of the min and max intensity
				if (container.getChildrenNumber() == 0)
				{
					minIntensity = intensitySF->getMin();
					maxIntensity = intensitySF->getMax();
				}
				else
				{
					minIntensity = std::min(minIntensity,intensitySF->getMin());
					maxIntensity = std::max(maxIntensity,intensitySF->getMax());
				}

				cloud->showSF(true);
				cloud->setCurrentDisplayedScalarField(intensitySFIndex);
			}

			if (hasIndexGrid && result != CC_FERR_CANCELED_BY_USER)
			{
				//determine best sensor parameters (mainly yaw and pitch steps)
				ccGBLSensor* sensor = ccGriddedTools::ComputeBestSensor(cloud,indexGrid,width,height);
				if (sensor)
				{
					cloud->addChild(sensor);
				}

#ifndef _DEBUG
				if (cloudIndex == 0)
				{
					//shall we ask the user if he wants to compute normals or not?
					computeNormals = ccGriddedTools::HandleAutoComputeNormalsFeature(parameters.autoComputeNormals);
				}
#endif
				if (computeNormals)
				{
					//try to compute normals
					bool canceledByUser = false;
					if (!ccGriddedTools::ComputeNormals(cloud,indexGrid,static_cast<int>(width),static_cast<int>(height),&canceledByUser))
					{
						if (canceledByUser)
						{
							//if the user cancelled the normal process, we cancel everything!
							result = CC_FERR_CANCELED_BY_USER;
						}
						else
						{
							computeNormals = false;
						}
					}
				}
			}

			//don't need it anymore
			indexGrid.clear();

			//we apply the transformation
			cloud->applyGLTransformation_recursive(&cloudTrans);

			cloud->setVisible(true);
			cloud->showColors(cloud->hasColors());
			cloud->showNormals(cloud->hasNormals());

			container.addChild(cloud);

#ifdef _DEBUG
			//break;
#endif
		}
	}

	//update scalar fields saturation (globally!)
	{
		for (unsigned i=0; i<container.getChildrenNumber(); ++i)
		{
			ccHObject* obj = container.getChild(i);
			assert(obj && obj->isA(CC_TYPES::POINT_CLOUD));
			CCLib::ScalarField* sf = static_cast<ccPointCloud*>(obj)->getScalarField(0);
			if (sf)
			{
				static_cast<ccScalarField*>(sf)->setSaturationStart(0/*minIntensity*/);
				static_cast<ccScalarField*>(sf)->setSaturationStop(maxIntensity);
			}
		}
	}

	return result;
}
