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

#include "PTXFilter.h"

//qCC_db
#include <ccColorScalesManager.h>
#include <ccGBLSensor.h>
#include <ccGriddedTools.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//Qt
#include <QMessageBox>
#include <QTextStream>

//System
#include <cassert>
#include <string>

const char CC_PTX_INTENSITY_FIELD_NAME[] = "Intensity";


PTXFilter::PTXFilter()
	: FileIOFilter( {
					"_PTX Filter",
					5.0f,	// priority
					QStringList{ "ptx" },
					"ptx",
					QStringList{ "PTX cloud (*.ptx)" },
					QStringList(),
					Import
					} )
{
}

static void CleanMatrix(ccGLMatrixd& mat)
{
	//make the transform a little bit cleaner (necessary as it's read from ASCII!)
	{
//#ifdef QT_DEBUG
//		//test the matrix quality
//		ccGLMatrixd before = mat;
//		CCVector3d X0(before.getColumn(0));
//		CCVector3d Y0(before.getColumn(1));
//		CCVector3d Z0(before.getColumn(2));
//		double normX0 = X0.norm();
//		double normY0 = Y0.norm();
//		double normZ0 = Z0.norm();
//#endif
		CCVector3d X(mat.getColumn(0));
		CCVector3d Y(mat.getColumn(1));
		CCVector3d Z(mat.getColumn(2));
		CCVector3d T = mat.getTranslationAsVec3D();
		Z = X.cross(Y);
		Y = Z.cross(X);
		X.normalize();
		Y.normalize();
		Z.normalize();
		mat = ccGLMatrixd(X, Y, Z, T);
//#ifdef QT_DEBUG
//		double dot = CCVector3d(X).dot(X0);
//		dot /= (normX0 * CCVector3d(X).norm());
//		double alpha = acos(dot);
//
//		dot = CCVector3d(Y).dot(Y0);
//		dot /= (normY0 * CCVector3d(Y).norm());
//		double beta = acos(dot);
//
//		dot = CCVector3d(Z).dot(Z0);
//		dot /= (normZ0 * CCVector3d(Z).norm());
//		double gamma = acos(dot);
//#endif
	}
}

CC_FILE_ERROR PTXFilter::loadFile(	const QString& filename,
									ccHObject& container,
									LoadParameters& parameters)
{
	//open ASCII file for reading
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		return CC_FERR_READING;
	}

	QTextStream inFile(&file);

	CCVector3d PshiftTrans(0, 0, 0);
	CCVector3d PshiftCloud(0, 0, 0);
	bool preserveCoordinateShift = true;

	CC_FILE_ERROR result = CC_FERR_NO_LOAD;
	ScalarType minIntensity = 0;
	ScalarType maxIntensity = 0;

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Loading PTX file"));
		pDlg->setAutoClose(false);
	}

	//progress dialog (for normals computation)
	QScopedPointer<ccProgressDialog> normalsProgressDlg(nullptr);
	if (parameters.parentWidget && parameters.autoComputeNormals)
	{
		normalsProgressDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		normalsProgressDlg->setAutoClose(false);
		normalsProgressDlg->hide();
	}

	for (unsigned cloudIndex = 0; result == CC_FERR_NO_ERROR || result == CC_FERR_NO_LOAD; cloudIndex++)
	{
		unsigned width = 0;
		unsigned height = 0;
		ccGLMatrixd sensorTransD;
		ccGLMatrixd cloudTransD;

		//read header
		{
			QString line = inFile.readLine();
			if (line.isNull() && container.getChildrenNumber() != 0) //end of file?
				break;

			//read the width (number of columns) and the height (number of rows) on the two first lines
			//(DGM: we transpose the matrix right away)
			bool ok;
			height = line.toUInt(&ok);
			if (!ok)
				return CC_FERR_MALFORMED_FILE;
			line = inFile.readLine();
			width = line.toUInt(&ok);
			if (!ok)
				return CC_FERR_MALFORMED_FILE;

			ccLog::Print(QString("[PTX] Scan #%1 - grid size: %2 x %3").arg(cloudIndex + 1).arg(height).arg(width));

			//read sensor transformation matrix
			for (int i = 0; i < 4; ++i)
			{
				line = inFile.readLine();
				QStringList tokens = line.split(" ", QString::SkipEmptyParts);
				if (tokens.size() != 3)
					return CC_FERR_MALFORMED_FILE;

				double* colDest = nullptr;
				if (i == 0)
				{
					//Translation
					colDest = sensorTransD.getTranslation();
				}
				else
				{
					//X, Y and Z axis
					colDest = sensorTransD.getColumn(i - 1);
				}

				for (int j = 0; j < 3; ++j)
				{
					assert(colDest);
					colDest[j] = tokens[j].toDouble(&ok);
					if (!ok)
						return CC_FERR_MALFORMED_FILE;
				}
			}
			//make the transform a little bit cleaner (necessary as it's read from ASCII!)
			CleanMatrix(sensorTransD);

			//read cloud transformation matrix
			for (int i = 0; i < 4; ++i)
			{
				line = inFile.readLine();
				QStringList tokens = line.split(" ", QString::SkipEmptyParts);
				if (tokens.size() != 4)
					return CC_FERR_MALFORMED_FILE;

				double* col = cloudTransD.getColumn(i);
				for (int j = 0; j < 4; ++j)
				{
					col[j] = tokens[j].toDouble(&ok);
					if (!ok)
						return CC_FERR_MALFORMED_FILE;
				}
			}
			//make the transform a little bit cleaner (necessary as it's read from ASCII!)
			CleanMatrix(cloudTransD);

			//handle Global Shift directly on the first cloud's translation!
			if (cloudIndex == 0)
			{
				if (HandleGlobalShift(cloudTransD.getTranslationAsVec3D(), PshiftTrans, preserveCoordinateShift, parameters))
				{
					ccLog::Warning("[PTXFilter::loadFile] Cloud has be recentered! Translation: (%.2f ; %.2f ; %.2f)", PshiftTrans.x, PshiftTrans.y, PshiftTrans.z);
				}
			}

			//'remove' global shift from the sensor and cloud transformation matrices
			cloudTransD.setTranslation(cloudTransD.getTranslationAsVec3D() + PshiftTrans);
			sensorTransD.setTranslation(sensorTransD.getTranslationAsVec3D() + PshiftTrans);
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
			{
				container.getChild(0)->setName("unnamed - Cloud 1"); //update previous cloud name!
			}
			cloud->setName(QString("unnamed - Cloud %1").arg(container.getChildrenNumber() + 1));
		}

		unsigned gridSize = width * height;
		if (!cloud->reserve(gridSize))
		{
			result = CC_FERR_NOT_ENOUGH_MEMORY;
			delete cloud;
			cloud = nullptr;
			break;
		}

		//set global shift
		if (preserveCoordinateShift)
		{
			cloud->setGlobalShift(PshiftTrans);
		}

		//intensities
		ccScalarField* intensitySF = new ccScalarField(CC_PTX_INTENSITY_FIELD_NAME);
		if (!intensitySF->reserveSafe(static_cast<unsigned>(gridSize)))
		{
			ccLog::Warning("[PTX] Not enough memory to load intensities!");
			intensitySF->release();
			intensitySF = nullptr;
		}

		//grid structure
		ccPointCloud::Grid::Shared grid(new ccPointCloud::Grid);
		grid->w = width;
		grid->h = height;
		bool hasIndexGrid = true;
		try
		{
			grid->indexes.resize(gridSize, -1); //-1 means no cell/point
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning("[PTX] Not enough memory to load the grid structure");
			hasIndexGrid = false;
		}

		//read points
		{
			CCLib::NormalizedProgress nprogress(pDlg.data(), gridSize);
			if (pDlg)
			{
				pDlg->setInfo(qPrintable(QString("Number of cells: %1").arg(gridSize)));
				pDlg->start();
			}

			bool firstPoint = true;
			bool hasColors = false;
			bool loadColors = false;
			bool loadGridColors = false;
			size_t gridIndex = 0;

			for (unsigned j = 0; j < height; ++j)
			{
				for (unsigned i = 0; i < width; ++i, ++gridIndex)
				{
					QString line = inFile.readLine();
					QStringList tokens = line.split(" ", QString::SkipEmptyParts);

					if (firstPoint)
					{
						hasColors = (tokens.size() == 7);
						if (hasColors)
						{
							loadColors = cloud->reserveTheRGBTable();
							if (!loadColors)
							{
								ccLog::Warning("[PTX] Not enough memory to load RGB colors!");
							}
							else if (hasIndexGrid)
							{
								//we also load the colors into the grid (as invalid/missing points can have colors!)
								try
								{
									grid->colors.resize(gridSize, ccColor::Rgb(0, 0, 0));
									loadGridColors = true;
								}
								catch (const std::bad_alloc&)
								{
									ccLog::Warning("[PTX] Not enough memory to load the grid colors");
								}
							}
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
					for (int v = 0; v < 4; ++v)
					{
						bool ok;
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
					bool pointIsValid = (CCVector3d::fromArray(values).norm2() != 0);
					if (pointIsValid)
					{
						const double* Pd = values;
						//first point: check for 'big' coordinates
						if (firstPoint)
						{
							if (cloudIndex == 0 && !cloud->isShifted()) //in case the trans. matrix was ok!
							{
								CCVector3d P(Pd);
								if (HandleGlobalShift(P, PshiftCloud, preserveCoordinateShift, parameters))
								{
									if (preserveCoordinateShift)
									{
										cloud->setGlobalShift(PshiftCloud);
									}
									ccLog::Warning("[PTXFilter::loadFile] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", PshiftCloud.x, PshiftCloud.y, PshiftCloud.z);
								}
							}
							firstPoint = false;
						}

						//update index grid
						if (hasIndexGrid)
						{
							grid->indexes[gridIndex] = static_cast<int>(cloud->size()); // = index (default value = -1, means no point)
						}

						//add point
						cloud->addPoint(CCVector3(	static_cast<PointCoordinateType>(Pd[0] + PshiftCloud.x),
													static_cast<PointCoordinateType>(Pd[1] + PshiftCloud.y),
													static_cast<PointCoordinateType>(Pd[2] + PshiftCloud.z)) );

						//add intensity
						if (intensitySF)
						{
							intensitySF->addElement(static_cast<ScalarType>(values[3]));
						}
					}

					//color
					if (loadColors && (pointIsValid || loadGridColors))
					{
						ccColor::Rgb color;
						for (int c = 0; c < 3; ++c)
						{
							bool ok;
							unsigned temp = tokens[4 + c].toUInt(&ok);
							ok &= (temp <= 255);
							if (ok)
							{
								color.rgb[c] = static_cast<unsigned char>(temp);
							}
							else
							{
								result = CC_FERR_MALFORMED_FILE;
								//early stop
								j = height;
								break;
							}
						}

						if (pointIsValid)
						{
							cloud->addColor(color);
						}
						if (loadGridColors)
						{
							assert(!grid->colors.empty());
							grid->colors[gridIndex] = color;
						}
					}

					if (parameters.parentWidget && !nprogress.oneStep())
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
			cloud = nullptr;
			if (intensitySF)
			{
				intensitySF->release();
				intensitySF = nullptr;
			}

			ccLog::Warning(QString("[PTX] Scan #%1 is empty?!").arg(cloudIndex+1));
		}
		else
		{
			if (result == CC_FERR_NO_LOAD)
				result = CC_FERR_NO_ERROR; //to make clear that we have loaded at least something!
			
			cloud->resize(cloud->size());
			if (intensitySF)
			{
				assert(intensitySF->currentSize() == cloud->size());
				intensitySF->resize(cloud->size());
				intensitySF->computeMinAndMax();
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

			ccGBLSensor* sensor = nullptr;
			if (hasIndexGrid && result != CC_FERR_CANCELED_BY_USER)
			{
				//determine best sensor parameters (mainly yaw and pitch steps)
				ccGLMatrix cloudToSensorTrans((sensorTransD.inverse() * cloudTransD).data());
				sensor = ccGriddedTools::ComputeBestSensor(cloud, grid, &cloudToSensorTrans);
			}

			//we apply the transformation
			ccGLMatrix cloudTrans(cloudTransD.data());
			cloud->applyGLTransformation_recursive(&cloudTrans);
			//this transformation is of no interest for the user
			cloud->resetGLTransformationHistory_recursive();

			if (sensor)
			{
				ccGLMatrix sensorTrans(sensorTransD.data());
				sensor->setRigidTransformation(sensorTrans); //after cloud->applyGLTransformation_recursive!
				cloud->addChild(sensor);
			}

			//scan grid
			if (hasIndexGrid)
			{
				grid->validCount = static_cast<unsigned>(cloud->size());
				grid->minValidIndex = 0;
				grid->maxValidIndex = grid->validCount - 1;
				grid->sensorPosition = sensorTransD;
				cloud->addGrid(grid);

				//by default we don't compute normals without asking the user
				if (parameters.autoComputeNormals)
				{
					cloud->computeNormalsWithGrids(1.0, normalsProgressDlg.data());
				}
			}

			cloud->setVisible(true);
			cloud->showColors(cloud->hasColors());
			cloud->showNormals(cloud->hasNormals());

			container.addChild(cloud);

#ifdef QT_DEBUG
			//break;
#endif
		}
	}

	//update scalar fields saturation (globally!)
	{
		bool validIntensityRange = true;
		if (minIntensity < 0 || maxIntensity > 1.0)
		{
			ccLog::Warning("[PTX] Intensity values are invalid (they should all fall in [0 ; 1])");
			validIntensityRange = false;
		}

		for (unsigned i = 0; i < container.getChildrenNumber(); ++i)
		{
			ccHObject* obj = container.getChild(i);
			assert(obj && obj->isA(CC_TYPES::POINT_CLOUD));
			CCLib::ScalarField* sf = static_cast<ccPointCloud*>(obj)->getScalarField(0);
			if (sf)
			{
				ccScalarField* ccSF = static_cast<ccScalarField*>(sf);
				ccSF->setColorScale(ccColorScalesManager::GetDefaultScale(validIntensityRange ? ccColorScalesManager::ABS_NORM_GREY : ccColorScalesManager::GREY));
				ccSF->setSaturationStart(0/*minIntensity*/);
				ccSF->setSaturationStop(maxIntensity);
			}
		}
	}

	return result;
}
