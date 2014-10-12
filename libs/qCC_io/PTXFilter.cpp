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

//CCLib
#include <Neighbourhood.h>
#include <GenericIndexedMesh.h>

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

//! Association of an angle and the corresponding number of rows/columns
typedef std::pair<PointCoordinateType,unsigned> AngleAndSpan;

//default behavior regarding normal computation
PTXFilter::ComputeNormalsBehavior PTXFilter::s_normalCompBehavior = PTXFilter::ASK_USER;

void PTXFilter::SetNormalsComputationBehavior(ComputeNormalsBehavior option)
{
	PTXFilter::s_normalCompBehavior = option;
}

CC_FILE_ERROR ComputeNormals(ccPointCloud* cloud, const std::vector<unsigned>& indexGrid, int width, int height)
{
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//try to compute normals
	if (cloud->reserveTheNormsTable())
	{
		//progress dialog
		ccProgressDialog pdlg(true);
		CCLib::NormalizedProgress nprogress(&pdlg,cloud->size());
		pdlg.setMethodTitle("Compute PTX normals");
		pdlg.setInfo(qPrintable(QString("Number of points: %1").arg(cloud->size())));
		pdlg.start();

		const unsigned* _indexGrid = &(indexGrid[0]);
		CCLib::ReferenceCloud knn(cloud);
		
		//neighborhood 'half-width' (total width = 1 + 2*nw) 
		const int nw = 3;
		//max number of neighbours: (1+2*nw)^2
		knn.reserve((1+2*nw)*(1+2*nw));

		//for each grid cell
		for (int j=0; j<height; ++j)
		{
			for (int i=0; i<width; ++i, ++_indexGrid)
			{
				if (*_indexGrid != 0)
				{
					//add the point itself
					knn.clear(false);
					knn.addPointIndex(*_indexGrid-1); //warning: indexes are shifted (0 = no point)
					const CCVector3* P = cloud->getPoint(*_indexGrid-1);

					//look for neighbors
					for (int v=std::max(0,j-nw); v<std::min<int>(height,j+nw); ++v)
					{
						if (v != j)
						{
							for (int u=std::max(0,i-nw); u<std::min<int>(width,i+nw); ++u)
							{
								if (u != i)
								{
									unsigned indexN = indexGrid[v*width + u];
									if (indexN != 0)
									{
										//we don't consider points with a too much different depth than the central point
										const CCVector3* Pn = cloud->getPoint(indexN-1);
										if (fabs(Pn->z - P->z) <= std::max(fabs(Pn->x - P->x),fabs(Pn->y - P->y)))
											knn.addPointIndex(indexN-1); //warning: indexes are shifted (0 = no point)
									}
								}
							}
						}
					}

					CCVector3 N(0,0,1);
					if (knn.size() >= 3)
					{
						CCLib::Neighbourhood Z(&knn);

						//compute normal with quadratic func. (if we have enough points)
						if (knn.size() >= 6)
						{
							uchar hfDims[3];
							const PointCoordinateType* h = Z.getHeightFunction(hfDims);
							if (h)
							{
								const CCVector3* gv = Z.getGravityCenter();
								assert(gv);

								const uchar& iX = hfDims[0];
								const uchar& iY = hfDims[1];
								const uchar& iZ = hfDims[2];

								PointCoordinateType lX = P->u[iX] - gv->u[iX];
								PointCoordinateType lY = P->u[iY] - gv->u[iY];

								N.u[iX] = h[1] + (2 * h[3] * lX) + (h[4] * lY);
								N.u[iY] = h[2] + (2 * h[5] * lY) + (h[4] * lX);
								N.u[iZ] = -PC_ONE;

								N.normalize();
							}
						}
						//else
						//{
						//	//compute normal with best fit plane
						//	const CCVector3* _N = Z.getLSQPlaneNormal();
						//	if (_N)
						//		N = *_N;
						//}
						else
						{
							//compute normals with 2D1/2 triangulation
							CCLib::GenericIndexedMesh* theMesh = Z.triangulateOnPlane();
							if (theMesh)
							{
								unsigned faceCount = theMesh->size();
								N.z = 0;

								//for all triangles
								theMesh->placeIteratorAtBegining();
								for (unsigned j=0; j<faceCount; ++j)
								{
									const CCLib::TriangleSummitsIndexes* tsi = theMesh->getNextTriangleIndexes();
									//we look if the central point is one of the triangle's vertices
									if (tsi->i1 == 0 || tsi->i2 == 0|| tsi->i3 == 0)
									{
										const CCVector3 *A = knn.getPoint(tsi->i1);
										const CCVector3 *B = knn.getPoint(tsi->i2);
										const CCVector3 *C = knn.getPoint(tsi->i3);

										CCVector3 no = (*B - *A).cross(*C - *A);
										//no.normalize();
										N += no;
									}
								}

								delete theMesh;
								theMesh = 0;

								//normalize the 'mean' vector
								N.normalize();
							}
						}
					}

					//check normal vector sign
					CCVector3 viewVector = *P /*- cloudTrans.getTranslationAsVec3D()*/; //clouds are still in their local coordinate system!
					if (viewVector.dot(N) > 0)
						N *= -PC_ONE;
					cloud->addNorm(N);

					//progress
					if (!nprogress.oneStep())
					{
						result = CC_FERR_CANCELED_BY_USER;
						//early stop
						j = height;
						break;
					}
				}
			}
		}

		if (result == CC_FERR_CANCELED_BY_USER)
		{
			cloud->unallocateNorms();
		}
	}
	else
	{
		result = CC_FERR_NOT_ENOUGH_MEMORY;
	}

	return result;
}

bool ComputeBestSensor(ccPointCloud* cloud, const std::vector<unsigned>& indexGrid, unsigned width, unsigned height)
{
	PointCoordinateType minPhi = static_cast<PointCoordinateType>(M_PI), maxPhi = -minPhi;
	PointCoordinateType minTheta = static_cast<PointCoordinateType>(M_PI), maxTheta = -minTheta;
	PointCoordinateType deltaPhiRad = 0, deltaThetaRad = 0;
	PointCoordinateType maxRange = 0;

	//we must test if the angles are shifted (i.e the scan spans above theta = pi)
	//we'll compute all parameters for both cases, and choose the best one at the end!
	PointCoordinateType minPhiShifted = minPhi, maxPhiShifted = maxPhi;
	PointCoordinateType minThetaShifted = minTheta, maxThetaShifted = maxTheta;

	try
	{
		//determine the PITCH angular step
		{
			std::vector< AngleAndSpan > angles;
			std::vector< AngleAndSpan > anglesShifted;

			//for each LINE we determine the min and max valid grid point (i.e. != (0,0,0))
			const unsigned* _indexGrid = &(indexGrid[0]);
			for (unsigned j=0; j<height; ++j)
			{
				unsigned minIndex = width;
				unsigned maxIndex = 0;
				for (unsigned i=0; i<width; ++i)
				{
					if (_indexGrid[i] != 0)
					{
						if (i < minIndex)
							minIndex = i;
						if (i > maxIndex)
							maxIndex = i;
					}
				}

				if (maxIndex > minIndex)
				{
					PointCoordinateType minPhiCurrentLine = 0, maxPhiCurrentLine = 0;
					PointCoordinateType minPhiCurrentLineShifted = 0, maxPhiCurrentLineShifted = 0;
					for (unsigned k=minIndex; k<=maxIndex; ++k)
					{
						unsigned index = _indexGrid[k];
						if (index != 0)
						{
							//warning: indexes are shifted (0 = no point)
							const CCVector3* P = cloud->getPoint(index-1);
							PointCoordinateType p = atan2(P->z,sqrt(P->x*P->x + P->y*P->y)); //see ccGBLSensor::projectPoint
							PointCoordinateType pShifted = (p < 0 ? p + static_cast<PointCoordinateType>(2.0*M_PI) : p);
							if (k != minIndex)
							{
								if (minPhiCurrentLine > p)
									minPhiCurrentLine = p;
								else if (maxPhiCurrentLine < p)
									maxPhiCurrentLine = p;

								if (minPhiCurrentLineShifted > pShifted)
									minPhiCurrentLineShifted = pShifted;
								else if (maxPhiCurrentLineShifted < pShifted)
									maxPhiCurrentLineShifted = pShifted;
							}
							else
							{
								minPhiCurrentLine = maxPhiCurrentLine = p;
								minPhiCurrentLineShifted = maxPhiCurrentLineShifted = pShifted;
							}

							//find max range
							PointCoordinateType range = P->norm();
							if (range > maxRange)
								maxRange = range;
						}
					}

					if (minPhi > minPhiCurrentLine)
						minPhi = minPhiCurrentLine;
					if (maxPhi < maxPhiCurrentLine)
						maxPhi = maxPhiCurrentLine;

					if (minPhiShifted > minPhiCurrentLineShifted)
						minPhiShifted = minPhiCurrentLineShifted;
					if (maxPhiShifted < maxPhiCurrentLineShifted)
						maxPhiShifted = maxPhiCurrentLineShifted;

					unsigned span = maxIndex-minIndex+1;
					ScalarType angle_rad = static_cast<ScalarType>((maxPhiCurrentLine-minPhiCurrentLine) / span);
					angles.push_back(AngleAndSpan(angle_rad,span));

					ScalarType angleShifted_rad = static_cast<ScalarType>((maxPhiCurrentLineShifted-minPhiCurrentLineShifted) / span);
					anglesShifted.push_back(AngleAndSpan(angleShifted_rad,span));
				}

				_indexGrid += width;
			}

			if (!angles.empty())
			{
				//check the 'shifted' hypothesis
				PointCoordinateType spanShifted = maxPhiShifted - minPhiShifted;
				PointCoordinateType span = maxPhi - minPhi;
				if (spanShifted < 0.99 * span)
				{
					//we prefer the shifted version!
					angles = anglesShifted;
					minPhi = minPhiShifted;
					maxPhi = maxPhiShifted;
				}

				//we simply take the biggest step evaluation for the widest span!
				size_t maxSpanIndex = 0;
				for (size_t i=1; i<angles.size(); ++i)
				{
					if (	angles[i].second > angles[maxSpanIndex].second
						||	(angles[i].second == angles[maxSpanIndex].second && angles[i].first > angles[maxSpanIndex].first) )
					{
						maxSpanIndex = i;
					}
				}

				deltaPhiRad = static_cast<PointCoordinateType>(angles[maxSpanIndex].first);
				ccLog::Print(QString("[PTX] Detected pitch step: %1 degrees (span [%2 - %3])").arg(deltaPhiRad * CC_RAD_TO_DEG).arg(minPhi * CC_RAD_TO_DEG).arg(maxPhi * CC_RAD_TO_DEG));
			}
			else
			{
				ccLog::Warning("[PTX] Not enough valid points to compute sensor angular step (pitch)!");
				return false;
			}
		}

		//now determine the YAW angular step
		{
			std::vector< AngleAndSpan > angles;
			std::vector< AngleAndSpan > anglesShifted;

			//for each COLUMN we determine the min and max valid grid point (i.e. != (0,0,0))
			for (unsigned i=0; i<width; ++i)
			{
				const unsigned* _indexGrid = &(indexGrid[i]);

				unsigned minIndex = height;
				unsigned maxIndex = 0;
				for (unsigned j=0; j<height; ++j)
				{
					if (_indexGrid[j*width] != 0)
					{
						if (j < minIndex)
							minIndex = j;
						if (j > maxIndex)
							maxIndex = j;
					}
				}

				if (maxIndex > minIndex)
				{
					PointCoordinateType minThetaCurrentCol = 0, maxThetaCurrentCol = 0;
					PointCoordinateType minThetaCurrentColShifted = 0, maxThetaCurrentColShifted = 0;
					for (unsigned k=minIndex; k<=maxIndex; ++k)
					{
						unsigned index = _indexGrid[k*width];
						if (index != 0)
						{
							//warning: indexes are shifted (0 = no point)
							const CCVector3* P = cloud->getPoint(index-1);
							PointCoordinateType t = atan2(P->y,P->x); //see ccGBLSensor::projectPoint
							PointCoordinateType tShifted = (t < 0 ? t + static_cast<PointCoordinateType>(2.0*M_PI) : t);
							if (k != minIndex)
							{
								if (minThetaCurrentColShifted > tShifted)
									minThetaCurrentColShifted = tShifted;
								else if (maxThetaCurrentColShifted < tShifted)
									maxThetaCurrentColShifted = tShifted;

								if (minThetaCurrentCol > t)
									minThetaCurrentCol = t;
								else if (maxThetaCurrentCol < t)
									maxThetaCurrentCol = t;
							}
							else
							{
								minThetaCurrentCol = maxThetaCurrentCol = t;
								minThetaCurrentColShifted = maxThetaCurrentColShifted = tShifted;
							}
						}
					}

					if (minTheta > minThetaCurrentCol)
						minTheta = minThetaCurrentCol;
					if (maxTheta < maxThetaCurrentCol)
						maxTheta = maxThetaCurrentCol;

					if (minThetaShifted > minThetaCurrentColShifted)
						minThetaShifted = minThetaCurrentColShifted;
					if (maxThetaShifted < maxThetaCurrentColShifted)
						maxThetaShifted = maxThetaCurrentColShifted;

					unsigned span = maxIndex-minIndex;
					ScalarType angle_rad = static_cast<ScalarType>((maxThetaCurrentCol-minThetaCurrentCol) / span);
					angles.push_back(AngleAndSpan(angle_rad,span));

					ScalarType angleShifted_rad = static_cast<ScalarType>((maxThetaCurrentColShifted-minThetaCurrentColShifted) / span);
					anglesShifted.push_back(AngleAndSpan(angleShifted_rad,span));
				}
			}

			if (!angles.empty())
			{
				//check the 'shifted' hypothesis
				PointCoordinateType spanShifted = maxThetaShifted - minThetaShifted;
				PointCoordinateType span = maxTheta - minTheta;
				if (spanShifted < 0.99 * span)
				{
					//we prefer the shifted version!
					angles = anglesShifted;
					minTheta = minThetaShifted;
					maxTheta = maxThetaShifted;
				}

				//we simply take the biggest step evaluation for the widest span!
				size_t maxSpanIndex = 0;
				for (size_t i=1; i<angles.size(); ++i)
				{
					if (	angles[i].second > angles[maxSpanIndex].second
						||	(angles[i].second == angles[maxSpanIndex].second && angles[i].first > angles[maxSpanIndex].first) )
					{
						maxSpanIndex = i;
					}
				}

				deltaThetaRad = static_cast<PointCoordinateType>(angles[maxSpanIndex].first);
				ccLog::Print(QString("[PTX] Detected yaw step: %1 degrees (span [%2 - %3])").arg(deltaThetaRad * CC_RAD_TO_DEG).arg(minTheta * CC_RAD_TO_DEG).arg(maxTheta * CC_RAD_TO_DEG));
			}
			else
			{
				ccLog::Warning("[PTX] Not enough valid points to compute sensor angular steps!");
				return false;
			}
		}
	}
	catch (std::bad_alloc)
	{
		ccLog::Warning("[PTX] Not enough memory to compute sensor angular steps!");
		return false;
	}

	ccGBLSensor* sensor = new ccGBLSensor(ccGBLSensor::YAW_THEN_PITCH);
	if (sensor)
	{
		sensor->setPitchStep(deltaPhiRad);
		sensor->setPitchRange(minPhi,maxPhi);
		sensor->setYawStep(deltaThetaRad);
		sensor->setYawRange(minTheta,maxTheta);
		sensor->setSensorRange(maxRange);
		//sensor->setRigidTransformation(cloudTrans/*sensorTrans*/); //will be called later by applyGLTransformation_recursive
		sensor->setGraphicScale(PC_ONE/2);
		sensor->setVisible(true);
		sensor->setEnabled(false);
		cloud->addChild(sensor);
	}

	return sensor != 0;
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
	bool computeNormals = (s_normalCompBehavior == ALWAYS);

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
		std::vector<unsigned> indexGrid;
		bool hasIndexGrid = true;
		try
		{
			indexGrid.resize(gridSize,0);
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
			unsigned* _indexGrid = hasIndexGrid ? &(indexGrid[0]) : 0;

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

						//add point
						cloud->addPoint(CCVector3(	static_cast<PointCoordinateType>(Pd[0] + PshiftCloud.x),
													static_cast<PointCoordinateType>(Pd[1] + PshiftCloud.y),
													static_cast<PointCoordinateType>(Pd[2] + PshiftCloud.z)) );

						//update index grid
						if (hasIndexGrid)
							*_indexGrid = cloud->size(); // = index + 1 (default value = 0, means no point)

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
								ok &= (temp <= static_cast<unsigned>(MAX_COLOR_COMP));
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
				ComputeBestSensor(cloud,indexGrid,width,height);

#ifndef _DEBUG
				//shall we ask the user if he wants to compute normals or not?
				if (s_normalCompBehavior == ASK_USER && cloudIndex == 0)
				{
					QMessageBox msgBox(QMessageBox::Question,"PTX normals computation","Compute normals? (this process is a bit long\nbut generally cleaner than standard methods)");
					msgBox.addButton(QMessageBox::Yes);
					msgBox.addButton(QMessageBox::YesToAll);
					msgBox.addButton(QMessageBox::No);
					msgBox.addButton(QMessageBox::NoToAll);
					
					msgBox.exec();
					if (msgBox.clickedButton() == msgBox.button(QMessageBox::Yes))
					{
						computeNormals = true;
					}
					else if (msgBox.clickedButton() == msgBox.button(QMessageBox::No))
					{
						computeNormals = false;
					}
					else if (msgBox.clickedButton() == msgBox.button(QMessageBox::YesToAll))
					{
						computeNormals = true;
						SetNormalsComputationBehavior(ALWAYS);
					}
					else if (msgBox.clickedButton() == msgBox.button(QMessageBox::NoToAll))
					{
						computeNormals = true;
						SetNormalsComputationBehavior(NEVER);
					}
				}
#endif
				if (computeNormals)
				{
					//try to compute normals
					CC_FILE_ERROR normalResult = ComputeNormals(cloud,indexGrid,static_cast<int>(width),static_cast<int>(height));

					if (normalResult == CC_FERR_CANCELED_BY_USER)
					{
						ccLog::Warning("[PTX] Normal computation canceled by user!");
						//if the user cancelled the normal process, we cancel everything!
						result = CC_FERR_CANCELED_BY_USER;
					}
					else if (normalResult == CC_FERR_NOT_ENOUGH_MEMORY)
					{
						ccLog::Warning("[PTX] Not enough memory to compute normals!");
						computeNormals = false;
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
