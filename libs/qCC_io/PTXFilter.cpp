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

//System
#include <assert.h>
#include <string.h>

const char CC_PTX_INTENSITY_FIELD_NAME[] = "Intensity";

CC_FILE_ERROR PTXFilter::saveToFile(ccHObject* entity, QString filename)
{
	//not supported
	return CC_FERR_WRONG_FILE_TYPE;
}

typedef std::pair<PointCoordinateType,unsigned> AngleAndSpan;

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

	CCVector3d PshiftTrans(0,0,0);
	CCVector3d PshiftCloud(0,0,0);

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	ScalarType minIntensity = 0;
	ScalarType maxIntensity = 0;
	for (unsigned cloudIndex = 0; result == CC_FERR_NO_ERROR; cloudIndex++)
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

			ccLog::Print(QString("[PTX] Sub-cloud #%1 - grid size: %2 x %3").arg(cloudIndex+1).arg(width).arg(height));

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
		unsigned size = width * height;
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

		if (!cloud->reserve(size))
		{
			delete cloud;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		//set global shift
		cloud->setGlobalShift(PshiftTrans);

		//intensities
		ccScalarField* intensitySF = new ccScalarField(CC_PTX_INTENSITY_FIELD_NAME);
		if (!intensitySF->reserve(static_cast<unsigned>(size)))
		{
			ccLog::Warning("[PTX] Not enough memory to load intensities!");
			intensitySF->release();
			intensitySF = 0;
		}

		//grid for computing normals
		unsigned* indexGrid = new unsigned[size];
		if (indexGrid)
		{
			memset(indexGrid,0,sizeof(unsigned)*size);
		}
		else
		{
			ccLog::Warning("[PTX] Not enough memory to save grid structure (required to compute normals!)");
		}

		//read points
		{
			//progress dialog
			ccProgressDialog pdlg(true);
			CCLib::NormalizedProgress nprogress(&pdlg,size);
			pdlg.setMethodTitle("Loading PTX file");
			pdlg.setInfo(qPrintable(QString("Number of cells: %1").arg(size)));
			pdlg.start();

			bool firstPoint = true;
			bool hasColors = false;
			bool loadColors = false;

			for (unsigned j=0; j<height; ++j)
			{
				for (unsigned i=0; i<width; ++i)
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
						if (indexGrid)
							indexGrid[j*width + i] = cloud->size(); // = index + 1 (as default value = 0)

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
			if (intensitySF)
				intensitySF->release();
			if (result == CC_FERR_NO_ERROR)
				result = CC_FERR_NO_LOAD;
		}
		else if (cloud->size() <= cloud->capacity())
		{
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

			if (indexGrid && result != CC_FERR_CANCELED_BY_USER)
			{
				//update sensor information
				{
					bool validSensor = true;
					PointCoordinateType minPhi = static_cast<PointCoordinateType>(M_PI), maxPhi = -minPhi;
					PointCoordinateType minTheta = static_cast<PointCoordinateType>(M_PI), maxTheta = -minTheta;
					double deltaThetaRad = 0, deltaPhiRad = 0;
					std::vector< AngleAndSpan > angles;
					
					try
					{
						//determine the longitudinal angular step
						//for each line we determine the min and max valid grid point (i.e. != (0,0,0))
						{
							const unsigned* _indexGrid = indexGrid;
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

								PointCoordinateType minPhiLine = 0, maxPhiLine = 0;
								for (unsigned k=minIndex; k<=maxIndex; ++k)
								{
									unsigned index = _indexGrid[k];
									if (index != 0)
									{
										//warning: indexes are shifted (0 = no point)
										const CCVector3* P = cloud->getPoint(index-1);
										PointCoordinateType p = atan2(P->z,sqrt(P->x*P->x + P->y*P->y));
										if (k != minIndex)
										{
											if (minPhiLine > p)
												minPhiLine = p;
											else if (maxPhiLine < p)
												maxPhiLine = p;
										}
										else
										{
											minPhiLine = maxPhiLine = p;
										}
									}

									if (minPhi > minPhiLine)
										minPhi = minPhiLine;
									if (maxPhi < maxPhiLine)
										maxPhi = maxPhiLine;

									unsigned span = maxIndex-minIndex;
									ScalarType angle_rad = static_cast<ScalarType>((maxPhiLine-minPhiLine) / span);
									angles.push_back(AngleAndSpan(angle_rad,span));
								}

								_indexGrid += width;
							}

							if (!angles.empty())
							{
								size_t maxSpanIndex = 0;
								for (size_t i=1; i<angles.size(); ++i)
								{
									if (	angles[i].second > angles[maxSpanIndex].second
										||	(angles[i].second == angles[maxSpanIndex].second && angles[i].first > angles[maxSpanIndex].first) )
									{
										maxSpanIndex = i;
									}
								}
								deltaPhiRad = angles[maxSpanIndex].first;
								ccLog::Print(QString("[PTX] Detected latitudinal step: %1 degrees").arg(deltaPhiRad * CC_RAD_TO_DEG));
							}
							else
							{
								ccLog::Warning("[PTX] Not enough valid points to compute sensor angular steps!");
							}
						}

						//now determine the latitudinal angular step
						if (validSensor)
						{
							angles.clear();
							//for each column we determine the min and max valid grid point (i.e. != (0,0,0))
							for (unsigned i=0; i<width; ++i)
							{
								const unsigned* _indexGrid = indexGrid + i;
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
							
								PointCoordinateType minThetaCol = 0, maxThetaCol = 0;
								for (unsigned k=minIndex; k<=maxIndex; ++k)
								{
									unsigned index = _indexGrid[k*width];
									if (index != 0)
									{
										//warning: indexes are shifted (0 = no point)
										const CCVector3* P = cloud->getPoint(index-1);
										PointCoordinateType t = atan2(P->x,P->y);
										if (k != minIndex)
										{
											if (minThetaCol > t)
												minThetaCol = t;
											else if (maxThetaCol < t)
												maxThetaCol = t;
										}
										else
										{
											minThetaCol = maxThetaCol = t;
										}
									}

									if (minTheta > minThetaCol)
										minTheta = minThetaCol;
									if (maxTheta < maxThetaCol)
										maxTheta = maxThetaCol;

									unsigned span = maxIndex-minIndex;
									ScalarType angle_rad = static_cast<ScalarType>((maxThetaCol-minThetaCol) / span);
									angles.push_back(AngleAndSpan(angle_rad,span));
								}
							}

							if (!angles.empty())
							{
								size_t maxSpanIndex = 0;
								for (size_t i=1; i<angles.size(); ++i)
								{
									if (	angles[i].second > angles[maxSpanIndex].second
										||	(angles[i].second == angles[maxSpanIndex].second && angles[i].first > angles[maxSpanIndex].first) )
									{
										maxSpanIndex = i;
									}
								}
								deltaThetaRad = angles[maxSpanIndex].first;
								ccLog::Print(QString("[PTX] Detected longitudinal step: %1 degrees").arg(deltaThetaRad * CC_RAD_TO_DEG));
							}
							else
							{
								ccLog::Warning("[PTX] Not enough valid points to compute sensor angular steps!");
								validSensor = false;
							}
						}
					}
					catch (std::bad_alloc)
					{
						ccLog::Warning("[PTX] Not enough memory to compute sensor angular steps!");
						validSensor = false;
					}

					ccGBLSensor* sensor = new ccGBLSensor(/*ccGBLSensor::PHI_THETA*/);
					if (sensor)
					{
						sensor->setDeltaPhi(deltaPhiRad);
						sensor->setPhi(minPhi,maxPhi);
						sensor->setDeltaTheta(deltaThetaRad);
						sensor->setTheta(minTheta,maxTheta);
						//sensor->setRigidTransformation(cloudTrans/*sensorTrans*/); //will be called later by applyGLTransformation_recursive
						sensor->setGraphicScale(PC_ONE/2);
						sensor->setVisible(true);
						sensor->setEnabled(false);
						cloud->addChild(sensor);
					}
				}

				//try to compute normals
#ifdef _DEBUG
				if (false)
#else
				if (cloud->reserveTheNormsTable())
#endif
				{
					//progress dialog
					ccProgressDialog pdlg(true);
					CCLib::NormalizedProgress nprogress(&pdlg,cloud->size());
					pdlg.setMethodTitle("Compute PTX normals");
					pdlg.setInfo(qPrintable(QString("Number of points: %1").arg(cloud->size())));
					pdlg.start();

					const unsigned* _indexGrid = indexGrid;
					CCLib::ReferenceCloud knn(cloud);
					const int nw = 3;
					knn.reserve((1+2*nw)*(1+2*nw));
					for (int j=0; j<static_cast<int>(height); ++j)
					{
						for (int i=0; i<static_cast<int>(width); ++i, ++_indexGrid)
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
											N.u[iZ] = -1.0;

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
									N *= -1;
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
						ccLog::Warning("[PTX] Normal computation canceled by user!");
						cloud->unallocateNorms();
					}
				}
				else
				{
					ccLog::Warning("[PTX] Not enough memory to compute normals!");
				}
			}

			if (indexGrid)
			{
				//test: export as depth map
				//if (true)
				//{
				//	_indexGrid = indexGrid;
				//	for (int j=0; j<static_cast<int>(height); ++j)
				//	{
				//		for (int i=0; i<static_cast<int>(width); ++i, ++_indexGrid)
				//		{
				//			if (*_indexGrid != 0)
				//			{
				//				const CCVector3* P = cloud->getPoint(*_indexGrid-1);
				//				const_cast<CCVector3*>(P)->x = static_cast<PointCoordinateType>(j);
				//				const_cast<CCVector3*>(P)->y = static_cast<PointCoordinateType>(i);
				//			}
				//		}
				//	}
				//}

				//don't need it anymore
				delete[] indexGrid;
				indexGrid = 0;
			}

			//we apply the transformation
			cloud->applyGLTransformation_recursive(&cloudTrans);

			cloud->setVisible(true);
			cloud->showColors(cloud->hasColors());
			cloud->showNormals(cloud->hasNormals());

			container.addChild(cloud);

#ifdef _DEBUG
			//FIXME
			//break;
#endif
		}

		if (indexGrid)
		{
			delete[] indexGrid;
			indexGrid = 0;
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
