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

#include "ccGriddedTools.h"

//Local
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#include "ccGBLSensor.h"
#include "ccLog.h"

//CCLib
#include <ReferenceCloud.h>
#include <GenericIndexedMesh.h>
#include <Neighbourhood.h>

//Qt
#include <QMessageBox>

//! Association of an angle and the corresponding number of rows/columns
typedef std::pair<PointCoordinateType,unsigned> AngleAndSpan;

bool ccGriddedTools::ComputeNormals(ccPointCloud* cloud,
									const std::vector<int>& indexGrid,
									int width,
									int height,
									bool* canceledByUser/*=0*/,
									int kernelWidth/*=3*/ )
{
	//init
	bool result = true;
	if (canceledByUser)
		*canceledByUser = false;

	//try to compute normals
	if (cloud->reserveTheNormsTable())
	{
		//progress dialog
		ccProgressDialog pdlg(true);
		CCLib::NormalizedProgress nprogress(&pdlg,cloud->size());
		pdlg.setMethodTitle("Compute normals");
		pdlg.setInfo(qPrintable(QString("Number of points: %1").arg(cloud->size())));
		pdlg.start();

		const int* _indexGrid = &(indexGrid[0]);
		CCLib::ReferenceCloud knn(cloud);
		
		//neighborhood 'half-width' (total width = 1 + 2*kernelWidth) 
		//max number of neighbours: (1+2*nw)^2
		knn.reserve((1+2*kernelWidth)*(1+2*kernelWidth));

		//for each grid cell
		for (int j=0; j<height; ++j)
		{
			for (int i=0; i<width; ++i, ++_indexGrid)
			{
				if (*_indexGrid >= 0)
				{
					knn.clear(false);

					unsigned pointIndex = static_cast<unsigned>(*_indexGrid);
					const CCVector3* P = cloud->getPoint(pointIndex);

					//look for neighbors
					for (int v=std::max(0,j-kernelWidth); v<std::min<int>(height,j+kernelWidth); ++v)
					{
						for (int u=std::max(0,i-kernelWidth); u<std::min<int>(width,i+kernelWidth); ++u)
						{
							if (v != j || u != i)
							{
								int indexN = indexGrid[v*width + u];
								if (indexN >= 0)
								{
									//we don't consider points with a too much different depth than the central point
									const CCVector3* Pn = cloud->getPoint(static_cast<unsigned>(indexN));
									if (fabs(Pn->z - P->z) <= std::max(fabs(Pn->x - P->x),fabs(Pn->y - P->y)))
										knn.addPointIndex(static_cast<unsigned>(indexN));
								}
							}
						}
					}

					CCVector3 N(0,0,1);
					if (knn.size() >= 3)
					{
						CCLib::Neighbourhood Z(&knn);

						//compute normal with quadratic func. (if we have enough points)
						if (false/*knn.size() >= 6*/)
						{
							Tuple3ub dims;
							const PointCoordinateType* h = Z.getQuadric(&dims);
							if (h)
							{
								const CCVector3* gv = Z.getGravityCenter();
								assert(gv);

								const uchar& iX = dims.x;
								const uchar& iY = dims.y;
								const uchar& iZ = dims.z;

								PointCoordinateType lX = P->u[iX] - gv->u[iX];
								PointCoordinateType lY = P->u[iY] - gv->u[iY];

								N.u[iX] = h[1] + (2 * h[3] * lX) + (h[4] * lY);
								N.u[iY] = h[2] + (2 * h[5] * lY) + (h[4] * lX);
								N.u[iZ] = -PC_ONE;

								N.normalize();
							}
						}
						else
#define USE_LS_PLANE
#ifdef USE_LS_PLANE
						{
							//compute normal with best fit plane
							const CCVector3* _N = Z.getLSPlaneNormal();
							if (_N)
								N = *_N;
						}
#else
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
									const CCLib::VerticesIndexes* tsi = theMesh->getNextTriangleVertIndexes();
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
#endif
					}

					//check normal vector sign
					CCVector3 viewVector = *P /*- cloudTrans.getTranslationAsVec3D()*/; //clouds are still in their local coordinate system!
					if (viewVector.dot(N) > 0)
						N *= -PC_ONE;
					cloud->addNorm(N);

					//progress
					if (!nprogress.oneStep())
					{
						result = false;
						if (canceledByUser)
							*canceledByUser = true;
						ccLog::Warning("[ComputeNormals] Process canceled by user!");
						//early stop
						j = height;
						break;
					}
				}
			}
		}

		if (!result)
		{
			cloud->unallocateNorms();
		}
	}
	else
	{
		ccLog::Warning("[ComputeNormals] Not enough memory!");
	}

	return result;
}

ccGBLSensor* ccGriddedTools::ComputeBestSensor(ccPointCloud* cloud, const std::vector<int>& indexGrid, unsigned width, unsigned height, ccGLMatrix* cloudToSensorTrans/*=0*/)
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
			const int* _indexGrid = &(indexGrid[0]);
			for (unsigned j=0; j<height; ++j)
			{
				unsigned minIndex = width;
				unsigned maxIndex = 0;
				for (unsigned i=0; i<width; ++i)
				{
					if (_indexGrid[i] >= 0)
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
						int index = _indexGrid[k];
						if (index >= 0)
						{
							CCVector3 P = *(cloud->getPoint(static_cast<unsigned>(index)));
							if (cloudToSensorTrans)
								cloudToSensorTrans->apply(P);
							PointCoordinateType p = atan2(P.z,sqrt(P.x*P.x + P.y*P.y)); //see ccGBLSensor::projectPoint
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
							PointCoordinateType range = P.norm();
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
				return 0;
			}
		}

		//now determine the YAW angular step
		{
			std::vector< AngleAndSpan > angles;
			std::vector< AngleAndSpan > anglesShifted;

			//for each COLUMN we determine the min and max valid grid point (i.e. != (0,0,0))
			for (unsigned i=0; i<width; ++i)
			{
				const int* _indexGrid = &(indexGrid[i]);

				unsigned minIndex = height;
				unsigned maxIndex = 0;
				for (unsigned j=0; j<height; ++j)
				{
					if (_indexGrid[j*width] >= 0)
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
						int index = _indexGrid[k*width];
						if (index >= 0)
						{
							//warning: indexes are shifted (0 = no point)
							CCVector3 P = *(cloud->getPoint(static_cast<unsigned>(index)));
							if (cloudToSensorTrans)
								cloudToSensorTrans->apply(P);
							PointCoordinateType t = atan2(P.y,P.x); //see ccGBLSensor::projectPoint
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
				return 0;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[PTX] Not enough memory to compute sensor angular steps!");
		return 0;
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
	}

	return sensor;
}

bool ccGriddedTools::HandleAutoComputeNormalsFeature(ComputeNormalsBehavior& behavior)
{
	//shall we ask the user if he wants to compute normals or not?
	switch(behavior)
	{
	case ALWAYS:
		return true;

	case ASK_USER:
	{
		QMessageBox msgBox(QMessageBox::Question,"Normals computation","Compute normals? (this process is a bit long\nbut generally cleaner than standard methods)");
		msgBox.addButton(QMessageBox::Yes);
		msgBox.addButton(QMessageBox::YesToAll);
		msgBox.addButton(QMessageBox::No);
		msgBox.addButton(QMessageBox::NoToAll);

		msgBox.exec();
		if (msgBox.clickedButton() == msgBox.button(QMessageBox::Yes))
		{
			return true;
		}
		else if (msgBox.clickedButton() == msgBox.button(QMessageBox::No))
		{
			return false;
		}
		else if (msgBox.clickedButton() == msgBox.button(QMessageBox::YesToAll))
		{
			behavior = ALWAYS;
			return true;
		}
		else if (msgBox.clickedButton() == msgBox.button(QMessageBox::NoToAll))
		{
			behavior = NEVER;
			return false;
		}
	}
	break;

	case NEVER:
		return false;
	}

	assert(false);
	return false;
}