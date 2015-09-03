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
#include "ccGBLSensor.h"
#include "ccLog.h"

//! Association of an angle and the corresponding number of rows/columns
typedef std::pair<PointCoordinateType,unsigned> AngleAndSpan;

ccGBLSensor* ccGriddedTools::ComputeBestSensor(ccPointCloud* cloud, ccPointCloud::Grid::Shared grid, ccGLMatrix* cloudToSensorTrans/*=0*/)
{
	if (!cloud || !grid)
	{
		assert(false);
		return 0;
	}

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
			const int* _indexGrid = &(grid->indexes[0]);
			for (unsigned j=0; j<grid->h; ++j)
			{
				unsigned minIndex = grid->w;
				unsigned maxIndex = 0;
				for (unsigned i=0; i<grid->w; ++i)
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

				_indexGrid += grid->w;
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
				ccLog::Print(QString("[Scan grid] Detected pitch step: %1 degrees (span [%2 - %3])").arg(deltaPhiRad * CC_RAD_TO_DEG).arg(minPhi * CC_RAD_TO_DEG).arg(maxPhi * CC_RAD_TO_DEG));
			}
			else
			{
				ccLog::Warning("[Scan grid] Not enough valid points to compute sensor angular step (pitch)!");
				return 0;
			}
		}

		//now determine the YAW angular step
		{
			std::vector< AngleAndSpan > angles;
			std::vector< AngleAndSpan > anglesShifted;

			//for each COLUMN we determine the min and max valid grid point (i.e. != (0,0,0))
			for (unsigned i=0; i<grid->w; ++i)
			{
				const int* _indexGrid = &(grid->indexes[i]);

				unsigned minIndex = grid->h;
				unsigned maxIndex = 0;
				for (unsigned j=0; j<grid->h; ++j)
				{
					if (_indexGrid[j*grid->w] >= 0)
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
						int index = _indexGrid[k*grid->w];
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
				ccLog::Print(QString("[Scan grid] Detected yaw step: %1 degrees (span [%2 - %3])").arg(deltaThetaRad * CC_RAD_TO_DEG).arg(minTheta * CC_RAD_TO_DEG).arg(maxTheta * CC_RAD_TO_DEG));
			}
			else
			{
				ccLog::Warning("[Scan grid] Not enough valid points to compute sensor angular steps!");
				return 0;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[Scan grid] Not enough memory to compute sensor angular steps!");
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
