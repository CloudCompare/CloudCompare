// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccGriddedTools.h"

// Local
#include "ccGBLSensor.h"
#include "ccLog.h"

//! Association of an angle and the corresponding number of rows/columns
using AngleAndSpan = std::pair<PointCoordinateType, unsigned>;

bool ccGriddedTools::DetectParameters(const ccPointCloud*              cloud,
                                      const ccPointCloud::Grid::Shared grid,
                                      GridParameters&                  parameters,
                                      bool                             verbose /*=false*/,
                                      ccGLMatrixd*                     cloudToSensorTrans /*=nullptr*/)
{
	if (!cloud || !grid)
	{
		assert(false);
		return false;
	}

	parameters.minPhi        = M_PI;
	parameters.maxPhi        = -parameters.minPhi;
	parameters.minTheta      = M_PI;
	parameters.maxTheta      = -parameters.minTheta;
	parameters.deltaPhiRad   = 0;
	parameters.deltaThetaRad = 0;
	parameters.maxRange      = 0;
	parameters.maxRange      = 0;

	// we must test if the angles are shifted (i.e the scan spans above theta = pi)
	// we'll compute all parameters for both cases, and choose the best one at the end!
	double minPhiShifted   = parameters.minPhi;
	double maxPhiShifted   = parameters.maxPhi;
	double minThetaShifted = parameters.minTheta;
	double maxThetaShifted = parameters.maxTheta;

	try
	{
		// determine the PITCH angular step
		{
			std::vector<AngleAndSpan> angles;
			std::vector<AngleAndSpan> anglesShifted;

			// for each ROW we determine the min and max valid grid point (i.e. index >= 0)
			const int* _indexGrid = grid->indexes.data();
			for (unsigned j = 0; j < grid->h; ++j)
			{
				unsigned minIndex = grid->w;
				unsigned maxIndex = 0;
				for (unsigned i = 0; i < grid->w; ++i)
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
					double minPhiCurrentLine        = 0;
					double maxPhiCurrentLine        = 0;
					double minPhiCurrentLineShifted = 0;
					double maxPhiCurrentLineShifted = 0;
					for (unsigned k = minIndex; k <= maxIndex; ++k)
					{
						int index = _indexGrid[k];
						if (index >= 0)
						{
							CCVector3d P;
							cloud->getGlobalPoint(static_cast<unsigned>(index), P);
							if (cloudToSensorTrans)
								cloudToSensorTrans->apply(P);
							double p        = atan2(P.z, sqrt(P.x * P.x + P.y * P.y)); // see ccGBLSensor::projectPoint
							double pShifted = (p < 0 ? p + 2.0 * M_PI : p);
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

							// find max range
							double range = P.norm();
							if (range > parameters.maxRange)
								parameters.maxRange = range;
						}
					}

					if (parameters.minPhi > minPhiCurrentLine)
						parameters.minPhi = minPhiCurrentLine;
					if (parameters.maxPhi < maxPhiCurrentLine)
						parameters.maxPhi = maxPhiCurrentLine;

					if (minPhiShifted > minPhiCurrentLineShifted)
						minPhiShifted = minPhiCurrentLineShifted;
					if (maxPhiShifted < maxPhiCurrentLineShifted)
						maxPhiShifted = maxPhiCurrentLineShifted;

					unsigned   span      = maxIndex - minIndex + 1;
					ScalarType angle_rad = static_cast<ScalarType>((maxPhiCurrentLine - minPhiCurrentLine) / span);
					angles.emplace_back(angle_rad, span);

					ScalarType angleShifted_rad = static_cast<ScalarType>((maxPhiCurrentLineShifted - minPhiCurrentLineShifted) / span);
					anglesShifted.emplace_back(angleShifted_rad, span);
				}

				_indexGrid += grid->w;
			}

			if (!angles.empty())
			{
				// check the 'shifted' hypothesis
				double spanShifted = maxPhiShifted - minPhiShifted;
				double span        = parameters.maxPhi - parameters.minPhi;
				if (spanShifted < 0.99 * span)
				{
					// we prefer the shifted version!
					angles            = anglesShifted;
					parameters.minPhi = minPhiShifted;
					parameters.maxPhi = maxPhiShifted;
				}

				// we simply take the biggest step evaluation for the widest span!
				size_t maxSpanIndex = 0;
				for (size_t i = 1; i < angles.size(); ++i)
				{
					if (angles[i].second > angles[maxSpanIndex].second
					    || (angles[i].second == angles[maxSpanIndex].second && angles[i].first > angles[maxSpanIndex].first))
					{
						maxSpanIndex = i;
					}
				}

				parameters.deltaPhiRad = angles[maxSpanIndex].first;
				if (verbose)
				{
					ccLog::Print(QStringLiteral("[Scan grid] Detected pitch step: %1 degrees (span [%2 - %3])")
					                 .arg(CCCoreLib::RadiansToDegrees(parameters.deltaPhiRad))
					                 .arg(CCCoreLib::RadiansToDegrees(parameters.minPhi))
					                 .arg(CCCoreLib::RadiansToDegrees(parameters.maxPhi)));
				}
			}
			else
			{
				ccLog::Warning("[Scan grid] Not enough valid points to compute the scan angular step (pitch)!");
				return false;
			}
		}

		// now determine the YAW angular step
		{
			std::vector<AngleAndSpan> angles;
			std::vector<AngleAndSpan> anglesShifted;

			// for each COLUMN we determine the min and max valid grid point (i.e. index >= 0)
			for (unsigned i = 0; i < grid->w; ++i)
			{
				const int* _indexGrid = &(grid->indexes[i]);

				unsigned minIndex = grid->h;
				unsigned maxIndex = 0;
				for (unsigned j = 0; j < grid->h; ++j)
				{
					if (_indexGrid[j * grid->w] >= 0)
					{
						if (j < minIndex)
							minIndex = j;
						if (j > maxIndex)
							maxIndex = j;
					}
				}

				if (maxIndex > minIndex)
				{
					double minThetaCurrentCol        = 0;
					double maxThetaCurrentCol        = 0;
					double minThetaCurrentColShifted = 0;
					double maxThetaCurrentColShifted = 0;
					for (unsigned k = minIndex; k <= maxIndex; ++k)
					{
						int index = _indexGrid[k * grid->w];
						if (index >= 0)
						{
							// warning: indexes are shifted (0 = no point)
							CCVector3d P;
							cloud->getGlobalPoint(static_cast<unsigned>(index), P);
							if (cloudToSensorTrans)
								cloudToSensorTrans->apply(P);
							double t        = atan2(P.y, P.x); // see ccGBLSensor::projectPoint
							double tShifted = (t < 0 ? t + 2.0 * M_PI : t);
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

					if (parameters.minTheta > minThetaCurrentCol)
						parameters.minTheta = minThetaCurrentCol;
					if (parameters.maxTheta < maxThetaCurrentCol)
						parameters.maxTheta = maxThetaCurrentCol;

					if (minThetaShifted > minThetaCurrentColShifted)
						minThetaShifted = minThetaCurrentColShifted;
					if (maxThetaShifted < maxThetaCurrentColShifted)
						maxThetaShifted = maxThetaCurrentColShifted;

					unsigned   span      = maxIndex - minIndex;
					ScalarType angle_rad = static_cast<ScalarType>((maxThetaCurrentCol - minThetaCurrentCol) / span);
					angles.emplace_back(angle_rad, span);

					ScalarType angleShifted_rad = static_cast<ScalarType>((maxThetaCurrentColShifted - minThetaCurrentColShifted) / span);
					anglesShifted.emplace_back(angleShifted_rad, span);
				}
			}

			if (!angles.empty())
			{
				// check the 'shifted' hypothesis
				double spanShifted = maxThetaShifted - minThetaShifted;
				double span        = parameters.maxTheta - parameters.minTheta;
				if (spanShifted < 0.99 * span)
				{
					// we prefer the shifted version!
					angles              = anglesShifted;
					parameters.minTheta = minThetaShifted;
					parameters.maxTheta = maxThetaShifted;
				}

				// we simply take the biggest step evaluation for the widest span!
				size_t maxSpanIndex = 0;
				for (size_t i = 1; i < angles.size(); ++i)
				{
					if (angles[i].second > angles[maxSpanIndex].second
					    || (angles[i].second == angles[maxSpanIndex].second && angles[i].first > angles[maxSpanIndex].first))
					{
						maxSpanIndex = i;
					}
				}

				parameters.deltaThetaRad = angles[maxSpanIndex].first;
				if (verbose)
				{
					ccLog::Print(QStringLiteral("[Scan grid] Detected yaw step: %1 degrees (span [%2 - %3])")
					                 .arg(CCCoreLib::RadiansToDegrees(parameters.deltaThetaRad))
					                 .arg(CCCoreLib::RadiansToDegrees(parameters.minTheta))
					                 .arg(CCCoreLib::RadiansToDegrees(parameters.maxTheta)));
				}
			}
			else
			{
				ccLog::Warning("[Scan grid] Not enough valid points to compute the scan angular steps!");
				return false;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[Scan grid] Not enough memory to compute the scan angular steps!");
		return false;
	}

	return true;
}

ccGBLSensor* ccGriddedTools::ComputeBestSensor(ccPointCloud*              cloud,
                                               ccPointCloud::Grid::Shared grid,
                                               ccGLMatrixd*               cloudToSensorTrans /*=nullptr*/)
{
	GridParameters parameters;
	if (!DetectParameters(cloud, grid, parameters, true, cloudToSensorTrans))
	{
		return nullptr;
	}

	ccGBLSensor* sensor = new ccGBLSensor(ccGBLSensor::YAW_THEN_PITCH);
	if (sensor)
	{
		sensor->setPitchStep(parameters.deltaPhiRad);
		sensor->setPitchRange(parameters.minPhi, parameters.maxPhi);
		sensor->setYawStep(parameters.deltaThetaRad);
		sensor->setYawRange(parameters.minTheta, parameters.maxTheta);
		sensor->setSensorRange(parameters.maxRange);
		sensor->setGraphicScale(CCCoreLib::PC_ONE / 2);
		sensor->setVisible(true);
		sensor->setEnabled(false);
	}

	return sensor;
}
