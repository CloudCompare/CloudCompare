//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#             COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015              #
//#                                                                        #
//##########################################################################

#include "ViewInterpolate.h"

//qCC_db
#include <ccPolyline.h>

ViewInterpolate::ViewInterpolate(	const ExtendedViewportParameters& viewParams1,
									const ExtendedViewportParameters& viewParams2,
									unsigned int stepCount )
	: m_view1(viewParams1)
	, m_view2(viewParams2)
	, m_totalSteps(stepCount)
	, m_currentStep(0)
	, smoothTrajectory(nullptr)
	, smoothTrajectoryReversed(nullptr)
	, smoothTrajStartIndex(0)
	, smoothTrajStopIndex(0)
	, smoothTrajCurrentIndex(0)
	, smoothSegmentLength(0)
	, smoothCurrentLength(0)
{
}

void ViewInterpolate::setSmoothTrajectory(	ccPolyline* _smoothTrajectory,
											ccPolyline* _smoothTrajectoryReversed,
											unsigned i1,
											unsigned i2,
											PointCoordinateType length)
{
	smoothTrajectory = _smoothTrajectory;
	smoothTrajectoryReversed = _smoothTrajectoryReversed;
	smoothTrajCurrentIndex = smoothTrajStartIndex = i1;
	smoothTrajStopIndex = i2;
	smoothSegmentLength = length;
	smoothCurrentLength = 0;
}

//helper function for interpolating between simple numerical types
template <class T> T InterpolateNumber(T start, T end, double interpolationFraction)
{
	double dStart = static_cast<double>(start);
	double dEnd = static_cast<double>(end);
	if (std::isnan(dStart))
	{
		if (std::isnan(dEnd))
		{
			return std::numeric_limits<T>::quiet_NaN();
		}
		else
		{
			return (interpolationFraction < 0.5 ? std::numeric_limits<T>::quiet_NaN() : end);
		}
	}
	else if (std::isnan(dEnd))
	{
		return (interpolationFraction < 0.5 ? start : std::numeric_limits<T>::quiet_NaN());
	}
	return static_cast<T> ( static_cast<double>(start) + (static_cast<double>(end) - static_cast<double>(start)) * interpolationFraction );
}

bool ViewInterpolate::interpolate(ExtendedViewportParameters& interpView, double interpolateFraction) const
{
	if (	interpolateFraction < 0.0
		||	interpolateFraction > 1.0)
	{
		return false;
	}

    interpView.params = m_view1.params;
	{
		interpView.params.defaultPointSize       = InterpolateNumber ( m_view1.params.defaultPointSize, m_view2.params.defaultPointSize, interpolateFraction );
		interpView.params.defaultLineWidth       = InterpolateNumber ( m_view1.params.defaultLineWidth, m_view2.params.defaultLineWidth, interpolateFraction );
		interpView.params.zNearCoef              = InterpolateNumber ( m_view1.params.zNearCoef, m_view2.params.zNearCoef, interpolateFraction );
		interpView.params.zNear                  = InterpolateNumber ( m_view1.params.zNear, m_view2.params.zNear, interpolateFraction );
		interpView.params.zFar                   = InterpolateNumber ( m_view1.params.zFar, m_view2.params.zFar, interpolateFraction );
		interpView.params.nearClippingDepth      = InterpolateNumber ( m_view1.params.nearClippingDepth, m_view2.params.nearClippingDepth, interpolateFraction );
		interpView.params.farClippingDepth       = InterpolateNumber ( m_view1.params.farClippingDepth, m_view2.params.farClippingDepth, interpolateFraction );
		interpView.params.fov_deg                = InterpolateNumber ( m_view1.params.fov_deg, m_view2.params.fov_deg, interpolateFraction );
		interpView.params.cameraAspectRatio      = InterpolateNumber ( m_view1.params.cameraAspectRatio, m_view2.params.cameraAspectRatio, interpolateFraction );
		interpView.params.viewMat                = ccGLMatrixd::Interpolate(interpolateFraction, m_view1.params.viewMat, m_view2.params.viewMat );
		interpView.params.setPivotPoint          (m_view1.params.getPivotPoint() + (m_view2.params.getPivotPoint() - m_view1.params.getPivotPoint()) * interpolateFraction, false );
		interpView.params.setCameraCenter        (m_view1.params.getCameraCenter() + (m_view2.params.getCameraCenter() - m_view1.params.getCameraCenter()) * interpolateFraction, true );
		interpView.params.setFocalDistance       ( InterpolateNumber(m_view1.params.getFocalDistance(), m_view2.params.getFocalDistance(), interpolateFraction) );

		// custom light
		if (m_view1.customLightEnabled && m_view2.customLightEnabled)
		{
			interpView.customLightEnabled = true;
			interpView.customLightPos.x = InterpolateNumber(m_view1.customLightPos.x, m_view2.customLightPos.x, interpolateFraction);
			interpView.customLightPos.y = InterpolateNumber(m_view1.customLightPos.y, m_view2.customLightPos.y, interpolateFraction);
			interpView.customLightPos.z = InterpolateNumber(m_view1.customLightPos.z, m_view2.customLightPos.z, interpolateFraction);
		}
		else
		{
			interpView.customLightEnabled = false;
		}
	}

    return true;
}

bool ViewInterpolate::nextView(ExtendedViewportParameters& outViewport)
{
	if (m_currentStep >= m_totalSteps)
	{
		return false;
	}

	//interpolation fraction
	double interpolateFraction = static_cast <double>(m_currentStep) / m_totalSteps;

	return interpolate(outViewport, interpolateFraction);
}
