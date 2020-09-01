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

ViewInterpolate::ViewInterpolate(const ccViewportParameters& viewParams1, const ccViewportParameters& m_view2, unsigned int stepCount)
	: m_view1(viewParams1)
	, m_view2(m_view2)
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
											PointCoordinateType length )
{
	smoothTrajectory = _smoothTrajectory;
	smoothTrajectoryReversed = _smoothTrajectoryReversed;
	smoothTrajCurrentIndex = smoothTrajStartIndex = i1;
	smoothTrajStopIndex = i2;
	smoothSegmentLength = length;
	smoothCurrentLength = 0;
}

//helper function for interpolating between simple numerical types
template <class T> T InterpolateNumber( T start, T end, double interpolationFraction )
{
    return static_cast < T > ( static_cast<double>(start) + (static_cast<double>(end) - static_cast<double>(start)) * interpolationFraction );
}

bool ViewInterpolate::interpolate(ccViewportParameters& interpView, double interpolate_fraction) const
{
	if (	interpolate_fraction < 0.0
		||	interpolate_fraction > 1.0)
	{
		return false;
	}

    interpView = m_view1;
	{
		interpView.defaultPointSize       = InterpolateNumber ( m_view1.defaultPointSize, m_view2.defaultPointSize, interpolate_fraction );
		interpView.defaultLineWidth       = InterpolateNumber ( m_view1.defaultLineWidth, m_view2.defaultLineWidth, interpolate_fraction );
		interpView.zNearCoef              = InterpolateNumber ( m_view1.zNearCoef, m_view2.zNearCoef, interpolate_fraction );
		interpView.zNear                  = InterpolateNumber ( m_view1.zNear, m_view2.zNear, interpolate_fraction );
		interpView.zFar                   = InterpolateNumber ( m_view1.zFar, m_view2.zFar, interpolate_fraction );
		interpView.fov_deg                = InterpolateNumber ( m_view1.fov_deg, m_view2.fov_deg, interpolate_fraction );
		interpView.cameraAspectRatio      = InterpolateNumber ( m_view1.cameraAspectRatio, m_view2.cameraAspectRatio, interpolate_fraction );
		interpView.viewMat                = ccGLMatrixd::Interpolate(interpolate_fraction, m_view1.viewMat, m_view2.viewMat );
		interpView.setPivotPoint          ( m_view1.getPivotPoint() + (m_view2.getPivotPoint() - m_view1.getPivotPoint()) * interpolate_fraction, false );
		interpView.setCameraCenter        ( m_view1.getCameraCenter() + (m_view2.getCameraCenter() - m_view1.getCameraCenter()) * interpolate_fraction, true );
		interpView.setFocalDistance       ( InterpolateNumber(m_view1.getFocalDistance(), m_view2.getFocalDistance(), interpolate_fraction) );
	}

    return true;
}

bool ViewInterpolate::nextView(ccViewportParameters& outViewport)
{
	if (m_currentStep >= m_totalSteps)
	{
		return false;
	}

	//interpolation fraction
	double interpolate_fraction = static_cast <double>(m_currentStep) / m_totalSteps;

	return interpolate(outViewport, interpolate_fraction);
}
