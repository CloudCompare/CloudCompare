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


ViewInterpolate::ViewInterpolate()
    : m_view1 (nullptr)
    , m_view2 (nullptr)
    , m_totalSteps ( 0 )
    , m_currentStep ( 0 )
{
}

ViewInterpolate::ViewInterpolate(cc2DViewportObject * viewParams1,  cc2DViewportObject * viewParams2, unsigned int stepCount )
	: m_view1 ( viewParams1 )
    , m_view2 ( viewParams2 )
    , m_totalSteps ( stepCount )
    , m_currentStep ( 0 )
{
}

//helper function for interpolating between simple numerical types
template <class T> T InterpolateNumber( T start, T end, double interpolationFraction )
{
    return static_cast < T > ( static_cast<double>(start) + (static_cast<double>(end) - static_cast<double>(start)) * interpolationFraction );
}

bool ViewInterpolate::nextView ( cc2DViewportObject& outViewport )
{
    if (	m_currentStep >= m_totalSteps
		||	m_view1 == nullptr
		||	m_view2 == nullptr)
    {
        return false;
    }

    //initial and final views
    const ccViewportParameters& viewParams1 = m_view1->getParameters();
    const ccViewportParameters& viewParams2 = m_view2->getParameters();
    ccViewportParameters interpView = m_view1->getParameters();

    //interpolation fraction
    double interpolate_fraction = static_cast <double>(m_currentStep) / m_totalSteps;

    interpView.pixelSize              = InterpolateNumber ( viewParams1.pixelSize, viewParams2.pixelSize, interpolate_fraction );
    interpView.zoom                   = InterpolateNumber ( viewParams1.zoom, viewParams2.zoom, interpolate_fraction );
    interpView.defaultPointSize       = InterpolateNumber ( viewParams1.defaultPointSize, viewParams2.defaultPointSize, interpolate_fraction );
    interpView.defaultLineWidth       = InterpolateNumber ( viewParams1.defaultLineWidth, viewParams2.defaultLineWidth, interpolate_fraction );
    interpView.zNearCoef              = InterpolateNumber ( viewParams1.zNearCoef, viewParams2.zNearCoef, interpolate_fraction );
    interpView.zNear                  = InterpolateNumber ( viewParams1.zNear, viewParams2.zNear, interpolate_fraction );
    interpView.zFar                   = InterpolateNumber ( viewParams1.zFar, viewParams2.zFar, interpolate_fraction );
    interpView.fov                    = InterpolateNumber ( viewParams1.fov, viewParams2.fov, interpolate_fraction );
    interpView.perspectiveAspectRatio = InterpolateNumber ( viewParams1.perspectiveAspectRatio, viewParams2.perspectiveAspectRatio, interpolate_fraction );
    interpView.orthoAspectRatio       = InterpolateNumber ( viewParams1.orthoAspectRatio, viewParams2.orthoAspectRatio, interpolate_fraction );
	interpView.viewMat                = ccGLMatrixd::Interpolate(interpolate_fraction, viewParams1.viewMat, viewParams2.viewMat);
	interpView.pivotPoint             = viewParams1.pivotPoint + (viewParams2.pivotPoint - viewParams1.pivotPoint) * interpolate_fraction;
	interpView.cameraCenter           = viewParams1.cameraCenter + (viewParams2.cameraCenter - viewParams1.cameraCenter) * interpolate_fraction;

    outViewport.setParameters( interpView );

    ++m_currentStep;

    return true;
}
