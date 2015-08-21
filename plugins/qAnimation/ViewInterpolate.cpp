//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#         COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015				   #
//#                                                                        #
//##########################################################################

#include "ViewInterpolate.h"

//qCC_db
#include <ccGenericGLDisplay.h>

ViewInterpolate::ViewInterpolate()
    : m_view1 ( NULL )
    , m_view2 ( NULL )
    , m_totalSteps ( 0 )
    , m_currentStep ( 0 )
{
}

ViewInterpolate::ViewInterpolate(cc2DViewportObject * view1,  cc2DViewportObject * view2, unsigned int stepCount )
	: m_view1 ( view1 )
    , m_view2 ( view2 )
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
		||	m_view1 == NULL
		||	m_view2 == NULL )
    {
        return false;
    }

    //initial and final views
    const ccViewportParameters& view1 = m_view1->getParameters();
    const ccViewportParameters& view2 = m_view2->getParameters();
    ccViewportParameters interpView = m_view1->getParameters();

    //interpolation fraction
    double interpolate_fraction = static_cast <double>(m_currentStep) / m_totalSteps;

    interpView.pixelSize              = InterpolateNumber ( view1.pixelSize, view2.pixelSize, interpolate_fraction );
    interpView.zoom                   = InterpolateNumber ( view1.zoom, view2.zoom, interpolate_fraction );
    interpView.defaultPointSize       = InterpolateNumber ( view1.defaultPointSize, view2.defaultPointSize, interpolate_fraction );
    interpView.defaultLineWidth       = InterpolateNumber ( view1.defaultLineWidth, view2.defaultLineWidth, interpolate_fraction );
    interpView.zNearCoef              = InterpolateNumber ( view1.zNearCoef, view2.zNearCoef, interpolate_fraction );
    interpView.zNear                  = InterpolateNumber ( view1.zNear, view2.zNear, interpolate_fraction );
    interpView.zFar                   = InterpolateNumber ( view1.zFar, view2.zFar, interpolate_fraction );
    interpView.fov                    = InterpolateNumber ( view1.fov, view2.fov, interpolate_fraction );
    interpView.perspectiveAspectRatio = InterpolateNumber ( view1.perspectiveAspectRatio, view2.perspectiveAspectRatio, interpolate_fraction );
    interpView.orthoAspectRatio       = InterpolateNumber ( view1.orthoAspectRatio, view2.orthoAspectRatio, interpolate_fraction );
	interpView.viewMat                = ccGLMatrixd::Interpolate(interpolate_fraction, view1.viewMat, view2.viewMat);
	interpView.pivotPoint             = view1.pivotPoint + (view2.pivotPoint - view1.pivotPoint) * interpolate_fraction;
	interpView.cameraCenter           = view1.cameraCenter + (view2.cameraCenter - view1.cameraCenter) * interpolate_fraction;

    outViewport.setParameters( interpView );

    ++m_currentStep;

    return true;
}
