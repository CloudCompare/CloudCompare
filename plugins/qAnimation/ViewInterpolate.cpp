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
    , m_total_steps ( 0 )
    , m_current_step (0)
{
}

ViewInterpolate::ViewInterpolate(cc2DViewportObject * a_first_view,  cc2DViewportObject * a_second_view, unsigned int a_max_steps )
	: m_view1 ( a_first_view )
    , m_view2 ( a_second_view )
    , m_total_steps ( a_max_steps )
    , m_current_step (0)
{
}

ViewInterpolate::~ViewInterpolate()
{
}

bool ViewInterpolate::nextView ( cc2DViewportObject& a_returned_viewport )
{
    if (	m_current_step >= m_total_steps
		||	m_view1 == NULL
		||	m_view2 == NULL )
    {
        return false;
    }

    //initial and final views
    const ccViewportParameters& view1 = m_view1->getParameters();
    const ccViewportParameters& view2 = m_view2->getParameters();
    ccViewportParameters working_view = m_view1->getParameters();

    //interpolation fraction
    double interpolate_fraction = static_cast <double> ( m_current_step ) / m_total_steps;

    working_view.pixelSize              = InterpolateNumber ( view1.pixelSize, view2.pixelSize, interpolate_fraction );
    working_view.zoom                   = InterpolateNumber ( view1.zoom, view2.zoom, interpolate_fraction );
    working_view.defaultPointSize       = InterpolateNumber ( view1.defaultPointSize, view2.defaultPointSize, interpolate_fraction );
    working_view.defaultLineWidth       = InterpolateNumber ( view1.defaultLineWidth, view2.defaultLineWidth, interpolate_fraction );
    working_view.zNearCoef              = InterpolateNumber ( view1.zNearCoef, view2.zNearCoef, interpolate_fraction );
    working_view.zNear                  = InterpolateNumber ( view1.zNear, view2.zNear, interpolate_fraction );
    working_view.zFar                   = InterpolateNumber ( view1.zFar, view2.zFar, interpolate_fraction );
    working_view.fov                    = InterpolateNumber ( view1.fov, view2.fov, interpolate_fraction );
    working_view.perspectiveAspectRatio = InterpolateNumber ( view1.perspectiveAspectRatio, view2.perspectiveAspectRatio, interpolate_fraction );
    working_view.orthoAspectRatio       = InterpolateNumber ( view1.orthoAspectRatio, view2.orthoAspectRatio, interpolate_fraction );
	working_view.viewMat                = ccGLMatrixd::Interpolate(interpolate_fraction, view1.viewMat, view2.viewMat);
	working_view.pivotPoint             = view1.pivotPoint + (view2.pivotPoint - view1.pivotPoint) * interpolate_fraction;
	working_view.cameraCenter           = view1.cameraCenter + (view2.cameraCenter - view1.cameraCenter) * interpolate_fraction;

    a_returned_viewport.setParameters( working_view );

    ++m_current_step;

    return true;
}
