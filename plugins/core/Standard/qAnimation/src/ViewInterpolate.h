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


#ifndef VIEWINTERPOLATE_H
#define VIEWINTERPOLATE_H

//qCC_db
#include <cc2DViewportObject.h>

class ccViewportParameters;

//! The ViewInterpolate class
/** This class takes pointers to two viewport objects, and returns intermediate viewports between over a set number of steps.
**/
class ViewInterpolate
{
public:

	//! Default constructor
    ViewInterpolate( );

	//! Constructor from two viewports and a number of steps
    ViewInterpolate(cc2DViewportObject * viewPort0, cc2DViewportObject * viewPort1,  cc2DViewportObject * viewPort2,  cc2DViewportObject * viewPort3, double dt0, double dt1, double dt2, double dt3, bool doCubic, unsigned int stepCount = 0);

    //! Sets the first viewport object
	inline void setView1 ( cc2DViewportObject * view ) { m_view1 = view; }
    //! Returns the first viewport object
	inline cc2DViewportObject * view1 () const { return m_view1; }

    // Sets the second viewport object
	inline void setView2 ( cc2DViewportObject * view ) {  m_view2 = view; }
    // Returns the second viewport object
	inline const cc2DViewportObject * view2 () const { return m_view2; }

    //! Returns the next viewport
    bool nextView ( cc2DViewportObject& a_returned_viewport );

    //! Returns the current step
	inline unsigned int currentStep () { return m_currentStep; }
    //! Sets the current step
	inline void setCurrentStep ( unsigned int step ) { m_currentStep = step; }

    //! Returns the max number of steps
	inline unsigned int maxStep() { return m_totalSteps; }
    //! Sets the max number of steps
	inline void setMaxStep ( unsigned int stepCount ) { m_totalSteps = stepCount; }

	//! Resets the interpolator
	inline void reset() { m_currentStep = 0; }

private:

    cc2DViewportObject* m_view0;

    cc2DViewportObject* m_view1;

    cc2DViewportObject* m_view2;

    cc2DViewportObject* m_view3;

    bool m_doCubic;
    double m_ts[4];
    double m_cam_cntr_x[4];
    double m_cam_cntr_y[4];
    double m_cam_cntr_z[4];
    double m_pvt_pt_x[4];
    double m_pvt_pt_y[4];
    double m_pvt_pt_z[4];
    double m_pixelSize[4];
    double m_zoom[4];
    double m_defaultPointSize[4];
    double m_defaultLineWidth[4];
    double m_zNearCoef[4];
    double m_zNear[4];
    double m_zFar[4];
    double m_fov[4];
    double m_perspectiveAspectRatio[4];
    double m_orthoAspectRatio[4];
    double m_vm_alpha[4];
    double m_vm_axis_x[4];
    double m_vm_axis_y[4];
    double m_vm_axis_z[4];
    double m_vm_translation_x[4];
    double m_vm_translation_y[4];
    double m_vm_translation_z[4];

    unsigned int m_totalSteps;

    unsigned int m_currentStep;
};

#endif // VIEWINTERPOLATE_H
