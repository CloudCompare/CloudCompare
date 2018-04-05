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
    ViewInterpolate( cc2DViewportObject * view1,  cc2DViewportObject * view2, unsigned int stepCount = 0 );

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

    cc2DViewportObject* m_view1;

    cc2DViewportObject* m_view2;

    unsigned int m_totalSteps;

    unsigned int m_currentStep;
};

#endif // VIEWINTERPOLATE_H
