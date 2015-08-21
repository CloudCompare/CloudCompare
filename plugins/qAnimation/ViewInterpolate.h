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


#ifndef VIEWINTERPOLATE_H
#define VIEWINTERPOLATE_H

//qCC_db
#include <cc2DViewportObject.h>

class ccViewportParameters;

/**
 * @brief The ViewInterpolate class
 * This class takes pointers to two viewport objects, and returns intermediate viewports between over a set number of steps.
 */
class ViewInterpolate
{
public:

	//! Default constructor
    ViewInterpolate( );

	//! Constructor from two viewports and a number of steps
    ViewInterpolate( cc2DViewportObject * a_first_view,  cc2DViewportObject * a_second_view, unsigned int a_max_steps = 0 );

	//! Destructor
    virtual ~ViewInterpolate();

    //! Sets the first viewport object
	inline void setView1 ( cc2DViewportObject * a_first_view ) { m_view1 = a_first_view; }
    //! Returns the first viewport object
	inline cc2DViewportObject * view1 () {  return m_view1; }

    // Sets the second viewport object
	inline void setView2 ( cc2DViewportObject * a_second_view ) {  m_view2 = a_second_view; }
    // Returns the second viewport object
	inline cc2DViewportObject * view2 () { return m_view2; }

    //! Returns the next viewport
    bool nextView ( cc2DViewportObject& a_returned_viewport );

    //! Returns the current step
	inline unsigned int currentStep () { return m_current_step; }
    //! Sets the current step
	inline void setCurrentStep ( unsigned int a_current_step ) { m_current_step = a_current_step; }

    //! Returns the max number of steps
	inline unsigned int maxStep() { return m_total_steps; }
    //! Sets the max number of steps
	inline void setMaxStep ( unsigned int a_max_step ) { m_total_steps = a_max_step; }

private:

    cc2DViewportObject * m_view1;

    cc2DViewportObject * m_view2;

    unsigned int m_total_steps;

    unsigned int m_current_step;
};

//helper function for interpolating between simple numerical types
template <class T>
T InterpolateNumber ( T start_num, T end_num, double interpolation_fraction )
{
    double start_double ( static_cast <double> ( start_num ) );
    double end_double ( static_cast <double> ( end_num ) );

    return static_cast < T > ( start_double + (end_double - start_double) * interpolation_fraction );
}

#endif // VIEWINTERPOLATE_H
