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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1874                                                              $
//$LastChangedDate:: 2011-08-28 23:39:04 +0200 (dim., 28 août 2011)       $
//**************************************************************************
//

#ifndef CC_RENDERING_TOOLS_HEADER
#define CC_RENDERING_TOOLS_HEADER

//qCC_db
#include <ccDrawableObject.h>

class ccGBLSensor;
class QWidget;

class ccRenderingTools
{
public:

	static void ShowDepthBuffer(ccGBLSensor* lidar, QWidget* parent=0);

    //! Displays the colored scale corresponding to the currently activated context scalar field
	/** Its appearance depends on the scalar fields min and max displayed
        values, min and max saturation values, and also the selected
		color ramp.
		\param context OpenGL context description
    **/
	static void DrawColorRamp(const CC_DRAW_CONTEXT& context);

protected:

    //! Graphical scale atomical element
    struct ScaleElement
    {
        //! Starting value
        DistanceType value;
        //! Specifies whether the value should be displayed
        bool textDisplayed;
		//! Specifies whether the cube is condensed or not
		bool condensed;

		//! Default constructor
        ScaleElement(DistanceType val, bool dispText = true, bool isCondensed = false)
			: value(val)
			, textDisplayed(dispText)
			, condensed(isCondensed)
        {
        }
    };
};

#endif
