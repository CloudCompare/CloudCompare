//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_RENDERING_TOOLS_HEADER
#define CC_RENDERING_TOOLS_HEADER

//qCC_db
#include <ccDrawableObject.h>

class QWidget;
class ccGBLSensor;
class ccScalarField;
class ccGLWindow;

//! Misc. tools for rendering of advanced structures
class ccRenderingTools
{
public:

	//! Displays a depth buffer as an image
	static void ShowDepthBuffer(ccGBLSensor* lidar, QWidget* parent = nullptr, unsigned maxDim = 1024);

	//! Displays the colored scale corresponding to the currently activated context scalar field
	/** Its appearance depends on the scalar fields min and max displayed
		values, min and max saturation values, and also the selected
		color ramp.
		\param context OpenGL context description
	**/
	static void DrawColorRamp(const CC_DRAW_CONTEXT& context);

	//! See other version of DrawColorRamp
	static void DrawColorRamp(const CC_DRAW_CONTEXT& context, const ccScalarField* sf, ccGLWindow* win, int glW, int glH, float renderZoom = 1.0f);
};

#endif //CC_RENDERING_TOOLS_HEADER
