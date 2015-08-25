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

#ifndef VIDEO_STEP_ITEM_HEADER
#define VIDEO_STEP_ITEM_HEADER

#include "ViewInterpolate.h"

class VideoStepItem
{
public:

	VideoStepItem()
		: duration_sec(1.0)
	{}

	VideoStepItem(cc2DViewportObject* view1, cc2DViewportObject* view2)
		: interpolator(view1, view2)
		, duration_sec(1.0)
	{}

    ViewInterpolate interpolator;
    double duration_sec;
};

#endif // VIDEO_STEP_ITEM_HEADER
