#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

// Local
#include "qCC_glWindow.h"

// qCC_db
#include <ccColorTypes.h>

class ccGLWindowInterface;

//! X-ray preferences are separate from the shared ccGui::ParamStruct layout.
struct CCGLWINDOW_LIB_API ccXRaySettings
{
	// A window override is owned by its existing Qt signal emitter.
	static ccXRaySettings Parameters(const ccGLWindowInterface* window = nullptr);
	static void           Set(const ccXRaySettings& settings, ccGLWindowInterface* window = nullptr);
	static void           ClearOverride(ccGLWindowInterface& window);

	bool operator==(const ccXRaySettings& other) const;
	void fromPersistentSettings();
	void toPersistentSettings() const;

	//! Background colors for the X-ray effect, with optional gradient.
	struct Background
	{
		//! Equality operator for Background struct
		bool operator==(const Background& other) const
		{
			return color == other.color && secondColor == other.secondColor && gradient == other.gradient;
		}

		ccColor::Rgb color;            //!< Primary background color
		ccColor::Rgb secondColor;      //!< Secondary background color for gradient
		bool         gradient = false; //!< Whether to use a gradient between the primary and secondary colors
	};

	Background background{ccColor::black, ccColor::Rgb(32, 32, 32), false};
	Background reverseBackground{ccColor::white, ccColor::Rgb(224, 224, 224), false};

	double exposure = -3.0;  //!< Exposure value for the X-ray effect
	bool   reverse  = false; //!< Whether to reverse the X-ray effect (light on dark vs. dark on light)
};
