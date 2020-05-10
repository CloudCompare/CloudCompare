//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#ifndef CC_MEASUREMENT_HEADER
#define CC_MEASUREMENT_HEADER

#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#include <ccGLWindow.h>

/*
A template class for all "measurements" made with ccCompass. Contains basic stuff like highligts, draw colours etc.
*/
class ccMeasurement
{
public:

	ccMeasurement() {}

	virtual ~ccMeasurement() {}

	//drawing stuff
	void setDefaultColor  (const ccColor::Rgb& col) { m_normal_colour    = col; }
	void setHighlightColor(const ccColor::Rgb& col) { m_highlight_colour = col; }
	void setActiveColor   (const ccColor::Rgb& col) { m_active_colour    = col; }
	void setAlternateColor(const ccColor::Rgb& col) { m_alternate_colour = col; }

	//returns the colour of this measurment object given the active/highlighted state
	ccColor::Rgb getMeasurementColour() const
	{
		if (m_isActive)
		{
			return m_active_colour;
		}
		else if (m_isAlternate)
		{
			return m_alternate_colour;
		}
		else if (m_isHighlighted)
		{
			return m_highlight_colour;
		}
		return m_normal_colour;
	}

	//set draw state of this measurment
	void setActive   (bool isActive) { m_isActive      = isActive; }
	void setHighlight(bool isActive) { m_isHighlighted = isActive; }
	void setAlternate(bool isActive) { m_isAlternate   = isActive;  }
	void setNormal() { m_isActive = false; m_isHighlighted = false; m_isAlternate = false; }

protected:
	//drawing variables
	bool m_isActive = false;
	bool m_isHighlighted = false;
	bool m_isAlternate = false;
	ccColor::Rgb m_active_colour = ccColor::yellow;
	ccColor::Rgb m_highlight_colour = ccColor::green;
	ccColor::Rgb m_alternate_colour = ccColor::cyan;
	ccColor::Rgb m_normal_colour = ccColor::blue;
};

#endif