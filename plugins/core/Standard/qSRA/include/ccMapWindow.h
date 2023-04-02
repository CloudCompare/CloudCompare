#pragma once

//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

//qCC
#include <ccGLWindow.h>

//qCC_db
#include <ccScalarField.h>

//! 2D map display window
class ccMapWindow : public ccGLWindow
{
public:

	//! Default constructor
	explicit ccMapWindow(QOpenGLWidget* parent = nullptr)
		: ccGLWindow(nullptr, parent, true)
		, m_sfForRampDisplay(nullptr)
		, m_showSF(true)
	{}

	//! Destructor
	virtual ~ccMapWindow()
	{
		setAssociatedScalarField(nullptr);
	}

	//! Sets associated scalar-field
	/** This scalar field will be used for color ramp display.
	**/
	void setAssociatedScalarField(ccScalarField* sf)
	{
		if (m_sfForRampDisplay != sf)
		{
			if (m_sfForRampDisplay)
				m_sfForRampDisplay->release();
			
			m_sfForRampDisplay = sf;
			
			if (m_sfForRampDisplay)
				m_sfForRampDisplay->link();
		}
	}

	//! Whether to show associated SF or not
	void showSF(bool state) { m_showSF = state; }

	//! Returns whether associated SF should be shown or not
	bool sfShown() const { return m_showSF; }

	//! Returns associated scalar field
	ccScalarField* getAssociatedScalarField() const { return m_sfForRampDisplay; }

	//inherited from ccGLWindow
	void getContext(CC_DRAW_CONTEXT& context) override
	{
		ccGLWindow::getContext(context);

		if (m_showSF)
		{
			//override sf that will be used for color ramp display
			context.sfColorScaleToDisplay = m_sfForRampDisplay;
		}
	}

protected:

	//! Associated scalar field
	ccScalarField* m_sfForRampDisplay;

	//! Whether to show or not the associated SF
	bool m_showSF;
};
