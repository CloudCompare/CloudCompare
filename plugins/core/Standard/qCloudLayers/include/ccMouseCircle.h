#pragma once

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

#include <ccGLWindowInterface.h>
#include <cc2DViewportObject.h>

class QEvent;

//Qt
#include <QObject>

//! This is a custom 2DViewportLabel which takes up the entire viewport but is entirely transparent,
//! except for a circle with radius r around the mouse.
class ccMouseCircle : public cc2DViewportObject, public QObject
{
public:
	//! Constructor
	explicit ccMouseCircle(ccGLWindowInterface* owner, QString name = QString("MouseCircle"));

	//! Destructor
	~ccMouseCircle() override;

	//! Returns the circle radius in px
	inline int getRadiusPx() const { return m_radius; }

	//! Sets whether scroll is allowed or not
	inline void setAllowScroll(bool state) { m_allowScroll = state; }
	
protected:
	//! Draws a circle around the mouse cursor
	void draw(CC_DRAW_CONTEXT& context) override;

private:
	//! Event filter to get mouse move and repaint eventss
	bool eventFilter(QObject* obj, QEvent* event) override;

private:
	//! The ccGLWindowInterface instance this overlay object is attached to
	ccGLWindowInterface* m_owner;

	//! Pixel size
	float m_pixelSize;
	//! Circle radius
	int m_radius;
	//! Increments of circle radius (when changed with the mouse wheel)
	int m_radiusStep;
	//! Whether to allow 'scrolling' (i.e. changing the circle radius)
	bool m_allowScroll;
};

