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

#ifndef CC_MOUSE_CIRCLE_HEADER
#define CC_MOUSE_CIRCLE_HEADER

/**
This is a custom 2DViewportLabel which takes up the entire viewport but is entirely transparent,
except for a circle with radius r around the mouse. 
*/
#include "ccStdPluginInterface.h"
#include <ccGLWindow.h>
#include <cc2DViewportObject.h>
#include <qevent.h>
#include <QPoint>
#include <QObject>

class ccMouseCircle : public cc2DViewportObject, public QObject
{
public:
	//constructor
	explicit ccMouseCircle(ccGLWindow* owner, QString name = QString("MouseCircle"));

	//deconstructor
	~ccMouseCircle();

	//get the circle radius in px
	inline int getRadiusPx() const { return m_radius; }

	//get the circle radius in world coordinates
	float getRadiusWorld();

	//removes the link with the owner (no cleanup)
	inline void ownerIsDead() { m_owner = nullptr; }

protected:
	//draws a circle of radius r around the mouse
	void draw(CC_DRAW_CONTEXT& context);

private:
	//ccGLWindow this overlay is attached to -> used to get mouse position & events
	ccGLWindow* m_owner;
	float m_pixelSize;

	//event to get mouse-move updates & trigger repaint
	bool eventFilter(QObject* obj, QEvent* event) override;

	int m_radius;
	int m_radiusStep;
};

#endif
