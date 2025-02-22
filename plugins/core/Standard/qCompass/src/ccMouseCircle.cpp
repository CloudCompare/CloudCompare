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

#include "ccMouseCircle.h"

//Qt
#include <QWheelEvent>

//System
#include <cmath>

//! Unit circle
struct Circle
{
	Circle()
	{
		//setup unit circle
		for (int n = 0; n < Resolution; n++)
		{
			double heading = n * (2 * M_PI / Resolution); //heading in radians
			vertices[n][0] = std::cos(heading);
			vertices[n][1] = std::sin(heading);
		}
	}
	
	static const int Resolution = 100;
	double vertices[Resolution][2];
};
static Circle s_unitCircle;

ccMouseCircle::ccMouseCircle(ccGLWindowInterface* owner, QString name) 
	: cc2DViewportObject(name.isEmpty() ? "label" : name)
	, m_owner(nullptr)
	, m_pixelSize(0.0f)
	, m_radius(50)
	, m_radiusStep(4)
{
	setVisible(true);
	setEnabled(false);

	//attach to owner
	assert(owner); //check valid pointer
	ccMouseCircle::m_owner = owner;
	m_owner->asQObject()->installEventFilter(this);
	m_owner->addToOwnDB(this, true);
}

ccMouseCircle::~ccMouseCircle()
{
	//cleanup event listner
	if (m_owner)
	{
		m_owner->asQObject()->removeEventFilter(this);
		m_owner->removeFromOwnDB(this);
	}
}

//get the circle radius in world coordinates
float ccMouseCircle::getRadiusWorld()
{
	float r = getRadiusPx() * m_pixelSize;
	ccLog::Print(QString("Radius_w = %1 (= %2 x %3)").arg(r).arg(getRadiusPx()).arg(m_pixelSize));
	return r;
}

//override draw function
void ccMouseCircle::draw(CC_DRAW_CONTEXT& context)
{
	if (!m_owner)
	{
		assert(false);
		return;
	}

	//only draw when visible
	if (!ccMouseCircle::isVisible())
	{
		return;
	}

	//only draw in 2D foreground mode
	if (!MACRO_Foreground(context) || !MACRO_Draw2D(context))
	{
		return;
	}

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);
	if (glFunc == nullptr)
	{
		return;
	}

	//test viewport parameters
	const ccViewportParameters& params = context.display->getViewportParameters();

	//ccLog::Print(QString("WidthAtFocalDist = %1 (= %2 x %3)").arg(params.computeWidthAtFocalDist()).arg(params.computeDistanceToWidthRatio()).arg(params.getFocalDistance()));
	m_pixelSize = (context.glW != 0 ? params.computeWidthAtFocalDist() / context.glW : 0);

	//get mouse position
	QPoint p = m_owner->asWidget()->mapFromGlobal(QCursor::pos()) * context.devicePixelRatio;
	int mx = p.x(); //mouse x-coord
	int my = context.glH - 1 - p.y(); //mouse y-coord in OpenGL coordinates (origin at bottom left, not top left)
	
	//calculate circle location
	int cx = mx - context.glW / 2;
	int cy = my - context.glH / 2;

	//draw circle
	{
		//thick dotted line
		{
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(2);
			glFunc->glLineStipple(1, 0xAAAA);
			glFunc->glEnable(GL_LINE_STIPPLE);
		}
		ccGL::Color(glFunc, ccColor::red);
		glFunc->glBegin(GL_LINE_LOOP);
		for (int n = 0; n < Circle::Resolution; n++)
		{
			glFunc->glVertex2d(s_unitCircle.vertices[n][0] * m_radius + cx, s_unitCircle.vertices[n][1] * m_radius + cy);
		}
		glFunc->glEnd();
		glFunc->glPopAttrib();
	}
}

//get mouse move events
bool ccMouseCircle::eventFilter(QObject* obj, QEvent* event)
{
	//only process events when visible
	if (!ccMouseCircle::isVisible())
		return false;

	if (event->type() == QEvent::MouseMove)
	{
		if (m_owner)
		{
			m_owner->redraw(true, false); //redraw 2D graphics
		}
	}

	if (event->type() == QEvent::Wheel)
	{
		QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);

		//is control down
		if (wheelEvent->modifiers().testFlag(Qt::ControlModifier))
		{
			//adjust radius (+ avoid really small radius)
			m_radius = std::max(m_radiusStep, m_radius - static_cast<int>(m_radiusStep * (wheelEvent->delta() / 100.0)));
			//repaint
			m_owner->redraw(true, false);
		}
	}
	return false; //pass event to other listeners
}
