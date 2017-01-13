#include "ccMouseCircle.h"

#include <math.h>

ccMouseCircle::ccMouseCircle(ccGLWindow* owner, QString name) 
	: cc2DViewportObject(name.isEmpty() ? "label" : name)
{
	setVisible(true);
	setEnabled(false);

	//setup unit circle
	for (int n = 0; n < ccMouseCircle::RESOLUTION; n++)
	{
		float heading = n * (2 * M_PI / (float) ccMouseCircle::RESOLUTION); //heading in radians
		ccMouseCircle::UNIT_CIRCLE[n][0] = cos(heading);
		ccMouseCircle::UNIT_CIRCLE[n][1] = sin(heading);
	}

	//attach to owner
	assert(owner); //check valid pointer
	ccMouseCircle::m_owner = owner;
	m_owner->installEventFilter(this);
}

ccMouseCircle::~ccMouseCircle()
{
	//cleanup event listner
	if (m_owner)
	{
		m_owner->removeEventFilter(this);
	}
}

//get the circle radius in px
int ccMouseCircle::getRadiusPx()
{
	return ccMouseCircle::RADIUS;
}

//get the circle radius in world coordinates
float ccMouseCircle::getRadiusWorld()
{
	return getRadiusPx() / m_winTotalZoom;
}

//override draw function
void ccMouseCircle::draw(CC_DRAW_CONTEXT& context)
{
	//only draw when visible
	if (!ccMouseCircle::isVisible())
		return;

	//only draw in 2D foreground mode
	if (!MACRO_Foreground(context) || !MACRO_Draw2D(context))
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	//test viewport parameters
	const ccViewportParameters& params = context.display->getViewportParameters();
	glFunc->glPushAttrib(GL_LINE_BIT);

	float relativeZoom = 1.0f;
	float dx = 0.0f;
	float dy = 0.0f;
	if (!m_params.perspectiveView) //ortho mode
	{
		//Screen pan & pivot compensation
		float totalZoom = m_params.zoom / m_params.pixelSize;
		m_winTotalZoom = params.zoom / params.pixelSize;
		relativeZoom = m_winTotalZoom / totalZoom;

		CCVector3d dC = m_params.cameraCenter - params.cameraCenter;
		
		CCVector3d P = m_params.pivotPoint - params.pivotPoint;
		m_params.viewMat.apply(P);

		static_cast<float>(dC.x + P.x);
		static_cast<float>(dC.y + P.y);

		dx *= m_winTotalZoom;
		dy *= m_winTotalZoom;
	}

	//thick dotted line
	glFunc->glLineWidth(2);
	glFunc->glLineStipple(1, 0xAAAA);
	glFunc->glEnable(GL_LINE_STIPPLE);

	const unsigned char* defaultColor = m_selected ? ccColor::red.rgba : context.textDefaultCol.rgb;
	glFunc->glColor3ubv(ccColor::red.rgba);

	//get height & width
	int halfW = static_cast<int>(context.glW / 2.0f);
	int halfH = static_cast<int>(context.glH / 2.0f);
	
	//get mouse position
	QPoint p = m_owner->asWidget()->mapFromGlobal(QCursor::pos());
	int mx = p.x(); //mouse x-coord
	int my = 2*halfH - p.y(); //mouse y-coord in OpenGL coordinates (origin at bottom left, not top left)
	
	//calculate circle location
	int cx = dx+mx-halfW;
	int cy = dy+my-halfH;

	//draw circle
	glFunc->glBegin(GL_LINE_LOOP);
	for (int n = 0; n < ccMouseCircle::RESOLUTION; n++)
	{
		glFunc->glVertex2f(ccMouseCircle::UNIT_CIRCLE[n][0] * ccMouseCircle::RADIUS + cx, ccMouseCircle::UNIT_CIRCLE[n][1] * ccMouseCircle::RADIUS + cy);
	}
	glFunc->glEnd();
	glFunc->glPopAttrib();
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
		QWheelEvent* wheelEvent = static_cast<QWheelEvent *>(event);

		//is control down
		if (wheelEvent->modifiers().testFlag(Qt::ControlModifier))
		{
			//adjust radius
			ccMouseCircle::RADIUS -= ccMouseCircle::RADIUS_STEP*(wheelEvent->delta()/100.0);

			//avoid really small radius
			if (ccMouseCircle::RADIUS < ccMouseCircle::RADIUS_STEP)
			{
				ccMouseCircle::RADIUS = ccMouseCircle::RADIUS_STEP;
			}
			//repaint
			m_owner->redraw(true, false);
		}
	}
	return false; //pass event to other listeners
}