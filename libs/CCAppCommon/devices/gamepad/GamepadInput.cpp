#include "GamepadInput.h"

//Qt
#include <QGamepadManager>

//qCC_gl
#include <ccGLWindowInterface.h>

//system
#include <assert.h>

GamepadInput::GamepadInput(QObject* parent/*=nullptr*/)
	: QGamepad(0, parent)
	, m_hasPanning(false)
	, m_hasTranslation(false)
	, m_hasRotation(false)
	, m_zoom(0)
{
	connect(&m_timer, &QTimer::timeout, this, &GamepadInput::updateInternalState);
}

GamepadInput::~GamepadInput()
{
}

void GamepadInput::start()
{
	m_timer.start(0);
}

void GamepadInput::stop()
{
	m_timer.stop();
}

void GamepadInput::update(ccGLWindowInterface* win)
{
	if (!win)
	{
		assert(false);
		return;
	}

	//rotation
	if (m_hasRotation)
	{
		win->rotateBaseViewMat(m_rotation);
	}

	const ccViewportParameters& viewParams = win->getViewportParameters();

	//panning
	if (m_hasPanning)
	{
		double screenWidth3D = viewParams.computeWidthAtFocalDist();
		if (!viewParams.objectCenteredView)
		{
			screenWidth3D = -screenWidth3D;
		}
		CCVector3d v(-m_panning.x*screenWidth3D, -m_panning.y*screenWidth3D, 0);
		win->moveCamera(v);
	}

	//zoom
	if (m_hasTranslation)
	{
		double X = m_translation.x;
		//double Y = m_translation.y; //always 0
		double Z = m_translation.z;

		if (	CCCoreLib::GreaterThanEpsilon(std::abs(X))
			||	CCCoreLib::GreaterThanEpsilon(std::abs(Z)))
		{
			if (viewParams.perspectiveView)
			{
				X *= win->getViewportParameters().computeDistanceToHalfWidthRatio();
			}
			double screenWidth3D = viewParams.computeWidthAtFocalDist();
			if (!viewParams.objectCenteredView)
			{
				screenWidth3D = -screenWidth3D;
			}
			CCVector3d v(-X * screenWidth3D, 0, -Z * screenWidth3D);
			win->moveCamera(v);
		}
	}

	win->redraw();
}

static double s_gamepadSpeed = 0.005;
static double s_gamepadRotSpeed = 0.02;

void GamepadInput::updateInternalState()
{
	//reset
	m_panning = CCVector3(0, 0, 0);
	m_hasPanning = false;
	m_translation = CCVector3(0, 0, 0);
	m_hasTranslation = false;
	m_rotation.toIdentity();
	m_hasRotation = false;
	m_zoom = 0;

	//rotation
	{
		CCVector3d u(axisRightX() * s_gamepadRotSpeed, -axisRightY() * s_gamepadRotSpeed, 1);
		u.normalize();

		if (u.z != 1.0)
		{
			m_rotation = ccGLMatrixd::FromToRotation(CCVector3d(0, 0, 1), u);
			m_hasRotation = true;
		}

		if (buttonL2() || buttonR2())
		{
			double angle_deg = buttonL2() ? 20 * s_gamepadRotSpeed : -20 * s_gamepadRotSpeed;
			ccGLMatrixd rot;
			rot.initFromParameters( CCCoreLib::DegreesToRadians( angle_deg ), CCVector3d(0, 0, 1), CCVector3d(0, 0, 0) );
			m_rotation = rot * m_rotation;
			m_hasRotation = true;
		}
	}

	//panning
	{
		if (buttonLeft())
		{
			m_panning.x = -s_gamepadSpeed;
			m_hasPanning = true;
		}
		else if (buttonRight())
		{
			m_panning.x = s_gamepadSpeed;
			m_hasPanning = true;
		}
		if (buttonUp())
		{
			m_panning.y = s_gamepadSpeed;
			m_hasPanning = true;
		}
		else if (buttonDown())
		{
			m_panning.y = -s_gamepadSpeed;
			m_hasPanning = true;
		}
	}

	//translation
	{
		double x = axisLeftX();
		double z = axisLeftY();
		if (x != 0 || z != 0)
		{
			m_translation = CCVector3(x * s_gamepadRotSpeed, 0, z * s_gamepadRotSpeed);
			m_hasTranslation = true;
		}
	}

	//zoom
	{
		m_zoom = -axisLeftY() * s_gamepadSpeed;
	}

	if (m_hasRotation || m_hasPanning || m_hasTranslation || m_zoom != 0)
	{
		Q_EMIT updated();
	}
}
