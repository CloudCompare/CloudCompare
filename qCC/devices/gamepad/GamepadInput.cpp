#include "GamepadInput.h"

//Qt
#include <QGamepadManager>

//qCC_gl
#include <ccGLWindow.h>

//system
#include <assert.h>

GamepadInput::GamepadInput(QObject* parent/*=0*/)
	: QGamepad(0, parent)
	, m_hasPanning(false)
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

void GamepadInput::update(ccGLWindow* win)
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

	if (viewParams.perspectiveView)
	{
		//translation
		if (m_hasPanning)
		{
			float scale = static_cast<float>(std::min(win->glWidth(), win->glHeight()) * viewParams.pixelSize);
			scale /= win->computePerspectiveZoom();

			float tanFOV = tan(static_cast<float>(viewParams.fov * CC_DEG_TO_RAD)/*/2*/);
			m_panning.x *= tanFOV;
			m_panning.y *= tanFOV;

			if (!viewParams.objectCenteredView)
			{
				scale = -scale;
			}
			win->moveCamera(-m_panning.x*scale, -m_panning.y*scale, 0);
		}
		else if (m_hasTranslation)
		{
			float scale = static_cast<float>(std::min(win->glWidth(), win->glHeight()) * viewParams.pixelSize);
			scale /= win->computePerspectiveZoom();

			float tanFOV = tan(static_cast<float>(viewParams.fov * CC_DEG_TO_RAD)/*/2*/);
			m_translation.x *= tanFOV;
			m_translation.y *= tanFOV;
			
			if (!viewParams.objectCenteredView)
			{
				scale = -scale;
			}
			win->moveCamera(-m_translation.x*scale, m_translation.y*scale, -m_translation.z*scale);
		}
	}
	else
	{
		//panning
		if (m_hasPanning)
		{
			float scale = static_cast<float>(std::min(win->glWidth(), win->glHeight()) * viewParams.pixelSize);
			scale /= win->getViewportParameters().zoom;
			win->moveCamera(-m_panning.x*scale, -m_panning.y*scale, 0);
		}

		//zoom
		if (m_hasTranslation)
		{
			if (m_translation.x != 0 && !m_hasPanning)
			{
				float scale = static_cast<float>(std::min(win->glWidth(), win->glHeight()) * viewParams.pixelSize);
				scale /= win->getViewportParameters().zoom;
				win->moveCamera(-m_translation.x*scale, 0, 0);
			}
			if (m_translation.z != 0)
			{
				win->updateZoom(1.0f + m_translation.z);
			}
		}
	}

	win->redraw();
}

static double s_gamepadSpeed = 0.005;
static double s_gamepadRotSpeed = 0.01;

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
			double angle = buttonL2() ? 10 * s_gamepadRotSpeed : -10 * s_gamepadRotSpeed;
			ccGLMatrixd rot;
			rot.initFromParameters(angle * CC_DEG_TO_RAD, CCVector3d(0, 0, 1), CCVector3d(0, 0, 0));
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
		emit updated();
	}
}
