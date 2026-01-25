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
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// qCC
#include "ccGLWindowStereo.h"

// qCC_db
#include <ccHObject.h>

// CCFbo
#include <ccFrameBufferObject.h>

// Qt
#include <QMessageBox>
#include <QOpenGLPaintDevice>
#include <QResizeEvent>

ccGLWindowStereo::ccGLWindowStereo(QSurfaceFormat* format /*=nullptr*/,
                                   QWindow*        parent /*=nullptr*/,
                                   bool            silentInitialization /*=false*/)
    : QWindow(parent)
    , ccGLWindowInterface(this, silentInitialization)
    , m_context(nullptr)
    , m_parentWidget(nullptr)
{
	setSurfaceType(QWindow::OpenGLSurface);

	m_format = format ? *format : requestedFormat();

	// default picking mode
	setPickingMode(DEFAULT_PICKING);

	// default interaction mode
	setInteractionMode(MODE_TRANSFORM_CAMERA);

	// signal/slot connections
	connect(m_signalEmitter, &ccGLWindowSignalEmitter::itemPickedFast, this, &ccGLWindowStereo::onItemPickedFastSlot, Qt::DirectConnection);
	connect(&m_scheduleTimer, &QTimer::timeout, [&]()
	        { checkScheduledRedraw(); });
	connect(&m_autoRefreshTimer, &QTimer::timeout, this, [&]()
	        { update(); });
	connect(&m_deferredPickingTimer, &QTimer::timeout, this, [&]()
	        { doPicking(); });

	QString windowTitle = QString("3D View Stereo %1").arg(m_uniqueID);
	setWindowTitle(windowTitle);
	setObjectName(windowTitle);
}

ccGLWindowStereo::~ccGLWindowStereo()
{
	// disable the stereo mode (mainly to release the FBO of stereo glasses ;)
	disableStereoMode();

	uninitializeGL();

	if (m_context)
	{
		m_context->doneCurrent();
	}
}

void ccGLWindowStereo::grabMouse()
{
	setMouseGrabEnabled(true);
}

void ccGLWindowStereo::releaseMouse()
{
	setMouseGrabEnabled(false);
}

void ccGLWindowStereo::setParentWidget(QWidget* widget)
{
	m_parentWidget = widget;

	if (widget)
	{
		// drag & drop handling
		widget->setAcceptDrops(true);
		widget->setAttribute(Qt::WA_AcceptTouchEvents, true);
		widget->setAttribute(Qt::WA_OpaquePaintEvent, true);

		m_parentWidget->setObjectName(windowTitle());
	}
}

void ccGLWindowStereo::doMakeCurrent()
{
	if (m_context)
	{
		m_context->makeCurrent(this);
	}

	if (m_activeFbo)
	{
		m_activeFbo->start();
	}
}

bool ccGLWindowStereo::preInitialize(bool& firstTime)
{
	firstTime = false;
	if (!m_context)
	{
		m_context = new QOpenGLContext(this);
		m_context->setFormat(m_format);
		m_context->setShareContext(QOpenGLContext::globalShareContext());
		if (!m_context->create())
		{
			ccLog::Error("Failed to create the OpenGL context");
			return false;
		}
		firstTime = true;
	}
	else if (!m_context->isValid())
	{
		return false;
	}

	m_context->makeCurrent(this);

	return true;
}

bool ccGLWindowStereo::postInitialize(bool firstTime)
{
	if (firstTime)
	{
		resizeGL(width(), height());
	}

	return true;
}

bool ccGLWindowStereo::event(QEvent* evt)
{
	// process generic events
	if (processEvents(evt))
	{
		return true;
	}

	switch (evt->type())
	{
	case QEvent::Resize:
	{
		QSize newSize = static_cast<QResizeEvent*>(evt)->size();
		resizeGL(newSize.width(), newSize.height());
		evt->accept();
	}
		return true;

	case QEvent::Expose:
	{
		if (isExposed())
		{
			requestUpdate();
		}
		evt->accept();
	}
		return true;

	case QEvent::UpdateRequest:
	case QEvent::Show:
	case QEvent::Paint:
	{
		requestUpdate();
		evt->accept();
	}
		return true;

	default:
		// nothing to do
		break;
	}

	return QWindow::event(evt);
}

void ccGLWindowStereo::resizeGL(int w, int h)
{
	onResizeGL(w, h);

	requestUpdate();
}

GLuint ccGLWindowStereo::defaultQtFBO() const
{
	return 0;
}

void ccGLWindowStereo::requestUpdate()
{
	if (!m_autoRefresh)
	{
		update();
	}
}

bool ccGLWindowStereo::initPaintGL()
{
	if (!isExposed())
	{
		return false;
	}
	if (!m_initialized && !initialize())
	{
		return false;
	}

	doMakeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glViewport(m_glViewport.x(), m_glViewport.y(), m_glViewport.width(), m_glViewport.height());

	return true;
}

void ccGLWindowStereo::swapGLBuffers()
{
	if (!m_stereoModeEnabled)
	{
		if (m_context)
		{
			m_context->swapBuffers(this);
		}
		else
		{
			assert(false);
		}
	}
}

bool ccGLWindowStereo::enableStereoMode(const StereoParams& params)
{
	return ccGLWindowInterface::enableStereoMode(params);
}

void ccGLWindowStereo::disableStereoMode()
{
	ccGLWindowInterface::disableStereoMode();
}

void ccGLWindowStereo::Create(ccGLWindowStereo*& window, QWidget*& widget, bool silentInitialization /*=false*/)
{
	QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	format.setStereo(true);

	window = new ccGLWindowStereo(&format, nullptr, silentInitialization);
	widget = new ccGLStereoWidget(window);
}

ccGLWindowStereo* ccGLWindowStereo::FromWidget(QWidget* widget)
{
	ccGLStereoWidget* myWidget = qobject_cast<ccGLStereoWidget*>(widget);
	if (myWidget)
	{
		return myWidget->associatedWindow();
	}
	else
	{
		assert(false);
		return nullptr;
	}
}

void ccGLWindowStereo::doSetMouseTracking(bool enable)
{
	if (m_parentWidget)
	{
		m_parentWidget->setMouseTracking(enable);
	}
}
