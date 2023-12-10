//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//qCC
#include "ccGLWindow.h"

//qCC_db
#include <ccHObject.h>

//CCFbo
#include <ccFrameBufferObject.h>

//Qt
#include <QMessageBox>
#include <QResizeEvent>

ccGLWindow::ccGLWindow(	QSurfaceFormat* format/*=nullptr*/,
						QOpenGLWidget* parent/*=nullptr*/,
						bool silentInitialization/*=false*/)
	: QOpenGLWidget(parent)
	, ccGLWindowInterface(this, silentInitialization)
{
	m_font = font();

	//drag & drop handling
	setAcceptDrops(true);

	if (format)
	{
		setFormat(*format);
	}

	//default picking mode
	setPickingMode(DEFAULT_PICKING);

	//default interaction mode
	setInteractionMode(MODE_TRANSFORM_CAMERA);

	//signal/slot connections
	connect(m_signalEmitter, &ccGLWindowSignalEmitter::itemPickedFast, this, &ccGLWindow::onItemPickedFastSlot, Qt::DirectConnection);
	connect(&m_scheduleTimer, &QTimer::timeout, [&]() { checkScheduledRedraw(); });
	connect(&m_autoRefreshTimer, &QTimer::timeout, this, [&]() { update(); });
	connect(&m_deferredPickingTimer, &QTimer::timeout, this, [&]() { doPicking(); });

	setAttribute(Qt::WA_AcceptTouchEvents, true);
	setAttribute(Qt::WA_OpaquePaintEvent, true);

	QString windowTitle = QString("3D View %1").arg(m_uniqueID);
	setWindowTitle(windowTitle);
	setObjectName(windowTitle);
}

ccGLWindow::~ccGLWindow()
{
	uninitializeGL();

	cancelScheduledRedraw();
}

void ccGLWindow::doMakeCurrent()
{
	QOpenGLWidget::makeCurrent();

	if (m_activeFbo)
	{
		m_activeFbo->start();
	}
}

bool ccGLWindow::event(QEvent* evt)
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
		update();
	}
	break;

	default:
		// nothing to do
	break;
	
	}

	return QOpenGLWidget::event(evt);
}

GLuint ccGLWindow::defaultQtFBO() const
{
	if (m_stereoModeEnabled && (m_stereoParams.glassType == StereoParams::NVIDIA_VISION || m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY))
	{
		return 0;
	}
	else
	{
		return defaultFramebufferObject();
	}
}

void ccGLWindow::requestUpdate()
{
	if (!m_autoRefresh)
	{
		update();
	}
}

bool ccGLWindow::enableStereoMode(const StereoParams& params)
{
	if (params.glassType == StereoParams::OCULUS)
	{
		QMessageBox::critical(asWidget(), "Oculus", "The Oculus device is not supported by this type of 3D view");
		return false;
	}
	else
	{
		return ccGLWindowInterface::enableStereoMode(params);
	}
}

void ccGLWindow::Create(ccGLWindow*& window, QWidget*& widget, bool silentInitialization/*=false*/)
{
	QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	format.setStereo(false);

	window = new ccGLWindow(&format, nullptr, silentInitialization);
	widget = window;
}

ccGLWindow* ccGLWindow::FromWidget(QWidget* widget)
{
	ccGLWindow* myWidget = qobject_cast<ccGLWindow*>(widget);
	if (myWidget)
	{
		return myWidget;
	}
	else
	{
		assert(false);
		return nullptr;
	}
}
