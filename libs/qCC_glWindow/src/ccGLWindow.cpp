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
#include "ccRenderingTools.h"

//qCC_db
#include <cc2DLabel.h>
#include <ccClipBox.h>
#include <ccColorRampShader.h>
#include <ccHObject.h>
#include <ccInteractor.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccSphere.h> //for the pivot symbol

//CCFbo
#include <ccGlFilter.h>
#include <ccFrameBufferObject.h>

//Qt
#include <QApplication>
#include <QLayout>
#include <QMessageBox>
#include <QMimeData>
#include <QSettings>
#include <QTouchEvent>
#include <QWheelEvent>
#include <QOpenGLDebugLogger>

#ifdef CC_GL_WINDOW_USE_QWINDOW
#include <QOpenGLPaintDevice>
#endif

#ifdef CC_OCULUS_SUPPORT
#include "oculus/ccOculus.h"
static OculusHMD s_oculus;
#endif //CC_OCULUS_SUPPORT

#ifdef USE_VLD
#include <vld.h>
#endif

//Max click duration for enabling picking mode (in ms)
constexpr int CC_MAX_PICKING_CLICK_DURATION_MS = 200;

//invalid GL list index
constexpr GLuint GL_INVALID_LIST_ID = (~0);

//stereo passes
static const unsigned char MONO_OR_LEFT_RENDERING_PASS = 0;
static const unsigned char RIGHT_RENDERING_PASS = 1;

/*** Persistent settings ***/

constexpr char c_ps_groupName[] = "ccGLWindow";
constexpr char c_ps_stereoGlassType[] = "stereoGlassType";

//! Rendering params
struct ccGLWindow::RenderingParams
{
	// Next LOD state
	LODState nextLODState;

	// Pass info
	unsigned char passIndex = 0;
	unsigned char passCount = 1;

	// 2D background
	bool drawBackground = true;
	bool clearDepthLayer = true;
	bool clearColorLayer = true;

	// 3D central layer
	bool draw3DPass = true;
	bool useFBO = false;
	bool draw3DCross = false;

	// 2D foreground
	bool drawForeground = true;

	//! Candidate pivot point(s) (will be used when the mouse is released)
	/** Up to 2 candidates, if stereo mode is enabled **/
	CCVector3d autoPivotCandidates[2];
	bool hasAutoPivotCandidates[2] = { false, false };
};

ccGLWindow::ccGLWindow(	QSurfaceFormat* format/*=nullptr*/,
						ccGLWindowParent* parent/*=nullptr*/,
						bool silentInitialization/*=false*/)
	: ccGLWindowInterface(parent, silentInitialization)
#ifdef CC_GL_WINDOW_USE_QWINDOW
	, m_context(nullptr)
	, m_device(new QOpenGLPaintDevice)
	, m_parentWidget(nullptr)
#endif
{
#ifdef CC_GL_WINDOW_USE_QWINDOW
	setSurfaceType(QWindow::OpenGLSurface);

	m_format = format ? *format : requestedFormat();
#else
	m_font = font();

	//drag & drop handling
	setAcceptDrops(true);

	if (format)
	{
		setFormat(*format);
	}

	//drag & drop handling
	setAcceptDrops(true);
#endif

	//default picking mode
	setPickingMode(DEFAULT_PICKING);

	//default interaction mode
	setInteractionMode(MODE_TRANSFORM_CAMERA);

	//signal/slot connections
	connect(m_signalEmitter, &ccGLWindowSignalEmitter::itemPickedFast, this, &ccGLWindow::onItemPickedFast, Qt::DirectConnection);
	connect(&m_scheduleTimer, &QTimer::timeout, this, &ccGLWindow::checkScheduledRedraw);
	connect(&m_autoRefreshTimer, &QTimer::timeout, this, [=]() { update(); });
	connect(&m_deferredPickingTimer, &QTimer::timeout, this, &ccGLWindow::doPicking);

#ifndef CC_GL_WINDOW_USE_QWINDOW
	setAcceptDrops(true);
	setAttribute(Qt::WA_AcceptTouchEvents, true);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
#endif
}

ccGLWindow::~ccGLWindow()
{
	cancelScheduledRedraw();

	//disable the stereo mode (mainly to release the Oculus FBO ;)
	disableStereoMode();

#ifdef CC_GL_WINDOW_USE_QWINDOW
	if (m_context)
		m_context->doneCurrent();

	delete m_device;
	m_device = nullptr;
#endif
}

#ifdef CC_GL_WINDOW_USE_QWINDOW

void ccGLWindow::grabMouse()
{
	setMouseGrabEnabled(true);
}

void ccGLWindow::releaseMouse()
{
	setMouseGrabEnabled(false);
}

void ccGLWindow::setParentWidget(QWidget* widget)
{
	m_parentWidget = widget;

	if (widget)
	{
		//drag & drop handling
		widget->setAcceptDrops(true);
		widget->setAttribute(Qt::WA_AcceptTouchEvents, true);
		widget->setAttribute(Qt::WA_OpaquePaintEvent, true);

		m_parentWidget->setObjectName(windowTitle());
	}
}

#endif

void ccGLWindow::makeCurrent()
{
#ifdef CC_GL_WINDOW_USE_QWINDOW
	if (m_context)
	{
		m_context->makeCurrent(this);
	}
#else
	QOpenGLWidget::makeCurrent();
#endif

	if (m_activeFbo)
	{
		m_activeFbo->start();
	}
}

void ccGLWindow::setInteractionMode(INTERACTION_FLAGS flags)
{
	m_interactionFlags = flags;

	//we need to explicitely enable 'mouse tracking' to track the mouse when no button is clicked
#ifdef CC_GL_WINDOW_USE_QWINDOW
	if (m_parentWidget)
	{
		m_parentWidget->setMouseTracking(flags & (INTERACT_CLICKABLE_ITEMS | INTERACT_SIG_MOUSE_MOVED));
	}
#else
	setMouseTracking(flags & (INTERACT_CLICKABLE_ITEMS | INTERACT_SIG_MOUSE_MOVED));
#endif

	if ((flags & INTERACT_CLICKABLE_ITEMS) == 0)
	{
		//auto-hide the embedded icons if they are disabled
		m_clickableItemsVisible = false;
	}
}

bool ccGLWindow::initialize()
{
#ifdef CC_GL_WINDOW_USE_QWINDOW
	bool firstTime = false;
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
#endif

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//initialize can be called again when switching to exclusive full screen!
	if (!m_initialized)
	{
		if (!glFunc->initializeOpenGLFunctions()) //DGM: seems to be necessary at least with Qt 5.4
		{
			assert(false);
			return false;
		}

		//we init the model view and projection matrices with identity
		m_viewMatd.toIdentity();
		m_projMatd.toIdentity();

		//we init the OpenGL ones with the same values
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glLoadIdentity();
		glFunc->glMatrixMode(GL_PROJECTION);
		glFunc->glLoadIdentity();

		//we emit the 'baseViewMatChanged' signal
		Q_EMIT m_signalEmitter->baseViewMatChanged(m_viewportParams.viewMat);

		//set viewport and visu. as invalid
		invalidateViewport();
		invalidateVisualization();
		deprecate3DLayer();
		
		//FBO support (TODO: catch error?)
		m_glExtFuncSupported = m_glExtFunc.initializeOpenGLFunctions();

		//OpenGL version
		const char* vendorName = reinterpret_cast<const char*>(glFunc->glGetString(GL_VENDOR));
		const QString vendorNameStr = QString(vendorName).toUpper();
		if (!m_silentInitialization)
		{
			ccLog::Print("[3D View %i] Graphics card manufacturer: %s", m_uniqueID, vendorName);
			ccLog::Print("[3D View %i] Renderer: %s", m_uniqueID, glFunc->glGetString(GL_RENDERER));
			ccLog::Print("[3D View %i] GL version: %s", m_uniqueID, glFunc->glGetString(GL_VERSION));
			ccLog::Print("[3D View %i] GLSL Version: %s", m_uniqueID, glFunc->glGetString(GL_SHADING_LANGUAGE_VERSION));
		}

		ccGui::ParamStruct params = getDisplayParameters();

		//VBO support
		if (context()->hasExtension(QByteArrayLiteral("GL_ARB_vertex_buffer_object")))
		{
			QStringList glVersion = QString((const char*)glFunc->glGetString(GL_VERSION)).split('.');
			int majorVersion = 0;
			int minorVersion = 0;
			if (glVersion.size() >= 2)
			{
				majorVersion = glVersion[0].toInt();
				minorVersion = glVersion[1].toInt();
			}
			else
			{
				assert(false);
			}

			if (params.useVBOs && (!vendorName || (vendorNameStr.startsWith("ATI") && (majorVersion < 4 || (majorVersion == 4 && minorVersion < 6))))) //only if OpenGL version is earlier than 4.6
			{
				if (!m_silentInitialization)
					ccLog::Warning("[3D View %i] VBO support has been disabled as it may not work on %s cards!\nYou can manually activate it in the display settings (at your own risk!)", m_uniqueID, vendorName);
				params.useVBOs = false;
			}
			else if (!m_silentInitialization)
			{
				ccLog::Print("[3D View %i] VBOs available", m_uniqueID);
			}
		}
		else
		{
			params.useVBOs = false;
		}

		//Shaders and other OpenGL extensions
		m_shadersEnabled =	context()->hasExtension(QByteArrayLiteral("GL_ARB_shading_language_100"))
						&&	context()->hasExtension(QByteArrayLiteral("GL_ARB_shader_objects"))
						&&	context()->hasExtension(QByteArrayLiteral("GL_ARB_vertex_shader"))
						&&	context()->hasExtension(QByteArrayLiteral("GL_ARB_fragment_shader"));
		if (!m_shadersEnabled)
		{
			//if no shader, no GL filter!
			if (!m_silentInitialization)
				ccLog::Warning("[3D View %i] Shaders and GL filters unavailable", m_uniqueID);
		}
		else
		{
			if (!m_silentInitialization)
				ccLog::Print("[3D View %i] Shaders available", m_uniqueID);

			m_glFiltersEnabled = context()->hasExtension(QByteArrayLiteral("GL_EXT_framebuffer_object"));
			if (m_glFiltersEnabled)
			{
				if (!m_silentInitialization)
					ccLog::Print("[3D View %i] GL filters available", m_uniqueID);
				m_alwaysUseFBO = true;
			}
			else if (!m_silentInitialization)
			{
				ccLog::Warning("[3D View %i] GL filters unavailable (FBO not supported)", m_uniqueID);
			}

			//color ramp shader
			if (!m_colorRampShader)
			{
				//we will update global parameters
				params.colorScaleShaderSupported = false;

				GLint maxBytes = 0;
				glFunc->glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_COMPONENTS, &maxBytes);

				const GLint minRequiredBytes = ccColorRampShader::MinRequiredBytes();
				if (maxBytes < minRequiredBytes)
				{
					if (!m_silentInitialization)
						ccLog::Warning("[3D View %i] Not enough memory on shader side to use color ramp shader! (max=%i/%i bytes)", m_uniqueID, maxBytes, minRequiredBytes);
				}
				else
				{
					ccColorRampShader* colorRampShader = new ccColorRampShader();
					QString error;
					const QString shaderPath = QStringLiteral( "%1/ColorRamp/color_ramp.frag" ).arg( GetShaderPath() );
					
					if (!colorRampShader->loadProgram(QString(), shaderPath, error))
					{
						if (!m_silentInitialization)
							ccLog::Warning(QString("[3D View %1] Failed to load color ramp shader: '%2'").arg(m_uniqueID).arg(error));
						delete colorRampShader;
						colorRampShader = nullptr;
					}
					else
					{
						if (!m_silentInitialization)
							ccLog::Print("[3D View %i] Color ramp shader loaded successfully", m_uniqueID);
						m_colorRampShader = colorRampShader;
						params.colorScaleShaderSupported = true;

						//if global parameter is not yet defined
						if (!getDisplayParameters().isInPersistentSettings("colorScaleUseShader"))
						{
							bool shouldUseShader = true;
							if (!vendorName || vendorNameStr.startsWith("ATI") || vendorNameStr.startsWith("VMWARE"))
							{
								if (!m_silentInitialization)
								{
									ccLog::Warning("[3D View %i] Color ramp shader will remain disabled as it may not work on %s cards!\nYou can manually activate it in the display settings (at your own risk!)", m_uniqueID, vendorName);
								}
								shouldUseShader = false;
							}
							params.colorScaleUseShader = shouldUseShader;
						}
					}
				}
			}

			//stereo mode
			if (!m_silentInitialization)
			{
				GLboolean isStereoEnabled = 0;
				glFunc->glGetBooleanv(GL_STEREO, &isStereoEnabled);
				ccLog::Print(QString("[3D View %1] Stereo mode: %2").arg(m_uniqueID).arg(isStereoEnabled ? "supported" : "not supported"));
			}
		}

#ifdef QT_DEBUG
		//KHR extension (debug)
		if (context()->hasExtension(QByteArrayLiteral("GL_KHR_debug")))
		{
			if (!m_silentInitialization)
				ccLog::Print("[3D View %i] GL KHR (debug) extension available", m_uniqueID);

			QOpenGLDebugLogger* logger = new QOpenGLDebugLogger(this);
			logger->initialize();

			connect(logger, &QOpenGLDebugLogger::messageLogged, this, &ccGLWindow::handleLoggedMessage);
			logger->enableMessages();
			logger->disableMessages(QOpenGLDebugMessage::AnySource,
									QOpenGLDebugMessage::DeprecatedBehaviorType
									| QOpenGLDebugMessage::PortabilityType
									| QOpenGLDebugMessage::PerformanceType
									| QOpenGLDebugMessage::OtherType);
			logger->startLogging(QOpenGLDebugLogger::SynchronousLogging);
		}
#endif

		//apply (potentially) updated parameters;
		setDisplayParameters(params, hasOverriddenDisplayParameters());

#if 0
		//OpenGL 3.3+ rendering shader
		if ( QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_3_3 )
		{
			bool vaEnabled = ccFBOUtils::CheckVAAvailability();
			if (vaEnabled && !m_customRenderingShader)
			{
				ccGui::ParamStruct params = getDisplayParameters();

				ccShader* renderingShader = new ccShader();
				QString shadersPath = ccGLWindow::getShadersPath();
				QString error;
				if (!renderingShader->fromFile(shadersPath+QString("/Rendering"),QString("rendering"),error))
				{
					if (!m_silentInitialization)
						ccLog::Warning(QString("[3D View %i] Failed to load custom rendering shader: '%2'").arg(m_uniqueID).arg(error));
					delete renderingShader;
					renderingShader = nullptr;
				}
				else
				{
					m_customRenderingShader = renderingShader;
				}
				setDisplayParameters(params,hasOverriddenDisplayParameters());
			}
		}
#endif

		if (!m_silentInitialization)
		{
			ccLog::Print("[ccGLWindow] 3D view initialized");
		}

		m_initialized = true;
	}

	//transparency off by default
	glFunc->glDisable(GL_BLEND);
	glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//nicest rendering for points by default (when GL_POINT_SMOOTH is enabled)
	glFunc->glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

	//no global ambient
	glFunc->glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ccColor::night.rgba);

	logGLError("ccGLWindow::initialize");

#ifdef CC_GL_WINDOW_USE_QWINDOW
	if (firstTime)
	{
		resizeGL(width(), height());
	}
#endif

	return true;
}

void ccGLWindow::uninitializeGL()
{
	if (!m_initialized)
	{
		return;
	}

	assert(!m_captureMode.enabled);
	makeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	if (m_trihedronGLList != GL_INVALID_LIST_ID)
	{
		glFunc->glDeleteLists(m_trihedronGLList, 1);
		m_trihedronGLList = GL_INVALID_LIST_ID;
	}
	if (m_pivotGLList != GL_INVALID_LIST_ID)
	{
		glFunc->glDeleteLists(m_pivotGLList, 1);
		m_pivotGLList = GL_INVALID_LIST_ID;
	}

	m_initialized = false;
}

bool ccGLWindow::event(QEvent* evt)
{
	switch (evt->type())
	{
	//Gesture start/stop
	case QEvent::TouchBegin:
	case QEvent::TouchEnd:
	{
		QTouchEvent* touchEvent = static_cast<QTouchEvent*>(evt);
		touchEvent->accept();
		m_touchInProgress = (evt->type() == QEvent::TouchBegin);
		m_touchBaseDist = 0.0;
		ccLog::PrintDebug(QString("Touch event %1").arg(m_touchInProgress ? "begins" : "ends"));
	}
	return true;

	case QEvent::Close:
	{
		if (m_unclosable)
		{
			evt->ignore();
		}
		else
		{
			evt->accept();
		}
	}
	return true;

	case QEvent::DragEnter:
	{
		dragEnterEvent(static_cast<QDragEnterEvent*>(evt));
	}
	return true;

	case QEvent::Drop:
	{
		dropEvent(static_cast<QDropEvent*>(evt));
	}
	return true;

	case QEvent::TouchUpdate:
	{
		//Gesture update
		if (m_touchInProgress && !m_viewportParams.perspectiveView)
		{
			QTouchEvent* touchEvent = static_cast<QTouchEvent*>(evt);
			const QList<QTouchEvent::TouchPoint>& touchPoints = touchEvent->touchPoints();
			if (touchPoints.size() == 2)
			{
				QPointF D = (touchPoints[1].pos() - touchPoints[0].pos());
				qreal dist = std::sqrt(D.x()*D.x() + D.y()*D.y());
				if (m_touchBaseDist != 0.0)
				{
					float pseudo_wheelDelta_deg = dist < m_touchBaseDist ? -15.0f : 15.0f;
					onWheelEvent(pseudo_wheelDelta_deg);

					Q_EMIT m_signalEmitter->mouseWheelRotated(pseudo_wheelDelta_deg);
				}
				m_touchBaseDist = dist;
				evt->accept();
				return true;
			}
		}
		ccLog::PrintDebug(QString("Touch update (%1 points)").arg(static_cast<QTouchEvent*>(evt)->touchPoints().size()));
	}
	break;

#ifdef CC_GL_WINDOW_USE_QWINDOW
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
#else
	case QEvent::Resize:
	{
		update();
	}
	break;
#endif

	default:
	{
		//ccLog::Print("Unhandled event: %i", evt->type());
	}
	break;
	
	}

#ifdef _DEBUG
	QEvent::Type evtType = evt->type();
#endif
	return ccGLWindowParent::event(evt);
}

void ccGLWindow::setGLViewport(const QRect& rect)
{
	//correction for HD screens
	const int retinaScale = devicePixelRatio();
	m_glViewport = QRect(rect.left() * retinaScale, rect.top() * retinaScale, rect.width() * retinaScale, rect.height() * retinaScale);
	invalidateViewport();

	if (context() && context()->isValid())
	{
		makeCurrent();

		functions()->glViewport(m_glViewport.x(), m_glViewport.y(), m_glViewport.width(), m_glViewport.height());
	}
}

void ccGLWindow::resizeGL(int w, int h)
{
	//update OpenGL viewport
	setGLViewport(0, 0, w, h);

	invalidateVisualization();
	deprecate3DLayer();

	if (m_initialized)
	{
		//filters
		if (m_fbo || m_alwaysUseFBO)
			initFBO(w, h);
		if (m_activeGLFilter)
			initGLFilter(w, h, true);

		//pivot symbol is dependent on the screen size!
		if (m_pivotGLList != GL_INVALID_LIST_ID)
		{
			functions()->glDeleteLists(m_pivotGLList, 1);
			m_pivotGLList = GL_INVALID_LIST_ID;
		}

		logGLError("ccGLWindow::resizeGL");
	}

	setLODEnabled(true, true);
	m_currentLODState.level = 0;
	if (m_hotZone)
	{
		m_hotZone->topCorner = QPoint(0, 0);
	}

	displayNewMessage(	QString("New size = %1 * %2 (px)").arg(glWidth()).arg(glHeight()),
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						SCREEN_SIZE_MESSAGE);

#ifdef CC_GL_WINDOW_USE_QWINDOW
	requestUpdate();
#else
	logGLError("ccGLWindow::resizeGL");
#endif
}

//Framerate test
static const qint64 FRAMERATE_TEST_DURATION_MSEC = 10000;
static const unsigned FRAMERATE_TEST_MIN_FRAMES = 50;
static bool s_frameRateTestInProgress = false;
static ccGLMatrixd s_frameRateBackupMat;
static QTimer s_frameRateTimer;
static QElapsedTimer s_frameRateElapsedTimer;
static qint64 s_frameRateElapsedTime_ms = 0; //i.e. not initialized
static unsigned s_frameRateCurrentFrame = 0;

void ccGLWindow::startFrameRateTest()
{
	if (s_frameRateTestInProgress)
	{
		ccLog::Error("Framerate test already in progress!");
		return;
	}
	s_frameRateTestInProgress = true;

	//we save the current view matrix
	s_frameRateBackupMat = m_viewportParams.viewMat;

	connect(&s_frameRateTimer, &QTimer::timeout, this, [=] () {
		redraw();
	}, Qt::QueuedConnection);

	displayNewMessage("[Framerate test in progress]",
		ccGLWindow::UPPER_CENTER_MESSAGE,
		true,
		3600);

	stopLODCycle();

	//let's start
	s_frameRateCurrentFrame = 0;
	s_frameRateElapsedTime_ms = 0;
	s_frameRateElapsedTimer.start();
	s_frameRateTimer.start(0);
};

void ccGLWindow::stopFrameRateTest()
{
	if (s_frameRateTestInProgress)
	{
		s_frameRateTimer.stop();
		s_frameRateTimer.disconnect();
	}
	s_frameRateTestInProgress = false;

	//we restore the original view mat
	m_viewportParams.viewMat = s_frameRateBackupMat;
	invalidateVisualization();

	displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE); //clear message in the upper center area
	if (s_frameRateElapsedTime_ms > 0)
	{
		QString message = QString("Framerate: %1 fps").arg((s_frameRateCurrentFrame*1.0e3) / s_frameRateElapsedTime_ms, 0, 'f', 3);
		displayNewMessage(message, ccGLWindow::LOWER_LEFT_MESSAGE, true);
		ccLog::Print(message);
	}
	else
	{
		ccLog::Error("An error occurred during framerate test!");
	}

	redraw();
}

void ccGLWindow::refresh(bool only2D/*=false*/)
{
	if (m_shouldBeRefreshed && isVisible())
	{
		redraw(only2D);
	}
}

void ccGLWindow::redraw(bool only2D/*=false*/, bool resetLOD/*=true*/)
{
	if (m_currentLODState.inProgress && resetLOD)
	{
		//reset current LOD cycle
		m_LODPendingIgnore = true;
		m_LODPendingRefresh = false;
		stopLODCycle();
	}

	if (!only2D)
	{
		//force the 3D layer to be redrawn
		deprecate3DLayer();
	}

	if (isVisible() && !m_autoRefresh)
	{
		requestUpdate();
	}
}

GLuint ccGLWindow::defaultQtFBO() const
{
#ifdef CC_GL_WINDOW_USE_QWINDOW
	return 0;
#else

	if (m_stereoModeEnabled && (m_stereoParams.glassType == StereoParams::NVIDIA_VISION || m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY))
	{
		return 0;
	}
	else
	{
		return defaultFramebufferObject();
	}
#endif
}

void ccGLWindow::requestUpdate()
{
	if (!m_autoRefresh)
	{
		update();
	}
}

void ccGLWindow::paintGL()
{
#ifdef CC_GL_WINDOW_USE_QWINDOW
	if (!isExposed())
	{
		return;
	}
	if (!m_initialized && !initialize())
	{
		return;
	}
	assert(m_context);
	makeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glViewport(m_glViewport.x(), m_glViewport.y(), m_glViewport.width(), m_glViewport.height());
#endif

//#define DEBUG_TIMINGS
#ifdef DEBUG_TIMINGS
	std::vector<qint64> debugTimings;
	debugTimings.push_back(m_timer.nsecsElapsed());
#endif
	qint64 startTime_ms = m_currentLODState.inProgress ? m_timer.elapsed() : 0;

	//reset the texture pool index
	m_texturePoolLastIndex = 0;

	if (m_scheduledFullRedrawTime != 0)
	{
		//scheduled redraw is (about to be) done
		cancelScheduledRedraw();
	}

	//we update font size (for text display)
	setFontPointSize(getFontPointSize());

	//context initialization
	CC_DRAW_CONTEXT CONTEXT;
	getContext(CONTEXT);

	//rendering parameters
	RenderingParams renderingParams;
	renderingParams.drawBackground = false;
	renderingParams.draw3DPass = false;
	renderingParams.drawForeground = true;

	//here are all the reasons for which we would like to update the main 3D layer
	if (!m_fbo ||
		m_updateFBO ||
		m_captureMode.enabled ||
		m_currentLODState.inProgress
		)
	{
		//we must update the FBO (or display without FBO)
		renderingParams.drawBackground = true;
		renderingParams.draw3DPass = true;
	}

	//other rendering options
	renderingParams.useFBO = (m_fbo != nullptr);

	renderingParams.draw3DCross = getDisplayParameters().displayCross;
	renderingParams.passCount = m_stereoModeEnabled ? 2 : 1;

	//clean the outdated messages
	{
		std::list<MessageToDisplay>::iterator it = m_messagesToDisplay.begin();
		qint64 currentTime_sec = m_timer.elapsed() / 1000;
		//ccLog::PrintDebug(QString("[paintGL] Current time: %1 s.").arg(currentTime_sec));

		while (it != m_messagesToDisplay.end())
		{
			//no more valid? we delete the message
			if (it->messageValidity_sec < currentTime_sec)
			{
				it = m_messagesToDisplay.erase(it);
			}
			else
			{
				++it;
			}
		}
	}

#ifdef DEBUG_TIMINGS
	debugTimings.push_back(m_timer.nsecsElapsed());
#endif

	//start the rendering passes
	for (renderingParams.passIndex = 0; renderingParams.passIndex < renderingParams.passCount; ++renderingParams.passIndex)
	{
		fullRenderingPass(CONTEXT, renderingParams);
#ifdef DEBUG_TIMINGS
		debugTimings.push_back(m_timer.nsecsElapsed());
#endif
	}

#ifdef CC_GL_WINDOW_USE_QWINDOW
	if (	!m_stereoModeEnabled
		||	m_stereoParams.glassType != StereoParams::OCULUS
#ifdef CC_OCULUS_SUPPORT
		||	s_oculus.mirror.texture
#endif
		)
	{
		m_context->swapBuffers(this);
	}
#endif

	m_shouldBeRefreshed = false;

	if (	m_autoPickPivotAtCenter
		&&	!m_mouseMoved
		&&	(renderingParams.hasAutoPivotCandidates[0] || (m_stereoModeEnabled && renderingParams.hasAutoPivotCandidates[1]))
		&&	!renderingParams.nextLODState.inProgress)
	{
		CCVector3d pivot;
		if (renderingParams.hasAutoPivotCandidates[0])
		{
			pivot = renderingParams.autoPivotCandidates[0];
		}
		if (m_stereoModeEnabled && renderingParams.hasAutoPivotCandidates[1])
		{
			pivot += renderingParams.autoPivotCandidates[1];
			if (renderingParams.hasAutoPivotCandidates[0])
				pivot /= 2;
		}
		setPivotPoint(pivot, true, false);
	}
#ifdef DEBUG_TIMINGS
	debugTimings.push_back(m_timer.nsecsElapsed());
#endif

	if (renderingParams.nextLODState.inProgress)
	{
		//if the LOD display process is not finished
		m_currentLODState = renderingParams.nextLODState;
	}
	else
	{
		//we have reached the final level
		stopLODCycle();

		if (m_LODAutoDisable)
		{
			setLODEnabled(false);
		}
	}

	//For frame rate test
	if (s_frameRateTestInProgress)
	{
		s_frameRateElapsedTime_ms = s_frameRateElapsedTimer.elapsed();
		if (++s_frameRateCurrentFrame > FRAMERATE_TEST_MIN_FRAMES && s_frameRateElapsedTime_ms > FRAMERATE_TEST_DURATION_MSEC)
		{
			QTimer::singleShot(0, this, &ccGLWindow::stopFrameRateTest);
		}
		else
		{
			//rotate base view matrix
			ccGLMatrixd rotMat;
			rotMat.initFromParameters(2 * M_PI / FRAMERATE_TEST_MIN_FRAMES, CCVector3d(0, 1, 0), CCVector3d(0, 0, 0));
			m_viewportParams.viewMat = rotMat * m_viewportParams.viewMat;
			invalidateVisualization();
		}
	}
	else
	{
		//should we render a new LOD level?
		if (m_currentLODState.inProgress)
		{
			if ((!m_LODPendingRefresh || m_LODPendingIgnore) && !m_mouseMoved && !m_mouseButtonPressed)
			{
				qint64 displayTime_ms = m_timer.elapsed() - startTime_ms;
				//we try to refresh LOD levels at a regular pace
				static const qint64 baseLODRefreshTime_ms = 50;

				m_LODPendingRefresh = true;
				m_LODPendingIgnore = false;

				ccLog::PrintDebug(QString("[QPaintGL] New LOD pass scheduled with timer"));
				QTimer::singleShot(std::max(baseLODRefreshTime_ms - displayTime_ms, Q_INT64_C(0)), this, &ccGLWindow::renderNextLODLevel);
			}
		}
		else
		{
			//just in case
			m_LODPendingRefresh = false;
		}
	}
#ifdef DEBUG_TIMINGS
	debugTimings.push_back(m_timer.nsecsElapsed());

	QString debugTimingsMessage;
	for (size_t i = 1; i < debugTimings.size(); ++i)
	{
		debugTimingsMessage += QString("[DT%1 = %2]").arg(i).arg((debugTimings[i]- debugTimings[i - 1])/1000);
	}
	debugTimingsMessage += QString("[DT TOTAL = %2]").arg((debugTimings.back() - debugTimings.front()) / 1000);
	ccLog::Print(debugTimingsMessage);
#endif
}

void ccGLWindow::drawBackground(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	/****************************************/
	/****  PASS: 2D/BACKGROUND/NO LIGHT  ****/
	/****************************************/
	glFunc->glPointSize(m_viewportParams.defaultPointSize);
	glFunc->glLineWidth(m_viewportParams.defaultLineWidth);
	glFunc->glDisable(GL_DEPTH_TEST);

	CONTEXT.drawingFlags = CC_DRAW_2D;
	if (m_interactionFlags & INTERACT_TRANSFORM_ENTITIES)
	{
		CONTEXT.drawingFlags |= CC_VIRTUAL_TRANS_ENABLED;
	}

	setStandardOrthoCenter();

	//clear background
	{
		GLbitfield clearMask = GL_NONE;

		if (renderingParams.clearDepthLayer)
		{
			clearMask |= GL_DEPTH_BUFFER_BIT;
		}
		if (renderingParams.clearColorLayer)
		{
			const ccGui::ParamStruct& displayParams = getDisplayParameters();
			if (displayParams.drawBackgroundGradient)
			{
				//draw the default gradient color background
				int w = glWidth() / 2 + 1;
				int h = glHeight() / 2 + 1;

				const ccColor::Rgbub& bkgCol = getDisplayParameters().backgroundCol;
				const ccColor::Rgbub& frgCol = getDisplayParameters().textDefaultCol;

				//Gradient "texture" drawing
				glFunc->glBegin(GL_QUADS);
				{
					//we use the default background color for gradient start
					glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, bkgCol);
					glFunc->glVertex2i(-w, h);
					glFunc->glVertex2i(w, h);
					//and the inverse of the text color for gradient stop
					glFunc->glColor3ub(255 - frgCol.r, 255 - frgCol.g, 255 - frgCol.b);
					glFunc->glVertex2i(w, -h);
					glFunc->glVertex2i(-w, -h);
				}
				glFunc->glEnd();
			}
			else
			{
				//use plain color as specified by the user
				const ccColor::Rgbub& bkgCol = displayParams.backgroundCol;
				const ccColor::Rgbaf backgroundColor(	bkgCol.r / 255.0f,
														bkgCol.g / 255.0f,
														bkgCol.b / 255.0f,
														1.0f);

				glFunc->glClearColor(	backgroundColor.r,
										backgroundColor.g,
										backgroundColor.b,
										backgroundColor.a);

				clearMask |= GL_COLOR_BUFFER_BIT;
			}
		}

		//we clear the background
		if (clearMask != GL_NONE)
		{
			glFunc->glClear(clearMask);
		}
	}

	//draw 2D background primitives
	//DGM: useless for now
#if 0
	if (false)
	{
		//we draw 2D entities
		if (m_globalDBRoot)
			m_globalDBRoot->draw(CONTEXT);
		if (m_winDBRoot)
			m_winDBRoot->draw(CONTEXT);
	}
#endif
	
	logGLError("ccGLWindow::drawBackground");
}

void ccGLWindow::fullRenderingPass(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
{
	//visual traces
	QStringList diagStrings;
	if (m_showDebugTraces)
	{
		diagStrings << QString("Stereo mode %1 (pass %2)").arg(m_stereoModeEnabled && renderingParams.passCount == 2 ? "ON" : "OFF").arg(renderingParams.passIndex);
		diagStrings << QString("FBO %1").arg(m_fbo && renderingParams.useFBO ? "ON" : "OFF");
		diagStrings << QString("FBO2 %1").arg(m_fbo2 && renderingParams.useFBO ? "ON" : "OFF");
		diagStrings << QString("GL filter %1").arg(m_fbo && renderingParams.useFBO && m_activeGLFilter ? "ON" : "OFF");
		diagStrings << QString("LOD %1 (level %2)").arg(m_currentLODState.inProgress ? "ON" : "OFF").arg(m_currentLODState.level);
	}

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//backup the current viewport
	QRect originViewport = m_glViewport;
	bool modifiedViewport = false;

	ccFrameBufferObject* currentFBO = renderingParams.useFBO ? m_fbo : nullptr;
	if (m_stereoModeEnabled)
	{
		if (	m_stereoParams.glassType == StereoParams::NVIDIA_VISION
			||	m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
		{
			if (renderingParams.useFBO && renderingParams.passIndex == RIGHT_RENDERING_PASS)
			{
				currentFBO = m_fbo2;
			}
		}
#ifdef CC_OCULUS_SUPPORT
		else if (m_stereoParams.glassType == StereoParams::OCULUS && s_oculus.session)
		{
			renderingParams.useFBO = true;
			renderingParams.drawBackground = (CONTEXT.currentLODLevel == 0);
			renderingParams.draw3DPass = true;
			currentFBO = s_oculus.fbo;

			if (renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS)
			{
				//Get both eye poses simultaneously, with IPD offset already included.
				double displayMidpointSeconds = ovr_GetPredictedDisplayTime(s_oculus.session, 0);
				//Query the HMD for the current tracking state.
				ovrTrackingState hmdState = ovr_GetTrackingState(s_oculus.session, displayMidpointSeconds, ovrTrue);
				if (hmdState.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked)) 
				{
					//compute the eye positions
					ovr_CalcEyePoses(hmdState.HeadPose.ThePose, s_oculus.hmdToEyeViewOffset, s_oculus.layer.RenderPose);
					s_oculus.hasLastOVRPos = true;
				}
				else
				{
					s_oculus.hasLastOVRPos = false;
				}

				//Increment to use next texture, just before writing
				int currentIndex = 0;
				ovr_GetTextureSwapChainCurrentIndex(s_oculus.session, s_oculus.textureSwapChain, &currentIndex);

				unsigned int colorTexID = 0;
				ovr_GetTextureSwapChainBufferGL(s_oculus.session, s_oculus.textureSwapChain, currentIndex, &colorTexID);
				s_oculus.fbo->attachColor(colorTexID);

				GLuint depthTexID = s_oculus.depthTextures[currentIndex];
				s_oculus.fbo->attachDepth(depthTexID);
			}

			//set the right viewport
			{
				bindFBO(s_oculus.fbo);

				//s_oculus.fbo->setDrawBuffer(renderingParams.passIndex);
				const ovrRecti& vp = s_oculus.layer.Viewport[renderingParams.passIndex];
				setGLViewport(vp.Pos.x, vp.Pos.y, vp.Size.w, vp.Size.h);
				CONTEXT.glW = vp.Size.w;
				CONTEXT.glH = vp.Size.h;
				modifiedViewport = true;

				bindFBO(nullptr);
			}
		}
#endif //CC_OCULUS_SUPPORT
	}

	//if a FBO is activated
	if (	currentFBO
		&&	renderingParams.useFBO)
	{
		if (renderingParams.drawBackground || renderingParams.draw3DPass)
		{
			bindFBO(currentFBO);
			logGLError("ccGLWindow::fullRenderingPass (FBO start)");

			if (!m_currentLODState.inProgress || m_currentLODState.level == 0)
			{
				renderingParams.drawBackground = renderingParams.draw3DPass = true; //DGM: we must update the FBO completely!
			}
			else
			{
				renderingParams.drawBackground = false;
				assert(renderingParams.draw3DPass);
			}

			if (m_showDebugTraces)
			{
				diagStrings << "FBO updated";
			}
		}
	}
	else if (!m_captureMode.enabled) //capture mode doesn't use double buffering by default!
	{
		GLboolean isStereoEnabled = 0;
		glFunc->glGetBooleanv(GL_STEREO, &isStereoEnabled);
		//ccLog::Warning(QString("[fullRenderingPass:%0][NO FBO] Stereo test: %1").arg(renderingParams.passIndex).arg(isStereoEnabled));

		if (isStereoEnabled)
		{
			if (m_stereoModeEnabled
				&& (	m_stereoParams.glassType == StereoParams::NVIDIA_VISION
					||	m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
				)
			{
				glFunc->glDrawBuffer(renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS ? GL_BACK_LEFT : GL_BACK_RIGHT);
			}
			else
			{
				glFunc->glDrawBuffer(GL_BACK);
			}
		}
	}

	/******************/
	/*** BACKGROUND ***/
	/******************/
	if (renderingParams.drawBackground)
	{
		if (m_showDebugTraces)
		{
			diagStrings << "draw background";
		}

		//shall we clear the background (depth and/or color)
		if (m_currentLODState.level == 0)
		{
			if (m_stereoModeEnabled && m_stereoParams.isAnaglyph())
			{
				//we don't want to clear the color layer between two anaglyph rendering steps!
				renderingParams.clearColorLayer = (renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS);
			}
		}
		else
		{
			renderingParams.clearDepthLayer = false;
			renderingParams.clearColorLayer = false;
		}

		drawBackground(CONTEXT, renderingParams);
	}

	/*********************/
	/*** MAIN 3D LAYER ***/
	/*********************/
	if (renderingParams.draw3DPass)
	{
		if (m_showDebugTraces)
		{
			diagStrings << "draw 3D";
		}

		if (m_stereoModeEnabled && m_stereoParams.isAnaglyph())
		{
			//change color filter
			static GLboolean RED[3]  = { GL_TRUE, GL_FALSE, GL_FALSE };
			static GLboolean BLUE[3] = { GL_FALSE, GL_FALSE, GL_TRUE };
			static GLboolean CYAN[3] = { GL_FALSE, GL_TRUE, GL_TRUE };
			const GLboolean* RGB = nullptr;
			switch (m_stereoParams.glassType)
			{
			case StereoParams::RED_BLUE:
				RGB = (renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS ? RED : BLUE);
				break;
			
			case StereoParams::BLUE_RED:
				RGB = (renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS ? BLUE : RED);
				break;

			case StereoParams::RED_CYAN:
				RGB = (renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS ? RED : CYAN);
				break;

			case StereoParams::CYAN_RED:
				RGB = (renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS ? CYAN : RED);
				break;

			default:
				assert(false);
			}

			if (RGB)
			{
				glFunc->glColorMask(RGB[0], RGB[1], RGB[2], GL_TRUE);
			}
		}

		draw3D(CONTEXT, renderingParams);

		if (m_stereoModeEnabled && m_stereoParams.isAnaglyph())
		{
			//restore default color mask
			glFunc->glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		}
	}

	//display traces
	if (!diagStrings.isEmpty())
	{
		int x = glWidth() / 2 - 100;
		int y = 0;

		if (m_stereoModeEnabled && m_stereoParams.glassType != StereoParams::OCULUS)
		{
			if (renderingParams.passIndex == RIGHT_RENDERING_PASS)
				x += glWidth() / 2;
		}

		setStandardOrthoCorner();
		glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
		glFunc->glDisable(GL_DEPTH_TEST);

		//draw black background
		{
			int height = (diagStrings.size() + 1) * 14;
			glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::black);
			glFunc->glBegin(GL_QUADS);
			glFunc->glVertex2i(x, glHeight() - y);
			glFunc->glVertex2i(x, glHeight() - (y + height));
			glFunc->glVertex2i(x + 200, glHeight() - (y + height));
			glFunc->glVertex2i(x + 200, glHeight() - y);
			glFunc->glEnd();
		}

		glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::yellow);
		for (const QString &str : diagStrings)
		{
			renderText(x + 10, y + 10, str);
			y += 14;
		}

		glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT
	}

	//restore viewport if necessary
	if (modifiedViewport)
	{
		setGLViewport(originViewport);
		CONTEXT.glW = originViewport.width();
		CONTEXT.glH = originViewport.height();
		modifiedViewport = false;
	}

	bool oculusMode = (m_stereoModeEnabled && m_stereoParams.glassType == StereoParams::OCULUS);
	glFunc->glFlush();

	//process and/or display the FBO (if any)
	if (currentFBO && renderingParams.useFBO)
	{
		//we disable fbo (if any)
		if (renderingParams.drawBackground || renderingParams.draw3DPass)
		{
			logGLError("ccGLWindow::fullRenderingPass (FBO stop)");
			bindFBO(nullptr);
			m_updateFBO = false;
		}

		if (!oculusMode)
		{
			GLuint screenTex = 0;
			if (m_activeGLFilter && (!m_stereoModeEnabled || m_stereoParams.glassType != StereoParams::OCULUS)) //not supported with Oculus right now!
			{
				//we apply the GL filter
				GLuint depthTex = currentFBO->getDepthTexture();
				GLuint colorTex = currentFBO->getColorTexture();
				//minimal set of viewport parameters necessary for GL filters
				ccGlFilter::ViewportParameters parameters;
				{
					parameters.perspectiveMode = m_viewportParams.perspectiveView;
					parameters.zFar = m_viewportParams.zFar;
					parameters.zNear = m_viewportParams.zNear;
				}
				//apply shader
				m_activeGLFilter->shade(depthTex, colorTex, parameters);
				logGLError("ccGLWindow::paintGL/glFilter shade");
				bindFBO(nullptr); //in case the active filter has used a FBOs!

				//if capture mode is ON: we only want to capture it, not to display it
				if (!m_captureMode.enabled)
				{
					screenTex = m_activeGLFilter->getTexture();
					//ccLog::PrintDebug(QString("[QPaintGL] Will use the shader output texture (tex ID = %1)").arg(screenTex));
				}
			}
			else if (!m_captureMode.enabled)
			{
				//screenTex = currentFBO->getDepthTexture();
				screenTex = currentFBO->getColorTexture();
				//ccLog::PrintDebug(QString("[QPaintGL] Will use the standard FBO (tex ID = %1)").arg(screenTex));
			}

			//we display the FBO texture fullscreen (if any)
			if (glFunc->glIsTexture(screenTex))
			{
				setStandardOrthoCorner();

				glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
				glFunc->glDisable(GL_DEPTH_TEST);

				//select back left or back right buffer
				//DGM: as we couldn't call it before (because of the FBO) we have to do it now!
				GLboolean isStereoEnabled = 0;
				glFunc->glGetBooleanv(GL_STEREO, &isStereoEnabled);
				//ccLog::Warning(QString("[fullRenderingPass:%0][FBO] Stereo test: %1").arg(renderingParams.passIndex).arg(isStereoEnabled));
				if (isStereoEnabled)
				{
					if (m_stereoModeEnabled
						&& (	m_stereoParams.glassType == StereoParams::NVIDIA_VISION
							||	m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
						)
					{
						glFunc->glDrawBuffer(renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS ? GL_BACK_LEFT : GL_BACK_RIGHT);
					}
					else
					{
						glFunc->glDrawBuffer(GL_BACK);
					}
				}

				ccGLUtils::DisplayTexture2DPosition(screenTex, 0, 0, glWidth(), glHeight());

				//warning: we must set the original FBO texture as default
				glFunc->glBindTexture(GL_TEXTURE_2D, this->defaultQtFBO());

				glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT

				//we don't need the depth info anymore!
				//glFunc->glClear(GL_DEPTH_BUFFER_BIT);
			}
		}
	}
	
#ifdef CC_OCULUS_SUPPORT
	if (oculusMode && s_oculus.session && renderingParams.passIndex == RIGHT_RENDERING_PASS)
	{
		ovr_CommitTextureSwapChain(s_oculus.session, s_oculus.textureSwapChain);
		
		// Submit frame
		ovrLayerHeader* layers = &s_oculus.layer.Header;
		//glFunc->glEnable(GL_FRAMEBUFFER_SRGB);
		ovrResult result = ovr_SubmitFrame(s_oculus.session, 0, nullptr, &layers, 1);
		//glFunc->glDisable(GL_FRAMEBUFFER_SRGB);

		if (s_oculus.mirror.texture)
		{
			bindFBO(nullptr);

			assert(m_glExtFuncSupported);
			m_glExtFunc.glBindFramebuffer(GL_READ_FRAMEBUFFER, s_oculus.mirror.fbo);
			m_glExtFunc.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
			//compute the size of the destination texture
			int ow = s_oculus.mirror.size.width();
			int oh = s_oculus.mirror.size.height();
			int sx = 0;
			int sy = 0;
			int sw = width();
			int sh = height();

			GLfloat cw = static_cast<GLfloat>(sw) / ow;
			GLfloat ch = static_cast<GLfloat>(sh) / oh;
			GLfloat zoomFactor = std::min(cw, ch);
			int sw2 = static_cast<int>(ow * zoomFactor);
			int sh2 = static_cast<int>(oh * zoomFactor);
			sx += (sw - sw2) / 2;
			sy += (sh - sh2) / 2;
			sw = sw2;
			sh = sh2;

			m_glExtFunc.glBlitFramebuffer(0, oh, ow, 0, sx, sy, sx + sw, sy + sh, GL_COLOR_BUFFER_BIT, GL_NEAREST);
			m_glExtFunc.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		}

	}
#endif //CC_OCULUS_SUPPORT

	/******************/
	/*** FOREGROUND ***/
	/******************/
	if (renderingParams.drawForeground && !oculusMode)
	{
		drawForeground(CONTEXT, renderingParams);
	}

	glFunc->glFlush();
}

void ccGLWindow::draw3D(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glPointSize(m_viewportParams.defaultPointSize);
	glFunc->glLineWidth(m_viewportParams.defaultLineWidth);

	glFunc->glEnable(GL_DEPTH_TEST);

	CONTEXT.drawingFlags = CC_DRAW_3D | CC_DRAW_FOREGROUND;
	if (m_interactionFlags & INTERACT_TRANSFORM_ENTITIES)
	{
		CONTEXT.drawingFlags |= CC_VIRTUAL_TRANS_ENABLED;
	}

	setStandardOrthoCenter();

	/****************************************/
	/****    PASS: 3D/FOREGROUND/LIGHT   ****/
	/****************************************/
	if (m_customLightEnabled || m_sunLightEnabled)
	{
		CONTEXT.drawingFlags |= CC_LIGHT_ENABLED;

		//we enable absolute sun light (if activated)
		if (m_sunLightEnabled)
		{
			glEnableSunLight();
		}
	}

	//we activate the current shader (if any)
	if (m_activeShader)
	{
		m_activeShader->bind();
	}

	//color ramp shader for fast dynamic color ramp lookup-up
	if (m_colorRampShader && getDisplayParameters().colorScaleUseShader)
	{
		CONTEXT.colorRampShader = m_colorRampShader;
	}

	//custom rendering shader (OpenGL 3.3+)
	{
		//FIXME: work in progress
		CONTEXT.customRenderingShader = m_customRenderingShader;
	}

	//LOD
	if (	isLODEnabled()
		&&	!s_frameRateTestInProgress
		&&	(!m_stereoModeEnabled || m_stereoParams.glassType != StereoParams::OCULUS)
		)
	{
		CONTEXT.drawingFlags |= CC_LOD_ACTIVATED;

		//LOD rendering level (for clouds only)
		if (CONTEXT.decimateCloudOnMove)
		{
			//ccLog::Print(QString("[LOD] Rendering level %1").arg(m_currentLODState.level));
			m_currentLODState.inProgress = true;
			CONTEXT.currentLODLevel = m_currentLODState.level;
			CONTEXT.higherLODLevelsAvailable = false;
			CONTEXT.moreLODPointsAvailable = false;
		}
	}

	//model and projection matrices
	ccGLMatrixd modelViewMat;
	ccGLMatrixd projectionMat;

	//setup camera projection (DGM: AFTER THE LIGHTS)
	if (m_stereoModeEnabled)
	{
		CONTEXT.stereoPassIndex = renderingParams.passIndex;

#ifdef CC_OCULUS_SUPPORT

		if (m_stereoParams.glassType == StereoParams::OCULUS && s_oculus.session)
		{
			//we use the standard modelview matrix
			modelViewMat = getModelViewMatrix();

			if (s_oculus.hasLastOVRPos)
			{
				s_oculus.layer.RenderPose[renderingParams.passIndex].Position;
				OVR::Quatf q(s_oculus.layer.RenderPose[renderingParams.passIndex].Orientation);
#ifdef QT_DEBUG
				float hmdYaw, hmdPitch, hmdRoll;
				q.GetEulerAngles<OVR::Axis::Axis_Y, OVR::Axis::Axis_X, OVR::Axis::Axis_Z>(&hmdYaw, &hmdPitch, &hmdRoll);
				hmdYaw = OVR::RadToDegree(hmdYaw);
				hmdPitch = OVR::RadToDegree(hmdPitch);
				hmdRoll = OVR::RadToDegree(hmdRoll);
#endif
				OVR::Matrix4f sensorRot(q);

				ccGLMatrixd sensorMat = FromOVRMat(sensorRot);
				const ovrVector3f& P = s_oculus.layer.RenderPose[renderingParams.passIndex].Position;
				sensorMat.setTranslation(sensorMat.getTranslationAsVec3D() + CCVector3d(P.x, P.y, P.z));

				modelViewMat = sensorMat.inverse() * modelViewMat;
			}

			//projection matrix
			m_viewportParams.zNear = 0.001;
			m_viewportParams.zFar = 1000.0;
			OVR::Matrix4f proj = ovrMatrix4f_Projection(s_oculus.layer.Fov[renderingParams.passIndex],
														static_cast<float>(m_viewportParams.zNear),
														static_cast<float>(m_viewportParams.zFar),
														ovrProjection_ClipRangeOpenGL);
			projectionMat = FromOVRMat(proj);
		}
		else
#endif //CC_OCULUS_SUPPORT
		{
			//we use the standard modelview matrix
			modelViewMat = getModelViewMatrix();

			//change eye position
			double eyeOffset = renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS ? -1.0 : 1.0;

			//update the projection matrix
			projectionMat = computeProjectionMatrix
				(
				false,
				nullptr,
				&eyeOffset
				); //eyeOffset will be scaled

			//apply the eye shift
			ccGLMatrixd eyeShiftMatrix; //identity by default
			eyeShiftMatrix.setTranslation(CCVector3d(-eyeOffset, 0.0, 0.0));
			projectionMat = projectionMat * eyeShiftMatrix;
		}
	}
	else //mono vision mode
	{
		modelViewMat = getModelViewMatrix();
		projectionMat = getProjectionMatrix();
	}

	//enable clipping planes
	glFunc->glPushAttrib(GL_ENABLE_BIT);
	if (m_clippingPlanesEnabled)
	{
		if (!std::isnan(m_viewportParams.nearClippingDepth))
		{
			double equation[4] = { 0.0, 0.0, -1.0, -m_viewportParams.nearClippingDepth };
			glFunc->glClipPlane(GL_CLIP_PLANE0, equation);
			glFunc->glEnable(GL_CLIP_PLANE0);
		}

		if (!std::isnan(m_viewportParams.farClippingDepth))
		{
			double equation[4] = { 0.0, 0.0, 1.0, m_viewportParams.farClippingDepth };
			glFunc->glClipPlane(GL_CLIP_PLANE1, equation);
			glFunc->glEnable(GL_CLIP_PLANE1);
		}
	}

	//setup the projection matrix
	{
		glFunc->glMatrixMode(GL_PROJECTION);
		glFunc->glLoadMatrixd(projectionMat.data());
	}
	//setup the default view matrix
	{
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glLoadMatrixd(modelViewMat.data());
	}

	//we enable relative custom light (if activated)
	if (m_customLightEnabled)
	{
		//DGM: warning, must be enabled/displayed AFTER the 3D MV and MP matrices have been set!)
		glEnableCustomLight();

		if (!m_captureMode.enabled
			&&	m_currentLODState.level == 0
			&& (!m_stereoModeEnabled || !m_stereoParams.isAnaglyph()))
		{
			//we display it as a litle 3D star
			drawCustomLight();
		}
	}

	//we draw 3D entities
	if (m_globalDBRoot)
	{
		m_globalDBRoot->draw(CONTEXT);
	}

	if (m_winDBRoot)
	{
		m_winDBRoot->draw(CONTEXT);
	}

	glFunc->glPopAttrib(); // GL_ENABLE_BIT

	//do this before drawing the pivot!
	if (	m_autoPickPivotAtCenter
		&&	(!m_stereoModeEnabled || renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS))
	{
		CCVector3d P;
		if (getClick3DPos(glWidth() / 2, glHeight() / 2, P, !m_stereoModeEnabled)) //can't use PBO in stereo mode
		{
			renderingParams.autoPivotCandidates[renderingParams.passIndex] = P;
			renderingParams.hasAutoPivotCandidates[renderingParams.passIndex] = true;
		}
	}

	if (m_globalDBRoot && m_globalDBRoot->getChildrenNumber())
	{
		//draw pivot
		drawPivot();
	}

	//for connected items
	if (m_currentLODState.level == 0)
	{
		Q_EMIT m_signalEmitter->drawing3D();
	}

	//update LOD information
	if (renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS) //only the first pass is meaningful
														//(the second one is just a duplicate of the first)
	{
		renderingParams.nextLODState = LODState();
		if (m_currentLODState.inProgress)
		{
			if (CONTEXT.moreLODPointsAvailable || CONTEXT.higherLODLevelsAvailable)
			{
				renderingParams.nextLODState = m_currentLODState;

				if (CONTEXT.moreLODPointsAvailable)
				{
					//either we increase the start index
					//renderingParams.nextLODState.startIndex += MAX_POINT_COUNT_PER_LOD_RENDER_PASS;
				}
				else
				{
					//or the level
					renderingParams.nextLODState.level++;
					renderingParams.nextLODState.startIndex = 0;
				}
			}
			else
			{
				//no more geometry to display
			}
		}
	}

	//reset context
	CONTEXT.colorRampShader = nullptr;
	CONTEXT.customRenderingShader = nullptr;

	//we disable shader (if any)
	if (m_activeShader)
	{
		m_activeShader->release();
	}

	//we disable lights
	if (m_customLightEnabled)
	{
		glDisableCustomLight();
	}
	if (m_sunLightEnabled)
	{
		glDisableSunLight();
	}

	//we display the cross at the end (and in orthographic mode)
	if (renderingParams.draw3DCross
		&&	m_currentLODState.level == 0
		&& !m_captureMode.enabled
		&&	!m_viewportParams.perspectiveView
		&& (!renderingParams.useFBO || !m_activeGLFilter))
	{
		setStandardOrthoCenter();
		drawCross();
	}

	logGLError("ccGLWindow::draw3D");
}

void ccGLWindow::drawForeground(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	/****************************************/
	/****  PASS: 2D/FOREGROUND/NO LIGHT  ****/
	/****************************************/
	setStandardOrthoCenter();
	glFunc->glDisable(GL_DEPTH_TEST);

	CONTEXT.drawingFlags = CC_DRAW_2D | CC_DRAW_FOREGROUND;
	if (m_interactionFlags & INTERACT_TRANSFORM_ENTITIES)
	{
		CONTEXT.drawingFlags |= CC_VIRTUAL_TRANS_ENABLED;
	}

	//we draw 2D entities
	if (m_globalDBRoot)
		m_globalDBRoot->draw(CONTEXT);
	if (m_winDBRoot)
		m_winDBRoot->draw(CONTEXT);

	//current displayed scalar field color ramp (if any)
	ccRenderingTools::DrawColorRamp(CONTEXT);

	m_clickableItems.clear();

	/*** overlay entities ***/
	{
		//default overlay color
		const ccColor::Rgbub& textCol = getDisplayParameters().textDefaultCol;

		if (!m_captureMode.enabled || m_captureMode.renderOverlayItems)
		{
			//scale: only in ortho mode
			if (m_showScale && !m_viewportParams.perspectiveView)
			{
				drawScale(textCol);
			}

			if (m_showTrihedron)
			{
				//trihedron
				drawTrihedron();
			}
		}

		if (!m_captureMode.enabled)
		{
			int yStart = 0;

			//transparent border at the top of the screen
			bool showGLFilterRibbon = renderingParams.useFBO && m_activeGLFilter;
			showGLFilterRibbon &= !exclusiveFullScreen(); //we hide it in fullscreen mode!
			if (showGLFilterRibbon)
			{
				const float w = glWidth() / 2.0f;
				const float h = glHeight() / 2.0f;
				const int borderHeight = getGlFilterBannerHeight();

				glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
				glFunc->glEnable(GL_BLEND);

				glFunc->glColor4f(1.0f, 1.0f, 0.0f, 0.6f);
				glFunc->glBegin(GL_QUADS);
				glFunc->glVertex2f(w, h);
				glFunc->glVertex2f(-w, h);
				glFunc->glVertex2f(-w, h - borderHeight);
				glFunc->glVertex2f(w, h - borderHeight);
				glFunc->glEnd();

				glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT

				glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::black);
				renderText(	10,
							borderHeight - CC_GL_FILTER_BANNER_MARGIN - CC_GL_FILTER_BANNER_MARGIN / 2,
							QString("[GL filter] ") + m_activeGLFilter->getDescription(),
							static_cast<uint16_t>(RenderTextReservedIDs::GLFilterLabel)
							/*, m_font*/); //we ignore the custom font size

				yStart += borderHeight;
			}

			//current messages (if valid)
			if (!m_messagesToDisplay.empty())
			{
				glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, textCol);

				int ll_currentHeight = glHeight() - 10; //lower left
				int uc_currentHeight = 10; //upper center

				for (const auto& message : m_messagesToDisplay)
				{
					uint16_t textureID = 0;
					if (message.type != CUSTOM_MESSAGE)
					{
						textureID = static_cast<uint16_t>(RenderTextReservedIDs::StandardMessagePrefix) + static_cast<uint16_t>(message.type);
					}
						
					switch (message.position)
					{
					case LOWER_LEFT_MESSAGE:
					{
						renderText(10, ll_currentHeight, message.message, textureID, m_font);
						int messageHeight = QFontMetrics(m_font).height();
						ll_currentHeight -= (messageHeight * 5) / 4; //add a 25% margin
					}
					break;
					
					case UPPER_CENTER_MESSAGE:
					{
						QRect rect = QFontMetrics(m_font).boundingRect(message.message);
						//take the GL filter banner into account!
						int x = (glWidth() - rect.width()) / 2;
						int y = uc_currentHeight + rect.height();
						if (showGLFilterRibbon)
						{
							y += getGlFilterBannerHeight();
						}
						renderText(x, y, message.message, textureID, m_font);
						uc_currentHeight += (rect.height() * 5) / 4; //add a 25% margin
					}
					break;
					
					case SCREEN_CENTER_MESSAGE:
					{
						QFont newFont(m_font); //no need to take zoom into account!
						newFont.setPointSize(12 * devicePixelRatio());
						QRect rect = QFontMetrics(newFont).boundingRect(message.message);
						//only one message supported in the screen center (for the moment ;)
						renderText((glWidth() - rect.width()) / 2, (glHeight() - rect.height()) / 2, message.message, textureID, newFont);
					}
					break;
					}
				}
			}

			//hot-zone
			{
				drawClickableItems(0, yStart);
			}

			if (renderingParams.nextLODState.inProgress)
			{
				renderingParams.nextLODState.progressIndicator++;

				//draw LOD in progress 'icon'
				static const int lodIconSize = 32;
				static const int margin = 6;
				static const unsigned lodIconParts = 12;
				static const float lodPartsRadius = 3.0f;
				int x = margin;
				yStart += margin;

				static const float radius = lodIconSize / 2.0f - lodPartsRadius;
				static const float alpha = static_cast<float>((2 * M_PI) / lodIconParts);
				int cx = x + lodIconSize / 2 - glWidth() / 2;
				int cy = glHeight() / 2 - (yStart + lodIconSize / 2);

				glFunc->glPushAttrib(GL_POINT_BIT | GL_DEPTH_BUFFER_BIT);
				glFunc->glPointSize(lodPartsRadius);
				glFunc->glEnable(GL_POINT_SMOOTH);
				glFunc->glDisable(GL_DEPTH_TEST);

				//draw spinning circles
				glFunc->glBegin(GL_POINTS);
				for (unsigned i = 0; i < lodIconParts; ++i)
				{
					float intensity = static_cast<float>((i + renderingParams.nextLODState.progressIndicator) % lodIconParts) / (lodIconParts - 1);
					intensity /= ccColor::MAX;
					float col[3] = {	textCol.rgb[0] * intensity,
										textCol.rgb[1] * intensity,
										textCol.rgb[2] * intensity };
					glFunc->glColor3fv(col);
					glFunc->glVertex3f(cx + radius*std::cos(i*alpha), cy+radius*std::sin(i*alpha), 0);
				}
				glFunc->glEnd();

				glFunc->glPopAttrib(); //GL_POINT_BIT | GL_DEPTH_BUFFER_BIT

				yStart += lodIconSize + margin;
			}
		}
	}

	logGLError("ccGLWindow::drawForeground");
}

void ccGLWindow::dragEnterEvent(QDragEnterEvent *event)
{
	const QMimeData* mimeData = event->mimeData();

	//Display all MIME info
	//for (unsigned i=0; i<mimeData->formats().size(); ++i)
	//{
	//	QString format = mimeData->formats().at(i);
	//	ccLog::Print(QString("Drop format: %1").arg(format));
	//	if (mimeData->hasFormat("FileNameW"))
	//	{
	//		QByteArray byteData = mimeData->data(format);
	//		ccLog::Print(QString("\tdata: %1").arg(QString::fromUtf16((ushort*)byteData.data(), byteData.size() / 2)));
	//	}
	//	else
	//	{
	//		ccLog::Print(QString("\tdata: %1").arg(QString(mimeData->data(format))));
	//	}
	//}

	if (mimeData->hasFormat("text/uri-list"))
	{
		event->acceptProposedAction();
	}
}

void ccGLWindow::dropEvent(QDropEvent *event)
{
	const QMimeData* mimeData = event->mimeData();

	if (mimeData && mimeData->hasFormat("text/uri-list"))
	{
		QStringList fileNames;
		for (const QUrl &url : mimeData->urls())
		{
			QString fileName = url.toLocalFile();
			fileNames.append(fileName);
#ifdef QT_DEBUG
			ccLog::Print(QString("File dropped: %1").arg(fileName));
#endif
		}

		if (!fileNames.empty())
		{
			Q_EMIT m_signalEmitter->filesDropped(fileNames);
		}

		event->acceptProposedAction();
	}

	event->ignore();
}

void ccGLWindow::mousePressEvent(QMouseEvent *event)
{
	m_mouseMoved = false;
	m_mouseButtonPressed = true;
	m_ignoreMouseReleaseEvent = false;
	m_lastMousePos = event->pos();

	if ((event->buttons() & Qt::RightButton)
#ifdef CC_MAC_OS
		|| (QApplication::keyboardModifiers() & Qt::MetaModifier)
#endif
		)
	{
		//right click = panning (2D translation)
		if (	(m_interactionFlags & INTERACT_PAN)
			||	((QApplication::keyboardModifiers() & Qt::ControlModifier) && (m_interactionFlags & INTERACT_CTRL_PAN))
			)
		{
			setCursor(QCursor(Qt::SizeAllCursor));
		}

		if (m_interactionFlags & INTERACT_SIG_RB_CLICKED)
		{
			Q_EMIT m_signalEmitter->rightButtonClicked(event->x(), event->y());
		}
	}
	else if (event->buttons() & Qt::LeftButton)
	{
		m_lastClickTime_ticks = m_timer.elapsed(); //in msec

		//left click = rotation
		if (m_interactionFlags & INTERACT_ROTATE)
		{
			setCursor(QCursor(Qt::ClosedHandCursor));
		}

		if (m_interactionFlags & INTERACT_SIG_LB_CLICKED)
		{
			Q_EMIT m_signalEmitter->leftButtonClicked(event->x(), event->y());
		}
	}
	if (event->buttons() & Qt::MiddleButton)
	{
		//middle click = zooming
		if (m_interactionFlags & INTERACT_SIG_MB_CLICKED)
		{
			Q_EMIT m_signalEmitter->middleButtonClicked(event->x(), event->y());
		}
	}
	else
	{
		event->ignore();
	}
}

void ccGLWindow::mouseDoubleClickEvent(QMouseEvent *event)
{
	m_deferredPickingTimer.stop(); //prevent the picking process from starting
	m_ignoreMouseReleaseEvent = true;

	const int x = event->x();
	const int y = event->y();

	CCVector3d P;
	if (getClick3DPos(x, y, P, false))
	{
		setPivotPoint(P, true, true);
	}
}

void ccGLWindow::mouseMoveEvent(QMouseEvent *event)
{
//#define DEBUG_MOUSE_MOVE_FREQ
#ifdef DEBUG_MOUSE_MOVE_FREQ
	static QElapsedTimer s_timer;
	static size_t s_counter = 0;
	if (s_counter == 0)
	{
		s_timer.start();
	}
	else
	{
		qint64 elapsed_ms = s_timer.elapsed();
		if (elapsed_ms >= 1000)
		{
			ccLog::Print("mouseMoveEvent frequency: " + QString::number(s_counter / (elapsed_ms / 1000.0), 'f', 2) + QString(" Hz (mouse tracking: %1)").arg(hasMouseTracking() ? "ON" : "OFF"));
			s_timer.restart();
			s_counter = 0;
		}
	}
	++s_counter;
#endif

	const int x = event->x();
	const int y = event->y();

	if (m_interactionFlags & INTERACT_SIG_MOUSE_MOVED)
	{
		Q_EMIT m_signalEmitter->mouseMoved(x, y, event->buttons());
		event->accept();
	}

	//no button pressed
	if (event->buttons() == Qt::NoButton)
	{
		if (m_interactionFlags & INTERACT_CLICKABLE_ITEMS)
		{
			//what would be the size of the 'hot zone' if it was displayed with all options
			if (!m_hotZone)
			{
				m_hotZone = new HotZone(this);
			}
			QRect areaRect = m_hotZone->rect(true, m_bubbleViewModeEnabled, exclusiveFullScreen());

			const int retinaScale = devicePixelRatio();
			bool inZone = (	x * retinaScale * 3 < m_hotZone->topCorner.x() + areaRect.width()  * 4   //25% margin
						&&	y * retinaScale * 2 < m_hotZone->topCorner.y() + areaRect.height() * 4); //50% margin

			if (inZone != m_clickableItemsVisible)
			{
				m_clickableItemsVisible = inZone;
				redraw(true, false);
			}
			event->accept();
		}
		
		//display the 3D coordinates of the pixel below the mouse cursor (if possible)
		if (m_showCursorCoordinates)
		{
			CCVector3d P;
			QString message = QString("2D (%1 ; %2)").arg(x).arg(y);
			if (getClick3DPos(x, y, P, false))
			{
				message += QString(" --> 3D (%1 ; %2 ; %3)").arg(P.x).arg(P.y).arg(P.z);
			}
			this->displayNewMessage(message, LOWER_LEFT_MESSAGE, false, 2, SCREEN_SIZE_MESSAGE);
			redraw(true, false);
		}

		//don't need to process any further
		return;
	}

	int dx = x - m_lastMousePos.x();
	int dy = y - m_lastMousePos.y();
	setLODEnabled(true, false);

	if ((event->buttons() & Qt::RightButton)
#ifdef CC_MAC_OS
		|| (QApplication::keyboardModifiers() & Qt::MetaModifier)
#endif
		)
	{
		//right button = panning / translating
		if (m_interactionFlags & INTERACT_PAN)
		{
			//displacement vector (in "3D")
			double pixSize = computeActualPixelSize();
			CCVector3d u(dx * pixSize, -dy * pixSize, 0.0);

			const int retinaScale = devicePixelRatio();
			u *= retinaScale;

			bool entityMovingMode = (m_interactionFlags & INTERACT_TRANSFORM_ENTITIES)
				|| ((QApplication::keyboardModifiers() & Qt::ControlModifier) && m_customLightEnabled);
			if (entityMovingMode)
			{
				//apply inverse view matrix
				m_viewportParams.viewMat.transposed().applyRotation(u);

				if (m_interactionFlags & INTERACT_TRANSFORM_ENTITIES)
				{
					Q_EMIT m_signalEmitter->translation(u);
				}
				else if (m_customLightEnabled)
				{
					//update custom light position
					m_customLightPos[0] += static_cast<float>(u.x);
					m_customLightPos[1] += static_cast<float>(u.y);
					m_customLightPos[2] += static_cast<float>(u.z);
					invalidateViewport();
					deprecate3DLayer();
				}
			}
			else //camera moving mode
			{
				if (m_viewportParams.objectCenteredView)
				{
					//inverse displacement in object-based mode
					u = -u;
				}
				moveCamera(u);
			}

		} //if (m_interactionFlags & INTERACT_PAN)
	}
	else if (event->buttons() & Qt::LeftButton) //rotation
	{
		if (m_interactionFlags & INTERACT_2D_ITEMS)
		{
			//on the first time, let's check if the mouse is on a (selected) 2D item
			if (!m_mouseMoved)
			{
				if (m_pickingMode != NO_PICKING
					//DGM: in fact we still need to move labels in those modes below (see the 'Point Picking' tool of CloudCompare for instance)
					//&&	m_pickingMode != POINT_PICKING
					//&&	m_pickingMode != TRIANGLE_PICKING
					//&&	m_pickingMode != POINT_OR_TRIANGLE_PICKING
					//&&	m_pickingMode != POINT_OR_TRIANGLE_OR_LABEL_PICKING
					&& (	QApplication::keyboardModifiers() == Qt::NoModifier
						||	QApplication::keyboardModifiers() == Qt::ControlModifier) )
				{
					updateActiveItemsList(m_lastMousePos.x(), m_lastMousePos.y(), true);
				}
			}
		}
		else
		{
			assert(m_activeItems.empty());
		}

		//specific case: move active item(s)
		if (!m_activeItems.empty())
		{
			//displacement vector (in "3D")
			double pixSize = computeActualPixelSize();
			CCVector3d u(dx * pixSize, -dy * pixSize, 0.0);
			m_viewportParams.viewMat.transposed().applyRotation(u);

			const int retinaScale = devicePixelRatio();
			u *= retinaScale;

			for (auto& activeItem : m_activeItems)
			{
				if (activeItem->move2D(x * retinaScale, y * retinaScale, dx * retinaScale, dy * retinaScale, glWidth(), glHeight()))
				{
					invalidateViewport();
				}
				else if (activeItem->move3D(u))
				{
					invalidateViewport();
					deprecate3DLayer();
				}
			}
		}
		else
		{
			//specific case: rectangular polyline drawing (for rectangular area selection mode)
			if (m_allowRectangularEntityPicking
				&& (m_pickingMode == ENTITY_PICKING || m_pickingMode == ENTITY_RECT_PICKING)
				&& (m_rectPickingPoly || (QApplication::keyboardModifiers() & Qt::AltModifier)))
			{
				//first time: initialization of the rectangle
				if (!m_rectPickingPoly)
				{
					ccPointCloud* vertices = new ccPointCloud("rect.vertices");
					m_rectPickingPoly = new ccPolyline(vertices);
					m_rectPickingPoly->addChild(vertices);
					if (vertices->reserve(4) && m_rectPickingPoly->addPointIndex(0, 4))
					{
						m_rectPickingPoly->setForeground(true);
						m_rectPickingPoly->setColor(ccColor::green);
						m_rectPickingPoly->showColors(true);
						m_rectPickingPoly->set2DMode(true);
						m_rectPickingPoly->setDisplay(this);
						m_rectPickingPoly->setVisible(true);
						QPointF posA = toCenteredGLCoordinates(m_lastMousePos.x(), m_lastMousePos.y());

						CCVector3 A(static_cast<PointCoordinateType>(posA.x()),
									static_cast<PointCoordinateType>(posA.y()),
									0);
						//we add 4 times the same point (just to fill the cloud!)
						vertices->addPoint(A);
						vertices->addPoint(A);
						vertices->addPoint(A);
						vertices->addPoint(A);
						m_rectPickingPoly->setClosed(true);
						addToOwnDB(m_rectPickingPoly, false);
					}
					else
					{
						ccLog::Warning("[ccGLWindow] Failed to create seleciton polyline! Not enough memory!");
						delete m_rectPickingPoly;
						m_rectPickingPoly = nullptr;
						vertices = nullptr;
					}
				}

				if (m_rectPickingPoly)
				{
					CCCoreLib::GenericIndexedCloudPersist* vertices = m_rectPickingPoly->getAssociatedCloud();
					assert(vertices);
					CCVector3* B = const_cast<CCVector3*>(vertices->getPointPersistentPtr(1));
					CCVector3* C = const_cast<CCVector3*>(vertices->getPointPersistentPtr(2));
					CCVector3* D = const_cast<CCVector3*>(vertices->getPointPersistentPtr(3));
					QPointF posD = toCenteredGLCoordinates(event->x(), event->y());
					B->x = C->x = static_cast<PointCoordinateType>(posD.x());
					C->y = D->y = static_cast<PointCoordinateType>(posD.y());
				}
			}
			else if (m_interactionFlags & INTERACT_ROTATE) //standard rotation around the current pivot
			{
				//choose the right rotation mode
				enum RotationMode { StandardMode, BubbleViewMode, LockedAxisMode };
				RotationMode rotationMode = StandardMode;
				if ((m_interactionFlags & INTERACT_TRANSFORM_ENTITIES) != INTERACT_TRANSFORM_ENTITIES)
				{
					if (m_bubbleViewModeEnabled)
						rotationMode = BubbleViewMode;
					else if (m_rotationAxisLocked)
						rotationMode = LockedAxisMode;
				}

				ccGLMatrixd rotMat;
				switch (rotationMode)
				{
				case BubbleViewMode:
				{
					QPoint posDelta = m_lastMousePos - event->pos();

					if (std::abs(posDelta.x()) != 0)
					{
						double delta_deg = (posDelta.x() * static_cast<double>(m_bubbleViewFov_deg)) / height();
						//rotation about the sensor Z axis
						CCVector3d axis = m_viewportParams.viewMat.getColumnAsVec3D(2);
						rotMat.initFromParameters( CCCoreLib::DegreesToRadians( delta_deg ), axis, CCVector3d(0, 0, 0) );
					}

					if (std::abs(posDelta.y()) != 0)
					{
						double delta_deg = (posDelta.y() * static_cast<double>(m_bubbleViewFov_deg)) / height();
						//rotation about the local X axis
						ccGLMatrixd rotX;
						rotX.initFromParameters( CCCoreLib::DegreesToRadians( delta_deg ), CCVector3d(1, 0, 0), CCVector3d(0, 0, 0) );
						rotMat = rotX * rotMat;
					}
				}
				break;

				case StandardMode:
				{
					static CCVector3d s_lastMouseOrientation;
					CCVector3d currentMouseOrientation = convertMousePositionToOrientation(x, y);

					if (QApplication::keyboardModifiers() & Qt::ShiftModifier)
					{
						//rotate around the current viewport (roll camera)
						double angle_rad = 2.0 * M_PI * dx/width();
						rotMat.initFromParameters(angle_rad, CCVector3d(0, 0, 1), CCVector3d(0, 0, 0));
					}
					else
					{
						if (!m_mouseMoved)
						{
							//on the first time, we must compute the previous orientation (the camera hasn't moved yet)
							s_lastMouseOrientation = convertMousePositionToOrientation(m_lastMousePos.x(), m_lastMousePos.y());
						}
						// unconstrained rotation following mouse position
						rotMat = ccGLMatrixd::FromToRotation(s_lastMouseOrientation, currentMouseOrientation);
					}

					s_lastMouseOrientation = currentMouseOrientation;
				}
				break;

				case LockedAxisMode:
				{
					//apply rotation about the locked axis
					CCVector3d axis = m_lockedRotationAxis;
					getBaseViewMat().applyRotation(axis);


					//determine whether we are in a side or top view
					bool topView = (std::abs(axis.z) > 0.5);

					//m_viewportParams.objectCenteredView
					ccGLCameraParameters camera;
					getGLCameraParameters(camera);

					if (topView)
					{
						//rotation origin
						CCVector3d C2D;
						if (m_viewportParams.objectCenteredView)
						{
							//project the current pivot point on screen
							camera.project(m_viewportParams.getPivotPoint(), C2D);
							C2D.z = 0.0;
						}
						else
						{
							C2D = CCVector3d(width() / 2.0, height() / 2.0, 0.0);
						}

						CCVector3d previousMousePos(static_cast<double>(m_lastMousePos.x()), static_cast<double>(height() - m_lastMousePos.y()), 0.0);
						CCVector3d currentMousePos(static_cast<double>(x), static_cast<double>(height() - y), 0.0);

						CCVector3d a = (currentMousePos - C2D);
						CCVector3d b = (previousMousePos - C2D);
						CCVector3d u = a * b;
						double u_norm = std::abs(u.z); //a and b are in the XY plane
						if (u_norm > 1.0e-6)
						{
							double sin_angle = u_norm / (a.norm() * b.norm());

							//determine the rotation direction
							if (u.z * m_lockedRotationAxis.z > 0)
							{
								sin_angle = -sin_angle;
							}

							double angle_rad = asin(sin_angle); //in [-pi/2 ; pi/2]
							rotMat.initFromParameters(angle_rad, axis, CCVector3d(0, 0, 0));
						}
					}
					else //side view
					{
						//project the current pivot point on screen
						CCVector3d A2D;
						CCVector3d B2D;
						if (	camera.project(m_viewportParams.getPivotPoint(), A2D)
							&&	camera.project(m_viewportParams.getPivotPoint() + m_viewportParams.zFar * m_lockedRotationAxis, B2D))
						{
							CCVector3d lockedRotationAxis2D = B2D - A2D;
							lockedRotationAxis2D.z = 0; //just in case
							lockedRotationAxis2D.normalize();

							CCVector3d mouseShift(static_cast<double>(dx), -static_cast<double>(dy), 0.0);
							mouseShift -= mouseShift.dot(lockedRotationAxis2D) * lockedRotationAxis2D; //we only keep the orthogonal part
							double angle_rad = 2.0 * M_PI * mouseShift.norm() / (width() + height());
							if ((lockedRotationAxis2D * mouseShift).z > 0.0)
							{
								angle_rad = -angle_rad;
							}

							rotMat.initFromParameters(angle_rad, axis, CCVector3d(0, 0, 0));
						}
					}
				}
				break;

				default:
					assert(false);
					break;
				}

				if (m_interactionFlags & INTERACT_TRANSFORM_ENTITIES)
				{
					rotMat = m_viewportParams.viewMat.transposed() * rotMat * m_viewportParams.viewMat;

					//feedback for 'interactive transformation' mode
					Q_EMIT m_signalEmitter->rotation(rotMat);
				}
				else
				{
					rotateBaseViewMat(rotMat);

					showPivotSymbol(true);

					//feedback for 'echo' mode
					Q_EMIT m_signalEmitter->viewMatRotated(rotMat);
				}
			}
		}
	}
	else if ((event->buttons() & Qt::MiddleButton)) // zoom
	{
		//middle button = zooming
		float pseudo_wheelDelta_deg = static_cast<float>(-dy);
		onWheelEvent(pseudo_wheelDelta_deg);

		Q_EMIT m_signalEmitter->mouseWheelRotated(pseudo_wheelDelta_deg);
	}

	m_mouseMoved = true;
	m_lastMousePos = event->pos();

	event->accept();

	if (m_interactionFlags != INTERACT_TRANSFORM_ENTITIES)
	{
		redraw(true);
	}
}

void ccGLWindow::mouseReleaseEvent(QMouseEvent *event)
{
	if (m_ignoreMouseReleaseEvent)
	{
		m_ignoreMouseReleaseEvent = false;
		return;
	}
	
	bool mouseHasMoved = m_mouseMoved;
	//setLODEnabled(false, false); //DGM: why?

	//reset to default state
	m_mouseButtonPressed = false;
	m_mouseMoved = false;
	setCursor(m_defaultCursorShape);

	if (m_interactionFlags & INTERACT_SIG_BUTTON_RELEASED)
	{
		event->accept();
		Q_EMIT m_signalEmitter->buttonReleased();
	}

	if (m_pivotSymbolShown)
	{
		if (m_pivotVisibility == PIVOT_SHOW_ON_MOVE)
		{
			toBeRefreshed();
		}
		showPivotSymbol(m_pivotVisibility == PIVOT_ALWAYS_SHOW);
	}

	if ((event->button() == Qt::RightButton)
#ifdef CC_MAC_OS
		|| (QApplication::keyboardModifiers () & Qt::MetaModifier)
#endif
		)
	{
		if (mouseHasMoved)
		{
			event->accept();
			toBeRefreshed();
		}
		else if (m_interactionFlags & INTERACT_2D_ITEMS)
		{
			//interaction with 2D item(s)
			updateActiveItemsList(event->x(), event->y(), false);
			if (!m_activeItems.empty())
			{
				ccInteractor* item = *m_activeItems.begin();
				m_activeItems.clear();
				if (item->acceptClick(event->x(), height() - 1 - event->y(), Qt::RightButton))
				{
					event->accept();
					toBeRefreshed();
				}
			}
		}
	}
	else if (event->button() == Qt::LeftButton)
	{
		if (mouseHasMoved)
		{
			//if a rectangular picking area has been defined
			if (m_rectPickingPoly)
			{
				CCCoreLib::GenericIndexedCloudPersist* vertices = m_rectPickingPoly->getAssociatedCloud();
				assert(vertices);
				const CCVector3* A = vertices->getPointPersistentPtr(0);
				const CCVector3* C = vertices->getPointPersistentPtr(2);

				int pickX = static_cast<int>(A->x + C->x) / 2;
				int pickY = static_cast<int>(A->y + C->y) / 2;
				int pickW = static_cast<int>(std::abs(C->x - A->x));
				int pickH = static_cast<int>(std::abs(C->y - A->y));

				removeFromOwnDB(m_rectPickingPoly);
				m_rectPickingPoly = nullptr;
				vertices = nullptr;

				PickingParameters params(ENTITY_RECT_PICKING, pickX + width() / 2, height() / 2 - pickY, pickW, pickH);
				startPicking(params);
			}

			event->accept();
			toBeRefreshed();
		}
		else
		{
			//picking?
			if (m_timer.elapsed() < m_lastClickTime_ticks + CC_MAX_PICKING_CLICK_DURATION_MS) //in msec
			{
				int x = m_lastMousePos.x();
				int y = m_lastMousePos.y();

				//first test if the user has clicked on a particular item on the screen
				if (!processClickableItems(x, y))
				{
					m_lastMousePos = event->pos(); //just in case (it should be already at this position)
					const ccGui::ParamStruct& displayParams = getDisplayParameters();
					if (displayParams.singleClickPicking)
					{
						m_deferredPickingTimer.start();
					}
					//doPicking();
				}
			}
		}

		m_activeItems.clear();
	}
	else if (event->button() == Qt::MiddleButton)
	{
		if (mouseHasMoved)
		{
			event->accept();
			toBeRefreshed();
		}
	}

	refresh(false);
}

void ccGLWindow::wheelEvent(QWheelEvent* event)
{
	bool doRedraw = false;

	Qt::KeyboardModifiers keyboardModifiers = QApplication::keyboardModifiers();
	if (keyboardModifiers & Qt::AltModifier)
	{
		event->accept();

		//same shortcut as Meshlab: change the point size
		float sizeModifier = (event->delta() < 0 ? -1.0f : 1.0f);
		setPointSize(m_viewportParams.defaultPointSize + sizeModifier);

		doRedraw = true;
	}
	else if (keyboardModifiers & Qt::ControlModifier)
	{
		event->accept();

		//same shortcut as Meshlab: change the zNear or zFar clipping planes
		double increment = (event->delta() < 0 ? -1.0 : 1.0) * computeDefaultIncrement();
		bool shiftPressed = (keyboardModifiers & Qt::ShiftModifier);
		if (shiftPressed)
		{
			//if ((increment < 0.0) || (!std::isnan(m_viewportParams.farClippingDepth)))
			{
				double farClippingDepth = (std::isnan(m_viewportParams.farClippingDepth) ? m_viewportParams.zFar : m_viewportParams.farClippingDepth);
				if (setFarClippingPlaneDepth(std::max(0.0, farClippingDepth + increment)))
				{
					doRedraw = true;
				}
			}
		}
		else
		{
			if ((increment > 0.0) || (!std::isnan(m_viewportParams.nearClippingDepth)))
			{
				double nearClippingDepth = (std::isnan(m_viewportParams.nearClippingDepth) ? m_viewportParams.zNear : m_viewportParams.nearClippingDepth);
				if (setNearClippingPlaneDepth(std::max(0.0, nearClippingDepth + increment)))
				{
					doRedraw = true;
				}
			}
		}
	}
	else if (keyboardModifiers & Qt::ShiftModifier)
	{
		event->accept();
		
		//same shortcut as Meshlab: change the fov value
		float newFOV = (getFov() + (event->delta() < 0 ? -1.0f : 1.0f));
		newFOV = std::min(std::max(1.0f, newFOV), 180.0f);
		if (newFOV != getFov())
		{
			setFov(newFOV);
			doRedraw = true;
		}
	}
	else if (m_interactionFlags & INTERACT_ZOOM_CAMERA)
	{
		event->accept();

		//see QWheelEvent documentation ("distance that the wheel is rotated, in eighths of a degree")
		float wheelDelta_deg = event->delta() / 8.0f;
		onWheelEvent(wheelDelta_deg);

		Q_EMIT m_signalEmitter->mouseWheelRotated(wheelDelta_deg);

		doRedraw = true;
	}

	if (doRedraw)
	{
		setLODEnabled(true, true);
		m_currentLODState.level = 0;

		redraw();
	}
}

//draw a unit circle in a given plane (0=YZ, 1 = XZ, 2=XY) 
static void glDrawUnitCircle(QOpenGLContext* context, unsigned char dim, unsigned steps = 64)
{
	assert(context);
	QOpenGLFunctions_2_1* glFunc = context->versionFunctions<QOpenGLFunctions_2_1>();
	if (!glFunc)
	{
		return;
	}

	double thetaStep = (2.0 * M_PI) / steps;
	unsigned char dimX = (dim < 2 ? dim + 1 : 0);
	unsigned char dimY = (dimX < 2 ? dimX + 1 : 0);

	CCVector3d P(0, 0, 0);

	glFunc->glBegin(GL_LINE_LOOP);
	for (unsigned i = 0; i < steps; ++i)
	{
		double theta = thetaStep * i;
		P.u[dimX] = std::cos(theta);
		P.u[dimY] = std::sin(theta);
		glFunc->glVertex3dv(P.u);
	}
	glFunc->glEnd();
}

void ccGLWindow::drawPivot()
{
	if (	!m_viewportParams.objectCenteredView
		||	(m_pivotVisibility == PIVOT_HIDE)
		||	(m_pivotVisibility == PIVOT_SHOW_ON_MOVE && !m_pivotSymbolShown))
	{
		return;
	}

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	//place origin on pivot point
	const CCVector3d& pivotPoint = m_viewportParams.getPivotPoint();
	glFunc->glTranslated(pivotPoint.x, pivotPoint.y, pivotPoint.z);

	//compute actual symbol radius
	double symbolRadius = CC_DISPLAYED_PIVOT_RADIUS_PERCENT * std::min(glWidth(), glHeight()) / 2.0;

	if (m_pivotGLList == GL_INVALID_LIST_ID)
	{
		m_pivotGLList = glFunc->glGenLists(1);
		glFunc->glNewList(m_pivotGLList, GL_COMPILE);

		//draw a small sphere
		{
			ccSphere sphere(static_cast<PointCoordinateType>(10.0 / symbolRadius));
			sphere.setColor(ccColor::yellow);
			sphere.showColors(true);
			sphere.setVisible(true);
			sphere.setEnabled(true);
			//force lighting for proper sphere display
			glFunc->glPushAttrib(GL_LIGHTING_BIT);
			glEnableSunLight();
			CC_DRAW_CONTEXT CONTEXT;
			getContext(CONTEXT);
			CONTEXT.drawingFlags = CC_DRAW_3D | CC_DRAW_FOREGROUND | CC_LIGHT_ENABLED;
			CONTEXT.display = nullptr;
			sphere.draw(CONTEXT);
			glFunc->glPopAttrib(); //GL_LIGHTING_BIT
		}

		//draw 3 circles
		glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LINE_BIT);
		glFunc->glEnable(GL_BLEND);
		glFunc->glLineWidth(2.0f);

		//default transparency
		const ColorCompType c_alpha = static_cast<ColorCompType>(ccColor::MAX * 0.6f);

		//pivot symbol: 3 circles
		static const ccColor::Rgba RedAlpha(ccColor::redRGB, c_alpha);
		ccGL::Color(glFunc, RedAlpha);
		glDrawUnitCircle(context(), 0);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(-1.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(1.0f, 0.0f, 0.0f);
		glFunc->glEnd();

		static const ccColor::Rgba GreenAlpha(ccColor::greenRGB, c_alpha);
		ccGL::Color(glFunc, GreenAlpha);
		glDrawUnitCircle(context(), 1);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(0.0f, -1.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 1.0f, 0.0f);
		glFunc->glEnd();

		static const ccColor::Rgba BlueCCAlpha(ccColor::blueCCRGB, c_alpha);
		ccGL::Color(glFunc, BlueCCAlpha);
		glDrawUnitCircle(context(), 2);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(0.0f, 0.0f, -1.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 1.0f);
		glFunc->glEnd();

		glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT | GL_LINE_BIT

		glFunc->glEndList();
	}

	//constant scale
	const double scale = symbolRadius * computeActualPixelSize();
	glFunc->glScaled(scale, scale, scale);

	glFunc->glCallList(m_pivotGLList);

	glFunc->glPopMatrix();
}

QImage ccGLWindow::renderToImage(	float zoomFactor/*=1.0f*/,
									bool dontScaleFeatures/*=false*/,
									bool renderOverlayItems/*=false*/,
									bool silent/*=false*/)
{
	QImage outputImage;

	if (!m_glExtFuncSupported) //no FBO support?!
	{
#ifdef CC_GL_WINDOW_USE_QWINDOW
		if (!silent)
		{
			ccLog::Error("Direct screen capture without FBO is not supported anymore!");
			return QImage();
		}
#else
		//if no shader or fbo --> we grab the screen directly
		if (m_activeShader)
		{
			if (!silent)
				ccLog::Error("Direct screen capture with shader is not supported!");
		}
		else
		{
			outputImage = grabFramebuffer();
			if (outputImage.isNull())
			{
				if (!silent)
					ccLog::Error("Direct screen capture failed! (not enough memory?)");
			}
		}
		return outputImage;
#endif
	}

	//otherwise FBOs are supported
	if (!silent)
	{
		ccLog::Print("[Render screen via FBO]");
	}

	makeCurrent();

	//current window size (in pixels)
	int Wp = static_cast<int>(width() * zoomFactor);
	int Hp = static_cast<int>(height() * zoomFactor);

	if (zoomFactor != 1.0f)
	{
		setGLViewport(0, 0, Wp, Hp); //warning: this will modify m_glViewport
	}

	//try to reserve memory for the output image
	outputImage = QImage(m_glViewport.size(), QImage::Format_ARGB32);
	GLubyte* data = outputImage.bits();
	if (!data)
	{
		//failure :(
		if (!silent)
		{
			ccLog::Error("Not enough memory!");
		}
		if (zoomFactor != 1.0f)
		{
			setGLViewport(0, 0, width(), height()); //restore m_glViewport
		}
		return QImage();
	}

	//we activate 'capture' mode
	m_captureMode.enabled = true;
	m_captureMode.zoomFactor = zoomFactor;
	m_captureMode.renderOverlayItems = renderOverlayItems;

	//current viewport parameters backup
	float _defaultPointSize = m_viewportParams.defaultPointSize;
	float _defaultLineWidth = m_viewportParams.defaultLineWidth;

	if (!dontScaleFeatures)
	{
		//we update point size (for point clouds)
		setPointSize(_defaultPointSize * zoomFactor, true);
		//we update line width (for bounding-boxes, etc.)
		setLineWidth(_defaultLineWidth * zoomFactor);
		//we update font size (for text display)
		setFontPointSize(getFontPointSize());
	}

	ccFrameBufferObject* fbo = nullptr;
	ccGlFilter* glFilter = nullptr;
	if (m_fbo && zoomFactor == 1.0f)
	{
		//we can use the existing FBO
		fbo = m_fbo;
		//and the existing GL filter
		glFilter = m_activeGLFilter;
	}
	else
	{
		//otherwise we create a new temporary one
		fbo = new ccFrameBufferObject();

		bool success = (	fbo->init(glWidth(), glHeight())
						&&	fbo->initColor()
						&&	fbo->initDepth());
		if (!success)
		{
			delete fbo;
			fbo = nullptr;

			if (!silent)
			{
				ccLog::Error("[FBO] Initialization failed! (not enough memory?)");
			}
			if (zoomFactor != 1.0f)
			{
				setGLViewport(0, 0, width(), height()); //restore m_glViewport
			}
			return QImage();
		}

		//and we change the current GL filter size (temporarily)
		if (m_activeGLFilter)
		{
			QString error;
			if (!m_activeGLFilter->init(glWidth(), glHeight(), GetShaderPath(), error))
			{
				if (!silent)
				{
					ccLog::Warning(QString("[GL Filter] GL filter can't be used for rendering: %1").arg(error));
				}
			}
			else
			{
				glFilter = m_activeGLFilter;
			}
		}
	}
	assert(fbo);

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	CC_DRAW_CONTEXT CONTEXT;
	getContext(CONTEXT);
	CONTEXT.renderZoom = zoomFactor;

	//just to be sure
	stopLODCycle();

	RenderingParams renderingParams;
	renderingParams.drawForeground = false;
	renderingParams.useFBO = false; //DGM: make sure that no FBO is used internally!
	bool stereoModeWasEnabled = m_stereoModeEnabled;
	m_stereoModeEnabled = false;

	//disable LOD!
	bool wasLODEnabled = isLODEnabled();
	setLODEnabled(false);

	//enable the FBO
	bindFBO(fbo);
	logGLError("ccGLWindow::renderToFile/FBO start");

	fullRenderingPass(CONTEXT, renderingParams);

	//disable the FBO
	logGLError("ccGLWindow::renderToFile/FBO stop");
	bindFBO(nullptr);

	setLODEnabled(wasLODEnabled);

	m_stereoModeEnabled = stereoModeWasEnabled;

	CONTEXT.drawingFlags = CC_DRAW_2D | CC_DRAW_FOREGROUND;
	if (m_interactionFlags == INTERACT_TRANSFORM_ENTITIES)
	{
		CONTEXT.drawingFlags |= CC_VIRTUAL_TRANS_ENABLED;
	}

	glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
	glFunc->glDisable(GL_DEPTH_TEST);

	if (glFilter)
	{
		//we process GL filter
		GLuint depthTex = fbo->getDepthTexture();
		GLuint colorTex = fbo->getColorTexture();
		//minimal set of viewport parameters necessary for GL filters
		ccGlFilter::ViewportParameters parameters;
		{
			parameters.perspectiveMode = m_viewportParams.perspectiveView;
			parameters.zFar = m_viewportParams.zFar;
			parameters.zNear = m_viewportParams.zNear;
			parameters.zoomFactor = zoomFactor;
		}
		//apply shader
		glFilter->shade(depthTex, colorTex, parameters);
		logGLError("ccGLWindow::renderToFile/glFilter shade");

		//in render mode we only want to capture it, not to display it
		bindFBO(fbo);

		setStandardOrthoCorner();
		ccGLUtils::DisplayTexture2DPosition(glFilter->getTexture(), 0, 0, CONTEXT.glW, CONTEXT.glH);

		bindFBO(nullptr);
	}

	bindFBO(fbo);
	setStandardOrthoCenter();

	//we draw 2D entities (mainly for the color ramp!)
	if (m_globalDBRoot)
		m_globalDBRoot->draw(CONTEXT);
	if (m_winDBRoot)
		m_winDBRoot->draw(CONTEXT);

	//current displayed scalar field color ramp (if any)
	ccRenderingTools::DrawColorRamp(CONTEXT);

	if (m_captureMode.renderOverlayItems)
	{
		//scale: only in ortho mode
		if (!m_viewportParams.perspectiveView)
		{
			//DGM FIXME: with a zoom > 1, the renderText call inside drawScale will result in the wrong FBO being used?!
			drawScale(getDisplayParameters().textDefaultCol);
		}

		if (m_showTrihedron)
		{
			//trihedron
			drawTrihedron();
		}
	}

	glFunc->glFlush();

	//read from fbo
	glFunc->glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	//to avoid memory issues, we read line by line
	for (int i = 0; i < glHeight(); ++i)
	{
		glFunc->glReadPixels(0, i, glWidth(), 1, GL_BGRA, GL_UNSIGNED_BYTE, data + (glHeight() - 1 - i) * glWidth() * 4);
	}
	glFunc->glReadBuffer(GL_NONE);

	//restore the default FBO
	bindFBO(nullptr);

	glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT

	logGLError("ccGLWindow::renderToFile");

	if (m_fbo != fbo)
	{
		delete fbo;
	}
	fbo = nullptr;

	if (zoomFactor != 1.0f)
	{
		setGLViewport(0, 0, width(), height()); //restore m_glViewport
	}

	if (glFilter && zoomFactor != 1.0f)
	{
		QString error;
		m_activeGLFilter->init(glWidth(), glHeight(), GetShaderPath(), error);
	}

	//we restore viewport parameters
	setPointSize(_defaultPointSize, true);
	setLineWidth(_defaultLineWidth);
	m_captureMode.enabled = false;
	m_captureMode.zoomFactor = 1.0f;
	setFontPointSize(getFontPointSize());

	invalidateViewport();
	invalidateVisualization();
	redraw(true);

	return outputImage;
}

bool ccGLWindow::initFBO(int w, int h)
{
	makeCurrent();

	if (!initFBOSafe(m_fbo, w, h))
	{
		ccLog::Warning("[FBO] Initialization failed!");
		m_alwaysUseFBO = false;
		removeFBOSafe(m_fbo2);
		setLODEnabled(false, false);
		return false;
	}

	if (!m_stereoModeEnabled || (m_stereoParams.glassType != StereoParams::NVIDIA_VISION && m_stereoParams.glassType != StereoParams::GENERIC_STEREO_DISPLAY))
	{
		//we don't need it anymore
		if (m_fbo2)
		{
			removeFBOSafe(m_fbo2);
		}
	}
	else
	{
		if (!initFBOSafe(m_fbo2, w, h))
		{
			ccLog::Warning("[FBO] Failed to initialize secondary FBO!");
			m_alwaysUseFBO = false;
			removeFBOSafe(m_fbo);
			setLODEnabled(false, false);
			return false;
		}
	}

	deprecate3DLayer();
	return true;
}

bool ccGLWindow::enableStereoMode(const StereoParams& params)
{
	bool needSecondFBO = false;
	bool needAutoRefresh = false;
	
	if (params.glassType == StereoParams::OCULUS)
	{
#ifdef CC_OCULUS_SUPPORT
#ifdef CC_GL_WINDOW_USE_QWINDOW
		if (!s_oculus.session)
		{
			// Example use of ovr_Initialize() to specify a log callback.
			// The log callback can be called from other threads until ovr_Shutdown() completes.
			ovrInitParams params = {0, 0, nullptr, 0, 0, OVR_ON64("")};
			params.LogCallback = LogCallback;
			ovrResult result = ovr_Initialize(nullptr);
			if (OVR_FAILURE(result))
			{
				QMessageBox::critical(asWidget(), "Oculus", "Failed to initialize the Oculus SDK (ovr_Initialize)");
				return false;
			}

			ovrGraphicsLuid luid;
			ovrSession session;
			result = ovr_Create(&session, &luid);
			if (OVR_FAILURE(result))
			{
				QMessageBox::critical(asWidget(), "Oculus", "Failed to initialize the Oculus SDK (ovr_Create)");
				ovr_Shutdown();
				return false;
			}

			//get device description
			ovrHmdDesc desc = ovr_GetHmdDesc(s_oculus.session);
			ccLog::Print(QString("[Oculus] HMD '%0' detected (resolution: %1 x %2)").arg(desc.ProductName).arg(desc.Resolution.w).arg(desc.Resolution.h));

			s_oculus.setSesion(session);
			assert(s_oculus.session);
		}

		if (!s_oculus.initTextureSet(context()))
		{
			QMessageBox::critical(asWidget(), "Oculus", "Failed to initialize the swap texture set (ovr_CreateSwapTextureSetGL)");
			s_oculus.stop(true);
			return false;
		}

		if (m_glExtFuncSupported)
		{
			s_oculus.initMirrorTexture(width(), height(), m_glExtFunc);
		}

		//configure tracking
		{
			//No longer necessary
			//ovr_ConfigureTracking(	s_oculus.session,
			//						/*requested = */ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position,
			//						/*required  = */ovrTrackingCap_Orientation );

			//reset tracking
			s_oculus.hasLastOVRPos = false;
			ovr_RecenterTrackingOrigin(s_oculus.session);
		}

		displayNewMessage("Look into your headset", ccGLWindow::SCREEN_CENTER_MESSAGE, false, 3600);
		//force the screen update before we freeze it! (see paintGL)
		update();

		needSecondFBO = false;
		needAutoRefresh = true;

#else //no CC_GL_WINDOW_USE_QWINDOW
		QMessageBox::critical(asWidget(), "Oculus", "The Oculus device is not supported by this version\n(use the 'Stereo' version)");
		return false;
#endif //no CC_GL_WINDOW_USE_QWINDOW

#else //no CC_OCULUS_SUPPORT

		QMessageBox::critical(asWidget(), "Oculus", "The Oculus device is not supported by this version\n(use the 'Stereo' version)");
		return false;

#endif //no CC_OCULUS_SUPPORT
	}
	else if (params.glassType == StereoParams::NVIDIA_VISION || params.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
	{
		if (	!format().stereo()
			||	format().swapBehavior() != QSurfaceFormat::DoubleBuffer )
			
		{
			QMessageBox::critical(asWidget(), "Stereo", "Quad buffering not supported!");
			return false;
		}

		if (m_initialized)
		{
			GLboolean isStereoEnabled = 0;
			functions()->glGetBooleanv(GL_STEREO, &isStereoEnabled);
			if (isStereoEnabled == 0)
			{
				QMessageBox::critical(asWidget(), "Stereo", "OpenGL stereo mode not supported/enabled!");
				return false;
			}
		}

		if (!exclusiveFullScreen())
		{
			ccLog::Warning("3D window should be in exclusive full screen mode!");
			return false;
		}

		needSecondFBO = true;
		needAutoRefresh = false;
	}

	m_stereoParams = params;
	m_stereoModeEnabled = true;

	//In some cases we must init the secondary FBO
	if (needSecondFBO && !initFBO(width(), height()))
	{
		//well, we only lose the LOD mechanism :(
	}

	//auto-save last glass type
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		settings.setValue(c_ps_stereoGlassType, m_stereoParams.glassType);
		settings.endGroup();
	}

	if (needAutoRefresh)
	{
		toggleAutoRefresh(true);
	}

	return true;
}

void ccGLWindow::disableStereoMode()
{
	if (m_stereoModeEnabled)
	{
		if (m_stereoParams.glassType == StereoParams::OCULUS)
		{
			toggleAutoRefresh(false);
			displayNewMessage(QString(), ccGLWindow::SCREEN_CENTER_MESSAGE, false);

#ifdef CC_OCULUS_SUPPORT
			if (s_oculus.session)
			{
				if (m_glExtFuncSupported)
				{
					s_oculus.releaseMirrorTexture(m_glExtFunc);
				}

				s_oculus.stop(false);
			}
#endif
		}
	}

	m_stereoModeEnabled = false;

	if (m_fbo2)
	{
		//we don't need it anymore
		removeFBOSafe(m_fbo2);
	}
}

void ccGLWindow::toggleExclusiveFullScreen(bool state)
{
	QWidget* widget = asWidget();

	if (state)
	{
		//we are currently in normal screen mode
		if (!m_exclusiveFullscreen)
		{
			if (widget)
			{
				m_formerGeometry = widget->saveGeometry();
				m_formerParent = widget->parentWidget();
				if (m_formerParent && m_formerParent->layout())
				{
					m_formerParent->layout()->removeWidget(widget);
				}
				widget->setParent(nullptr);
			}

			m_exclusiveFullscreen = true;
			if (widget)
				widget->showFullScreen();
			else
				showFullScreen();
			displayNewMessage("Press F11 to disable full-screen mode", ccGLWindow::UPPER_CENTER_MESSAGE, false, 30, FULL_SCREEN_MESSAGE);
		}
	}
	else
	{
		if (m_stereoModeEnabled && m_stereoParams.glassType == StereoParams::NVIDIA_VISION)
		{
			//auto disable stereo mode (DGM: otherwise the application may crash!)
			disableStereoMode();
		}

		//if we are currently in full-screen mode
		if (m_exclusiveFullscreen)
		{
			if (m_formerParent && widget)
			{
				if (m_formerParent->layout())
				{
					m_formerParent->layout()->addWidget(widget);
				}
				else
				{
					widget->setParent(m_formerParent);
				}
				m_formerParent = nullptr;
			}
			m_exclusiveFullscreen = false;

			displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE, false, 0, FULL_SCREEN_MESSAGE); //remove any message
			if (widget)
			{
				widget->showNormal();
				if (!m_formerGeometry.isNull())
				{
					widget->restoreGeometry(m_formerGeometry);
					m_formerGeometry.clear();
				}
			}
			else
			{
				showNormal();
			}
		}
	}

	QCoreApplication::processEvents();
	if (widget)
	{
		widget->setFocus();
	}
	redraw();

	Q_EMIT m_signalEmitter->exclusiveFullScreenToggled(state);
}

void ccGLWindow::onItemPickedFast(ccHObject* pickedEntity, int pickedItemIndex, int x, int y)
{
	if (pickedEntity)
	{
		if (pickedEntity->isA(CC_TYPES::LABEL_2D))
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(pickedEntity);
			m_activeItems.insert(label);
		}
		else if (pickedEntity->isA(CC_TYPES::CLIPPING_BOX_PART))
		{
			ccClipBoxPart* cBoxPart = static_cast<ccClipBoxPart*>(pickedEntity);
			ccClipBox* cbox = cBoxPart->clipBox();
			assert(cbox);
			cbox->setActiveComponent(cBoxPart->partID());
			cbox->setClickedPoint(x, y, width(), height(), m_viewportParams.viewMat);

			m_activeItems.insert(cbox);
		}
	}

	Q_EMIT m_signalEmitter->fastPickingFinished();
}

void ccGLWindow::checkScheduledRedraw()
{
	if (m_scheduledFullRedrawTime && m_timer.elapsed() > m_scheduledFullRedrawTime)
	{
		redraw();
	}
}

void ccGLWindow::doPicking()
{
	int x = m_lastMousePos.x();
	int y = m_lastMousePos.y();

	if (x < 0 || y < 0 || x > width() || y > height())
	{
		// we can ignore clicks outside of the window
		return;
	}

	if ((m_pickingMode != NO_PICKING)
		|| (m_interactionFlags & INTERACT_2D_ITEMS))
	{
		if (m_interactionFlags & INTERACT_2D_ITEMS)
		{
			//label selection
			updateActiveItemsList(x, y, false);
			if (!m_activeItems.empty())
			{
				if (m_activeItems.size() == 1)
				{
					ccInteractor* pickedObj = *m_activeItems.begin();
					cc2DLabel* label = dynamic_cast<cc2DLabel*>(pickedObj);
					if (label && !label->isSelected())
					{
						Q_EMIT m_signalEmitter->entitySelectionChanged(label);
						QApplication::processEvents();
					}
				}

				//interaction with item(s) such as labels, etc.
				//DGM TODO: to activate only if some items take left clicks into account!
				//for (std::set<ccInteractor*>::iterator it=m_activeItems.begin(); it!=m_activeItems.end(); ++it)
				//if ((*it)->acceptClick(x,y,Qt::LeftButton))
				//{
				//	event->accept();
				//	redraw();
				//	return;
				//}
			}
		}
		else
		{
			assert(m_activeItems.empty());
		}

		if (m_activeItems.empty() && m_pickingMode != NO_PICKING)
		{
			//perform standard picking
			PICKING_MODE pickingMode = m_pickingMode;

			//shift+click = point/triangle picking
			if (pickingMode == ENTITY_PICKING && (QApplication::keyboardModifiers() & Qt::ShiftModifier))
			{
				pickingMode = LABEL_PICKING;
			}

			PickingParameters params(pickingMode, x, y, m_pickRadius, m_pickRadius);
			startPicking(params);
		}
	}
}

void ccGLWindow::renderNextLODLevel()
{
	ccLog::PrintDebug(QString("[renderNextLODLevel] About to draw new LOD level?"));
	m_LODPendingRefresh = false;
	if (m_currentLODState.inProgress && m_currentLODState.level != 0 && !m_LODPendingIgnore)
	{
		ccLog::PrintDebug(QString("[renderNextLODLevel] Level %1 - index %2 confirmed").arg(m_currentLODState.level).arg(m_currentLODState.startIndex));
		QApplication::processEvents();
		requestUpdate();
	}
	else
	{
		ccLog::WarningDebug(QString("[renderNextLODLevel] Ignored"));
	}
}

void ccGLWindow::Create(ccGLWindow*& window, QWidget*& widget, bool stereoMode/*=false*/, bool silentInitialization/*=false*/)
{
	QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	format.setStereo(stereoMode);

	window = new ccGLWindow(&format, nullptr, silentInitialization);

#ifdef CC_GL_WINDOW_USE_QWINDOW
	widget = new ccGLWidget(window);
#else
	widget = window;
#endif
}

ccGLWindow* ccGLWindow::FromWidget(QWidget* widget)
{
#ifdef CC_GL_WINDOW_USE_QWINDOW
	ccGLWidget* myWidget = qobject_cast<ccGLWidget*>(widget);
	if (myWidget)
	{
		return myWidget->associatedWindow();
	}
#else
	ccGLWindow* myWidget = qobject_cast<ccGLWindow*>(widget);
	if (myWidget)
	{
		return myWidget;
	}
#endif
	else
	{
		assert(false);
		return nullptr;
	}
}

void ccGLWindow::handleLoggedMessage(const QOpenGLDebugMessage& message)
{
	//Decode severity
	QString sevStr;
	switch (message.severity())
	{
	case QOpenGLDebugMessage::HighSeverity:
		sevStr = "high";
		break;
	case QOpenGLDebugMessage::MediumSeverity:
		sevStr = "medium";
		break;
	case QOpenGLDebugMessage::LowSeverity:
		sevStr = "low";
		return; //don't care about them! they flood the console in Debug mode :(
	case QOpenGLDebugMessage::NotificationSeverity:
	default:
		sevStr = "notification";
		break;
	}

	//Decode source
	QString sourceStr;
	switch (message.source())
	{
	case QOpenGLDebugMessage::APISource:
		sourceStr = "API";
		break;
	case QOpenGLDebugMessage::WindowSystemSource:
		sourceStr = "window system";
		break;
	case QOpenGLDebugMessage::ShaderCompilerSource:
		sourceStr = "shader compiler";
		break;
	case QOpenGLDebugMessage::ThirdPartySource:
		sourceStr = "third party";
		break;
	case QOpenGLDebugMessage::ApplicationSource:
		sourceStr = "application";
		break;
	case QOpenGLDebugMessage::OtherSource:
	default:
		sourceStr = "other";
		break;
	}

	//Decode type
	QString typeStr;
	switch (message.type())
	{
	case QOpenGLDebugMessage::ErrorType:
		typeStr = "error";
		break;
	case QOpenGLDebugMessage::DeprecatedBehaviorType:
		typeStr = "deprecated behavior";
		break;
	case QOpenGLDebugMessage::UndefinedBehaviorType:
		typeStr = "undefined behavior";
		break;
	case QOpenGLDebugMessage::PortabilityType:
		typeStr = "portability";
		break;
	case QOpenGLDebugMessage::PerformanceType:
		typeStr = "performance";
		break;
	case QOpenGLDebugMessage::OtherType:
	default:
		typeStr = "other";
		break;
	case QOpenGLDebugMessage::MarkerType:
		typeStr = "marker";
		break;
	}

	QString msg = QString("[OpenGL][Win %0]").arg(getUniqueID());
	msg += "[source: " + sourceStr + "]";
	msg += "[type: " + typeStr + "]";
	msg += "[severity: " + sevStr + "]";
	msg += " ";
	msg += message.message();

	if (message.severity() != QOpenGLDebugMessage::NotificationSeverity)
		ccLog::Warning(msg);
	else
		ccLog::Print(msg);
}

