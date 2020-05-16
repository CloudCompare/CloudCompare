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
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccSphere.h> //for the pivot symbol
#include <ccSubMesh.h>

//CCFbo
#include <ccFrameBufferObject.h>
#include <ccGlFilter.h>

//Qt
#include <QApplication>
#include <QLayout>
#include <QMessageBox>
#include <QMimeData>
#include <QOpenGLDebugLogger>
#include <QOpenGLTexture>
#include <QPushButton>
#include <QSettings>
#include <QTouchEvent>
#include <QWheelEvent>

#if defined( Q_OS_MAC ) || defined( Q_OS_LINUX )
#include <QDir>
#endif

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

// These extra definitions are required in C++11.
// In C++17, class-level "static constexpr" is implicitly inline, so these are not required.
constexpr float ccGLWindow::MIN_POINT_SIZE_F;
constexpr float ccGLWindow::MAX_POINT_SIZE_F;
constexpr float ccGLWindow::MIN_LINE_WIDTH_F;
constexpr float ccGLWindow::MAX_LINE_WIDTH_F;

//Min and max zoom ratio (relative)
constexpr float CC_GL_MAX_ZOOM_RATIO = 1.0e6f;
constexpr float CC_GL_MIN_ZOOM_RATIO = 1.0e-6f;

//Vaious overlay elements dimensions
constexpr double CC_DISPLAYED_PIVOT_RADIUS_PERCENT = 0.8; //percentage of the smallest screen dimension
constexpr double CC_DISPLAYED_CUSTOM_LIGHT_LENGTH = 10.0;
constexpr float  CC_DISPLAYED_TRIHEDRON_AXES_LENGTH = 25.0f;
constexpr float  CC_DISPLAYED_CENTER_CROSS_LENGTH = 10.0f;

//Max click duration for enabling picking mode (in ms)
constexpr int CC_MAX_PICKING_CLICK_DURATION_MS = 200;

//invalid GL list index
constexpr GLuint GL_INVALID_LIST_ID = (~0);

//GL filter banner margin (height = 2*margin + current font height)
constexpr int CC_GL_FILTER_BANNER_MARGIN = 5;

//stereo passes
static const unsigned char MONO_OR_LEFT_RENDERING_PASS = 0;
static const unsigned char RIGHT_RENDERING_PASS = 1;

/*** Persistent settings ***/

constexpr char c_ps_groupName[] = "ccGLWindow";
constexpr char c_ps_perspectiveView[] = "perspectiveView";
constexpr char c_ps_objectMode[] = "objectCenteredView";
constexpr char c_ps_sunLight[] = "sunLightEnabled";
constexpr char c_ps_customLight[] = "customLightEnabled";
constexpr char c_ps_pivotVisibility[] = "pivotVisibility";
constexpr char c_ps_stereoGlassType[] = "stereoGlassType";

//Unique GL window ID
static int s_GlWindowNumber = 0;

// Shader path
Q_GLOBAL_STATIC( QString, s_shaderPath );

//On some versions of Qt, QGLWidget::renderText seems to need glColorf instead of glColorub!
// See https://bugreports.qt-project.org/browse/QTBUG-6217
template<class QOpenGLFunctions> inline static void glColor3ubv_safe(QOpenGLFunctions* glFunc, const unsigned char* rgb)
{
	assert(glFunc);
	glFunc->glColor3f(	rgb[0] / 255.0f,
						rgb[1] / 255.0f,
						rgb[2] / 255.0f);
}
template<class QOpenGLFunctions> inline static void glColor4ubv_safe(QOpenGLFunctions* glFunc, const unsigned char* rgb)
{
	assert(glFunc);
	glFunc->glColor4f(	rgb[0] / 255.0f,
						rgb[1] / 255.0f,
						rgb[2] / 255.0f,
						rgb[3] / 255.0f);
}

//! Precomputed stuff for the 'hot zone'
struct HotZone
{
	//display font
	QFont font;
	//text height
	int textHeight;
	//text shift
	int yTextBottomLineShift;
	//default color
	unsigned char color[3];

	//bubble-view label rect.
	QString bbv_label;
	//bubble-view label rect.
	QRect bbv_labelRect;
	//bubble-view row width
	int bbv_totalWidth;

	//fullscreen label rect.
	QString fs_label;
	//fullscreen label rect.
	QRect fs_labelRect;
	//fullscreen row width
	int fs_totalWidth;

	//point size label
	QString psi_label;
	//point size label rect.
	QRect psi_labelRect;
	//point size row width
	int psi_totalWidth;

	//line size label
	QString lsi_label;
	//line size label rect.
	QRect lsi_labelRect;
	//line size row width
	int lsi_totalWidth;

	int margin;
	int iconSize;
	QPoint topCorner;

	explicit HotZone(ccGLWindow* win)
		: textHeight(0)
		, yTextBottomLineShift(0)
		, bbv_label("bubble-view mode")
		, fs_label("fullscreen mode")
		, psi_label("default point size")
		, lsi_label("default line width")
		, margin(16)
		, iconSize(16)
		, topCorner(0, 0)
	{
		//default color ("greenish")
		color[0] = 133;
		color[1] = 193;
		color[2] = 39;

		if (win)
		{
			font = win->font();
			int retinaScale = win->devicePixelRatio();
			font.setPointSize(12 * retinaScale);
			margin *= retinaScale;
			iconSize *= retinaScale;
			font.setBold(true);
		}

		QFontMetrics metrics(font);
		bbv_labelRect = metrics.boundingRect(bbv_label);
		fs_labelRect = metrics.boundingRect(fs_label);
		psi_labelRect = metrics.boundingRect(psi_label);
		lsi_labelRect = metrics.boundingRect(lsi_label);

		psi_totalWidth = /*margin() + */psi_labelRect.width() + margin + iconSize + margin + iconSize/* + margin*/;
		lsi_totalWidth = /*margin() + */lsi_labelRect.width() + margin + iconSize + margin + iconSize/* + margin*/;
		bbv_totalWidth = /*margin() + */bbv_labelRect.width() + margin + iconSize/* + margin*/;
		fs_totalWidth  = /*margin() + */fs_labelRect.width()  + margin + iconSize/* + margin*/;

		textHeight = std::max(psi_labelRect.height(), bbv_labelRect.height());
		textHeight = std::max(lsi_labelRect.height(), textHeight);
		textHeight = std::max(fs_labelRect.height(), textHeight);
		textHeight = (3 * textHeight) / 4; // --> factor: to recenter the baseline a little
		yTextBottomLineShift = (iconSize / 2) + (textHeight / 2);
	}

	QRect rect(bool clickableItemsVisible, bool bubbleViewModeEnabled, bool fullScreenEnabled) const
	{
		//total hot zone area size (without margin)
		int totalWidth = 0;
		if (clickableItemsVisible)
			totalWidth = std::max(psi_totalWidth, lsi_totalWidth);
		if (bubbleViewModeEnabled)
			totalWidth = std::max(totalWidth, bbv_totalWidth);
		if (fullScreenEnabled)
			totalWidth = std::max(totalWidth, fs_totalWidth);

		QPoint minAreaCorner(0         , std::min(0, yTextBottomLineShift - textHeight));
		QPoint maxAreaCorner(totalWidth, std::max(iconSize, yTextBottomLineShift));
		int rowCount = clickableItemsVisible ? 2 : 0;
		rowCount += bubbleViewModeEnabled ? 1 : 0;
		rowCount += fullScreenEnabled ? 1 : 0;
		maxAreaCorner.setY(maxAreaCorner.y() + (iconSize + margin) * (rowCount - 1));

		QRect areaRect(	minAreaCorner - QPoint(margin, margin) / 2,
						maxAreaCorner + QPoint(margin, margin) / 2);

		return areaRect;
	}
};

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
	/** Up to 2 candidates, ifstereo mode is enabled **/
	CCVector3d autoPivotCandidates[2];
	bool hasAutoPivotCandidates[2] = { false, false };
};

//! Optional output metrics (from computeProjectionMatrix)
struct ccGLWindow::ProjectionMetrics
{
	double zNear = 0.0;
	double zFar = 0.0;
	double cameraToBBCenterDist = 0.0;
	double bbHalfDiag = 0.0;
};

//! Picking parameters
struct ccGLWindow::PickingParameters
{
	//! Default constructor
	PickingParameters(	PICKING_MODE _mode = NO_PICKING,
						int _centerX = 0,
						int _centerY = 0,
						int _pickWidth = 5,
						int _pickHeight = 5,
						bool _pickInSceneDB = true,
						bool _pickInLocalDB = true)
		: mode(_mode)
		, centerX(_centerX)
		, centerY(_centerY)
		, pickWidth(_pickWidth)
		, pickHeight(_pickHeight)
		, pickInSceneDB(_pickInSceneDB)
		, pickInLocalDB(_pickInLocalDB)
	{}

	PICKING_MODE mode;
	int centerX;
	int centerY;
	int pickWidth;
	int pickHeight;
	bool pickInSceneDB;
	bool pickInLocalDB;
};


ccGLWindow::ccGLWindow(	QSurfaceFormat* format/*=0*/,
						ccGLWindowParent* parent/*=0*/,
						bool silentInitialization/*=false*/)
	: ccGLWindowParent(parent)
#ifdef CC_GL_WINDOW_USE_QWINDOW
	, m_context(nullptr)
	, m_device(new QOpenGLPaintDevice)
	, m_parentWidget(nullptr)
#endif
	, m_uniqueID(++s_GlWindowNumber) //GL window unique ID
	, m_initialized(false)
	, m_trihedronGLList(GL_INVALID_LIST_ID)
	, m_pivotGLList(GL_INVALID_LIST_ID)
	, m_lastMousePos(-1, -1)
	, m_validModelviewMatrix(false)
	, m_validProjectionMatrix(false)
	, m_cameraToBBCenterDist(0.0)
	, m_bbHalfDiag(0.0)
	, m_LODEnabled(true)
	, m_LODAutoDisable(false)
	, m_shouldBeRefreshed(false)
	, m_mouseMoved(false)
	, m_mouseButtonPressed(false)
	, m_unclosable(false)
	, m_interactionFlags(MODE_TRANSFORM_CAMERA)
	, m_pickingMode(NO_PICKING)
	, m_pickingModeLocked(false)
	, m_lastClickTime_ticks(0)
	, m_sunLightEnabled(true)
	, m_customLightEnabled(false)
	, m_clickableItemsVisible(false)
	, m_activeShader(nullptr)
	, m_shadersEnabled(false)
	, m_activeFbo(nullptr)
	, m_fbo(nullptr)
	, m_fbo2(nullptr)
	, m_alwaysUseFBO(false)
	, m_updateFBO(true)
	, m_colorRampShader(nullptr)
	, m_customRenderingShader(nullptr)
	, m_activeGLFilter(nullptr)
	, m_glFiltersEnabled(false)
	, m_winDBRoot(nullptr)
	, m_globalDBRoot(nullptr) //external DB
#ifdef CC_GL_WINDOW_USE_QWINDOW
	, m_font(QFont())
#else
	, m_font(font())
#endif
	, m_pivotVisibility(PIVOT_SHOW_ON_MOVE)
	, m_pivotSymbolShown(false)
	, m_allowRectangularEntityPicking(true)
	, m_rectPickingPoly(nullptr)
	, m_overridenDisplayParametersEnabled(false)
	, m_displayOverlayEntities(true)
	, m_silentInitialization(silentInitialization)
	, m_bubbleViewModeEnabled(false)
	, m_bubbleViewFov_deg(90.0f)
	, m_LODPendingRefresh(false)
	, m_touchInProgress(false)
	, m_touchBaseDist(0.0)
	, m_scheduledFullRedrawTime(0)
	, m_stereoModeEnabled(false)
	, m_formerParent(nullptr)
	, m_exclusiveFullscreen(false)
	, m_showDebugTraces(false)
	, m_pickRadius(DefaultPickRadius)
	, m_glExtFuncSupported(false)
	, m_autoRefresh(false)
	, m_hotZone(nullptr)
	, m_showCursorCoordinates(false)
	, m_autoPickPivotAtCenter(true)
	, m_ignoreMouseReleaseEvent(false)
	, m_rotationAxisLocked(false)
	, m_lockedRotationAxis(0, 0, 1)
{
	//start internal timer
	m_timer.start();

#ifdef CC_GL_WINDOW_USE_QWINDOW
	setSurfaceType(QWindow::OpenGLSurface);

	m_format = format ? *format : requestedFormat();
#else
	if (format)
	{
		setFormat(*format);
	}
#endif
	//GL window title
	setWindowTitle(QString("3D View %1").arg(m_uniqueID));

	//GL window own DB
	m_winDBRoot = new ccHObject(QString("DB.3DView_%1").arg(m_uniqueID));

	//lights
	m_sunLightEnabled = true;
	m_sunLightPos[0] = 0.0f;
	m_sunLightPos[1] = 0.0f;
	m_sunLightPos[2] = 1.0f;
	m_sunLightPos[3] = 0.0f;

	m_customLightEnabled = false;
	m_customLightPos[0] = 0.0f;
	m_customLightPos[1] = 0.0f;
	m_customLightPos[2] = 0.0f;
	m_customLightPos[3] = 1.0f; //positional light

	//matrices
	m_viewportParams.viewMat.toIdentity();
	m_viewportParams.cameraCenter.z = -1.0; //don't position the camera on the pivot by default!
	m_viewMatd.toIdentity();
	m_projMatd.toIdentity();

	//default modes
	setPickingMode(DEFAULT_PICKING);
	setInteractionMode(MODE_TRANSFORM_CAMERA);

#ifndef CC_GL_WINDOW_USE_QWINDOW
	//drag & drop handling
	setAcceptDrops(true);
#endif

	//auto-load previous perspective settings
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);

		//load parameters
		bool perspectiveView = settings.value(c_ps_perspectiveView, false).toBool();
		//DGM: we force object-centered view by default now, as the viewer-based perspective is too dependent
		//on what is displayed (so restoring this parameter at next startup is rarely a good idea)
		bool objectCenteredView = /*settings.value(c_ps_objectMode,		true								).toBool()*/true;
		m_sunLightEnabled = settings.value(c_ps_sunLight, true).toBool();
		m_customLightEnabled = settings.value(c_ps_customLight, false).toBool();
		int pivotVisibility = settings.value(c_ps_pivotVisibility, PIVOT_SHOW_ON_MOVE).toInt();
		int glassType = settings.value(c_ps_stereoGlassType, ccGLWindow::StereoParams::RED_BLUE).toInt();

		settings.endGroup();

		//update stereo parameters
		m_stereoParams.glassType = static_cast<ccGLWindow::StereoParams::GlassType>(glassType);

		//report current perspective
		if (!m_silentInitialization)
		{
			if (!perspectiveView)
				ccLog::Print("[ccGLWindow] Perspective is off by default");
			else
				ccLog::Print(QString("[ccGLWindow] Perspective is on by default (%1)").arg(objectCenteredView ? "object-centered" : "viewer-centered"));
		}

		//pivot visibility
		switch (pivotVisibility)
		{
		case PIVOT_HIDE:
			setPivotVisibility(PIVOT_HIDE);
			break;
		case PIVOT_SHOW_ON_MOVE:
			setPivotVisibility(PIVOT_SHOW_ON_MOVE);
			break;
		case PIVOT_ALWAYS_SHOW:
			setPivotVisibility(PIVOT_ALWAYS_SHOW);
			break;
		}

		//apply saved parameters
		setPerspectiveState(perspectiveView, objectCenteredView);
		if (m_customLightEnabled)
			displayNewMessage("Warning: custom light is ON", ccGLWindow::LOWER_LEFT_MESSAGE, false, 2, CUSTOM_LIGHT_STATE_MESSAGE);
		if (!m_sunLightEnabled)
			displayNewMessage("Warning: sun light is OFF", ccGLWindow::LOWER_LEFT_MESSAGE, false, 2, SUN_LIGHT_STATE_MESSAGE);
	}

	m_deferredPickingTimer.setSingleShot(true);
	m_deferredPickingTimer.setInterval(100);

	//signal/slot connections
	connect(this, &ccGLWindow::itemPickedFast, this, &ccGLWindow::onItemPickedFast, Qt::DirectConnection);
	connect(&m_scheduleTimer, &QTimer::timeout, this, &ccGLWindow::checkScheduledRedraw);
	connect(&m_autoRefreshTimer, &QTimer::timeout, this, [=] () { update();	});
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

	//we must unlink entities currently linked to this window
	if (m_globalDBRoot)
	{
		m_globalDBRoot->removeFromDisplay_recursive(this);
	}
	if (m_winDBRoot)
	{
		m_winDBRoot->removeFromDisplay_recursive(this);
	}

	delete m_winDBRoot;
	m_winDBRoot = nullptr;

	delete m_rectPickingPoly;
	m_rectPickingPoly = nullptr;

	delete m_activeGLFilter;
	m_activeGLFilter = nullptr;

	delete m_colorRampShader;
	m_colorRampShader = nullptr;

	delete m_customRenderingShader;
	m_customRenderingShader = nullptr;

	delete m_activeShader;
	m_activeShader = nullptr;

	delete m_fbo;
	m_fbo = nullptr;

	delete m_fbo2;
	m_fbo2 = nullptr;

#ifdef CC_GL_WINDOW_USE_QWINDOW
	if (m_context)
		m_context->doneCurrent();

	delete m_device;
	m_device = nullptr;
#endif

	delete m_hotZone;
	m_hotZone = nullptr;
}

#ifdef CC_GL_WINDOW_USE_QWINDOW

void ccGLWindow::setParentWidget(QWidget* widget)
{
	m_parentWidget = widget;

	if (widget)
	{
		//drag & drop handling
		widget->setAcceptDrops(true);
		widget->setAttribute(Qt::WA_AcceptTouchEvents, true);
		widget->setAttribute(Qt::WA_OpaquePaintEvent, true);
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

bool ccGLWindow::bindFBO(ccFrameBufferObject* fbo)
{
	if (fbo) //bind
	{
		if (fbo->start())
		{
			m_activeFbo = fbo;
			return true;
		}
		else
		{
			//failed to start the FBO?!
			m_activeFbo = nullptr;
			return false;

		}
	}
	else //unbind
	{
		m_activeFbo = nullptr;

		assert(m_glExtFuncSupported);
		//we automatically enable the QOpenGLWidget's default FBO
		m_glExtFunc.glBindFramebuffer(GL_FRAMEBUFFER_EXT, defaultQtFBO());

		return true;
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

const ccGui::ParamStruct& ccGLWindow::getDisplayParameters() const
{
	return m_overridenDisplayParametersEnabled ? m_overridenDisplayParameters : ccGui::Parameters();
}

void ccGLWindow::setDisplayParameters(const ccGui::ParamStruct &params, bool thisWindowOnly)
{
	if (thisWindowOnly)
	{
		m_overridenDisplayParametersEnabled = true;
		m_overridenDisplayParameters = params;
	}
	else
	{
		m_overridenDisplayParametersEnabled = false;
		ccGui::Set(params);
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
		emit baseViewMatChanged(m_viewportParams.viewMat);

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
					const QString shaderPath = QStringLiteral( "%1/ColorRamp/color_ramp.frag" ).arg( *s_shaderPath );
					
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
		setDisplayParameters(params, hasOverridenDisplayParameters());

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
				setDisplayParameters(params,hasOverridenDisplayParameters());
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
					float zoomFactor = dist / m_touchBaseDist;
					updateZoom(zoomFactor);
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

	displayNewMessage(	QString("New size = %1 * %2 (px)").arg(m_glViewport.width()).arg(m_glViewport.height()),
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

bool ccGLWindow::setLODEnabled(bool state, bool autoDisable/*=false*/)
{
	if (state && (!m_fbo || (m_stereoModeEnabled && !m_stereoParams.isAnaglyph() && !m_fbo2)))
	{
		//we need a valid FBO (or two ;) for LOD!!!
		return false;
	}

	m_LODEnabled = state;
	m_LODAutoDisable = autoDisable;
	return true;
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

void ccGLWindow::drawClickableItems(int xStart0, int& yStart)
{
	//we init the necessary parameters the first time we need them
	if (!m_hotZone)
	{
		m_hotZone = new HotZone(this);
	}
	//remember the last position of the 'top corner'
	m_hotZone->topCorner = QPoint(xStart0, yStart) + QPoint(m_hotZone->margin, m_hotZone->margin);

	bool fullScreenEnabled = exclusiveFullScreen();

	if (!m_clickableItemsVisible
		&&	!m_bubbleViewModeEnabled
		&&	!fullScreenEnabled)
	{
		//nothing to do
		return;
	}

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//"exit" icon
	static const QImage c_exitIcon = QImage(":/CC/images/ccExit.png").mirrored();

	int halfW = m_glViewport.width() / 2;
	int halfH = m_glViewport.height() / 2;

	glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
	glFunc->glEnable(GL_BLEND);

	//draw semi-transparent background
	{
		QRect areaRect = m_hotZone->rect(m_clickableItemsVisible, m_bubbleViewModeEnabled, fullScreenEnabled);
		areaRect.translate(m_hotZone->topCorner);

		//draw rectangle
		glFunc->glColor4ub(ccColor::darkGrey.r, ccColor::darkGrey.g, ccColor::darkGrey.b, 210);
		glFunc->glBegin(GL_QUADS);
		int x0 = -halfW + areaRect.x();
		int y0 =  halfH - areaRect.y();
		glFunc->glVertex2i(x0                   , y0);
		glFunc->glVertex2i(x0 + areaRect.width(), y0);
		glFunc->glVertex2i(x0 + areaRect.width(), y0 - areaRect.height());
		glFunc->glVertex2i(x0                   , y0 - areaRect.height());
		glFunc->glEnd();
	}

	yStart = m_hotZone->topCorner.y();

	if (fullScreenEnabled)
	{
		int xStart = m_hotZone->topCorner.x();

		//label
		glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, m_hotZone->color);
		renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->fs_label, m_hotZone->font);

		//icon
		xStart += m_hotZone->fs_labelRect.width() + m_hotZone->margin;

		//"full-screen" icon
		{
			ccGLUtils::DisplayTexture2DPosition(c_exitIcon, -halfW + xStart, halfH - (yStart + m_hotZone->iconSize), m_hotZone->iconSize, m_hotZone->iconSize);
			m_clickableItems.emplace_back(ClickableItem::LEAVE_FULLSCREEN_MODE, QRect(xStart, yStart, m_hotZone->iconSize, m_hotZone->iconSize));
			xStart += m_hotZone->iconSize;
		}

		yStart += m_hotZone->iconSize;
		yStart += m_hotZone->margin;
	}

	if (m_bubbleViewModeEnabled)
	{
		int xStart = m_hotZone->topCorner.x();

		//label
		glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, m_hotZone->color);
		renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->bbv_label, m_hotZone->font);

		//icon
		xStart += m_hotZone->bbv_labelRect.width() + m_hotZone->margin;

		//"exit" icon
		{
			ccGLUtils::DisplayTexture2DPosition(c_exitIcon, -halfW + xStart, halfH - (yStart + m_hotZone->iconSize), m_hotZone->iconSize, m_hotZone->iconSize);
			m_clickableItems.emplace_back(ClickableItem::LEAVE_BUBBLE_VIEW_MODE, QRect(xStart, yStart, m_hotZone->iconSize, m_hotZone->iconSize));
			xStart += m_hotZone->iconSize;
		}

		yStart += m_hotZone->iconSize;
		yStart += m_hotZone->margin;
	}

	if (m_clickableItemsVisible)
	{
		static const QImage c_minusPix = QImage(":/CC/images/ccMinus.png").mirrored();
		static const QImage c_plusPix = QImage(":/CC/images/ccPlus.png").mirrored();

		//default point size
		{
			int xStart = m_hotZone->topCorner.x();

			glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, m_hotZone->color);
			renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->psi_label, m_hotZone->font);

			//icons
			xStart += m_hotZone->psi_labelRect.width() + m_hotZone->margin;

			//"minus" icon
			{
				ccGLUtils::DisplayTexture2DPosition(c_minusPix, -halfW + xStart, halfH - (yStart + m_hotZone->iconSize), m_hotZone->iconSize, m_hotZone->iconSize);
				m_clickableItems.emplace_back(ClickableItem::DECREASE_POINT_SIZE, QRect(xStart, yStart, m_hotZone->iconSize, m_hotZone->iconSize));
				xStart += m_hotZone->iconSize;
			}

			//separator
			{
				glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, m_hotZone->color);
				glFunc->glPushAttrib(GL_POINT_BIT);
				glFunc->glPointSize(m_viewportParams.defaultPointSize);
				glFunc->glEnable(GL_POINT_SMOOTH);
				glFunc->glBegin(GL_POINTS);
				glFunc->glVertex2i(-halfW + xStart + m_hotZone->margin / 2, halfH - (yStart + m_hotZone->iconSize / 2));
				glFunc->glEnd();
				glFunc->glPopAttrib(); //GL_POINT_BIT
				xStart += m_hotZone->margin;
			}

			//"plus" icon
			{
				ccGLUtils::DisplayTexture2DPosition(c_plusPix, -halfW + xStart, halfH - (yStart + m_hotZone->iconSize), m_hotZone->iconSize, m_hotZone->iconSize);
				m_clickableItems.emplace_back(ClickableItem::INCREASE_POINT_SIZE, QRect(xStart, yStart, m_hotZone->iconSize, m_hotZone->iconSize));
				xStart += m_hotZone->iconSize;
			}

			yStart += m_hotZone->iconSize;
			yStart += m_hotZone->margin;
		}
		
		//default line size
		{
			int xStart = m_hotZone->topCorner.x();

			glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, m_hotZone->color);
			renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->lsi_label, m_hotZone->font);

			//icons
			xStart += m_hotZone->lsi_labelRect.width() + m_hotZone->margin;

			//"minus" icon
			{
				ccGLUtils::DisplayTexture2DPosition(c_minusPix, -halfW + xStart, halfH - (yStart + m_hotZone->iconSize), m_hotZone->iconSize, m_hotZone->iconSize);
				m_clickableItems.emplace_back(ClickableItem::DECREASE_LINE_WIDTH, QRect(xStart, yStart, m_hotZone->iconSize, m_hotZone->iconSize));
				xStart += m_hotZone->iconSize;
			}

			//separator
			{
				glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, m_hotZone->color);
				//we use a point to represent the line thickness
				glFunc->glPushAttrib(GL_POINT_BIT);
				glFunc->glPointSize(m_viewportParams.defaultLineWidth);
				glFunc->glBegin(GL_POINTS);
				glFunc->glVertex2i(-halfW + xStart + m_hotZone->margin / 2, halfH - (yStart + m_hotZone->iconSize / 2));
				glFunc->glEnd();
				glFunc->glPopAttrib(); //GL_POINT_BIT
				xStart += m_hotZone->margin;
			}

			//"plus" icon
			{
				ccGLUtils::DisplayTexture2DPosition(c_plusPix, -halfW + xStart, halfH - (yStart + m_hotZone->iconSize), m_hotZone->iconSize, m_hotZone->iconSize);
				m_clickableItems.emplace_back(ClickableItem::INCREASE_LINE_WIDTH, QRect(xStart, yStart, m_hotZone->iconSize, m_hotZone->iconSize));
				xStart += m_hotZone->iconSize;
			}

			yStart += m_hotZone->iconSize;
			yStart += m_hotZone->margin;
		}
	}

	glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT
}

void ccGLWindow::toBeRefreshed()
{
	m_shouldBeRefreshed = true;

	invalidateViewport();
	invalidateVisualization();
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

	qint64 startTime_ms = m_currentLODState.inProgress ? m_timer.elapsed() : 0;

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

	//start the rendering passes
	for (renderingParams.passIndex = 0; renderingParams.passIndex < renderingParams.passCount; ++renderingParams.passIndex)
	{
		fullRenderingPass(CONTEXT, renderingParams);
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
				int w = m_glViewport.width() / 2 + 1;
				int h = m_glViewport.height() / 2 + 1;

				const ccColor::Rgbub& bkgCol = getDisplayParameters().backgroundCol;
				const ccColor::Rgbub& frgCol = getDisplayParameters().textDefaultCol;

				//Gradient "texture" drawing
				glFunc->glBegin(GL_QUADS);
				{
					//we use the default background color for gradient start
					glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, bkgCol.rgb);
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

				bindFBO(0);
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
		int x = m_glViewport.width() / 2 - 100;
		int y = 0;

		if (m_stereoModeEnabled && m_stereoParams.glassType != StereoParams::OCULUS)
		{
			if (renderingParams.passIndex == RIGHT_RENDERING_PASS)
				x += m_glViewport.width() / 2;
		}

		setStandardOrthoCorner();
		glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
		glFunc->glDisable(GL_DEPTH_TEST);

		//draw black background
		{
			int height = (diagStrings.size() + 1) * 14;
			glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::black.rgba);
			glFunc->glBegin(GL_QUADS);
			glFunc->glVertex2i(x, m_glViewport.height() - y);
			glFunc->glVertex2i(x, m_glViewport.height() - (y + height));
			glFunc->glVertex2i(x + 200, m_glViewport.height() - (y + height));
			glFunc->glVertex2i(x + 200, m_glViewport.height() - y);
			glFunc->glEnd();
		}

		glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::yellow.rgba);
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
					parameters.zoom = m_viewportParams.perspectiveView ? computePerspectiveZoom() : m_viewportParams.zoom; //TODO: doesn't work well with EDL in perspective mode!
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

				ccGLUtils::DisplayTexture2DPosition(screenTex, 0, 0, m_glViewport.width(), m_glViewport.height());

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
			bindFBO(0);

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
				getRealCameraCenter(),
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

	//do this before drawing the pivot!
	if (	m_autoPickPivotAtCenter
		&&	(!m_stereoModeEnabled || renderingParams.passIndex == MONO_OR_LEFT_RENDERING_PASS))
	{
		CCVector3d P;
		if (getClick3DPos(m_glViewport.width() / 2, m_glViewport.height() / 2, P))
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
		emit drawing3D();
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
	if (m_displayOverlayEntities)
	{
		//default overlay color
		const ccColor::Rgbub& textCol = getDisplayParameters().textDefaultCol;

		if (!m_captureMode.enabled || m_captureMode.renderOverlayItems)
		{
			//scale: only in ortho mode
			if (!m_viewportParams.perspectiveView)
			{
				drawScale(textCol);
			}

			//trihedron
			drawTrihedron();
		}

		if (!m_captureMode.enabled)
		{
			int yStart = 0;

			//transparent border at the top of the screen
			bool showGLFilterRibbon = renderingParams.useFBO && m_activeGLFilter;
			showGLFilterRibbon &= !exclusiveFullScreen(); //we hide it in fullscreen mode!
			if (showGLFilterRibbon)
			{
				const float w = m_glViewport.width() / 2.0f;
				const float h = m_glViewport.height() / 2.0f;
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

				glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::black.rgba);
				renderText(	10,
							borderHeight - CC_GL_FILTER_BANNER_MARGIN - CC_GL_FILTER_BANNER_MARGIN / 2,
							QString("[GL filter] ") + m_activeGLFilter->getDescription()
							/*,m_font*/); //we ignore the custom font size

				yStart += borderHeight;
			}

			//current messages (if valid)
			if (!m_messagesToDisplay.empty())
			{
				glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, textCol.rgb);

				int ll_currentHeight = m_glViewport.height() - 10; //lower left
				int uc_currentHeight = 10; //upper center

				for (const auto &message : m_messagesToDisplay)
				{
					switch (message.position)
					{
					case LOWER_LEFT_MESSAGE:
					{
						renderText(10, ll_currentHeight, message.message, m_font);
						int messageHeight = QFontMetrics(m_font).height();
						ll_currentHeight -= (messageHeight * 5) / 4; //add a 25% margin
					}
					break;
					case UPPER_CENTER_MESSAGE:
					{
						QRect rect = QFontMetrics(m_font).boundingRect(message.message);
						//take the GL filter banner into account!
						int x = (m_glViewport.width() - rect.width()) / 2;
						int y = uc_currentHeight + rect.height();
						if (showGLFilterRibbon)
						{
							y += getGlFilterBannerHeight();
						}
						renderText(x, y, message.message, m_font);
						uc_currentHeight += (rect.height() * 5) / 4; //add a 25% margin
					}
					break;
					case SCREEN_CENTER_MESSAGE:
					{
						QFont newFont(m_font); //no need to take zoom into account!
						newFont.setPointSize(12 * devicePixelRatio());
						QRect rect = QFontMetrics(newFont).boundingRect(message.message);
						//only one message supported in the screen center (for the moment ;)
						renderText((m_glViewport.width() - rect.width()) / 2, (m_glViewport.height() - rect.height()) / 2, message.message, newFont);
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
				int cx = x + lodIconSize / 2 - m_glViewport.width() / 2;
				int cy = m_glViewport.height() / 2 - (yStart + lodIconSize / 2);

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

void ccGLWindow::stopLODCycle()
{
	//reset LOD rendering (if any)
	m_currentLODState = LODState();
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
		event->acceptProposedAction();
}

void ccGLWindow::dropEvent(QDropEvent *event)
{
	const QMimeData* mimeData = event->mimeData();

	if (mimeData && mimeData->hasFormat("text/uri-list"))
	{
		QStringList fileNames;
		for (const QUrl &url : mimeData->urls()) {
			QString fileName = url.toLocalFile();
			fileNames.append(fileName);
#ifdef QT_DEBUG
			ccLog::Print(QString("File dropped: %1").arg(fileName));
#endif
		}

		if (!fileNames.empty())
		{
			emit filesDropped(fileNames);
		}

		event->acceptProposedAction();
	}

	event->ignore();
}

bool ccGLWindow::objectPerspectiveEnabled() const
{
	return m_viewportParams.perspectiveView && m_viewportParams.objectCenteredView;
}

bool ccGLWindow::viewerPerspectiveEnabled() const
{
	return m_viewportParams.perspectiveView && !m_viewportParams.objectCenteredView;
}

bool ccGLWindow::getPerspectiveState(bool& objectCentered) const
{
	objectCentered = m_viewportParams.objectCenteredView;
	return m_viewportParams.perspectiveView;
}

void ccGLWindow::setUnclosable(bool state)
{
	m_unclosable = state;
}

ccHObject* ccGLWindow::getOwnDB()
{
	return m_winDBRoot;
}

void ccGLWindow::addToOwnDB(ccHObject* obj, bool noDependency/*=true*/)
{
	if (!obj)
	{
		assert(false);
		return;
	}

	if (m_winDBRoot)
	{
		m_winDBRoot->addChild(obj, noDependency ? ccHObject::DP_NONE : ccHObject::DP_PARENT_OF_OTHER);
		obj->setDisplay(this);
	}
	else
	{
		ccLog::Error("[ccGLWindow::addToOwnDB] Window has no DB!");
	}
}

void ccGLWindow::removeFromOwnDB(ccHObject* obj)
{
	if (m_winDBRoot)
		m_winDBRoot->removeChild(obj);
}

void ccGLWindow::zoomGlobal()
{
	updateConstellationCenterAndZoom();
}

void ccGLWindow::updateConstellationCenterAndZoom(const ccBBox* aBox/*=0*/)
{
	if (m_bubbleViewModeEnabled)
	{
		ccLog::Warning("[updateConstellationCenterAndZoom] Not when bubble-view is enabled!");
		return;
	}

	setZoom(1.0f);

	ccBBox zoomedBox;

	//the user has provided a valid bounding box
	if (aBox)
	{
		zoomedBox = (*aBox);
	}
	else //otherwise we'll take the default one (if possible)
	{
		getVisibleObjectsBB(zoomedBox);
	}

	if (!zoomedBox.isValid())
		return;

	//we get the bounding-box diagonal length
	double bbDiag = static_cast<double>(zoomedBox.getDiagNorm());

	if ( CCCoreLib::LessThanEpsilon( bbDiag ) )
	{
		ccLog::Warning("[ccGLWindow] Entity/DB has a null bounding-box! Can't zoom in...");
		return;
	}

	//we compute the pixel size (in world coordinates)
	{
		int minScreenSize = std::min(m_glViewport.width(), m_glViewport.height());
		setPixelSize(minScreenSize > 0 ? static_cast<float>(bbDiag / minScreenSize) : 1.0f);
	}

	//we set the pivot point on the box center
	CCVector3d P = CCVector3d::fromArray(zoomedBox.getCenter().u);
	setPivotPoint(P);

	CCVector3d cameraPos = P;
	if (m_viewportParams.perspectiveView) //camera is on the pivot in ortho mode
	{
		//we must go backward so as to see the object!
		float currentFov_deg = getFov();
		assert( CCCoreLib::GreaterThanEpsilon( currentFov_deg ) );
		double d = bbDiag / (2 * std::tan( CCCoreLib::DegreesToRadians( currentFov_deg / 2.0 ) ));

		CCVector3d cameraDir(0, 0, -1);
		if (!m_viewportParams.objectCenteredView)
			cameraDir = getCurrentViewDir();

		cameraPos -= cameraDir * d;
	}
	setCameraPos(cameraPos);

	invalidateViewport();
	invalidateVisualization();
	deprecate3DLayer();

	redraw();
}

bool ccGLWindow::areShadersEnabled() const
{
	return m_shadersEnabled;
}

bool ccGLWindow::areGLFiltersEnabled() const
{
	return m_glFiltersEnabled;
}

void ccGLWindow::setShader(ccShader* _shader)
{
	if (!m_shadersEnabled)
	{
		ccLog::Warning("[ccGLWindow::setShader] Shader ignored (not supported)");
		return;
	}

	delete m_activeShader;
	m_activeShader = _shader;

	redraw();
}

void ccGLWindow::setGlFilter(ccGlFilter* filter)
{
	if (!m_glFiltersEnabled)
	{
		ccLog::Warning("[ccGLWindow::setGlFilter] GL filter ignored (not supported)");
		return;
	}

	removeGLFilter();

	if (filter)
	{
		if (!m_fbo)
		{
			if (!initFBO(width(), height()))
			{
				redraw();
				return;
			}
		}

		m_activeGLFilter = filter;
		initGLFilter(width(), height());
	}

	if (!m_activeGLFilter && m_fbo && !m_alwaysUseFBO)
	{
		removeFBO();
	}

	redraw();
}

void ccGLWindow::setZoom(float value)
{
	//zoom should be avoided in bubble-view mode
	assert(!m_bubbleViewModeEnabled);

	if (value < CC_GL_MIN_ZOOM_RATIO)
		value = CC_GL_MIN_ZOOM_RATIO;
	else if (value > CC_GL_MAX_ZOOM_RATIO)
		value = CC_GL_MAX_ZOOM_RATIO;

	if (m_viewportParams.zoom != value)
	{
		m_viewportParams.zoom = value;
		invalidateViewport();
		invalidateVisualization();
		deprecate3DLayer();
	}
}

void ccGLWindow::setCameraPos(const CCVector3d& P)
{
	if ((m_viewportParams.cameraCenter - P).norm2d() != 0.0)
	{
		m_viewportParams.cameraCenter = P;
		emit cameraPosChanged(m_viewportParams.cameraCenter);

		invalidateViewport();
		invalidateVisualization();
		deprecate3DLayer();
	}
}

void ccGLWindow::moveCamera(float dx, float dy, float dz)
{
	if (dx != 0.0f || dy != 0.0f) //camera movement? (dz doesn't count as it only corresponds to a zoom)
	{
		//feedback for echo mode
		emit cameraDisplaced(dx, dy);
	}

	//current X, Y and Z viewing directions
	//correspond to the 'model view' matrix
	//lines.
	CCVector3d V(dx, dy, dz);
	if (!m_viewportParams.objectCenteredView)
	{
		m_viewportParams.viewMat.transposed().applyRotation(V);
	}

	setCameraPos(m_viewportParams.cameraCenter + V);
}

void ccGLWindow::setPivotPoint(	const CCVector3d& P,
								bool autoUpdateCameraPos/*=false*/,
								bool verbose/*=false*/)
{
	if (autoUpdateCameraPos && 
		(!m_viewportParams.perspectiveView || m_viewportParams.objectCenteredView))
	{
		//compute the equivalent camera center
		CCVector3d dP = m_viewportParams.pivotPoint - P;
		CCVector3d MdP = dP; m_viewportParams.viewMat.applyRotation(MdP);
		CCVector3d newCameraPos = m_viewportParams.cameraCenter + MdP - dP;
		setCameraPos(newCameraPos);
	}

	m_viewportParams.pivotPoint = P;
	emit pivotPointChanged(m_viewportParams.pivotPoint);

	if (verbose)
	{
		const unsigned& precision = getDisplayParameters().displayedNumPrecision;
		displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
		displayNewMessage(QString("Point (%1 ; %2 ; %3) set as rotation center")
			.arg(P.x, 0, 'f', precision)
			.arg(P.y, 0, 'f', precision)
			.arg(P.z, 0, 'f', precision),
			ccGLWindow::LOWER_LEFT_MESSAGE, true);
		redraw(true, false);
	}

	invalidateViewport();
	invalidateVisualization();
}

void ccGLWindow::setAutoPickPivotAtCenter(bool state)
{
	if (m_autoPickPivotAtCenter != state)
	{
		m_autoPickPivotAtCenter = state;

		if (state)
		{
			//force 3D redraw to update the coordinates of the 'auto' pivot center
			redraw(false);
		}
	}
}

void ccGLWindow::setPixelSize(float pixelSize)
{
	if (m_viewportParams.pixelSize != pixelSize)
	{
		m_viewportParams.pixelSize = pixelSize;
		emit pixelSizeChanged(pixelSize);
	}

	invalidateViewport();
	invalidateVisualization();
	deprecate3DLayer();
}

void ccGLWindow::setSceneDB(ccHObject* root)
{
	m_globalDBRoot = root;
	zoomGlobal();
}

ccHObject* ccGLWindow::getSceneDB()
{
	return m_globalDBRoot;
}

void ccGLWindow::drawCross()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);
	
	//force line width
	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(1.0f);

	//cross OpenGL drawing
	glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::lightGrey.rgba);
	glFunc->glBegin(GL_LINES);
	glFunc->glVertex3f(0.0f, -CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f);
	glFunc->glVertex3f(0.0f, CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f);
	glFunc->glVertex3f(-CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f, 0.0f);
	glFunc->glVertex3f(CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f, 0.0f);
	glFunc->glEnd();

	glFunc->glPopAttrib(); //GL_LINE_BIT
}

inline float RoundScale(float equivalentWidth)
{
	//we compute the scale granularity (to avoid width values with a lot of decimals)
	int k = static_cast<int>(std::floor(std::log(equivalentWidth) / std::log(10.0f)));
	float granularity = std::pow(10.0f, static_cast<float>(k)) / 2.0f;
	//we choose the value closest to equivalentWidth with the right granularity
	return std::floor(std::max(equivalentWidth / granularity, 1.0f))*granularity;
}

void ccGLWindow::drawScale(const ccColor::Rgbub& color)
{
	assert(!m_viewportParams.perspectiveView); //a scale is only valid in ortho. mode!

	float scaleMaxW = m_glViewport.width() / 4.0f; //25% of screen width
	if (m_captureMode.enabled)
	{
		//DGM: we have to fall back to the case 'render zoom = 1' (otherwise we might not get the exact same aspect)
		scaleMaxW /= m_captureMode.zoomFactor;
	}
	if (m_viewportParams.zoom < CC_GL_MIN_ZOOM_RATIO)
	{
		assert(false);
		return;
	}

	//we first compute the width equivalent to 25% of horizontal screen width
	//(this is why it's only valid in orthographic mode !)
	float equivalentWidthRaw = scaleMaxW * m_viewportParams.pixelSize / m_viewportParams.zoom;
	float equivalentWidth = RoundScale(equivalentWidthRaw);

	QFont font = getTextDisplayFont(); //we take rendering zoom into account!
	QFontMetrics fm(font);

	//we deduce the scale drawing width
	float scaleW_pix = equivalentWidth / m_viewportParams.pixelSize * m_viewportParams.zoom;
	if (m_captureMode.enabled)
	{
		//we can now safely apply the rendering zoom
		scaleW_pix *= m_captureMode.zoomFactor;
	}
	float trihedronLength = CC_DISPLAYED_TRIHEDRON_AXES_LENGTH * m_captureMode.zoomFactor;
	float dW = 2.0f * trihedronLength + 20.0f;
	float dH = std::max(fm.height() * 1.25f, trihedronLength + 5.0f);
	float w = m_glViewport.width() / 2.0f - dW;
	float h = m_glViewport.height() / 2.0f - dH;
	float tick = 3.0f * m_captureMode.zoomFactor;

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//force line width
	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(1.0f);

	//scale OpenGL drawing
	glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, color.rgb);
	glFunc->glBegin(GL_LINES);
	glFunc->glVertex3f(w - scaleW_pix, -h, 0.0f);
	glFunc->glVertex3f(w, -h, 0.0f);
	glFunc->glVertex3f(w - scaleW_pix, -h - tick, 0.0f);
	glFunc->glVertex3f(w - scaleW_pix, -h + tick, 0.0f);
	glFunc->glVertex3f(w, -h + tick, 0.0f);
	glFunc->glVertex3f(w, -h - tick, 0.0f);
	glFunc->glEnd();

	glFunc->glPopAttrib(); //GL_LINE_BIT

	QString text = QString::number(equivalentWidth);
	glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, color.rgb);
	renderText(m_glViewport.width() - static_cast<int>(scaleW_pix / 2 + dW) - fm.width(text) / 2, m_glViewport.height() - static_cast<int>(dH / 2) + fm.height() / 3, text, font);
}

void ccGLWindow::drawTrihedron()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	float trihedronLength = CC_DISPLAYED_TRIHEDRON_AXES_LENGTH * m_captureMode.zoomFactor;

	float w = m_glViewport.width() / 2.0f - trihedronLength - 10.0f;
	float h = m_glViewport.height() / 2.0f - trihedronLength - 5.0f;

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glTranslatef(w, -h, 0.0f);
	glFunc->glMultMatrixd(m_viewportParams.viewMat.data());

	//on first call, compile the GL list once and for all
	if (m_trihedronGLList == GL_INVALID_LIST_ID)
	{
		m_trihedronGLList = glFunc->glGenLists(1);
		glFunc->glNewList(m_trihedronGLList, GL_COMPILE);

		glFunc->glPushAttrib(GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);
		glFunc->glEnable(GL_LINE_SMOOTH);
		glFunc->glLineWidth(2.0f);
		glFunc->glClear(GL_DEPTH_BUFFER_BIT); //DGM: the trihedron is displayed in the foreground but still in 3D!
		glFunc->glEnable(GL_DEPTH_TEST);

		//trihedron OpenGL drawing
		glFunc->glBegin(GL_LINES);
		glFunc->glColor3f(1.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(CC_DISPLAYED_TRIHEDRON_AXES_LENGTH, 0.0f, 0.0f);
		glFunc->glColor3f(0.0f, 1.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, CC_DISPLAYED_TRIHEDRON_AXES_LENGTH, 0.0f);
		glFunc->glColor3f(0.0f, 0.7f, 1.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, CC_DISPLAYED_TRIHEDRON_AXES_LENGTH);
		glFunc->glEnd();

		glFunc->glPopAttrib(); //GL_LINE_BIT | GL_DEPTH_BUFFER_BIT

		glFunc->glEndList();
	}
	else if (m_captureMode.enabled)
	{
		glFunc->glScalef(m_captureMode.zoomFactor, m_captureMode.zoomFactor, m_captureMode.zoomFactor);
	}

	glFunc->glCallList(m_trihedronGLList);

	glFunc->glPopMatrix();
}

CCVector3d ccGLWindow::getRealCameraCenter() const
{
	//the camera center is always defined in perspective mode
	if (m_viewportParams.perspectiveView)
	{
		return m_viewportParams.cameraCenter;
	}

	//in orthographic mode, we put the camera at the center of the
	//visible objects (along the viewing direction)
	ccBBox box;
	getVisibleObjectsBB(box);

	return CCVector3d(	m_viewportParams.cameraCenter.x,
						m_viewportParams.cameraCenter.y,
						box.isValid() ? box.getCenter().z : 0.0);
}

void ccGLWindow::getVisibleObjectsBB(ccBBox& box) const
{
	//compute center of visible objects constellation
	if (m_globalDBRoot)
	{
		//get whole bounding-box
		box = m_globalDBRoot->getDisplayBB_recursive(false, this);
	}

	//incorporate window own db
	if (m_winDBRoot)
	{
		ccBBox ownBox = m_winDBRoot->getDisplayBB_recursive(false, this);
		if (ownBox.isValid())
		{
			box += ownBox;
		}
	}
}

void ccGLWindow::invalidateViewport()
{
	m_validProjectionMatrix = false;
}

ccGLMatrixd ccGLWindow::computeProjectionMatrix(const CCVector3d& cameraCenter, bool withGLfeatures, ProjectionMetrics* metrics/*=0*/, double* eyeOffset/*=0*/) const
{
	double bbHalfDiag = 1.0;
	CCVector3d bbCenter(0, 0, 0);

	//compute center of visible objects constellation
	if (m_globalDBRoot || m_winDBRoot)
	{
		//get whole bounding-box
		ccBBox box;
		getVisibleObjectsBB(box);
		if (box.isValid())
		{
			//get bbox center
			bbCenter = CCVector3d::fromArray(box.getCenter().u);
			//get half bbox diagonal length
			bbHalfDiag = box.getDiagNormd() / 2;
		}
	}

	if (metrics)
	{
		metrics->bbHalfDiag = bbHalfDiag;
		metrics->cameraToBBCenterDist = (cameraCenter - bbCenter).normd();
	}

	//virtual pivot point (i.e. to handle viewer-based mode smoothly)
	CCVector3d pivotPoint = (m_viewportParams.objectCenteredView ? m_viewportParams.pivotPoint : cameraCenter);

	//distance between the camera center and the pivot point
	//warning: in orthographic mode it's important to get the 'real' camera center
	//(i.e. with z == bbCenter(z) and not z == anything)
	//otherwise we (sometimes largely) overestimate the distance between the camera center
	//and the displayed objects if the camera has been shifted in the Z direction (e.g. after
	//switching from perspective to ortho. view).
	//While the user won't see the difference this has a great influence on GL filters
	//(as normalized depth map values depend on it)
	double CP = (cameraCenter - pivotPoint).norm();

	//distance between the pivot point and DB farthest point
	double MP = (bbCenter - pivotPoint).norm() + bbHalfDiag;

	//pivot symbol should always be visible in object-based mode (if enabled)
	if (m_pivotSymbolShown && m_pivotVisibility != PIVOT_HIDE && withGLfeatures && m_viewportParams.objectCenteredView)
	{
		double pivotActualRadius = CC_DISPLAYED_PIVOT_RADIUS_PERCENT * std::min(m_glViewport.width(), m_glViewport.height()) / 2;
		double pivotSymbolScale = pivotActualRadius * computeActualPixelSize();
		MP = std::max(MP, pivotSymbolScale);
	}
	MP *= 1.01; //for round-off issues

	if (withGLfeatures && m_customLightEnabled)
	{
		//distance from custom light to pivot point
		double distToCustomLight = (pivotPoint - CCVector3d::fromArray(m_customLightPos)).norm();
		MP = std::max(MP, distToCustomLight);
	}

	if (m_viewportParams.perspectiveView)
	{
		//we deduce zNear et zFar
		//DGM: the 'zNearCoef' must not be too small, otherwise the loss in accuracy
		//for the detph buffer is too high and the display is jeopardized, especially
		//for entities with big coordinates)
		//double zNear = MP * m_viewportParams.zNearCoef;
		//DGM: what was the purpose of this?!
		//if (m_viewportParams.objectCenteredView)
		//	zNear = std::max<double>(CP-MP, zNear);
		double zFar = std::max(CP + MP, 1.0);
		//double zNear = zFar * m_viewportParams.zNearCoef;
		double zNear = bbHalfDiag * m_viewportParams.zNearCoef; //we want a stable value!

		if (metrics)
		{
			metrics->zNear = zNear;
			metrics->zFar = zFar;
		}

		double currentFov_deg = getFov();

		//compute the aspect ratio
		double ar = static_cast<double>(m_glViewport.height()) / m_glViewport.width();

		double xMax = zNear * std::tan( CCCoreLib::DegreesToRadians( currentFov_deg / 2.0 ) );
		double yMax = xMax * ar;

		//DGM: we now take 'frustumAsymmetry' into account (for stereo rendering)
		double frustumAsymmetry = 0.0;
		if (eyeOffset)
		{
			//see 'NVIDIA 3D VISION PRO AND STEREOSCOPIC 3D' White paper (Oct 2010, p. 12)
			double convergence = bbHalfDiag;
			if (m_viewportParams.objectCenteredView)
			{
				CCVector3d viewDir = getCurrentViewDir();

				CCVector3d realCameraCenter = m_viewportParams.viewMat.inverse() * (cameraCenter - m_viewportParams.pivotPoint) + m_viewportParams.pivotPoint;
				convergence = std::fabs((realCameraCenter - pivotPoint).dot(viewDir))/* / 2.0*/;

				//if (*eyeOffset < 0.0)
				//	const_cast<ccGLWindow*>(this)->displayNewMessage(QString("view dir. = (%1 ; %2 ; %3) / cam = (%4 ; %5 ; %6) / P(%7 ; %8 ; %9) --> convergence = %10")
				//		.arg(viewDir.x, 0, 'f', 2).arg(viewDir.y, 0, 'f', 2).arg(viewDir.z, 0, 'f', 2)
				//		.arg(realCameraCenter.x, 0, 'f', 2).arg(realCameraCenter.y, 0, 'f', 2).arg(realCameraCenter.z, 0, 'f', 2)
				//		.arg(pivotPoint.x, 0, 'f', 2).arg(pivotPoint.y, 0, 'f', 2).arg(pivotPoint.z, 0, 'f', 2)
				//		.arg(convergence), ccGLWindow::LOWER_LEFT_MESSAGE, false, 2, ccGLWindow::PERSPECTIVE_STATE_MESSAGE);
			}

			//we assume zNear = screen distance
			double scale = zNear * m_stereoParams.stereoStrength / m_stereoParams.screenDistance_mm;
			double eyeSeperation = m_stereoParams.eyeSeparation_mm * scale;

			//on input 'eyeOffset' should be -1 (left) or +1 (right)
			*eyeOffset *= eyeSeperation;
			
			frustumAsymmetry = (*eyeOffset) * zNear / convergence;

			//if (*eyeOffset < 0.0)
			//	const_cast<ccGLWindow*>(this)->displayNewMessage(QString("eye sep. = %1 - convergence = %3").arg(*eyeOffset).arg(convergence), ccGLWindow::LOWER_LEFT_MESSAGE, false, 2, ccGLWindow::PERSPECTIVE_STATE_MESSAGE);
		}

		return ccGL::Frustum(-xMax - frustumAsymmetry, xMax - frustumAsymmetry, -yMax, yMax, zNear, zFar);
	}
	else
	{
		//max distance (camera to 'farthest' point)
		double maxDist = CP + MP;

		double maxDist_pix = maxDist / m_viewportParams.pixelSize * m_viewportParams.zoom;
		maxDist_pix = std::max(maxDist_pix, 1.0);

		double halfW = m_glViewport.width() / 2.0;
		double halfH = m_glViewport.height() / 2.0 * m_viewportParams.orthoAspectRatio;

		//save actual zNear and zFar parameters
		double zNear = -maxDist_pix;
		double zFar = maxDist_pix;

		if (metrics)
		{
			metrics->zNear = zNear;
			metrics->zFar = zFar;
		}

		return ccGL::Ortho(halfW, halfH, maxDist_pix);
	}
}

void ccGLWindow::updateProjectionMatrix()
{
	ProjectionMetrics metrics;

	m_projMatd = computeProjectionMatrix
	(
		getRealCameraCenter(),
		true,
		&metrics,
		nullptr
	); //no stereo vision by default!

	m_viewportParams.zNear = metrics.zNear;
	m_viewportParams.zFar = metrics.zFar;
	m_cameraToBBCenterDist = metrics.cameraToBBCenterDist;
	m_bbHalfDiag = metrics.bbHalfDiag;

	m_validProjectionMatrix = true;
}

void ccGLWindow::deprecate3DLayer()
{
	m_updateFBO = true;
}

void ccGLWindow::invalidateVisualization()
{
	m_validModelviewMatrix = false;
}

ccGLMatrixd ccGLWindow::computeModelViewMatrix(const CCVector3d& cameraCenter) const
{
	ccGLMatrixd viewMatd;
	viewMatd.toIdentity();

	//apply current camera parameters (see trunk/doc/rendering_pipeline.doc)
	if (m_viewportParams.objectCenteredView)
	{
		//place origin on pivot point
		viewMatd.setTranslation(/*viewMatd.getTranslationAsVec3D()*/ - m_viewportParams.pivotPoint);

		//rotation (viewMat is simply a rotation matrix around the pivot here!)
		viewMatd = m_viewportParams.viewMat * viewMatd;

		//go back to initial origin
		//then place origin on camera center
		viewMatd.setTranslation(viewMatd.getTranslationAsVec3D() + m_viewportParams.pivotPoint - cameraCenter);
	}
	else
	{
		//place origin on camera center
		viewMatd.setTranslation(/*viewMatd.getTranslationAsVec3D()*/ - cameraCenter);

		//rotation (viewMat is the rotation around the camera center here - no pivot)
		viewMatd = m_viewportParams.viewMat * viewMatd;
	}

	ccGLMatrixd scaleMatd;
	scaleMatd.toIdentity();
	if (m_viewportParams.perspectiveView) //perspective mode
	{
		//for proper aspect ratio handling
		if (m_glViewport.height() != 0)
		{
			double ar = static_cast<double>(m_glViewport.width() / (m_glViewport.height() * m_viewportParams.perspectiveAspectRatio));
			if (ar < 1.0)
			{
				//glScalef(ar, ar, 1.0);
				scaleMatd.data()[0] = ar;
				scaleMatd.data()[5] = ar;
			}
		}
	}
	else //ortho. mode
	{
		//apply zoom
		double totalZoom = m_viewportParams.zoom / m_viewportParams.pixelSize;
		//glScalef(totalZoom,totalZoom,totalZoom);
		scaleMatd.data()[0] = totalZoom;
		scaleMatd.data()[5] = totalZoom;
		scaleMatd.data()[10] = totalZoom;
	}

	return scaleMatd * viewMatd;
}

void ccGLWindow::updateModelViewMatrix()
{
	//we save visualization matrix
	m_viewMatd = computeModelViewMatrix(getRealCameraCenter());

	m_validModelviewMatrix = true;
}

const ccGLMatrixd& ccGLWindow::getBaseViewMat()
{
	return m_viewportParams.viewMat;
}

void ccGLWindow::setBaseViewMat(ccGLMatrixd& mat)
{
	m_viewportParams.viewMat = mat;

	invalidateVisualization();

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_viewportParams.viewMat);
}

void ccGLWindow::getGLCameraParameters(ccGLCameraParameters& params)
{
	//get/compute the modelview matrix
	{
		params.modelViewMat = getModelViewMatrix();
	}

	//get/compute the projection matrix
	{
		params.projectionMat = getProjectionMatrix();
	}

	params.viewport[0] = m_glViewport.x();
	params.viewport[1] = m_glViewport.y();
	params.viewport[2] = m_glViewport.width();
	params.viewport[3] = m_glViewport.height();

	params.perspective = m_viewportParams.perspectiveView;
	params.fov_deg = m_viewportParams.fov;
	params.pixelSize = m_viewportParams.pixelSize;
}

const ccGLMatrixd& ccGLWindow::getModelViewMatrix()
{
	if (!m_validModelviewMatrix)
		updateModelViewMatrix();

	return m_viewMatd;
}

const ccGLMatrixd& ccGLWindow::getProjectionMatrix()
{
	if (!m_validProjectionMatrix)
		updateProjectionMatrix();

	return m_projMatd;
}

void ccGLWindow::setStandardOrthoCenter()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glMatrixMode(GL_PROJECTION);
	glFunc->glLoadIdentity();
	double halfW = m_glViewport.width() / 2.0;
	double halfH = m_glViewport.height() / 2.0;
	double maxS = std::max(halfW, halfH);
	glFunc->glOrtho(-halfW, halfW, -halfH, halfH, -maxS, maxS);
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glLoadIdentity();
}

void ccGLWindow::setStandardOrthoCorner()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glMatrixMode(GL_PROJECTION);
	glFunc->glLoadIdentity();
	glFunc->glOrtho(0.0, m_glViewport.width(), 0.0, m_glViewport.height(), 0.0, 1.0);
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glLoadIdentity();
}

void ccGLWindow::getContext(CC_DRAW_CONTEXT& CONTEXT)
{
	//display size
	CONTEXT.glW = m_glViewport.width();
	CONTEXT.glH = m_glViewport.height();
	CONTEXT.devicePixelRatio = static_cast<float>(devicePixelRatio());
	CONTEXT.display = this;
	CONTEXT.qGLContext = this->context();
	CONTEXT.drawingFlags = 0;

	const ccGui::ParamStruct& guiParams = getDisplayParameters();

	//decimation options
	CONTEXT.decimateCloudOnMove = guiParams.decimateCloudOnMove;
	CONTEXT.minLODPointCount = guiParams.minLoDCloudSize;
	CONTEXT.decimateMeshOnMove = guiParams.decimateMeshOnMove && m_mouseMoved;
	CONTEXT.minLODTriangleCount = guiParams.minLoDMeshSize;
	CONTEXT.higherLODLevelsAvailable = false;
	CONTEXT.moreLODPointsAvailable = false;
	CONTEXT.currentLODLevel = 0;

	//scalar field color-bar
	CONTEXT.sfColorScaleToDisplay = nullptr;

	//point picking
	CONTEXT.labelMarkerSize = static_cast<float>(guiParams.labelMarkerSize * computeActualPixelSize());
	CONTEXT.labelMarkerTextShift_pix = 5; //5 pixels shift

	//text display
	CONTEXT.dispNumberPrecision = guiParams.displayedNumPrecision;
	//label opacity
	CONTEXT.labelOpacity = guiParams.labelOpacity;

	//default materials
	CONTEXT.defaultMat->setDiffuseFront(guiParams.meshFrontDiff);
	CONTEXT.defaultMat->setDiffuseBack(guiParams.meshBackDiff);
	CONTEXT.defaultMat->setAmbient(ccColor::bright);
	CONTEXT.defaultMat->setSpecular(guiParams.meshSpecular);
	CONTEXT.defaultMat->setEmission(ccColor::night);
	CONTEXT.defaultMat->setShininessFront(30);
	CONTEXT.defaultMat->setShininessBack(50);
	//default colors
	CONTEXT.pointsDefaultCol = guiParams.pointsDefaultCol;
	CONTEXT.textDefaultCol = guiParams.textDefaultCol;
	CONTEXT.labelDefaultBkgCol = guiParams.labelBackgroundCol;
	CONTEXT.labelDefaultMarkerCol = guiParams.labelMarkerCol;
	CONTEXT.bbDefaultCol = guiParams.bbDefaultCol;

	//display acceleration
	CONTEXT.useVBOs = guiParams.useVBOs;

	//other options
	CONTEXT.drawRoundedPoints = guiParams.drawRoundedPoints;
}

CCVector3d ccGLWindow::getCurrentViewDir() const
{
	//view direction is (the opposite of) the 3rd line of the current view matrix
	const double* M = m_viewportParams.viewMat.data();
	CCVector3d axis(-M[2], -M[6], -M[10]);
	axis.normalize();

	return axis;
}

CCVector3d ccGLWindow::getCurrentUpDir() const
{
	//if (m_viewportParams.objectCenteredView)
	//	return CCVector3d(0,1,0);

	//otherwise up direction is the 2nd line of the current view matrix
	const double* M = m_viewportParams.viewMat.data();
	CCVector3d axis(M[1], M[5], M[9]);
	axis.normalize();

	return axis;
}

//QString ToString(ccGLWindow::PICKING_MODE mode)
//{
//	switch (mode)
//	{
//	case ccGLWindow::NO_PICKING:
//		return "NO_PICKING";
//	case ccGLWindow::ENTITY_PICKING:
//		return "ENTITY_PICKING";
//	case ccGLWindow::ENTITY_RECT_PICKING:
//		return "ENTITY_RECT_PICKING";
//	case ccGLWindow::FAST_PICKING:
//		return "FAST_PICKING";
//	case ccGLWindow::POINT_PICKING:
//		return "POINT_PICKING";
//	case ccGLWindow::TRIANGLE_PICKING:
//		return "TRIANGLE_PICKING";
//	case ccGLWindow::POINT_OR_TRIANGLE_PICKING:
//		return "POINT_OR_TRIANGLE_PICKING";
//	case ccGLWindow::LABEL_PICKING:
//		return "LABEL_PICKING";
//	case ccGLWindow::DEFAULT_PICKING:
//		return "DEFAULT_PICKING";
//	}
//
//	assert(false);
//	return QString();
//}

void ccGLWindow::setPickingMode(PICKING_MODE mode/*=DEFAULT_PICKING*/)
{
	//is the picking mode locked?
	if (m_pickingModeLocked)
	{
		if ((mode != m_pickingMode) && (mode != DEFAULT_PICKING))
			ccLog::Warning("[ccGLWindow::setPickingMode] Picking mode is locked! Can't change it...");
		return;
	}

	switch (mode)
	{
	case DEFAULT_PICKING:
		mode = ENTITY_PICKING;
	case NO_PICKING:
	case ENTITY_PICKING:
		setCursor(QCursor(Qt::ArrowCursor));
		break;
	case POINT_OR_TRIANGLE_PICKING:
	case TRIANGLE_PICKING:
	case POINT_PICKING:
		setCursor(QCursor(Qt::PointingHandCursor));
		break;
	default:
		break;
	}

	m_pickingMode = mode;

	//ccLog::Warning(QString("[%1] Picking mode set to: ").arg(m_uniqueID) + ToString(m_pickingMode));
}

CCVector3d ccGLWindow::convertMousePositionToOrientation(int x, int y)
{
	double xc = width() / 2.0;
	double yc = height() / 2.0; //DGM FIXME: is it scaled coordinates or not?!

	CCVector3d Q2D;
	if (m_viewportParams.objectCenteredView)
	{
		//project the current pivot point on screen
		ccGLCameraParameters camera;
		getGLCameraParameters(camera);

		if (!camera.project(m_viewportParams.pivotPoint, Q2D))
		{
			//arbitrary direction
			return CCVector3d(0, 0, 1);
		}

		//we set the virtual rotation pivot closer to the actual one (but we always stay in the central part of the screen!)
		Q2D.x = std::min(Q2D.x, 3.0 * width() / 4.0);
		Q2D.x = std::max(Q2D.x, width() / 4.0);

		Q2D.y = std::min(Q2D.y, 3.0 * height() / 4.0);
		Q2D.y = std::max(Q2D.y, height() / 4.0);
	}
	else
	{
		Q2D.x = xc;
		Q2D.y = yc;
	}

	//invert y
	y = height() - 1 - y;

	CCVector3d v(x - Q2D.x, y - Q2D.y, 0.0);

	v.x = std::max(std::min(v.x / xc, 1.0), -1.0);
	v.y = std::max(std::min(v.y / yc, 1.0), -1.0);

	//square 'radius'
	double d2 = v.x*v.x + v.y*v.y;

	//projection on the unit sphere
	if (d2 > 1)
	{
		double d = std::sqrt(d2);
		v.x /= d;
		v.y /= d;
	}
	else
	{
		v.z = std::sqrt(1.0 - d2);
	}

	return v;
}

void ccGLWindow::updateActiveItemsList(int x, int y, bool extendToSelectedLabels/*=false*/)
{
	m_activeItems.clear();

	PickingParameters params(FAST_PICKING, x, y, 2, 2);

	startPicking(params);

	if (m_activeItems.size() == 1)
	{
		ccInteractor* pickedObj = m_activeItems.front();
		cc2DLabel* label = dynamic_cast<cc2DLabel*>(pickedObj);
		if (label)
		{
			if (!label->isSelected() || !extendToSelectedLabels)
			{
				//select it?
				//emit entitySelectionChanged(label);
				//QApplication::processEvents();
			}
			else
			{
				//we get the other selected labels as well!
				ccHObject::Container labels;
				if (m_globalDBRoot)
					m_globalDBRoot->filterChildren(labels, true, CC_TYPES::LABEL_2D);
				if (m_winDBRoot)
					m_winDBRoot->filterChildren(labels, true, CC_TYPES::LABEL_2D);

				for (auto & label : labels)
				{
					if (label->isA(CC_TYPES::LABEL_2D) && label->isVisible()) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
					{
						cc2DLabel* l = static_cast<cc2DLabel*>(label);
						if (l != label && l->isSelected())
						{
							m_activeItems.push_back(l);
						}
					}
				}
			}
		}
	}
}

void ccGLWindow::onItemPickedFast(ccHObject* pickedEntity, int pickedItemIndex, int x, int y)
{
	if (pickedEntity)
	{
		if (pickedEntity->isA(CC_TYPES::LABEL_2D))
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(pickedEntity);
			m_activeItems.push_back(label);
		}
		else if (pickedEntity->isA(CC_TYPES::CLIPPING_BOX))
		{
			ccClipBox* cbox = static_cast<ccClipBox*>(pickedEntity);
			cbox->setActiveComponent(pickedItemIndex);
			cbox->setClickedPoint(x, y, width(), height(), m_viewportParams.viewMat);

			m_activeItems.push_back(cbox);
		}
	}

	emit fastPickingFinished();
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
			QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
		}

		if (m_interactionFlags & INTERACT_SIG_RB_CLICKED)
		{
			emit rightButtonClicked(event->x(), event->y());
		}
	}
	else if (event->buttons() & Qt::LeftButton)
	{
		m_lastClickTime_ticks = m_timer.elapsed(); //in msec

		//left click = rotation
		if (m_interactionFlags & INTERACT_ROTATE)
		{
			QApplication::setOverrideCursor(QCursor(Qt::PointingHandCursor));
		}

		if (m_interactionFlags & INTERACT_SIG_LB_CLICKED)
		{
			emit leftButtonClicked(event->x(), event->y());
		}
	}
	if (event->buttons() & Qt::MiddleButton)
	{
		//middle click = zooming
		if (m_interactionFlags & INTERACT_SIG_MB_CLICKED)
		{
			emit middleButtonClicked(event->x(), event->y());
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
	if (getClick3DPos(x, y, P))
	{
		setPivotPoint(P, true, true);
	}
}

void ccGLWindow::mouseMoveEvent(QMouseEvent *event)
{
	const int x = event->x();
	const int y = event->y();

	if (m_interactionFlags & INTERACT_SIG_MOUSE_MOVED)
	{
		emit mouseMoved(x, y, event->buttons());
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
			if (getClick3DPos(x, y, P))
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
			if (!m_viewportParams.perspectiveView)
			{
				u.y *= m_viewportParams.orthoAspectRatio;
			}

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
					emit translation(u);
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
				moveCamera(static_cast<float>(u.x), static_cast<float>(u.y), static_cast<float>(u.z));
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
			CCVector3d u(dx*pixSize, -dy*pixSize, 0.0);
			m_viewportParams.viewMat.transposed().applyRotation(u);

			const int retinaScale = devicePixelRatio();
			u *= retinaScale;

			for (auto &activeItem : m_activeItems)
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
				//case LockedAxisMode:
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

					//if (rotationMode == LockedAxisMode)
					//{
					//	CCVector3d upAxis = m_lockedRotationAxis;
					//	getBaseViewMat().applyRotation(upAxis);
					//	upAxis.normalize();

					//	ccGLMatrixd upAxisToZ = ccGLMatrixd::FromToRotation(upAxis, CCVector3d(0, 0, 1));
					//	ccGLMatrixd rotMatInTempCS = upAxisToZ * rotMat;
					//	ccGLMatrixd rotMatInTempCSFiltered = rotMatInTempCS/*.zRotation()*/;
					//	ccGLMatrixd rotMatFiltered = upAxisToZ.inverse() * rotMatInTempCSFiltered;
					//	rotMat = rotMatFiltered;
					//}

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
							camera.project(m_viewportParams.pivotPoint, C2D);
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
						if (	camera.project(m_viewportParams.pivotPoint, A2D)
							&&	camera.project(m_viewportParams.pivotPoint + m_viewportParams.zFar * m_lockedRotationAxis, B2D))
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
					emit rotation(rotMat);
				}
				else
				{
					rotateBaseViewMat(rotMat);

					showPivotSymbol(true);
					QApplication::changeOverrideCursor(QCursor(Qt::ClosedHandCursor));

					//feedback for 'echo' mode
					emit viewMatRotated(rotMat);
				}
			}
		}
	}
	else if ((event->buttons() & Qt::MiddleButton)) // zoom
	{
		//middle button = zooming
		float pseudo_wheelDelta_deg = static_cast<float>(-dy);
		onWheelEvent(pseudo_wheelDelta_deg);

		emit mouseWheelRotated(pseudo_wheelDelta_deg);
	}

	m_mouseMoved = true;
	m_lastMousePos = event->pos();

	event->accept();

	if (m_interactionFlags != INTERACT_TRANSFORM_ENTITIES)
	{
		redraw(true);
	}
}

bool ccGLWindow::processClickableItems(int x, int y)
{
	if (m_clickableItems.empty())
	{
		//shortcut
		return false;
	}

	//correction for HD screens
	const int retinaScale = devicePixelRatio();
	x *= retinaScale;
	y *= retinaScale;

	ClickableItem::Role clickedItem = ClickableItem::NO_ROLE;
	for (std::vector<ClickableItem>::const_iterator it = m_clickableItems.begin(); it != m_clickableItems.end(); ++it)
	{
		if (it->area.contains(x, y))
		{
			clickedItem = it->role;
			break;
		}
	}

	switch (clickedItem)
	{
	case ClickableItem::NO_ROLE:
	{
		//nothing to do
	}
	break;
	
	case ClickableItem::INCREASE_POINT_SIZE:
	{
		setPointSize(m_viewportParams.defaultPointSize + 1.0f);
		redraw();
	}
	return true;
	
	case ClickableItem::DECREASE_POINT_SIZE:
	{
		setPointSize(m_viewportParams.defaultPointSize - 1.0f);
		redraw();
	}
	return true;
	
	case ClickableItem::INCREASE_LINE_WIDTH:
	{
		setLineWidth(m_viewportParams.defaultLineWidth + 1.0f);
		redraw();
	}
	return true;
	
	case ClickableItem::DECREASE_LINE_WIDTH:
	{
		setLineWidth(m_viewportParams.defaultLineWidth - 1.0f);
		redraw();
	}
	return true;
	
	case ClickableItem::LEAVE_BUBBLE_VIEW_MODE:
	{
		setBubbleViewMode(false);
		redraw();
	}
	return true;
	
	case ClickableItem::LEAVE_FULLSCREEN_MODE:
	{
		toggleExclusiveFullScreen(false);
	}
	return true;
	
	default:
	{
		//unhandled item?!
		assert(false);
	}
	break;
	}

	return false;
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
	QApplication::restoreOverrideCursor();

	if (m_interactionFlags & INTERACT_SIG_BUTTON_RELEASED)
	{
		event->accept();
		emit buttonReleased();
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
				ccInteractor* item = m_activeItems.front();
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
					m_deferredPickingTimer.start();
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

void ccGLWindow::doPicking()
{
	int x = m_lastMousePos.x();
	int y = m_lastMousePos.y();

	if (x < 0 || y < 0)
	{
		assert(false);
		return;
	}

	if (	(m_pickingMode != NO_PICKING)
		||	(m_interactionFlags & INTERACT_2D_ITEMS))
	{
		if (m_interactionFlags & INTERACT_2D_ITEMS)
		{
			//label selection
			updateActiveItemsList(x, y, false);
			if (!m_activeItems.empty())
			{
				if (m_activeItems.size() == 1)
				{
					ccInteractor* pickedObj = m_activeItems.front();
					cc2DLabel* label = dynamic_cast<cc2DLabel*>(pickedObj);
					if (label && !label->isSelected())
					{
						emit entitySelectionChanged(label);
						QApplication::processEvents();
					}
				}

				//interaction with item(s) such as labels, etc.
				//DGM TODO: to activate only if some items take left clicks into account!
				//for (std::list<ccInteractor*>::iterator it=m_activeItems.begin(); it!=m_activeItems.end(); ++it)
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

		if (m_viewportParams.perspectiveView)
		{
			//same shortcut as Meshlab: change the zNear value
			static const int MAX_INCREMENT = 150;
			int increment = ccViewportParameters::ZNearCoefToIncrement(m_viewportParams.zNearCoef, MAX_INCREMENT+1);
			int newIncrement = std::min(std::max(0, increment + (event->delta() < 0 ? -1 : 1)), MAX_INCREMENT); //the zNearCoef must be < 1! 
			if (newIncrement != increment)
			{
				double newCoef = ccViewportParameters::IncrementToZNearCoef(newIncrement, MAX_INCREMENT+1);
				setZNearCoef(newCoef);
				doRedraw = true;
			}
		}
	}
	else if (keyboardModifiers & Qt::ShiftModifier)
	{
		event->accept();
		
		if (m_viewportParams.perspectiveView)
		{
			//same shortcut as Meshlab: change the fov value
			float newFOV = (m_viewportParams.fov + (event->delta() < 0 ? -1.0f : 1.0f));
			newFOV = std::min(std::max(1.0f, newFOV), 180.0f);
			if (newFOV != m_viewportParams.fov)
			{
				setFov(newFOV);
				doRedraw = true;
			}
		}
	}
	else if (m_interactionFlags & INTERACT_ZOOM_CAMERA)
	{
		event->accept();

		//see QWheelEvent documentation ("distance that the wheel is rotated, in eighths of a degree")
		float wheelDelta_deg = event->delta() / 8.0f;
		onWheelEvent(wheelDelta_deg);

		emit mouseWheelRotated(wheelDelta_deg);

		doRedraw = true;
	}

	if (doRedraw)
	{
		setLODEnabled(true, true);
		m_currentLODState.level = 0;

		redraw();
	}
}

void ccGLWindow::onWheelEvent(float wheelDelta_deg)
{
	//in perspective mode, wheel event corresponds to 'walking'
	if (m_viewportParams.perspectiveView)
	{
		//to zoom in and out we simply change the fov in bubble-view mode!
		if (m_bubbleViewModeEnabled)
		{
			setBubbleViewFov(m_bubbleViewFov_deg - wheelDelta_deg / 3.6f); //1 turn = 100 degrees
		}
		else
		{
			//convert degrees in 'constant' walking speed in ... pixels ;)
			const double& deg2PixConversion = getDisplayParameters().zoomSpeed;
			double delta = deg2PixConversion * static_cast<double>(wheelDelta_deg * m_viewportParams.pixelSize);

			//if we are (clearly) outisde of the displayed objects bounding-box
			if (m_cameraToBBCenterDist > m_bbHalfDiag)
			{
				//we go faster if we are far from the entities
				delta *= 1.0 + std::log(m_cameraToBBCenterDist / m_bbHalfDiag);
			}

			moveCamera(0.0f, 0.0f, static_cast<float>(-delta));
		}
	}
	else //ortho. mode
	{
		//convert degrees in zoom 'power'
		static const float c_defaultDeg2Zoom = 20.0f;
		float zoomFactor = std::pow(1.1f, wheelDelta_deg / c_defaultDeg2Zoom);
		updateZoom(zoomFactor);
	}

	setLODEnabled(true, true);
	m_currentLODState.level = 0;

	redraw();

	//scheduleFullRedraw(1000);
}

void ccGLWindow::startPicking(PickingParameters& params)
{
	//correction for HD screens
	const int retinaScale = devicePixelRatio();
	params.centerX *= retinaScale;
	params.centerY *= retinaScale;

	if (!m_globalDBRoot && !m_winDBRoot)
	{
		//we must always emit a signal!
		processPickingResult(params, nullptr, -1);
		return;
	}

	if (	params.mode == POINT_OR_TRIANGLE_PICKING
		||	params.mode == POINT_PICKING
		||	params.mode == TRIANGLE_PICKING
		||	params.mode == LABEL_PICKING // = spawn a label on the clicked point or triangle
		)
	{
		//CPU-based point picking
		startCPUBasedPointPicking(params);
	}
	else
	{
		startOpenGLPicking(params);
	}
}

void ccGLWindow::processPickingResult(	const PickingParameters& params,
										ccHObject* pickedEntity,
										int pickedItemIndex,
										const CCVector3* nearestPoint/*=nullptr*/,
										const CCVector3d* nearestPointBC/*=nullptr*/,
										const std::unordered_set<int>* selectedIDs/*=nullptr*/)
{
	//standard "entity" picking
	if (params.mode == ENTITY_PICKING)
	{
		emit entitySelectionChanged(pickedEntity);
	}
	//rectangular "entity" picking
	else if (params.mode == ENTITY_RECT_PICKING)
	{
		if (selectedIDs)
			emit entitiesSelectionChanged(*selectedIDs);
		else
			assert(false);
	}
	//3D point or triangle picking
	else if (	params.mode == POINT_PICKING
			||	params.mode == TRIANGLE_PICKING
			||	params.mode == POINT_OR_TRIANGLE_PICKING)
	{
		assert(pickedEntity == nullptr || pickedItemIndex >= 0);
		assert(nearestPoint && nearestPointBC);

		emit itemPicked(pickedEntity, static_cast<unsigned>(pickedItemIndex), params.centerX, params.centerY, *nearestPoint, *nearestPointBC);
	}
	//fast picking (labels, interactors, etc.)
	else if (params.mode == FAST_PICKING)
	{
		emit itemPickedFast(pickedEntity, pickedItemIndex, params.centerX, params.centerY);
	}
	else if (params.mode == LABEL_PICKING)
	{
		if (m_globalDBRoot && pickedEntity && pickedItemIndex >= 0)
		{
			//qint64 stopTime = m_timer.nsecsElapsed();
			//ccLog::Print(QString("[Picking] entity ID %1 - item #%2 (time: %3 ms)").arg(selectedID).arg(pickedItemIndex).arg((stopTime-startTime) / 1.0e6));

			//auto spawn the right label
			cc2DLabel* label = nullptr;
			if (pickedEntity->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				label = new cc2DLabel();
				label->addPickedPoint(ccHObjectCaster::ToGenericPointCloud(pickedEntity), pickedItemIndex);
				pickedEntity->addChild(label);
			}
			else if (pickedEntity->isKindOf(CC_TYPES::MESH))
			{
				assert(nearestPointBC);
				label = new cc2DLabel();
				label->addPickedPoint(ccHObjectCaster::ToGenericMesh(pickedEntity), pickedItemIndex, CCVector2d(nearestPointBC->x, nearestPointBC->y));
				pickedEntity->addChild(label);
			}

			if (label)
			{
				label->setVisible(true);
				label->setDisplay(pickedEntity->getDisplay());
				label->setPosition(	static_cast<float>(params.centerX + 20) / m_glViewport.width(),
									static_cast<float>(params.centerY + 20) / m_glViewport.height());
				emit newLabel(static_cast<ccHObject*>(label));
				QApplication::processEvents();

				toBeRefreshed();
			}
		}
	}
}

//DGM: WARNING: OpenGL picking with the picking buffer is depreacted.
//We need to get rid of this code or change it to color-based selection...
void ccGLWindow::startOpenGLPicking(const PickingParameters& params)
{
	if (!params.pickInLocalDB && !params.pickInSceneDB)
	{
		assert(false);
		return;
	}

	//setup rendering context
	unsigned short flags = CC_DRAW_FOREGROUND;

	switch (params.mode)
	{
	case FAST_PICKING:
		flags |= CC_DRAW_FAST_NAMES_ONLY;
	case ENTITY_PICKING:
	case ENTITY_RECT_PICKING:
		flags |= CC_DRAW_ENTITY_NAMES;
		break;
	default:
		//unhandled mode?!
		assert(false);
		//we must always emit a signal!
		processPickingResult(params, nullptr, -1);
		return;
	}

	//OpenGL picking
	assert(!m_captureMode.enabled);
	makeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//no need to clear display, we don't draw anything new!
	//glFunc->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//OpenGL picking buffer size (= max hits number per 'OpenGL' selection pass)
	static const GLsizei CC_PICKING_BUFFER_SIZE = 65536;
	//GL names picking buffer
	static GLuint s_pickingBuffer[CC_PICKING_BUFFER_SIZE];

	//setup selection buffers
	memset(s_pickingBuffer, 0, sizeof(GLuint)*CC_PICKING_BUFFER_SIZE);
	glFunc->glSelectBuffer(CC_PICKING_BUFFER_SIZE, s_pickingBuffer);
	glFunc->glRenderMode(GL_SELECT);
	glFunc->glInitNames();

	//get viewport
	GLint viewport[4] = { m_glViewport.left(), m_glViewport.top(), m_glViewport.width(), m_glViewport.height() };
	//glFunc->glGetIntegerv(GL_VIEWPORT, viewport);

	//get context
	CC_DRAW_CONTEXT CONTEXT;
	getContext(CONTEXT);

	//3D objects picking
	{
		CONTEXT.drawingFlags = CC_DRAW_3D | flags;

		//projection matrix
		glFunc->glMatrixMode(GL_PROJECTION);
		//restrict drawing to the picking area
		{
			double pickMatrix[16];
			ccGL::PickMatrix(	static_cast<GLdouble>(params.centerX),
								static_cast<GLdouble>(viewport[3] - params.centerY),
								static_cast<GLdouble>(params.pickWidth),
								static_cast<GLdouble>(params.pickWidth),
								viewport,
								pickMatrix);
			glFunc->glLoadMatrixd(pickMatrix);
		}
		glFunc->glMultMatrixd(getProjectionMatrix().data());

		//model view matrix
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glLoadMatrixd(getModelViewMatrix().data());

		glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
		glFunc->glEnable(GL_DEPTH_TEST);

		//display 3D objects
		//DGM: all of them, even if we don't pick the own DB for instance, as they can hide the other objects!
		if (m_globalDBRoot)
			m_globalDBRoot->draw(CONTEXT);
		if (m_winDBRoot)
			m_winDBRoot->draw(CONTEXT);

		glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT

		logGLError("ccGLWindow::startPicking.draw(3D)");
	}

	//2D objects picking
	if (params.mode == ENTITY_PICKING || params.mode == ENTITY_RECT_PICKING || params.mode == FAST_PICKING)
	{
		CONTEXT.drawingFlags = CC_DRAW_2D | flags;

		//we must first grab the 2D ortho view projection matrix
		setStandardOrthoCenter();
		glFunc->glMatrixMode(GL_PROJECTION);
		double orthoProjMatd[OPENGL_MATRIX_SIZE];
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, orthoProjMatd);
		//restrict drawing to the picking area
		{
			double pickMatrix[16];
			ccGL::PickMatrix(	static_cast<GLdouble>(params.centerX),
								static_cast<GLdouble>(viewport[3] - params.centerY),
								static_cast<GLdouble>(params.pickWidth),
								static_cast<GLdouble>(params.pickWidth),
								viewport,
								pickMatrix);
			glFunc->glLoadMatrixd(pickMatrix);
		}
		glFunc->glMultMatrixd(orthoProjMatd);
		glFunc->glMatrixMode(GL_MODELVIEW);

		glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
		glFunc->glDisable(GL_DEPTH_TEST);

		//we display 2D objects
		//DGM: all of them, even if we don't pick the own DB for instance, as they can hide the other objects!
		if (m_globalDBRoot)
			m_globalDBRoot->draw(CONTEXT);
		if (m_winDBRoot)
			m_winDBRoot->draw(CONTEXT);

		glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT

		logGLError("ccGLWindow::startPicking.draw(2D)");
	}

	glFunc->glFlush();

	//back to the standard rendering mode
	int hits = glFunc->glRenderMode(GL_RENDER);

	logGLError("ccGLWindow::startPicking.render");

	ccLog::PrintDebug("[Picking] hits: %i", hits);
	if (hits < 0)
	{
		ccLog::Warning("[Picking] Too many items inside the picking area! Try to zoom in...");
		//we must always emit a signal!
		processPickingResult(params, nullptr, -1);
	}

	//process hits
	std::unordered_set<int> selectedIDs;
	int pickedItemIndex = -1;
	int selectedID = -1;
	try
	{
		GLuint minMinDepth = (~0);
		const GLuint* _selectBuf = s_pickingBuffer;

		for (int i = 0; i < hits; ++i)
		{
			const GLuint& n = _selectBuf[0]; //number of names on stack
			if (n) //if we draw anything outside of 'glPushName()... glPopName()' then it will appear here with as an empty set!
			{
				//n should be equal to 1 (CC_DRAW_ENTITY_NAMES mode) or 2 (CC_DRAW_POINT_NAMES/CC_DRAW_TRIANGLES_NAMES modes)!
				assert(n == 1 || n == 2);
				const GLuint& minDepth = _selectBuf[1];
				//const GLuint& maxDepth = _selectBuf[2];
				const GLuint& currentID = _selectBuf[3];

				if (params.mode == ENTITY_RECT_PICKING)
				{
					//pick them all!
					selectedIDs.insert(currentID);
				}
				else
				{
					//if there are multiple hits, we keep only the nearest
					if (selectedID < 0 || minDepth < minMinDepth)
					{
						selectedID = currentID;
						pickedItemIndex = (n > 1 ? _selectBuf[4] : -1);
						minMinDepth = minDepth;
					}
				}
			}

			_selectBuf += (3 + n);
		}

		//standard output is made through the 'selectedIDs' set
		if (params.mode != ENTITY_RECT_PICKING
			&&	selectedID != -1)
		{
			selectedIDs.insert(selectedID);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Warning("[Picking] Not enough memory!");
	}

	ccHObject* pickedEntity = nullptr;
	if (selectedID >= 0)
	{
		if (params.pickInSceneDB && m_globalDBRoot)
		{
			pickedEntity = m_globalDBRoot->find(selectedID);
		}
		if (!pickedEntity && params.pickInLocalDB && m_winDBRoot)
		{
			pickedEntity = m_winDBRoot->find(selectedID);
		}
	}

	CCVector3 P(0, 0, 0);
	CCVector3d PBC(0, 0, 0);
	CCVector3* pickedPoint = nullptr;
	CCVector3d* pickedBarycenter = nullptr;
	if (pickedEntity && pickedItemIndex >= 0)
	{
		//we need to retrieve the point coordinates
		//(and even the barycentric coordinates if the point is picked on a mesh!)
		if (pickedEntity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			P = *(static_cast<ccGenericPointCloud*>(pickedEntity)->getPoint(pickedItemIndex));
			pickedPoint = &P;
		}
		else if (pickedEntity->isKindOf(CC_TYPES::MESH))
		{
			CCVector2d clickedPos(params.centerX, m_glViewport.height() - 1 - params.centerY);
			ccGLCameraParameters camera;
			getGLCameraParameters(camera);
			CCVector3d Pd(0, 0, 0);
			static_cast<ccGenericMesh*>(pickedEntity)->trianglePicking(static_cast<unsigned>(pickedItemIndex), clickedPos, camera, Pd, &PBC);
			P = CCVector3::fromArray(Pd.u);
			pickedPoint = &P;
			pickedBarycenter = &PBC;
		}
	}

	//we must always emit a signal!
	processPickingResult(params, pickedEntity, pickedItemIndex, pickedPoint, pickedBarycenter, &selectedIDs);
}

void ccGLWindow::startCPUBasedPointPicking(const PickingParameters& params)
{
	//qint64 t0 = m_timer.elapsed();

	CCVector2d clickedPos(params.centerX, m_glViewport.height() - 1 - params.centerY);

	ccHObject* nearestEntity = nullptr;
	int nearestElementIndex = -1;
	double nearestElementSquareDist = -1.0;
	CCVector3 nearestPoint(0, 0, 0);
	CCVector3d nearestPointBC(0, 0, 0);
	static const unsigned MIN_POINTS_FOR_OCTREE_COMPUTATION = 128;

	static ccGui::ParamStruct::ComputeOctreeForPicking autoComputeOctreeThisSession = ccGui::ParamStruct::ASK_USER;
	bool autoComputeOctree = false;
	bool firstCloudWithoutOctree = true;

	ccGLCameraParameters camera;
	getGLCameraParameters(camera);

	try
	{
		ccHObject::Container toProcess;
		if (m_globalDBRoot)
			toProcess.push_back(m_globalDBRoot);
		if (m_winDBRoot)
			toProcess.push_back(m_winDBRoot);

		while (!toProcess.empty())
		{
			//get next item
			ccHObject* ent = toProcess.back();
			toProcess.pop_back();

			if (!ent->isEnabled())
				continue;

			bool ignoreSubmeshes = false;

			//we look for point cloud displayed in this window
			if (ent->isDisplayedIn(this))
			{
				if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);

					if (firstCloudWithoutOctree && !cloud->getOctree() && cloud->size() > MIN_POINTS_FOR_OCTREE_COMPUTATION) //no need to use the octree for a few points!
					{
						//can we compute an octree for picking?
						ccGui::ParamStruct::ComputeOctreeForPicking behavior = getDisplayParameters().autoComputeOctree;
						if (behavior == ccGui::ParamStruct::ASK_USER)
						{
							//we use the persistent parameter for this session
							behavior = autoComputeOctreeThisSession;
						}

						switch (behavior)
						{
						case ccGui::ParamStruct::ALWAYS:
							autoComputeOctree = true;
							break;

						case ccGui::ParamStruct::ASK_USER:
						{
							QMessageBox question(QMessageBox::Question,
								"Picking acceleration",
								"Automatically compute octree(s) to accelerate the picking process?\n(this behavior can be changed later in the Display Settings)",
								QMessageBox::NoButton,
								asWidget());

							QPushButton* yes = new QPushButton("Yes");
							question.addButton(yes, QMessageBox::AcceptRole);
							QPushButton* no = new QPushButton("No");
							question.addButton(no, QMessageBox::RejectRole);
							QPushButton* always = new QPushButton("Always");
							question.addButton(always, QMessageBox::AcceptRole);
							QPushButton* never = new QPushButton("Never");
							question.addButton(never, QMessageBox::RejectRole);

							question.exec();
							QAbstractButton* clickedButton = question.clickedButton();
							if (clickedButton == yes)
							{
								autoComputeOctree = true;
								autoComputeOctreeThisSession = ccGui::ParamStruct::ALWAYS;
							}
							else if (clickedButton == no)
							{
								autoComputeOctree = false;
								autoComputeOctreeThisSession = ccGui::ParamStruct::NEVER;
							}
							else if (clickedButton == always || clickedButton == never)
							{
								autoComputeOctree = (clickedButton == always);
								//update the global application parameters
								ccGui::ParamStruct globalParams = ccGui::Parameters();
								globalParams.autoComputeOctree = autoComputeOctree ? ccGui::ParamStruct::ALWAYS : ccGui::ParamStruct::NEVER;
								ccGui::Set(globalParams);
								globalParams.toPersistentSettings();
							}
						}
						break;

						case ccGui::ParamStruct::NEVER:
							autoComputeOctree = false;
							break;
						}

						firstCloudWithoutOctree = false;
					}

					int nearestPointIndex = -1;
					double nearestSquareDist = 0.0;

					if (cloud->pointPicking(clickedPos,
						camera,
						nearestPointIndex,
						nearestSquareDist,
						params.pickWidth,
						params.pickHeight,
						autoComputeOctree && cloud->size() > MIN_POINTS_FOR_OCTREE_COMPUTATION))
					{
						if (nearestElementIndex < 0 || (nearestPointIndex >= 0 && nearestSquareDist < nearestElementSquareDist))
						{
							nearestElementSquareDist = nearestSquareDist;
							nearestElementIndex = nearestPointIndex;
							nearestPoint = *(cloud->getPoint(nearestPointIndex));
							nearestEntity = cloud;
						}
					}
				}
				else if (	ent->isKindOf(CC_TYPES::MESH)
						&&	!ent->isA(CC_TYPES::MESH_GROUP)) //we don't need to process mesh groups as their children will be processed later
				{
					ignoreSubmeshes = true;

					ccGenericMesh* mesh = static_cast<ccGenericMesh*>(ent);
					if (mesh->isShownAsWire())
					{
						//skip meshes that are displayed in wireframe mode
						continue;
					}

					int nearestTriIndex = -1;
					double nearestSquareDist = 0.0;
					CCVector3d P;
					CCVector3d barycentricCoords;
					if (mesh->trianglePicking(	clickedPos,
												camera,
												nearestTriIndex,
												nearestSquareDist,
												P,
												&barycentricCoords))
					{
						if (nearestElementIndex < 0 || (nearestTriIndex >= 0 && nearestSquareDist < nearestElementSquareDist))
						{
							nearestElementSquareDist = nearestSquareDist;
							nearestElementIndex = nearestTriIndex;
							nearestPoint = CCVector3::fromArray(P.u);
							nearestEntity = mesh;
							nearestPointBC = barycentricCoords;
						}
					}
				}
			}

			//add children
			for (unsigned i = 0; i < ent->getChildrenNumber(); ++i)
			{
				//we ignore the sub-meshes of the current (mesh) entity
				//as their content is the same!
				if (ignoreSubmeshes
					&&	ent->getChild(i)->isKindOf(CC_TYPES::SUB_MESH)
					&& static_cast<ccSubMesh*>(ent)->getAssociatedMesh() == ent)
				{
					continue;
				}

				toProcess.push_back(ent->getChild(i));
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Warning("[Picking][CPU] Not enough memory!");
	}

	//qint64 dt = m_timer.elapsed() - t0;
	//ccLog::Print(QString("[Picking][CPU] Time: %1 ms").arg(dt));

	//we must always emit a signal!
	processPickingResult(params, nearestEntity, nearestElementIndex, &nearestPoint, &nearestPointBC);
}

void ccGLWindow::displayNewMessage(	const QString& message,
									MessagePosition pos,
									bool append/*=false*/,
									int displayMaxDelay_sec/*=2*/,
									MessageType type/*=CUSTOM_MESSAGE*/)
{
	if (message.isEmpty())
	{
		if (!append)
		{
			std::list<MessageToDisplay>::iterator it = m_messagesToDisplay.begin();
			while (it != m_messagesToDisplay.end())
			{
				//same position? we remove the message
				if (it->position == pos)
					it = m_messagesToDisplay.erase(it);
				else
					++it;
			}
		}
		else
		{
			ccLog::Warning("[ccGLWindow::displayNewMessage] Appending an empty message has no effect!");
		}
		return;
	}

	//shall we replace the equivalent message(if any)?
	if (!append)
	{
		//only if type is not 'custom'
		if (type != CUSTOM_MESSAGE)
		{
			for (std::list<MessageToDisplay>::iterator it = m_messagesToDisplay.begin(); it != m_messagesToDisplay.end();)
			{
				//same type? we remove it
				if (it->type == type)
					it = m_messagesToDisplay.erase(it);
				else
					++it;
			}
		}
	}
	else
	{
		if (pos == SCREEN_CENTER_MESSAGE)
		{
			ccLog::Warning("[ccGLWindow::displayNewMessage] Append is not supported for center screen messages!");
		}
	}

	MessageToDisplay mess;
	mess.message = message;
	mess.messageValidity_sec = m_timer.elapsed() / 1000 + displayMaxDelay_sec;
	mess.position = pos;
	mess.type = type;
	m_messagesToDisplay.push_back(mess);

	//ccLog::Print(QString("[displayNewMessage] New message valid until %1 s.").arg(mess.messageValidity_sec));
}

void ccGLWindow::setPointSize(float size, bool silent/*=false*/)
{
	float newSize = std::max(std::min(size, MAX_POINT_SIZE_F), MIN_POINT_SIZE_F);
	
	if (m_viewportParams.defaultPointSize != newSize)
	{
		m_viewportParams.defaultPointSize = newSize;
		deprecate3DLayer();
	
		if (!silent)
		{
			displayNewMessage(	QString("New default point size: %1").arg(newSize),
								ccGLWindow::LOWER_LEFT_MESSAGE,
								false,
								2,
								SCREEN_SIZE_MESSAGE); //DGM HACK: we cheat and use the same 'slot' as the window size
		}
	}
	else
	{
		if (!silent)
		{
			if (size < MIN_POINT_SIZE_F)
				ccLog::Print(QString("Defaut point size is already at minimum : %1").arg(MIN_POINT_SIZE_F));
			else
				ccLog::Print(QString("Defaut point size is already at maximum : %1").arg(MAX_POINT_SIZE_F));
		}
	}
}

void ccGLWindow::setLineWidth(float width, bool silent/*=false*/)
{
	float newWidth = std::max(std::min(width, MAX_LINE_WIDTH_F), MIN_LINE_WIDTH_F);
	
	if (m_viewportParams.defaultLineWidth != newWidth)
	{
		m_viewportParams.defaultLineWidth = newWidth;
		deprecate3DLayer();
		if (!silent)
		{
			displayNewMessage(QString("New default line width: %1").arg(newWidth),
				ccGLWindow::LOWER_LEFT_MESSAGE,
				false,
				2,
				SCREEN_SIZE_MESSAGE); //DGM HACK: we cheat and use the same 'slot' as the window size
		}
	}
	else
	{
		if (!silent)
		{
			if (width < MIN_LINE_WIDTH_F)
				ccLog::Print(QString("Defaut line width is already at minimum : %1").arg(MIN_LINE_WIDTH_F));
			else
				ccLog::Print(QString("Defaut line width is already at maximum : %1").arg(MAX_LINE_WIDTH_F));
		}
	}
}

int FontSizeModifier(int fontSize, float zoomFactor)
{
	int scaledFontSize = static_cast<int>(std::floor(fontSize * zoomFactor));
	if (zoomFactor >= 2.0f)
		scaledFontSize -= static_cast<int>(zoomFactor);
	if (scaledFontSize < 1)
		scaledFontSize = 1;
	return scaledFontSize;
}

int ccGLWindow::getFontPointSize() const
{
	return (m_captureMode.enabled ? FontSizeModifier(getDisplayParameters().defaultFontSize, m_captureMode.zoomFactor) : getDisplayParameters().defaultFontSize) * devicePixelRatio();
}

void ccGLWindow::setFontPointSize(int pixelSize)
{
	m_font.setPointSize(pixelSize);
}

QFont ccGLWindow::getTextDisplayFont() const
{
	return m_font;
}

int ccGLWindow::getLabelFontPointSize() const
{
	return (m_captureMode.enabled ? FontSizeModifier(getDisplayParameters().labelFontSize, m_captureMode.zoomFactor) : getDisplayParameters().labelFontSize) * devicePixelRatio();
}

QFont ccGLWindow::getLabelDisplayFont() const
{
	QFont font = m_font;
	font.setPointSize(getLabelFontPointSize());
	return font;
}

void ccGLWindow::glEnableSunLight()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glLightfv(GL_LIGHT0, GL_DIFFUSE, getDisplayParameters().lightDiffuseColor.rgba);
	glFunc->glLightfv(GL_LIGHT0, GL_AMBIENT, getDisplayParameters().lightAmbientColor.rgba);
	glFunc->glLightfv(GL_LIGHT0, GL_SPECULAR, getDisplayParameters().lightSpecularColor.rgba);
	glFunc->glLightfv(GL_LIGHT0, GL_POSITION, m_sunLightPos);
	glFunc->glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, getDisplayParameters().lightDoubleSided ? GL_TRUE : GL_FALSE);
	glFunc->glEnable(GL_LIGHT0);
}

void ccGLWindow::glDisableSunLight()
{
	functions()->glDisable(GL_LIGHT0);
}

void ccGLWindow::setSunLight(bool state)
{
	m_sunLightEnabled = state;
	displayNewMessage(	state ? "Sun light ON" : "Sun light OFF",
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						SUN_LIGHT_STATE_MESSAGE);
	redraw();

	//save parameter
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		settings.setValue(c_ps_sunLight, m_sunLightEnabled);
	}
}

void ccGLWindow::toggleSunLight()
{
	setSunLight(!m_sunLightEnabled);
}

void ccGLWindow::glEnableCustomLight()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glLightfv(GL_LIGHT1, GL_DIFFUSE, getDisplayParameters().lightDiffuseColor.rgba);
	glFunc->glLightfv(GL_LIGHT1, GL_AMBIENT, getDisplayParameters().lightAmbientColor.rgba);
	glFunc->glLightfv(GL_LIGHT1, GL_SPECULAR, getDisplayParameters().lightSpecularColor.rgba);
	glFunc->glLightfv(GL_LIGHT1, GL_POSITION, m_customLightPos);
	glFunc->glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glFunc->glEnable(GL_LIGHT1);
}

void ccGLWindow::glDisableCustomLight()
{
	functions()->glDisable(GL_LIGHT1);
}

void ccGLWindow::setCustomLight(bool state)
{
	m_customLightEnabled = state;
	displayNewMessage(	state ? "Custom light ON" : "Custom light OFF",
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						CUSTOM_LIGHT_STATE_MESSAGE);

	invalidateViewport();
	deprecate3DLayer();
	redraw();

	//save parameter
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		settings.setValue(c_ps_customLight, m_customLightEnabled);
	}
}

void ccGLWindow::toggleCustomLight()
{
	setCustomLight(!m_customLightEnabled);
}

void ccGLWindow::drawCustomLight()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glColor4ubv(ccColor::yellow.rgba);
	//ensure that the star size is constant (in pixels)
	GLfloat d = static_cast<GLfloat>(CC_DISPLAYED_CUSTOM_LIGHT_LENGTH * computeActualPixelSize());

	//force line width
	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(1.0f);

	glFunc->glBegin(GL_LINES);
	glFunc->glVertex3f(m_customLightPos[0] - d, m_customLightPos[1], m_customLightPos[2]);
	glFunc->glVertex3f(m_customLightPos[0] + d, m_customLightPos[1], m_customLightPos[2]);
	glFunc->glVertex3f(m_customLightPos[0], m_customLightPos[1] - d, m_customLightPos[2]);
	glFunc->glVertex3f(m_customLightPos[0], m_customLightPos[1] + d, m_customLightPos[2]);
	glFunc->glVertex3f(m_customLightPos[0], m_customLightPos[1], m_customLightPos[2] - d);
	glFunc->glVertex3f(m_customLightPos[0], m_customLightPos[1], m_customLightPos[2] + d);
	glFunc->glEnd();

	glFunc->glPopAttrib(); //GL_LINE_BIT
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

void ccGLWindow::setPivotVisibility(PivotVisibility vis)
{
	m_pivotVisibility = vis;

	//auto-save last pivot visibility settings
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		settings.setValue(c_ps_pivotVisibility, vis);
		settings.endGroup();
	}
}

ccGLWindow::PivotVisibility ccGLWindow::getPivotVisibility() const
{
	return m_pivotVisibility;
}

void ccGLWindow::showPivotSymbol(bool state)
{
	//is the pivot really going to be drawn?
	if (state && !m_pivotSymbolShown && m_viewportParams.objectCenteredView && m_pivotVisibility != PIVOT_HIDE)
	{
		invalidateViewport();
		deprecate3DLayer();
	}

	m_pivotSymbolShown = state;
}

void ccGLWindow::drawPivot()
{
	if (!m_viewportParams.objectCenteredView ||
			(m_pivotVisibility == PIVOT_HIDE) ||
			(m_pivotVisibility == PIVOT_SHOW_ON_MOVE && !m_pivotSymbolShown))
	{
		return;
	}

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	//place origin on pivot point
	glFunc->glTranslated(m_viewportParams.pivotPoint.x, m_viewportParams.pivotPoint.y, m_viewportParams.pivotPoint.z);

	//compute actual symbol radius
	double symbolRadius = CC_DISPLAYED_PIVOT_RADIUS_PERCENT * std::min(m_glViewport.width(), m_glViewport.height()) / 2.0;

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
		const float c_alpha = 0.6f;

		//pivot symbol: 3 circles
		glFunc->glColor4f(1.0f, 0.0f, 0.0f, c_alpha);
		glDrawUnitCircle(context(), 0);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(-1.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(1.0f, 0.0f, 0.0f);
		glFunc->glEnd();

		glFunc->glColor4f(0.0f, 1.0f, 0.0f, c_alpha);
		glDrawUnitCircle(context(), 1);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(0.0f, -1.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 1.0f, 0.0f);
		glFunc->glEnd();

		glFunc->glColor4f(0.0f, 0.7f, 1.0f, c_alpha);
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

void ccGLWindow::togglePerspective(bool objectCentered)
{
	if (m_viewportParams.objectCenteredView != objectCentered)
		setPerspectiveState(true, objectCentered);
	else
		setPerspectiveState(!m_viewportParams.perspectiveView, objectCentered);
}

double ccGLWindow::computeActualPixelSize() const
{
	if (!m_viewportParams.perspectiveView)
	{
		return m_viewportParams.pixelSize / m_viewportParams.zoom;
	}

	//int minScreenDim = std::min(m_glViewport.width(), m_glViewport.height());
	//if (minScreenDim <= 0)
	//	return 1.0;

	if (m_glViewport.width() <= 0)
		return 1.0;

	//Camera center to pivot vector
	double zoomEquivalentDist = (m_viewportParams.cameraCenter - m_viewportParams.pivotPoint).norm();

	//return zoomEquivalentDist * (2.0 * std::tan(std::min(getFov(), 75.0f) / 2.0 *CCCoreLib::CCCoreLib::DEG_TO_RAD )) / minScreenDim; //tan(75) = 3.73 (then it quickly increases!)
	return zoomEquivalentDist * (2.0 * std::tan( CCCoreLib::DegreesToRadians( std::min(getFov(), 75.0f) / 2.0 ) )) / m_glViewport.width(); //tan(75) = 3.73 (then it quickly increases!)
}

float ccGLWindow::computePerspectiveZoom() const
{
	//DGM: in fact it can be useful to compute it even in ortho mode :)
	//if (!m_viewportParams.perspectiveView)
	//	return m_viewportParams.zoom;

	//we compute the zoom equivalent to the corresponding camera position (inverse of above calculus)
	float currentFov_deg = getFov();
	if ( CCCoreLib::LessThanEpsilon( currentFov_deg ) )
	{
		return 1.0f;
	}
	
	//Camera center to pivot vector
	double zoomEquivalentDist = (m_viewportParams.cameraCenter - m_viewportParams.pivotPoint).norm();
	if ( CCCoreLib::LessThanEpsilon( zoomEquivalentDist ) )
	{
		return 1.0f;
	}
	
	//float screenSize = std::min(m_glViewport.width(), m_glViewport.height()) * m_viewportParams.pixelSize; //see how pixelSize is computed!
	float screenSize = m_glViewport.width() * m_viewportParams.pixelSize; //see how pixelSize is computed!
	return screenSize / static_cast<float>(zoomEquivalentDist * 2.0 * std::tan( CCCoreLib::DegreesToRadians( currentFov_deg / 2.0 ) ));
}

void ccGLWindow::setBubbleViewMode(bool state)
{
	//Backup the camera center before entering this mode!
	bool bubbleViewModeWasEnabled = m_bubbleViewModeEnabled;
	if (!m_bubbleViewModeEnabled && state)
	{
		m_preBubbleViewParameters = m_viewportParams;
	}

	if (state)
	{
		//bubble-view mode = viewer-based perspective mode
		//setPerspectiveState must be called first as it
		//automatically deactivates bubble-view mode!
		setPerspectiveState(true, false);

		m_bubbleViewModeEnabled = true;

		//when entering this mode, we reset the f.o.v.
		m_bubbleViewFov_deg = 0.0f; //to trick the signal emission mechanism
		setBubbleViewFov(90.0f);
	}
	else if (bubbleViewModeWasEnabled)
	{
		m_bubbleViewModeEnabled = false;
		setPerspectiveState(m_preBubbleViewParameters.perspectiveView, m_preBubbleViewParameters.objectCenteredView);

		//when leaving this mode, we restore the original camera center
		setViewportParameters(m_preBubbleViewParameters);
	}
}

void ccGLWindow::setPerspectiveState(bool state, bool objectCenteredView)
{
	//previous state
	bool perspectiveWasEnabled = m_viewportParams.perspectiveView;
	bool viewWasObjectCentered = m_viewportParams.objectCenteredView;

	//new state
	m_viewportParams.perspectiveView = state;
	m_viewportParams.objectCenteredView = objectCenteredView;

	//Camera center to pivot vector
	CCVector3d PC = m_viewportParams.cameraCenter - m_viewportParams.pivotPoint;

	if (m_viewportParams.perspectiveView)
	{
		if (!perspectiveWasEnabled) //from ortho. mode to perspective view
		{
			//we compute the camera position that gives 'quite' the same view as the ortho one
			//(i.e. we replace the zoom by setting the camera at the right distance from
			//the pivot point)
			double currentFov_deg = getFov();
			assert( CCCoreLib::GreaterThanEpsilon( currentFov_deg ) );
			//double screenSize = std::min(m_glViewport.width(), m_glViewport.height()) * m_viewportParams.pixelSize; //see how pixelSize is computed!
			double screenSize = m_glViewport.width() * m_viewportParams.pixelSize; //see how pixelSize is computed!
			if (screenSize > 0.0)
			{
				PC.z = screenSize / (m_viewportParams.zoom * 2.0 * std::tan( CCCoreLib::DegreesToRadians( currentFov_deg / 2.0 ) ));
			}
		}

		//display message
		displayNewMessage(objectCenteredView ? "Centered perspective ON" : "Viewer-based perspective ON",
			ccGLWindow::LOWER_LEFT_MESSAGE,
			false,
			2,
			PERSPECTIVE_STATE_MESSAGE);
	}
	else
	{
		m_viewportParams.objectCenteredView = true; //object-centered mode is forced for otho. view

		if (perspectiveWasEnabled) //from perspective view to ortho. view
		{
			//we compute the zoom equivalent to the corresponding camera position (inverse of above calculus)
			float newZoom = computePerspectiveZoom();
			setZoom(newZoom);
		}

		displayNewMessage("Perspective OFF",
			ccGLWindow::LOWER_LEFT_MESSAGE,
			false,
			2,
			PERSPECTIVE_STATE_MESSAGE);
	}

	//if we change form object-based to viewer-based visualization, we must
	//'rotate' around the object (or the opposite ;)
	if (viewWasObjectCentered && !m_viewportParams.objectCenteredView)
	{
		m_viewportParams.viewMat.transposed().apply(PC); //inverse rotation
	}
	else if (!viewWasObjectCentered && m_viewportParams.objectCenteredView)
	{
		m_viewportParams.viewMat.apply(PC);
	}

	setCameraPos(m_viewportParams.pivotPoint + PC);

	emit perspectiveStateChanged();

	//auto-save last perspective settings
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		//write parameters
		settings.setValue(c_ps_perspectiveView, m_viewportParams.perspectiveView);
		settings.setValue(c_ps_objectMode, m_viewportParams.objectCenteredView);
		settings.endGroup();
	}

	m_bubbleViewModeEnabled = false;
	invalidateViewport();
	invalidateVisualization();
	deprecate3DLayer();
}

void ccGLWindow::setAspectRatio(float ar)
{
	if (ar < 0.0f)
	{
		ccLog::Warning("[ccGLWindow::setAspectRatio] Invalid AR value!");
		return;
	}

	if (m_viewportParams.perspectiveAspectRatio != ar)
	{
		//update param
		m_viewportParams.perspectiveAspectRatio = ar;

		//and camera state (if perspective view is 'on')
		if (m_viewportParams.perspectiveView)
		{
			invalidateViewport();
			invalidateVisualization();
			deprecate3DLayer();
		}
	}
}

void ccGLWindow::setFov(float fov_deg)
{
	if ( CCCoreLib::LessThanEpsilon( fov_deg ) || (fov_deg > 180.0f))
	{
		ccLog::Warning("[ccGLWindow::setFov] Invalid FOV value!");
		return;
	}

	//derivation if we are in bubble-view mode
	if (m_bubbleViewModeEnabled)
	{
		setBubbleViewFov(fov_deg);
	}
	else if (m_viewportParams.fov != fov_deg)
	{
		//update param
		m_viewportParams.fov = fov_deg;
		//and camera state (if perspective view is 'on')
		if (m_viewportParams.perspectiveView)
		{
			invalidateViewport();
			invalidateVisualization();
			deprecate3DLayer();

			displayNewMessage(	QString("F.O.V. = %1 deg.").arg(fov_deg, 0, 'f', 1),
								ccGLWindow::LOWER_LEFT_MESSAGE, //DGM HACK: we cheat and use the same 'slot' as the window size
								false,
								2,
								SCREEN_SIZE_MESSAGE);
		}

		emit fovChanged(m_viewportParams.fov);
	}
}

float ccGLWindow::getFov() const
{
	return (m_bubbleViewModeEnabled ? m_bubbleViewFov_deg : m_viewportParams.fov);
}

void ccGLWindow::setBubbleViewFov(float fov_deg)
{
	if ( CCCoreLib::LessThanEpsilon( fov_deg ) || (fov_deg > 180.0f))
	{
		return;
	}
	
	if (fov_deg != m_bubbleViewFov_deg)
	{
		m_bubbleViewFov_deg = fov_deg;

		if (m_bubbleViewModeEnabled)
		{
			invalidateViewport();
			invalidateVisualization();
			deprecate3DLayer();
			emit fovChanged(m_bubbleViewFov_deg);
		}
	}
}

void ccGLWindow::setZNearCoef(double coef)
{
	if (coef <= 0.0 || coef >= 1.0)
	{
		ccLog::Warning("[ccGLWindow::setZNearCoef] Invalid coef. value!");
		return;
	}

	if (m_viewportParams.zNearCoef != coef)
	{
		//update param
		m_viewportParams.zNearCoef = coef;
		//and camera state (if perspective view is 'on')
		if (m_viewportParams.perspectiveView)
		{
			//invalidateViewport();
			//invalidateVisualization();

			//DGM: we update the projection matrix directly so as to get an up-to-date estimation of zNear
			updateProjectionMatrix();

			deprecate3DLayer();

			displayNewMessage(	QString("Near clipping = %1% of max depth (= %2)").arg(m_viewportParams.zNearCoef * 100.0, 0, 'f', 1).arg(m_viewportParams.zNear),
								ccGLWindow::LOWER_LEFT_MESSAGE, //DGM HACK: we cheat and use the same 'slot' as the window size
								false,
								2,
								SCREEN_SIZE_MESSAGE);
		}

		emit zNearCoefChanged(coef);
	}
}

void ccGLWindow::setViewportParameters(const ccViewportParameters& params)
{
	ccViewportParameters oldParams = m_viewportParams;
	m_viewportParams = params;

	if (m_stereoModeEnabled && !params.perspectiveView)
	{
		ccLog::Warning("Applied viewport projeciton is not perspective: stereo mode will be automatically disabled");
		disableStereoMode();
	}

	invalidateViewport();
	invalidateVisualization();
	deprecate3DLayer();

	emit baseViewMatChanged(m_viewportParams.viewMat);
	emit pivotPointChanged(m_viewportParams.pivotPoint);
	emit cameraPosChanged(m_viewportParams.cameraCenter);
	emit fovChanged(m_viewportParams.fov);
}

void ccGLWindow::rotateBaseViewMat(const ccGLMatrixd& rotMat)
{
	m_viewportParams.viewMat = rotMat * m_viewportParams.viewMat;

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_viewportParams.viewMat);

	invalidateVisualization();
	deprecate3DLayer();
}

void ccGLWindow::updateZoom(float zoomFactor)
{
	//no 'zoom' in viewer based perspective
	assert(!m_viewportParams.perspectiveView);

	if (zoomFactor > 0.0f && zoomFactor != 1.0f)
	{
		setZoom(m_viewportParams.zoom*zoomFactor);
	}
}

void ccGLWindow::setupProjectiveViewport(const ccGLMatrixd& cameraMatrix,
	float fov_deg/*=0.0f*/,
	float ar/*=1.0f*/,
	bool viewerBasedPerspective/*=true*/,
	bool bubbleViewMode/*=false*/)
{
	//perspective (viewer-based by default)
	if (bubbleViewMode)
		setBubbleViewMode(true);
	else
		setPerspectiveState(true, !viewerBasedPerspective);

	//field of view (= OpenGL 'fovy' but in degrees)
	if (fov_deg > 0.0f)
	{
		setFov(fov_deg);
	}

	//aspect ratio
	setAspectRatio(ar);

	//set the camera matrix 'translation' as OpenGL camera center
	CCVector3d T = cameraMatrix.getTranslationAsVec3D();
	setCameraPos(T);
	if (viewerBasedPerspective)
	{
		setPivotPoint(T);
	}

	//apply orientation matrix
	ccGLMatrixd trans = cameraMatrix;
	trans.clearTranslation();
	trans.invert();
	setBaseViewMat(trans);

	redraw();
}

void ccGLWindow::setCustomView(const CCVector3d& forward, const CCVector3d& up, bool forceRedraw/*=true*/)
{
	bool wasViewerBased = !m_viewportParams.objectCenteredView;
	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView, true);

	ccGLMatrixd viewMat = ccGLMatrixd::FromViewDirAndUpDir(forward, up);
	setBaseViewMat(viewMat);

	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView, false);

	if (forceRedraw)
		redraw();
}

void ccGLWindow::setView(CC_VIEW_ORIENTATION orientation, bool forceRedraw/*=true*/)
{
	bool wasViewerBased = !m_viewportParams.objectCenteredView;
	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView, true);

	m_viewportParams.viewMat = ccGLUtils::GenerateViewMat(orientation);

	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView, false);

	invalidateVisualization();
	deprecate3DLayer();

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_viewportParams.viewMat);

	if (forceRedraw)
		redraw();
}

bool ccGLWindow::renderToFile(	QString filename,
								float zoomFactor/*=1.0*/,
								bool dontScaleFeatures/*=false*/,
								bool renderOverlayItems/*=false*/)
{
	if (filename.isEmpty() || zoomFactor < 1.0e-2f)
	{
		return false;
	}

	QImage outputImage = renderToImage(zoomFactor, dontScaleFeatures, renderOverlayItems);

	if (outputImage.isNull())
	{
		//an error occurred (message should have already been issued!)
		return false;
	}

	if (getDisplayParameters().drawRoundedPoints)
	{
		//convert the image to plain RGB to avoid issues with points transparency when saving to PNG
		outputImage = outputImage.convertToFormat(QImage::Format_RGB32);
	}

	bool success = outputImage.convertToFormat(QImage::Format_RGB32).save(filename);
	if (success)
	{
		ccLog::Print(QString("[Snapshot] File '%1' saved! (%2 x %3 pixels)").arg(filename).arg(outputImage.width()).arg(outputImage.height()));
	}
	else
	{
		ccLog::Print(QString("[Snapshot] Failed to save file '%1'!").arg(filename));
	}

	return success;
}

void ccGLWindow::setShaderPath( const QString &path )
{
	(*s_shaderPath) = path;
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

		bool success = (	fbo->init(m_glViewport.width(), m_glViewport.height())
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
			if (!m_activeGLFilter->init(m_glViewport.width(), m_glViewport.height(), *s_shaderPath, error))
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

	//updateZoom(zoomFactor);
	float originalZoom = m_viewportParams.zoom;
	setZoom(m_viewportParams.zoom * zoomFactor);

	//disable LOD!
	bool wasLODEnabled = isLODEnabled();
	setLODEnabled(false);

	//enable the FBO
	bindFBO(fbo);
	logGLError("ccGLWindow::renderToFile/FBO start");

	fullRenderingPass(CONTEXT, renderingParams);

	setZoom(originalZoom);

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
			parameters.zoom = m_viewportParams.perspectiveView ? computePerspectiveZoom() : m_viewportParams.zoom * zoomFactor; //TODO: doesn't work well with EDL in perspective mode!
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

	if (m_displayOverlayEntities && m_captureMode.renderOverlayItems)
	{
		//scale: only in ortho mode
		if (!m_viewportParams.perspectiveView)
		{
			//DGM FIXME: with a zoom > 1, the renderText call inside drawScale will result in the wrong FBO being used?!
			drawScale(getDisplayParameters().textDefaultCol);
		}

		//trihedron
		drawTrihedron();
	}

	glFunc->glFlush();

	//read from fbo
	glFunc->glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	//to avoid memory issues, we read line by line
	for (int i = 0; i < m_glViewport.height(); ++i)
	{
		glFunc->glReadPixels(0, i, m_glViewport.width(), 1, GL_BGRA, GL_UNSIGNED_BYTE, data + (m_glViewport.height() - 1 - i) * m_glViewport.width() * 4);
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
		m_activeGLFilter->init(m_glViewport.width(), m_glViewport.height(), *s_shaderPath, error);
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

void ccGLWindow::removeFBO()
{
	removeFBOSafe(m_fbo);
	removeFBOSafe(m_fbo2);
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

void ccGLWindow::removeGLFilter()
{
	//we "disconnect" current glFilter, to avoid wrong display/errors
	//if QT tries to redraw window during object destruction
	ccGlFilter* _filter = nullptr;
	std::swap(_filter, m_activeGLFilter);

	if (_filter)
	{
		delete _filter;
		_filter = nullptr;
	}
}

bool ccGLWindow::initGLFilter(int w, int h, bool silent/*=false*/)
{
	if (!m_activeGLFilter)
	{
		return false;
	}

	makeCurrent();
	
	//correction for HD screens
	const int retinaScale = devicePixelRatio();
	w *= retinaScale;
	h *= retinaScale;

	//we "disconnect" current glFilter, to avoid wrong display/errors
	//if QT tries to redraw window during initialization
	ccGlFilter* _filter = nullptr;
	std::swap(_filter, m_activeGLFilter);

	QString error;
	if (!_filter->init(static_cast<unsigned>(w), static_cast<unsigned>(h), *s_shaderPath, error))
	{
		if (!silent)
		{
			ccLog::Warning(QString("[GL Filter] Initialization failed: ") + error.trimmed());
		}
		return false;
	}

	if (!silent)
	{
		ccLog::Print("[GL Filter] Filter initialized");
	}

	m_activeGLFilter = _filter;

	return true;
}

int ccGLWindow::getGlFilterBannerHeight() const
{
	return QFontMetrics(font()).height() + 2 * CC_GL_FILTER_BANNER_MARGIN;
}

void ccGLWindow::display3DLabel(const QString& str, const CCVector3& pos3D, const ccColor::Rgba* color/*=nullptr*/, const QFont& font/*=QFont()*/)
{
	glColor4ubv_safe<ccQOpenGLFunctions>(functions(), color ? color->rgba : getDisplayParameters().textDefaultCol.rgba);
	renderText(pos3D.x, pos3D.y, pos3D.z, str, font);
}

void ccGLWindow::displayText(	QString text,
								int x,
								int y,
								unsigned char align/*=ALIGN_HLEFT|ALIGN_VTOP*/,
								float bkgAlpha/*=0*/,
								const ccColor::Rgba* color/*=0*/,
								const QFont* font/*=0*/)
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	int x2 = x;
	int y2 = m_glViewport.height() - 1 - y;

	//actual text color
	const unsigned char* rgba = (color ? color->rgba : getDisplayParameters().textDefaultCol.rgba);

	QFont textFont = (font ? *font : m_font);

	QFontMetrics fm(textFont);
	int margin = fm.height() / 4;

	if (align != ALIGN_DEFAULT || bkgAlpha != 0.0f)
	{
		QRect rect = fm.boundingRect(text);

		//text alignment
		if (align & ALIGN_HMIDDLE)
			x2 -= rect.width() / 2;
		else if (align & ALIGN_HRIGHT)
			x2 -= rect.width();
		if (align & ALIGN_VMIDDLE)
			y2 += rect.height() / 2;
		else if (align & ALIGN_VBOTTOM)
			y2 += rect.height();

		//background is not totally transparent
		if (bkgAlpha != 0.0f)
		{
			glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
			glFunc->glEnable(GL_BLEND);

			//inverted color with a bit of transparency
			const float invertedCol[4] = {	(255 - rgba[0]) / 255.0f,
											(255 - rgba[1]) / 255.0f,
											(255 - rgba[2]) / 255.0f,
											bkgAlpha };
			glFunc->glColor4fv(invertedCol);

			int xB = x2 - m_glViewport.width() / 2;
			int yB = m_glViewport.height() / 2 - y2;
			//yB += margin/2; //empirical compensation

			glFunc->glMatrixMode(GL_PROJECTION);
			glFunc->glPushMatrix();
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();

			setStandardOrthoCenter();

			glFunc->glBegin(GL_POLYGON);
			glFunc->glVertex2d(xB - margin, yB - margin);
			glFunc->glVertex2d(xB - margin, yB + rect.height() + margin / 2);
			glFunc->glVertex2d(xB + rect.width() + margin, yB + rect.height() + margin / 2);
			glFunc->glVertex2d(xB + rect.width() + margin, yB - margin);
			glFunc->glEnd();

			glFunc->glMatrixMode(GL_PROJECTION);
			glFunc->glPopMatrix();
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPopMatrix();
			
			glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT
		}
	}

	if (align & ALIGN_VBOTTOM)
		y2 -= margin; //empirical compensation
	else if (align & ALIGN_VMIDDLE)
		y2 -= margin / 2; //empirical compensation

	glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, rgba);
	renderText(x2, y2, text, textFont);
}

CCVector3 ccGLWindow::backprojectPointOnTriangle(	const CCVector2i& P2D,
													const CCVector3& A3D,
													const CCVector3& B3D,
													const CCVector3& C3D)
{
	//viewing parameters
	ccGLCameraParameters camera;
	getGLCameraParameters(camera);

	CCVector3d A2D;
	CCVector3d B2D;
	CCVector3d C2D;
	camera.project(A3D, A2D);
	camera.project(B3D, B2D);
	camera.project(C3D, C2D);

	//barycentric coordinates
	GLdouble detT =  (B2D.y - C2D.y) * (A2D.x - C2D.x) + (C2D.x - B2D.x) * (A2D.y - C2D.y);
	GLdouble l1   = ((B2D.y - C2D.y) * (P2D.x - C2D.x) + (C2D.x - B2D.x) * (P2D.y - C2D.y)) / detT;
	GLdouble l2   = ((C2D.y - A2D.y) * (P2D.x - C2D.x) + (A2D.x - C2D.x) * (P2D.y - C2D.y)) / detT;

	//clamp everything between 0 and 1
	if (l1 < 0)
		l1 = 0;
	else if (l1 > 1.0)
		l1 = 1.0;
	if (l2 < 0)
		l2 = 0;
	else if (l2 > 1.0)
		l2 = 1.0;
	double l1l2 = l1 + l2;
	assert(l1l2 >= 0);
	if (l1l2 > 1.0)
	{
		l1 /= l1l2;
		l2 /= l1l2;
	}
	GLdouble l3 = 1.0 - l1 - l2;
	assert(l3 >= -1.0e-12);

	//now deduce the 3D position
	GLdouble G[3] = {	l1 * A3D.x + l2 * B3D.x + l3 * C3D.x,
						l1 * A3D.y + l2 * B3D.y + l3 * C3D.y,
						l1 * A3D.z + l2 * B3D.z + l3 * C3D.z };

	return CCVector3::fromArray(G);

}

void ccGLWindow::checkScheduledRedraw()
{
	if (m_scheduledFullRedrawTime && m_timer.elapsed() > m_scheduledFullRedrawTime)
	{
		redraw();
	}
}

void ccGLWindow::cancelScheduledRedraw()
{
	m_scheduledFullRedrawTime = 0;
	m_scheduleTimer.stop();
}

void ccGLWindow::scheduleFullRedraw(unsigned maxDelay_ms)
{
	m_scheduledFullRedrawTime = m_timer.elapsed() + maxDelay_ms;

	if (!m_scheduleTimer.isActive())
	{
		m_scheduleTimer.start(500);
	}
}

ccGLWindow::StereoParams::StereoParams()
	: screenWidth_mm(600)
	, screenDistance_mm(800)
	, eyeSeparation_mm(64)
	, stereoStrength(50)
	, glassType(RED_CYAN)
{}

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

	emit exclusiveFullScreenToggled(state);
}

void ccGLWindow::renderText(int x, int y, const QString & str, const QFont & font/*=QFont()*/)
{   
	if (m_activeFbo)
	{
		m_activeFbo->start();
	}
   
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//compute the text bounding rect
	// This adjustment and the change to x & y are to work around a crash with Qt 5.9.
	// At the time I (Andy) could not determine if it is a bug in CC or Qt.
	//		https://bugreports.qt.io/browse/QTBUG-61863
	//		https://github.com/CloudCompare/CloudCompare/issues/543
	QRect rect = QFontMetrics(font).boundingRect(str).adjusted( -1, -2, 1, 2 );
	
	x -= 1;	// magic number!
	y += 3;	// magic number!

	//first we create a QImage from the text
	QImage textImage(rect.width(), rect.height(), QImage::Format::Format_RGBA8888);
	rect = textImage.rect();
	
	textImage.fill(Qt::transparent);
	{
		QPainter painter(&textImage);

		float glColor[4];
		glFunc->glGetFloatv(GL_CURRENT_COLOR, glColor);
		QColor color;
		color.setRgbF( glColor[0], glColor[1], glColor[2], glColor[3] );
		
		painter.setPen( color );
		painter.setFont( font );
		painter.drawText( rect, Qt::AlignCenter, str );
	}
	
	//and then we convert this QImage to a texture!
	{
		glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
		glFunc->glEnable(GL_BLEND);
		glFunc->glDisable(GL_DEPTH_TEST);

		//set ortho view with center in the upper-left corner
		glFunc->glMatrixMode(GL_PROJECTION);
		glFunc->glPushMatrix();
		glFunc->glLoadIdentity();
		glFunc->glOrtho(0, m_glViewport.width(), 0, m_glViewport.height(), -1, 1);
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glPushMatrix();
		glFunc->glLoadIdentity();
		{
			//move to the right position on the screen
			glFunc->glTranslatef(x, m_glViewport.height() - 1 - y, 0);

			glFunc->glEnable(GL_TEXTURE_2D);         
			QOpenGLTexture textTex( textImage, QOpenGLTexture::DontGenerateMipMaps );
			textTex.setMinificationFilter( QOpenGLTexture::Linear );
			textTex.setMagnificationFilter( QOpenGLTexture::Linear );
			textTex.bind();

			glFunc->glColor4f(1.0f, 1.0f, 1.0f, 1.0f); //DGM: warning must be float colors to work properly?!
			glFunc->glBegin(GL_QUADS);
			glFunc->glTexCoord2f(0, 1); glFunc->glVertex3i(0, 0, 0);
			glFunc->glTexCoord2f(1, 1); glFunc->glVertex3i(rect.width(), 0, 0);
			glFunc->glTexCoord2f(1, 0); glFunc->glVertex3i(rect.width(), rect.height(), 0);
			glFunc->glTexCoord2f(0, 0); glFunc->glVertex3i(0, rect.height(), 0);
			glFunc->glEnd();

			textTex.release();
		}

		glFunc->glMatrixMode(GL_PROJECTION);
		glFunc->glPopMatrix();
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glPopMatrix();
		
		glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT
	}
}

void ccGLWindow::renderText(double x, double y, double z, const QString & str, const QFont & font/*=QFont()*/)
{
	makeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);
	
	//get the actual viewport / matrices
	ccGLCameraParameters camera;
	glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
	glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
	glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

	CCVector3d Q2D(0, 0, 0);
	if (camera.project(CCVector3d(x, y, z), Q2D))
	{
		Q2D.y = m_glViewport.height() - 1 - Q2D.y;
		renderText(Q2D.x, Q2D.y, str, font);
	}
}

void ccGLWindow::logGLError(const char* context) const
{
	if (m_initialized)
	{
		LogGLError(functions()->glGetError(), context);
	}
}

void ccGLWindow::LogGLError(GLenum err, const char* context)
{
	//see http://www.opengl.org/sdk/docs/man/xhtml/glGetError.xml
	switch (err)
	{
	case GL_NO_ERROR:
		break;
	case GL_INVALID_ENUM:
		ccLog::Warning("[%s] OpenGL error: invalid enumerator", context);
		break;
	case GL_INVALID_VALUE:
		ccLog::Warning("[%s] OpenGL error: invalid value", context);
		break;
	case GL_INVALID_OPERATION:
		ccLog::Warning("[%s] OpenGL error: invalid operation", context);
		break;
	case GL_STACK_OVERFLOW:
		ccLog::Error("[%s] OpenGL error: stack overflow", context);
		break;
	case GL_STACK_UNDERFLOW:
		ccLog::Error("[%s] OpenGL error: stack underflow", context);
		break;
	case GL_OUT_OF_MEMORY:
		ccLog::Error("[%s] OpenGL error: out of memory", context);
		break;
	case GL_INVALID_FRAMEBUFFER_OPERATION:
		ccLog::Warning("[%s] OpenGL error: invalid framebuffer operation", context);
		break;
	}
}

void ccGLWindow::toggleAutoRefresh(bool state, int period_ms/*=0*/)
{
	if (state == m_autoRefresh)
	{
		//nothing to do
		return;
	}
	
	m_autoRefresh = state;
	if (m_autoRefresh)
	{
		m_autoRefreshTimer.start(period_ms);
	}
	else
	{
		m_autoRefreshTimer.stop();
	}
}

QPointF ccGLWindow::toCenteredGLCoordinates(int x, int y) const
{
	return QPointF(x - width() / 2, height() / 2 - y) * devicePixelRatio();
}

QPointF ccGLWindow::toCornerGLCoordinates(int x, int y) const
{
	return QPointF(x, height() - 1 - y) * devicePixelRatio();
}

void ccGLWindow::removeFBOSafe(ccFrameBufferObject* &fbo)
{
	//we "disconnect" the current FBO to avoid wrong display/errors
	//if QT tries to redraw window during object destruction
	if (fbo)
	{
		ccFrameBufferObject* _fbo = fbo;
		fbo = nullptr;
		delete _fbo;
	}
}

bool ccGLWindow::initFBOSafe(ccFrameBufferObject* &fbo, int w, int h)
{
	//correction for HD screens
	const int retinaScale = devicePixelRatio();
	w *= retinaScale;
	h *= retinaScale;

	if (fbo && (fbo->width() == w) && (fbo->height() == h))
	{
		//nothing to do
		return true;
	}

	//we "disconnect" the current FBO to avoid wrong display/errors
	//if QT tries to redraw window during initialization
	ccFrameBufferObject* _fbo = fbo;
	fbo = nullptr;

	if (!_fbo)
	{
		_fbo = new ccFrameBufferObject();
	}

	if (!_fbo->init(w, h)
		|| !_fbo->initColor()
		|| !_fbo->initDepth())
	{
		delete _fbo;
		_fbo = nullptr;
		return false;
	}

	fbo = _fbo;
	return true;
}

GLfloat ccGLWindow::getGLDepth(int x, int y, bool extendToNeighbors/*=false*/)
{
	makeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	GLfloat z[9];
	int kernel[2] = { 1, 1 };

	if (extendToNeighbors)
	{
		if (x > 0 && x + 1 < m_glViewport.width())
		{
			kernel[0] = 3;
			--x;
		}
		if (y > 0 && y + 1 < m_glViewport.height())
		{
			kernel[1] = 3;
			--y;
		}
	}

	ccFrameBufferObject* formerFBO = m_activeFbo;
	if (m_fbo && m_activeFbo != m_fbo)
	{
		bindFBO(m_fbo);
	}
	glFunc->glReadPixels(x, y, kernel[0], kernel[1], GL_DEPTH_COMPONENT, GL_FLOAT, z);
	if (m_activeFbo != formerFBO)
	{
		bindFBO(formerFBO);
	}

	logGLError("getGLDepth");

	//by default, we take the center value (= pixel(x,y))
	int kernelSize = kernel[0] * kernel[1];
	GLfloat minZ = z[(kernelSize + 1) / 2 - 1];

	//if the depth is not defined...
	if (minZ == 1.0f && extendToNeighbors)
	{
		//...extend the search to the neighbors
		for (int i = 0; i < kernelSize; ++i)
		{
			minZ = std::min(minZ, z[i]);
		}
	}

	//test: read depth texture
	//if (m_fbo)
	//{
	//	GLfloat* windowDepth = new GLfloat[m_fbo->width() * m_fbo->height()];
	//	glFunc->glBindTexture(GL_TEXTURE_2D, m_fbo->getDepthTexture());
	//	glFunc->glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, windowDepth);
	//	minZ = windowDepth[x + y * m_fbo->width()];
	//	delete[] windowDepth;
	//}

	return minZ;
}

bool ccGLWindow::getClick3DPos(int x, int y, CCVector3d& P3D)
{
	ccGLCameraParameters camera;
	getGLCameraParameters(camera);

	y = m_glViewport.height() - 1 - y;
	GLfloat glDepth = getGLDepth(x, y);
	if (glDepth == 1.0f)
	{
		return false;
	}
	
	CCVector3d P2D(x, y, glDepth);
	return camera.unproject(P2D, P3D);
}

void ccGLWindow::lockRotationAxis(bool state, const CCVector3d& axis)
{
	m_rotationAxisLocked = state;
	m_lockedRotationAxis = axis;
	m_lockedRotationAxis.normalize();
}
