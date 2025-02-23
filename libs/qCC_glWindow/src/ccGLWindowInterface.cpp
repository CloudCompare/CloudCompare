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
#include "ccGLWindowStereo.h"
#include "ccRenderingTools.h"

//qCC_db
#include <cc2DLabel.h>
#include <ccClipBox.h>
#include <ccColorRampShader.h>
#include <ccHObjectCaster.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccSphere.h> //for the pivot symbol
#include <ccSubMesh.h>

//CCFbo
#include <ccFrameBufferObject.h>
#include <ccGlFilter.h>

//Qt
#include <QApplication>
#include <QMessageBox>
#include <QMimeData>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
#include <QOffscreenSurface>
#include <QPainter>
#include <QPushButton>
#include <QSettings>
#include <QTouchEvent>
#include <QWheelEvent>
#include <QNativeGestureEvent>

//STL
#include <array>

#if defined( Q_OS_MAC ) || defined( Q_OS_LINUX )
#include <QDir>
#endif

#ifdef USE_VLD
#include <vld.h>
#endif

// These extra definitions are required in C++11.
// In C++17, class-level "static constexpr" is implicitly inline, so these are not required.
constexpr float ccGLWindowInterface::MIN_POINT_SIZE_F;
constexpr float ccGLWindowInterface::MAX_POINT_SIZE_F;
constexpr float ccGLWindowInterface::MIN_LINE_WIDTH_F;
constexpr float ccGLWindowInterface::MAX_LINE_WIDTH_F;

//Max click duration for enabling picking mode (in ms)
constexpr int CC_MAX_PICKING_CLICK_DURATION_MS = 200;

//GL filter banner margin (height = 2*margin + current font height)
static constexpr int CC_GL_FILTER_BANNER_MARGIN = 5;

//Percentage of the smallest screen dimension
static constexpr double CC_DISPLAYED_PIVOT_RADIUS_PERCENT = 0.8;

//Default picking radius value
static const int DefaultPickRadius = 5;

//Various overlay elements dimensions
constexpr double CC_DISPLAYED_CUSTOM_LIGHT_LENGTH   = 10.0;
constexpr float  CC_DISPLAYED_TRIHEDRON_AXES_LENGTH = 25.0f;
constexpr float  CC_DISPLAYED_CENTER_CROSS_LENGTH   = 10.0f;
constexpr double CC_TRIHEDRON_TEXT_MARGIN           = 5.0;

//invalid GL list index
constexpr GLuint GL_INVALID_LIST_ID = (~0);

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
static QString s_shaderPath;

// Whether stereo (quad buffer) rendering is supported
static bool s_stereoSupported = false;
static bool s_stereoChecked = false;

// Reserved texture indexes
enum class RenderTextReservedIDs
{
	NotReserved = 0,
	FullScreenLabel,
	BubbleViewLabel,
	PointSizeLabel,
	LineSizeLabel,
	GLFilterLabel,
	ScaleLabel,
	trihedronX,
	trihedronY,
	trihedronZ,
	StandardMessagePrefix = 1024
};

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
	ccColor::Rgb color;

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
	qreal pixelDeviceRatio;

	explicit HotZone(ccGLWindowInterface* win)
		: textHeight(0)
		, yTextBottomLineShift(0)
		, color(133, 193, 39) //default color ("greenish")
		, bbv_label("bubble-view mode")
		, fs_label("fullscreen mode")
		, psi_label("default point size")
		, lsi_label("default line width")
		, margin(16)
		, iconSize(16)
		, topCorner(0, 0)
		, pixelDeviceRatio(1.0)
	{
		updateInternalVariables(win);
	}

	void updateInternalVariables(ccGLWindowInterface* win)
	{
		if (win)
		{
			font = win->getFont();
			pixelDeviceRatio = win->getDevicePixelRatio();
			font.setPointSize(12 * pixelDeviceRatio);
			margin = 16 * pixelDeviceRatio;
			iconSize = 16 * pixelDeviceRatio;
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
		fs_totalWidth = /*margin() + */fs_labelRect.width() + margin + iconSize/* + margin*/;

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

		QPoint minAreaCorner(0, std::min(0, yTextBottomLineShift - textHeight));
		QPoint maxAreaCorner(totalWidth, std::max(iconSize, yTextBottomLineShift));
		int rowCount = clickableItemsVisible ? 2 : 0;
		rowCount += bubbleViewModeEnabled ? 1 : 0;
		rowCount += fullScreenEnabled ? 1 : 0;
		maxAreaCorner.setY(maxAreaCorner.y() + (iconSize + margin) * (rowCount - 1));

		QRect areaRect(minAreaCorner - QPoint(margin, margin) / 2,
			maxAreaCorner + QPoint(margin, margin) / 2);

		return areaRect;
	}
};

void ccGLWindowInterface::SetStereoSupported(bool state)
{
	s_stereoSupported = state;
}

bool ccGLWindowInterface::StereoSupported()
{
	return s_stereoSupported;
}

//! Optional output metrics (from computeProjectionMatrix)
struct ccGLWindowInterface::ProjectionMetrics
{
	double zNear = 0.0;
	double zFar = 0.0;
	ccBBox visibleObjectsBBox;
};

ccGLWindowInterface::ccGLWindowInterface(QObject* parent/*=nullptr*/, bool silentInitialization/*=false*/)
	: m_uniqueID(++s_GlWindowNumber) //GL window unique ID
	, m_initialized(false)
	, m_trihedronGLList(GL_INVALID_LIST_ID)
	, m_pivotGLList(GL_INVALID_LIST_ID)
	, m_lastMousePos(-1, -1)
	, m_validModelviewMatrix(false)
	, m_validProjectionMatrix(false)
	, m_LODEnabled(true)
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
	, m_pickingFbo(nullptr)
	, m_alwaysUseFBO(false)
	, m_updateFBO(true)
	, m_colorRampShader(nullptr)
	, m_customRenderingShader(nullptr)
	, m_activeGLFilter(nullptr)
	, m_glFiltersEnabled(false)
	, m_winDBRoot(nullptr)
	, m_globalDBRoot(nullptr) //external DB
	, m_font{}
	, m_pivotVisibility(PIVOT_SHOW_ON_MOVE)
	, m_pivotSymbolShown(false)
	, m_allowRectangularEntityPicking(true)
	, m_rectPickingPoly(nullptr)
	, m_overriddenDisplayParametersEnabled(false)
	, m_showScale(true)
	, m_showTrihedron(true)
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
	, m_texturePoolLastIndex(0)
	, m_clippingPlanesEnabled(true)
	, m_defaultCursorShape(Qt::ArrowCursor)
	, m_signalEmitter(new ccGLWindowSignalEmitter(this, parent))
	, m_displayScale(1.0, 1.0)
{
	//start internal timer
	m_timer.start();

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
	m_viewMatd.toIdentity();
	m_projMatd.toIdentity();

	//auto-load previous perspective settings
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);

		//load parameters
		bool perspectiveView = settings.value(c_ps_perspectiveView, false).toBool();
		//DGM: we force object-centered view by default now, as the viewer-based perspective is too dependent
		//on what is displayed (so restoring this parameter at next startup is rarely a good idea)
		bool objectCenteredView = /*settings.value(c_ps_objectMode, true).toBool()*/true;
		m_sunLightEnabled = settings.value(c_ps_sunLight, true).toBool();
		m_customLightEnabled = settings.value(c_ps_customLight, false).toBool();
		int pivotVisibility = settings.value(c_ps_pivotVisibility, PIVOT_SHOW_ON_MOVE).toInt();
		int glassType = settings.value(c_ps_stereoGlassType, ccGLWindowInterface::StereoParams::RED_BLUE).toInt();

		settings.endGroup();

		//update stereo parameters
		m_stereoParams.glassType = static_cast<ccGLWindowInterface::StereoParams::GlassType>(glassType);

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
			displayNewMessage("Warning: custom light is ON", ccGLWindowInterface::LOWER_LEFT_MESSAGE, false, 2, CUSTOM_LIGHT_STATE_MESSAGE);
		if (!m_sunLightEnabled)
			displayNewMessage("Warning: sun light is OFF", ccGLWindowInterface::LOWER_LEFT_MESSAGE, false, 2, SUN_LIGHT_STATE_MESSAGE);
	}

	m_deferredPickingTimer.setSingleShot(true);
	m_deferredPickingTimer.setInterval(100);
}

ccGLWindowInterface::~ccGLWindowInterface()
{
	doMakeCurrent();

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

	delete m_pickingFbo;
	m_pickingFbo = nullptr;

	m_pickingPBO.release();

	delete m_hotZone;
	m_hotZone = nullptr;
}

// OpenGL KHR debug log
static void HandleLoggedMessage(const QOpenGLDebugMessage& message, int uniqueID)
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

	QString msg = QString("[OpenGL][Win %0]").arg(uniqueID);
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

bool ccGLWindowInterface::initialize()
{
	bool firstTime = true;
	if (!preInitialize(firstTime))
	{
		return false;
	}

	QOpenGLContext* glContext = getOpenGLContext();
	if (!glContext)
	{
		ccLog::Warning("Failed to retrieve the OpengGL context");
		assert(false);
		return false;
	}

	ccQOpenGLFunctions* glFunc = functions();
	if (!glFunc)
	{
		ccLog::Warning("Failed to retrieve the OpengGL functions");
		assert(false);
		return false;
	}

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
		if (glContext->hasExtension(QByteArrayLiteral("GL_ARB_vertex_buffer_object")))
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
		m_shadersEnabled =	glContext->hasExtension(QByteArrayLiteral("GL_ARB_shading_language_100"))
						&&	glContext->hasExtension(QByteArrayLiteral("GL_ARB_shader_objects"))
						&&	glContext->hasExtension(QByteArrayLiteral("GL_ARB_vertex_shader"))
						&&	glContext->hasExtension(QByteArrayLiteral("GL_ARB_fragment_shader"));
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

			m_glFiltersEnabled = glContext->hasExtension(QByteArrayLiteral("GL_EXT_framebuffer_object"));
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
					const QString shaderPath = QStringLiteral("%1/ColorRamp/color_ramp.frag").arg(GetShaderPath());

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
			{
				GLboolean isStereoEnabled = GL_FALSE;
				glFunc->glGetBooleanv(GL_STEREO, &isStereoEnabled);

				if (!s_stereoChecked || isStereoEnabled == GL_TRUE)
				{
					s_stereoSupported = (isStereoEnabled == GL_TRUE);
					s_stereoChecked = true;
				}

				if (!m_silentInitialization)
				{
					ccLog::Print(QString("[3D View %1] Stereo mode: %2").arg(m_uniqueID).arg(isStereoEnabled ? "supported" : "not supported"));
				}
			}
		}

#ifdef QT_DEBUG
		//KHR extension (debug)
		if (glContext->hasExtension(QByteArrayLiteral("GL_KHR_debug")))
		{
			if (!m_silentInitialization)
				ccLog::Print("[3D View %i] GL KHR (debug) extension available", m_uniqueID);

			QOpenGLDebugLogger* logger = new QOpenGLDebugLogger(asQObject());
			logger->initialize();

			QObject::connect(logger, &QOpenGLDebugLogger::messageLogged, [&](const QOpenGLDebugMessage& debugMessage) { HandleLoggedMessage(debugMessage, getUniqueID()); });
			logger->enableMessages();
			logger->disableMessages(	QOpenGLDebugMessage::AnySource,
										QOpenGLDebugMessage::DeprecatedBehaviorType
									|	QOpenGLDebugMessage::PortabilityType
									|	QOpenGLDebugMessage::PerformanceType
									|	QOpenGLDebugMessage::OtherType);
			logger->startLogging(QOpenGLDebugLogger::SynchronousLogging);
		}
#endif

		//apply (potentially) updated parameters;
		setDisplayParameters(params, hasOverriddenDisplayParameters());

#if 0
		//OpenGL 3.3+ rendering shader
		if (QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_3_3)
		{
			bool vaEnabled = ccFBOUtils::CheckVAAvailability();
			if (vaEnabled && !m_customRenderingShader)
			{
				ccGui::ParamStruct params = getDisplayParameters();

				ccShader* renderingShader = new ccShader();
				QString shadersPath = ccGLWindow::getShadersPath();
				QString error;
				if (!renderingShader->fromFile(shadersPath + QString("/Rendering"), QString("rendering"), error))
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
				setDisplayParameters(params, hasOverriddenDisplayParameters());
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

	return postInitialize(firstTime);
}

void ccGLWindowInterface::uninitializeGL()
{
	if (!m_initialized)
	{
		return;
	}

	assert(!m_captureMode.enabled);
	doMakeCurrent();

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

void ccGLWindowInterface::onResizeGL(int w, int h)
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

		logGLError("ccGLWindowInterface::onResizeGL");
	}

	setLODEnabled(true);
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

	logGLError("ccGLWindowInterface::onResizeGL");
}

void ccGLWindowInterface::refresh(bool only2D/*=false*/)
{
	if (m_shouldBeRefreshed && asWidget()->isVisible())
	{
		redraw(only2D);
	}
}

void ccGLWindowInterface::redraw(bool only2D/*=false*/, bool resetLOD/*=true*/)
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

	if (asWidget()->isVisible() && !m_autoRefresh)
	{
		requestUpdate();
	}
}

void ccGLWindowInterface::setGLViewport(const QRect& rect)
{
	//correction for HD screens
	const int devicePixelRatio = static_cast<int>(getDevicePixelRatio());
	m_glViewport = QRect(rect.left() * devicePixelRatio, rect.top() * devicePixelRatio, rect.width() * devicePixelRatio, rect.height() * devicePixelRatio);
	invalidateViewport();

	if (getOpenGLContext() && getOpenGLContext()->isValid())
	{
		doMakeCurrent();

		functions()->glViewport(m_glViewport.x(), m_glViewport.y(), m_glViewport.width(), m_glViewport.height());
	}
}

bool ccGLWindowInterface::bindFBO(ccFrameBufferObject* fbo)
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

const ccGui::ParamStruct& ccGLWindowInterface::getDisplayParameters() const
{
	return m_overriddenDisplayParametersEnabled ? m_overriddenDisplayParameters : ccGui::Parameters();
}

void ccGLWindowInterface::setDisplayParameters(const ccGui::ParamStruct &params, bool thisWindowOnly)
{
	if (thisWindowOnly)
	{
		m_overriddenDisplayParametersEnabled = true;
		m_overriddenDisplayParameters = params;
	}
	else
	{
		m_overriddenDisplayParametersEnabled = false;
		ccGui::Set(params);
	}
}

bool ccGLWindowInterface::setLODEnabled(bool state)
{
	if (state && (!m_fbo || (m_stereoModeEnabled && !m_stereoParams.isAnaglyph() && !m_fbo2)))
	{
		//we need a valid FBO (or two ;) for LOD!!!
		return false;
	}

	m_LODEnabled = state;
	return true;
}

void ccGLWindowInterface::drawClickableItems(int xStart0, int& yStart)
{
	//we init the necessary parameters the first time we need them
	if (!m_hotZone)
	{
		m_hotZone = new HotZone(this);
	}
	else if (getDevicePixelRatio() != m_hotZone->pixelDeviceRatio) // the device pixel ratio has changed (happens when changing screen for instance)
	{
		m_hotZone->updateInternalVariables(this);
	}
	//remember the last position of the 'top corner'
	m_hotZone->topCorner = QPoint(xStart0, yStart) + QPoint(m_hotZone->margin, m_hotZone->margin);

	bool fullScreenEnabled = exclusiveFullScreen();

	if (!m_clickableItemsVisible
		&& !m_bubbleViewModeEnabled
		&& !fullScreenEnabled)
	{
		//nothing to do
		return;
	}

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//"exit" icon
	static const QImage c_exitIcon = QImage(":/CC/images/ccExit.png").mirrored();

	int halfW = glWidth() / 2;
	int halfH = glHeight() / 2;

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
		int y0 = halfH - areaRect.y();
		glFunc->glVertex2i(x0, y0);
		glFunc->glVertex2i(x0 + areaRect.width(), y0);
		glFunc->glVertex2i(x0 + areaRect.width(), y0 - areaRect.height());
		glFunc->glVertex2i(x0, y0 - areaRect.height());
		glFunc->glEnd();
	}

	yStart = m_hotZone->topCorner.y();

	if (fullScreenEnabled)
	{
		int xStart = m_hotZone->topCorner.x();

		//label
		glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, m_hotZone->color);
		renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->fs_label, static_cast<uint16_t>(RenderTextReservedIDs::FullScreenLabel), m_hotZone->font);

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
		renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->bbv_label, static_cast<uint16_t>(RenderTextReservedIDs::BubbleViewLabel), m_hotZone->font);

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
			renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->psi_label, static_cast<uint16_t>(RenderTextReservedIDs::PointSizeLabel), m_hotZone->font);

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
			renderText(xStart, yStart + m_hotZone->yTextBottomLineShift, m_hotZone->lsi_label, static_cast<uint16_t>(RenderTextReservedIDs::LineSizeLabel), m_hotZone->font);

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

void ccGLWindowInterface::toBeRefreshed()
{
	m_shouldBeRefreshed = true;

	invalidateViewport();
	invalidateVisualization();
}

void ccGLWindowInterface::stopLODCycle()
{
	//reset LOD rendering (if any)
	m_currentLODState = LODState();
}

bool ccGLWindowInterface::objectPerspectiveEnabled() const
{
	return m_viewportParams.perspectiveView && m_viewportParams.objectCenteredView;
}

bool ccGLWindowInterface::viewerPerspectiveEnabled() const
{
	return m_viewportParams.perspectiveView && !m_viewportParams.objectCenteredView;
}

bool ccGLWindowInterface::getPerspectiveState(bool& objectCentered) const
{
	objectCentered = m_viewportParams.objectCenteredView;
	return m_viewportParams.perspectiveView;
}

void ccGLWindowInterface::addToOwnDB(ccHObject* obj, bool noDependency/*=true*/)
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
		ccLog::Error("[ccGLWindowInterface::addToOwnDB] Window has no DB!");
	}
}

void ccGLWindowInterface::removeFromOwnDB(ccHObject* obj)
{
	if (m_winDBRoot)
	{
		m_winDBRoot->removeChild(obj);
	}
}

void ccGLWindowInterface::zoomGlobal()
{
	updateConstellationCenterAndZoom();
}

void ccGLWindowInterface::updateConstellationCenterAndZoom(const ccBBox* boundingBox/*=nullptr*/)
{
	if (m_bubbleViewModeEnabled)
	{
		ccLog::Warning("[updateConstellationCenterAndZoom] Not when bubble-view is enabled!");
		return;
	}

	ccBBox zoomedBox;
	if (boundingBox)
	{
		//the user has provided a valid bounding box
		zoomedBox = *boundingBox;
	}
	else
	{
		//otherwise we'll take the default one (if possible)
		getVisibleObjectsBB(zoomedBox);
	}

	if (!zoomedBox.isValid())
	{
		return;
	}

	//we compute the bounding-box diagonal length
	double bbDiag = zoomedBox.getDiagNorm();
	if (CCCoreLib::LessThanEpsilon(bbDiag))
	{
		ccLog::Warning("[ccGLWindow] Entity/DB has a null bounding-box!");
		bbDiag = 1.0;
	}

	//we set the pivot point on the box center
	CCVector3d P = zoomedBox.getCenter();
	setPivotPoint(P, false, false);

	//compute the right distance for the camera to see the whole bounding-box
	double targetWidth = bbDiag;
	if (glHeight() < glWidth())
	{
		targetWidth *= static_cast<double>(glWidth()) / glHeight();
	}
	double focalDistance = targetWidth / m_viewportParams.computeDistanceToWidthRatio();

	//set the camera position
	setCameraPos(P);
	CCVector3d v(0, 0, focalDistance);
	moveCamera(v);

	//just in case
	invalidateViewport();
	invalidateVisualization();
	deprecate3DLayer();

	redraw();
}

void ccGLWindowInterface::setShader(ccShader* _shader)
{
	if (!m_shadersEnabled)
	{
		ccLog::Warning("[ccGLWindowInterface::setShader] Shader ignored (not supported)");
		return;
	}

	delete m_activeShader;
	m_activeShader = _shader;

	redraw();
}

void ccGLWindowInterface::setGlFilter(ccGlFilter* filter)
{
	if (!m_glFiltersEnabled)
	{
		ccLog::Warning("[ccGLWindowInterface::setGlFilter] GL filter ignored (not supported)");
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

void ccGLWindowInterface::setCameraFocalToFitWidth(double width)
{
	double focalDistance = width / m_viewportParams.computeDistanceToWidthRatio();

	setFocalDistance(focalDistance);
}

void ccGLWindowInterface::setFocalDistance(double focalDistance)
{
	if (focalDistance != m_viewportParams.getFocalDistance())
	{
		m_viewportParams.setFocalDistance(focalDistance);

		if (m_viewportParams.objectCenteredView)
		{
			Q_EMIT m_signalEmitter->cameraPosChanged(m_viewportParams.getCameraCenter());
		}

		invalidateViewport();
		invalidateVisualization();
		deprecate3DLayer();
	}
}

void ccGLWindowInterface::setCameraPos(const CCVector3d& P)
{
	if ((m_viewportParams.getCameraCenter() - P).norm2d() != 0.0)
	{
		m_viewportParams.setCameraCenter(P, true);

		//ccLog::Print(QString("[ccGLWindow] Focal distance = %1").arg(m_viewportParams.getFocalDistance()));

		Q_EMIT m_signalEmitter->cameraPosChanged(P);

		invalidateViewport();
		invalidateVisualization();
		deprecate3DLayer();
	}
}

void ccGLWindowInterface::moveCamera(CCVector3d& v)
{
	//current X, Y and Z viewing directions correspond to the 'model view' matrix rows
	if (!m_viewportParams.objectCenteredView)
	{
		m_viewportParams.viewMat.transposed().applyRotation(v);
	}

	setCameraPos(m_viewportParams.getCameraCenter() + v);
}

void ccGLWindowInterface::setPivotPoint(const CCVector3d& P,
										bool autoUpdateCameraPos/*=false*/,
										bool verbose/*=false*/)
{
	if (autoUpdateCameraPos && m_viewportParams.objectCenteredView)
	{
		//compute the equivalent camera center
		CCVector3d pivotShift = m_viewportParams.getPivotPoint() - P;
		CCVector3d pivotShiftInCameraCS = pivotShift;
		m_viewportParams.viewMat.applyRotation(pivotShiftInCameraCS);
		CCVector3d newCameraPos = m_viewportParams.getCameraCenter() + pivotShiftInCameraCS - pivotShift;

		if (!m_viewportParams.perspectiveView)
		{
			//in orthographic mode, make sure the level of 'zoom' is maintained by repositioning the camera
			newCameraPos.z = m_viewportParams.getFocalDistance() + P.z;
		}

		setCameraPos(newCameraPos); //will also update the pixel size
	}

	m_viewportParams.setPivotPoint(P, true);
	Q_EMIT m_signalEmitter->pivotPointChanged(P);

	if (verbose)
	{
		const unsigned& precision = getDisplayParameters().displayedNumPrecision;
		displayNewMessage(QString(), ccGLWindowInterface::LOWER_LEFT_MESSAGE, false); //clear previous message
		displayNewMessage(QString("Point (%1 ; %2 ; %3) set as rotation center")
			.arg(P.x, 0, 'f', precision)
			.arg(P.y, 0, 'f', precision)
			.arg(P.z, 0, 'f', precision),
			ccGLWindowInterface::LOWER_LEFT_MESSAGE, true);
		redraw(true, false);
	}

	invalidateViewport();
	invalidateVisualization();
}

void ccGLWindowInterface::setAutoPickPivotAtCenter(bool state)
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

void ccGLWindowInterface::setSceneDB(ccHObject* root)
{
	m_globalDBRoot = root;
	zoomGlobal();
}

void ccGLWindowInterface::computeColorRampAreaLimits(int& yStart, int& yStop) const
{
	const int defaultMargin = static_cast<int>(5 * m_captureMode.zoomFactor);

	//top of the area
	yStart = defaultMargin;

	//avoid the GL filter banner (if any)
	if (nullptr != getGlFilter())
	{
		yStart += getGlFilterBannerHeight();
	}
	else
	{
		yStart += 2 * defaultMargin; //we still add a margin
	}

	//bottom: only the trihedron
	yStop = glHeight() - defaultMargin;
	if (trihedronIsDisplayed())
	{
		int totalTrihedronHeight = 2 * static_cast<int>(computeTrihedronLength() + 2 * defaultMargin);
		yStop -= totalTrihedronHeight;
	}
}

void ccGLWindowInterface::getVisibleObjectsBB(ccBBox& box) const
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

ccGLMatrixd ccGLWindowInterface::computeProjectionMatrix(bool withGLfeatures, ProjectionMetrics* metrics/*=nullptr*/, double* eyeOffset/*=nullptr*/) const
{
	static const CCVector3 MinusOne(-CCCoreLib::PC_ONE / 2, -CCCoreLib::PC_ONE / 2, -CCCoreLib::PC_ONE / 2);
	static const CCVector3 PlusOne(CCCoreLib::PC_ONE / 2, CCCoreLib::PC_ONE / 2, CCCoreLib::PC_ONE / 2);
	ccBBox visibleObjectsBBox(MinusOne, PlusOne, true);

	// compute center of visible objects constellation
	if (m_globalDBRoot || m_winDBRoot)
	{
		// get whole bounding-box
		ccBBox box;
		getVisibleObjectsBB(box);
		if (box.isValid())
		{
			visibleObjectsBBox = box;
		}
	}

	// get bbox center
	CCVector3d bbCenter = visibleObjectsBBox.getCenter();
	//get half bbox diagonal length
	double bbHalfDiag = visibleObjectsBBox.getDiagNormd() / 2;

	// determine the min and max distance based on the current view matrix
	ccGLMatrixd viewMatd = m_viewportParams.computeViewMatrix();

	double zMin = std::numeric_limits<double>::quiet_NaN();
	double zMax = zMin;
	{
		// apply the view matrix to all 8 corners
		const CCVector3& bbMin = visibleObjectsBBox.minCorner();
		const CCVector3& bbMax = visibleObjectsBBox.maxCorner();
		std::array<CCVector3, 8> bbCorners
		{
			bbMin,
			CCVector3(bbMin.x, bbMin.y, bbMax.z),
			CCVector3(bbMin.x, bbMax.y, bbMin.z),
			CCVector3(bbMax.x, bbMin.y, bbMin.z),
			bbMax,
			CCVector3(bbMax.x, bbMax.y, bbMin.z),
			CCVector3(bbMax.x, bbMin.y, bbMax.z),
			CCVector3(bbMin.x, bbMax.y, bbMax.z)
		};

		for (const CCVector3& P : bbCorners)
		{
			CCVector3d Pd = viewMatd * CCVector3d::fromArray(P.u);
			double z = -Pd.z; // warning, the OpenGL camera looks toward -Z
			if (std::isnan(zMin) || z < zMin)
			{
				zMin = z;
			}
			if (std::isnan(zMax) || z > zMax)
			{
				zMax = z;
			}
		}
	}

	// predict the minimum and maximum displayed depths
	double zMinDisplayed = zMin;
	double zMaxDisplayed = zMax;
	{
		double pixelSize = computeActualPixelSize();

		if (	m_pivotSymbolShown
			&&	m_pivotVisibility != PIVOT_HIDE
			&&	withGLfeatures
			&&	m_viewportParams.objectCenteredView)
		{
			// take the pivot symbol into account
			double pivotActualRadius_pix = CC_DISPLAYED_PIVOT_RADIUS_PERCENT * std::min(glWidth(), glHeight()) / 2.0;
			double pivotSymbolScale = pivotActualRadius_pix * pixelSize;
			CCVector3d rotationCenterInCameraCS = viewMatd * m_viewportParams.getRotationCenter();
			zMinDisplayed = std::min(zMinDisplayed, -rotationCenterInCameraCS.z - pivotSymbolScale);
			zMaxDisplayed = std::max(zMaxDisplayed, -rotationCenterInCameraCS.z + pivotSymbolScale);
		}

		if (withGLfeatures && m_customLightEnabled)
		{
			// take the custom light into account
			CCVector3d customLightPosInCameraCS = viewMatd * CCVector3d::fromArray(m_customLightPos);
			double d = CC_DISPLAYED_CUSTOM_LIGHT_LENGTH * pixelSize;
			zMinDisplayed = std::min(zMinDisplayed, -customLightPosInCameraCS.z - d);
			zMaxDisplayed = std::max(zMaxDisplayed, -customLightPosInCameraCS.z + d);
		}
	}

	// compute the aspect ratio
	double ar = static_cast<double>(glHeight()) / glWidth();

	double zNear = zMinDisplayed;
	double zFar = zMaxDisplayed;

	double epsilon = std::max(bbHalfDiag / 1000.0, 1.0e-6);

	ccGLMatrixd projMatrix;
	if (m_viewportParams.perspectiveView)
	{
		double minZFar = static_cast<double>(CCCoreLib::ZERO_TOLERANCE_F) / m_viewportParams.zNearCoef;
		if (zFar < minZFar)
		{
			// no object in front of the camera! (or too small)
			// use some default values
			zFar = minZFar;
			zNear = epsilon;
		}
		else
		{
			zNear = std::max(zMinDisplayed, zFar * m_viewportParams.zNearCoef);
		}

		double delta = zFar / 2;
		if (zFar - zNear < delta)
		{
			// zNear can't be too close to zFar (for the EDL filter ;)
			zNear -= delta / 2;
			zFar += delta / 2;
		}

		double distanceToHalfWidthRatio = (m_bubbleViewModeEnabled ? std::tan(CCCoreLib::DegreesToRadians(m_bubbleViewFov_deg / 2.0)) : m_viewportParams.computeDistanceToHalfWidthRatio());
		double xMax = zNear * distanceToHalfWidthRatio;
		double yMax = xMax * ar;

		//DGM: we now take 'frustumAsymmetry' into account (for stereo rendering)
		double frustumAsymmetry = 0.0;
		if (eyeOffset)
		{
			//see 'NVIDIA 3D VISION PRO AND STEREOSCOPIC 3D' White paper (Oct 2010, p. 12)
			double convergence = std::abs(m_viewportParams.getFocalDistance());

			//we assume zNear = screen distance
			//double scale = zNear * m_stereoParams.stereoStrength / m_stereoParams.screenDistance_mm;	//DGM: we don't want to depend on the cloud size anymore
																										//as it can produce very strange visual effects on very large clouds
																										//we now keep something related to the focal distance (multiplied by
																										//the 'zNearCoef' that can be tweaked by the user if necessary)
			double scale = convergence * m_viewportParams.zNearCoef * m_stereoParams.stereoStrength / m_stereoParams.screenDistance_mm;
			double eyeSeperation = m_stereoParams.eyeSeparation_mm * scale;

			//on input 'eyeOffset' should be -1 (left) or +1 (right)
			*eyeOffset *= eyeSeperation;

			frustumAsymmetry = (*eyeOffset) * zNear / convergence;

			//if (*eyeOffset < 0.0)
			//	const_cast<ccGLWindow*>(this)->displayNewMessage(QString("eye sep. = %1 - convergence = %3").arg(*eyeOffset).arg(convergence), ccGLWindowInterface::LOWER_LEFT_MESSAGE, false, 2, ccGLWindowInterface::PERSPECTIVE_STATE_MESSAGE);
		}

		projMatrix = ccGL::Frustum(-xMax - frustumAsymmetry, xMax - frustumAsymmetry, -yMax, yMax, zNear, zFar);
	}
	else
	{
		zFar += epsilon;
		zNear -= epsilon;
		//ccLog::Print(QString("cameraCenterToPivotDist = %0 / zNear = %1 / zFar = %2").arg(cameraCenterToPivotDist).arg(zNear).arg(zFar));

		double xMax = std::abs(m_viewportParams.getFocalDistance()) * m_viewportParams.computeDistanceToHalfWidthRatio();
		double yMax = xMax * ar;

		projMatrix = ccGL::Ortho(-xMax, xMax, -yMax, yMax, zNear, zFar);
	}

	if (metrics)
	{
		//ccLog::PrintDebug(QString("[%1 ; %2] R = %3").arg(zNear).arg(zFar).arg(zFar / zNear));
		metrics->zNear = zNear;
		metrics->zFar = zFar;
		metrics->visibleObjectsBBox = visibleObjectsBBox;
	}

	return projMatrix;
}

void ccGLWindowInterface::updateProjectionMatrix()
{
	ProjectionMetrics metrics;

	m_projMatd = computeProjectionMatrix
	(
		true,
		&metrics,
		nullptr
	); //no stereo vision by default!

	m_viewportParams.zNear = metrics.zNear;
	m_viewportParams.zFar = metrics.zFar;
	m_visibleObjectsBBox = metrics.visibleObjectsBBox;

	m_validProjectionMatrix = true;
}

ccGLMatrixd ccGLWindowInterface::computeModelViewMatrix() const
{
	ccGLMatrixd viewMatd = m_viewportParams.computeViewMatrix();

	ccGLMatrixd scaleMatd = m_viewportParams.computeScaleMatrix(m_glViewport);

	scaleMatd.data()[0] *= m_displayScale.x;
	scaleMatd.data()[5] *= m_displayScale.y;

	return scaleMatd * viewMatd;
}

void ccGLWindowInterface::updateModelViewMatrix()
{
	//we save visualization matrix
	m_viewMatd = computeModelViewMatrix();

	m_validModelviewMatrix = true;
}

void ccGLWindowInterface::setBaseViewMat(ccGLMatrixd& mat)
{
	m_viewportParams.viewMat = mat;

	invalidateViewport();
	invalidateVisualization();

	//we emit the 'baseViewMatChanged' signal
	Q_EMIT m_signalEmitter->baseViewMatChanged(m_viewportParams.viewMat);
}

void ccGLWindowInterface::getGLCameraParameters(ccGLCameraParameters& params)
{
	//get/compute the modelview matrix
	params.modelViewMat = getModelViewMatrix();

	//get/compute the projection matrix
	params.projectionMat = getProjectionMatrix();

	//viewport
	params.viewport[0] = m_glViewport.x();
	params.viewport[1] = m_glViewport.y();
	params.viewport[2] = m_glViewport.width();
	params.viewport[3] = m_glViewport.height();

	//compute the pixel size
	params.pixelSize = computeActualPixelSize();

	//other parameters
	params.perspective = m_viewportParams.perspectiveView;
	params.fov_deg = getFov();
	params.nearClippingDepth = m_viewportParams.nearClippingDepth;
	params.farClippingDepth = m_viewportParams.farClippingDepth;
}

const ccGLMatrixd& ccGLWindowInterface::getModelViewMatrix()
{
	if (!m_validModelviewMatrix)
	{
		updateModelViewMatrix();
	}

	return m_viewMatd;
}

const ccGLMatrixd& ccGLWindowInterface::getProjectionMatrix()
{
	if (!m_validProjectionMatrix)
	{
		updateProjectionMatrix();
	}

	return m_projMatd;
}

void ccGLWindowInterface::setStandardOrthoCenter()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glMatrixMode(GL_PROJECTION);
	glFunc->glLoadIdentity();
	double halfW = glWidth() / 2.0;
	double halfH = glHeight() / 2.0;
	double maxS = std::max(halfW, halfH);
	glFunc->glOrtho(-halfW, halfW, -halfH, halfH, -maxS, maxS);
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glLoadIdentity();
}

void ccGLWindowInterface::setStandardOrthoCorner()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	glFunc->glMatrixMode(GL_PROJECTION);
	glFunc->glLoadIdentity();
	glFunc->glOrtho(0.0, glWidth(), 0.0, glHeight(), 0.0, 1.0);
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glLoadIdentity();
}

void ccGLWindowInterface::getContext(CC_DRAW_CONTEXT& CONTEXT)
{
	//display size
	CONTEXT.glW = glWidth();
	CONTEXT.glH = glHeight();
	CONTEXT.devicePixelRatio = static_cast<float>(getDevicePixelRatio());
	CONTEXT.display = this;
	CONTEXT.qGLContext = getOpenGLContext();
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

//QString ToString(ccGLWindowInterface::PICKING_MODE mode)
//{
//	switch (mode)
//	{
//	case ccGLWindowInterface::NO_PICKING:
//		return "NO_PICKING";
//	case ccGLWindowInterface::ENTITY_PICKING:
//		return "ENTITY_PICKING";
//	case ccGLWindowInterface::ENTITY_RECT_PICKING:
//		return "ENTITY_RECT_PICKING";
//	case ccGLWindowInterface::FAST_PICKING:
//		return "FAST_PICKING";
//	case ccGLWindowInterface::POINT_PICKING:
//		return "POINT_PICKING";
//	case ccGLWindowInterface::TRIANGLE_PICKING:
//		return "TRIANGLE_PICKING";
//	case ccGLWindowInterface::POINT_OR_TRIANGLE_PICKING:
//		return "POINT_OR_TRIANGLE_PICKING";
//	case POINT_OR_TRIANGLE_OR_LABEL_PICKING:
//		return "POINT_OR_TRIANGLE_OR_LABEL_PICKING";
//	case ccGLWindowInterface::LABEL_PICKING:
//		return "LABEL_PICKING";
//	case ccGLWindowInterface::DEFAULT_PICKING:
//		return "DEFAULT_PICKING";
//	}
//
//	assert(false);
//	return QString();
//}

void ccGLWindowInterface::setPickingMode(PICKING_MODE mode/*=DEFAULT_PICKING*/, Qt::CursorShape defaultCursorShape/*=Qt::ArrowCursor*/)
{
	//is the picking mode locked?
	if (m_pickingModeLocked)
	{
		if ((mode != m_pickingMode) && (mode != DEFAULT_PICKING))
			ccLog::Warning("[ccGLWindowInterface::setPickingMode] Picking mode is locked! Can't change it...");
		return;
	}

	switch (mode)
	{
	case DEFAULT_PICKING:
		mode = ENTITY_PICKING;
	case NO_PICKING:
	case ENTITY_PICKING:
		m_defaultCursorShape = defaultCursorShape;
		break;
	case POINT_OR_TRIANGLE_PICKING:
	case POINT_OR_TRIANGLE_OR_LABEL_PICKING:
	case TRIANGLE_PICKING:
	case POINT_PICKING:
	{
		const ccGui::ParamStruct& displayParams = getDisplayParameters();
		m_defaultCursorShape = displayParams.pickingCursorShape;
	}
	break;

	default:
		break;
	}

	m_pickingMode = mode;
	setWindowCursor(QCursor(m_defaultCursorShape));

	//ccLog::Warning(QString("[%1] Picking mode set to: ").arg(m_uniqueID) + ToString(m_pickingMode));
}

CCVector3d ccGLWindowInterface::convertMousePositionToOrientation(int x, int y)
{
	double xc = width() / 2.0;
	double yc = height() / 2.0; //DGM FIXME: is it scaled coordinates or not?!

	CCVector3d Q2D;
	if (m_viewportParams.objectCenteredView)
	{
		//project the current pivot point on screen
		ccGLCameraParameters camera;
		getGLCameraParameters(camera);

		if (!camera.project(m_viewportParams.getPivotPoint(), Q2D))
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

void ccGLWindowInterface::updateActiveItemsList(int x, int y, bool extendToSelectedLabels/*=false*/)
{
	m_activeItems.clear();

	PickingParameters params(FAST_PICKING, x, y, 2, 2);

	startPicking(params);

	if (m_activeItems.size() == 1)
	{
		ccInteractor* pickedObj = *m_activeItems.begin();
		cc2DLabel* label = dynamic_cast<cc2DLabel*>(pickedObj);
		if (label)
		{
			if (!label->isSelected() || !extendToSelectedLabels)
			{
				//select it?
				//Q_EMIT m_signalEmitter->entitySelectionChanged(label);
				//QCoreApplication::processEvents();
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
							m_activeItems.insert(l);
						}
					}
				}
			}
		}
	}
}

bool ccGLWindowInterface::processClickableItems(int x, int y)
{
	if (m_clickableItems.empty())
	{
		//shortcut
		return false;
	}

	//correction for HD screens
	const auto devicePixelRatio = getDevicePixelRatio();
	x = static_cast<int>(x *devicePixelRatio);
	y = static_cast<int>(y * devicePixelRatio);

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

double ccGLWindowInterface::computeDefaultIncrement() const
{
	return m_visibleObjectsBBox.getMaxBoxDim() / 250.0;
}

void ccGLWindowInterface::onWheelEvent(float wheelDelta_deg)
{
	if (m_bubbleViewModeEnabled)
	{
		//to zoom in and out we simply change the fov in bubble-view mode!
		setBubbleViewFov(m_bubbleViewFov_deg - wheelDelta_deg / 3.6f); //1 turn = 100 degrees
	}
	else
	{
		double delta = 0.0;
		if (m_viewportParams.perspectiveView)
		{
			delta = static_cast<double>(wheelDelta_deg * computeDefaultIncrement()) / 8.0 * getDisplayParameters().zoomSpeed;
			double speedRatio = 10.0 * m_viewportParams.zNear / m_visibleObjectsBBox.getMaxBoxDim();
			double speedCoef = std::min(16.0, exp(speedRatio));
			delta *= speedCoef;
		}
		else
		{
			double cameraCenterToPivotDist = m_viewportParams.getFocalDistance();
			delta = (std::abs(cameraCenterToPivotDist) / (wheelDelta_deg < 0.0 ? -20.0 : 20.0)) * getDisplayParameters().zoomSpeed;
		}

		CCVector3d v(0.0, 0.0, -delta);
		moveCamera(v);
	}

	setLODEnabled(true);
	m_currentLODState.level = 0;

	redraw();
}

void ccGLWindowInterface::startPicking(PickingParameters& params)
{
	//correction for HD screens
	const auto devicePixelRatio = getDevicePixelRatio();
	params.centerX = static_cast<int>(params.centerX * devicePixelRatio);
	params.centerY = static_cast<int>(params.centerY * devicePixelRatio);

	if (!m_globalDBRoot && !m_winDBRoot)
	{
		//we must always emit a signal!
		processPickingResult(params, nullptr, -1);
		return;
	}

	if (	params.mode == POINT_OR_TRIANGLE_PICKING
		||	params.mode == POINT_OR_TRIANGLE_OR_LABEL_PICKING
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

void ccGLWindowInterface::startOpenGLPicking(const PickingParameters& params)
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
		flags |= CC_FAST_ENTITY_PICKING;
	case ENTITY_PICKING:
	case ENTITY_RECT_PICKING:
		flags |= CC_ENTITY_PICKING;
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
	doMakeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	if (!initFBOSafe(m_pickingFbo, glWidth(), glHeight()))
	{
		ccLog::Warning("[FBO] Initialization failed!");
		//we must always emit a signal!
		processPickingResult(params, nullptr, -1);
		return;
	}
	bindFBO(m_pickingFbo);

	//we have to clear the display to be sure there's no color
	glFunc->glClearColor(0, 0, 0, 255);
	glFunc->glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	//get context
	CC_DRAW_CONTEXT CONTEXT;
	getContext(CONTEXT);

	//3D objects picking
	{
		CONTEXT.drawingFlags = CC_DRAW_3D | flags;

		//projection matrix
		glFunc->glMatrixMode(GL_PROJECTION);

		glFunc->glLoadMatrixd(getProjectionMatrix().data());

		//model view matrix
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glLoadMatrixd(getModelViewMatrix().data());

		glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
		glFunc->glEnable(GL_DEPTH_TEST);

		//display 3D objects
		//DGM: all of them, even if we don't pick the window own DB for instance, as they can hide the other objects!
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

		setStandardOrthoCenter();

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

	logGLError("ccGLWindow::startPicking.render");
	glFunc->glFlush();

	if (CONTEXT.entityPicking.getLastID() == 0)
	{
		//no pickable entity displayed
		//we must always emit a signal!
		bindFBO(nullptr);
		processPickingResult(params, nullptr, -1);
		return;
	}

	glFunc->glFinish();

	ccHObject* pickedEntity = nullptr;
	std::unordered_set<int> selectedIDs;

	try
	{
		// crop the picking rectangle so that's it strictly inside the displayed window
		int xCenter = params.centerX;
		int xTop = xCenter - params.pickWidth / 2;
		int xWidth = params.pickWidth;
		if (xTop < 0)
		{
			// crop pixels with negative X positions
			xWidth += xTop;
			xCenter += xTop;
			xTop = 0;
		}
		xWidth = std::min(xWidth, static_cast<int>(glWidth()) - xTop);
		if (xWidth <= 0)
		{
			bindFBO(nullptr);
			processPickingResult(params, nullptr, -1);
			return;
		}

		int yCenter = glHeight() - 1 - params.centerY;
		int yTop = yCenter - params.pickHeight / 2;
		int yWidth = params.pickHeight;
		if (yTop < 0)
		{
			// crop pixels with negative Y positions
			yWidth += yTop;
			yCenter += yTop;
			yTop = 0;
		}
		yWidth = std::min(yWidth, static_cast<int>(glHeight()) - yTop);
		if (yWidth <= 0)
		{
			bindFBO(nullptr);
			processPickingResult(params, nullptr, -1);
			return;
		}

		int pixelCount = xWidth * yWidth;
		std::vector<ccColor::Rgba> pickedPixels;
		pickedPixels.resize(pixelCount, ccColor::black);

		//read the pixel under the mouse
		glFunc->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		glFunc->glReadPixels(xTop, yTop, xWidth, yWidth, GL_RGBA, GL_UNSIGNED_BYTE, pickedPixels.data());

		bindFBO(nullptr);

		//QImage testImage(xWidth, yWidth, QImage::Format_RGB888);

		//process hits
		int minSquareDistToCenter = -1;
		ccColor::Rgba centerPixelColor(0, 0, 0, 0);
		ccColor::Rgba previousPixelColor(0, 0, 0, 0);

		ccColor::Rgba* _pickedPixels = pickedPixels.data();
		for (int j = 0; j < yWidth; ++j)
		{
			for (int i = 0; i < xWidth; ++i, ++_pickedPixels)
			{
				//testImage.setPixelColor(i, yWidth - 1 - j, QColor(_pickedPixels->r, _pickedPixels->g, _pickedPixels->b));
				if (_pickedPixels->r != 0 || _pickedPixels->g != 0 || _pickedPixels->b != 0)
				{
					if (params.mode == ENTITY_RECT_PICKING)
					{
						// avoid reprocessing the pixel corresponding to the same cloud over and over
						if (_pickedPixels->r != previousPixelColor.r || _pickedPixels->g != previousPixelColor.g || _pickedPixels->b != previousPixelColor.b)
						{
							previousPixelColor = *_pickedPixels;
							ccHObject* object = CONTEXT.entityPicking.objectFromColor(*_pickedPixels);
							if (object)
							{
								selectedIDs.insert(object->getUniqueID());
							}
							else
							{
								assert(false);
							}
						}
					}
					else
					{
						int dX = i - xCenter;
						int dY = j - yCenter;
						int squareDistToCenter = dX * dX + dY * dY;
						if (minSquareDistToCenter < 0 || squareDistToCenter < minSquareDistToCenter)
						{
							minSquareDistToCenter = squareDistToCenter;
							centerPixelColor = *_pickedPixels;
						}
					}
				}
			}
		}

		if (params.mode != ENTITY_RECT_PICKING && minSquareDistToCenter >= 0)
		{
			pickedEntity = CONTEXT.entityPicking.objectFromColor(centerPixelColor);
		}

		//testImage.save("C:\\Temp\\test.png");

		//standard output is made through the 'selectedIDs' set
		if (pickedEntity)
		{
			assert(params.mode != ENTITY_RECT_PICKING);
			selectedIDs.insert(pickedEntity->getUniqueID());
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Warning("[Picking] Not enough memory!");
	}

	processPickingResult(params, pickedEntity, -1, nullptr, nullptr, &selectedIDs);
}

void ccGLWindowInterface::processPickingResult(	const PickingParameters& params,
												ccHObject* pickedEntity,
												int pickedItemIndex,
												const CCVector3* nearestPoint/*=nullptr*/,
												const CCVector3d* nearestPointBC/*=nullptr*/,
												const std::unordered_set<int>* selectedIDs/*=nullptr*/)
{
	//standard "entity" picking
	if (params.mode == ENTITY_PICKING)
	{
		Q_EMIT m_signalEmitter->entitySelectionChanged(pickedEntity);
	}
	//rectangular "entity" picking
	else if (params.mode == ENTITY_RECT_PICKING)
	{
		if (selectedIDs)
			Q_EMIT m_signalEmitter->entitiesSelectionChanged(*selectedIDs);
	}
	//3D point or triangle or label picking
	else if (	params.mode == POINT_PICKING
			||	params.mode == TRIANGLE_PICKING
			||	params.mode == POINT_OR_TRIANGLE_PICKING
			||	params.mode == POINT_OR_TRIANGLE_OR_LABEL_PICKING)
	{
		assert(pickedEntity == nullptr || pickedItemIndex >= 0);
		assert(nearestPoint && nearestPointBC);

		Q_EMIT m_signalEmitter->itemPicked(pickedEntity, static_cast<unsigned>(pickedItemIndex), params.centerX, params.centerY, *nearestPoint, *nearestPointBC);
	}
	//fast picking (labels, interactors, etc.)
	else if (params.mode == FAST_PICKING)
	{
		Q_EMIT m_signalEmitter->itemPickedFast(pickedEntity, pickedItemIndex, params.centerX, params.centerY);
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
				label->setPosition(	static_cast<float>(params.centerX + 20) / glWidth(),
									static_cast<float>(params.centerY + 20) / glHeight());
				Q_EMIT m_signalEmitter->newLabel(static_cast<ccHObject*>(label));
				QCoreApplication::processEvents();

				redraw(false, false);
			}
		}
	}
}

void ccGLWindowInterface::startCPUBasedPointPicking(const PickingParameters& params)
{
	//qint64 t0 = m_timer.elapsed();

	CCVector2d clickedPos(params.centerX, glHeight() - 1 - params.centerY);
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
				else if (ent->isKindOf(CC_TYPES::MESH)
					&& !ent->isA(CC_TYPES::MESH_GROUP) //we don't need to process mesh groups as their children will be processed later
					&& !ent->isA(CC_TYPES::COORDINATESYSTEM) // we ignore coordinate system entities
					)
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
					if (mesh->trianglePicking(clickedPos,
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
							nearestPoint = P.toPC();
							nearestEntity = mesh;
							nearestPointBC = barycentricCoords;
						}
					}
				}
				else if (params.mode == PICKING_MODE::POINT_OR_TRIANGLE_OR_LABEL_PICKING && ent->isA(CC_TYPES::LABEL_2D))
				{
					cc2DLabel* label = static_cast<cc2DLabel*>(ent);

					int nearestPointIndex = -1;
					double nearestSquareDist = 0.0;

					if (label->pointPicking(clickedPos,
											camera,
											nearestPointIndex,
											nearestSquareDist))
					{
						if (nearestElementIndex < 0 || (nearestPointIndex >= 0 && nearestSquareDist < nearestElementSquareDist))
						{
							nearestElementSquareDist = nearestSquareDist;
							assert(nearestPointIndex < static_cast<int>(label->size()));
							nearestElementIndex = nearestPointIndex;
							nearestPoint = label->getPickedPoint(nearestPointIndex).getPointPosition();
							nearestEntity = label;
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

void ccGLWindowInterface::displayNewMessage(const QString& message,
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
			ccLog::Warning("[ccGLWindowInterface::displayNewMessage] Appending an empty message has no effect!");
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
			ccLog::Warning("[ccGLWindowInterface::displayNewMessage] Append is not supported for center screen messages!");
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

void ccGLWindowInterface::setPointSize(float size, bool silent/*=false*/)
{
	float newSize = std::max(std::min(size, MAX_POINT_SIZE_F), MIN_POINT_SIZE_F);

	if (m_viewportParams.defaultPointSize != newSize)
	{
		m_viewportParams.defaultPointSize = newSize;
		deprecate3DLayer();

		if (!silent)
		{
			displayNewMessage(	QString("New default point size: %1").arg(newSize),
								ccGLWindowInterface::LOWER_LEFT_MESSAGE,
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
				ccLog::Print(QString("Default point size is already at minimum : %1").arg(MIN_POINT_SIZE_F));
			else
				ccLog::Print(QString("Default point size is already at maximum : %1").arg(MAX_POINT_SIZE_F));
		}
	}
}

void ccGLWindowInterface::setLineWidth(float width, bool silent/*=false*/)
{
	if (!silent)
	{
		if (width < MIN_LINE_WIDTH_F)
			ccLog::Warning(QString("Line width is too small: %1/%2").arg(width).arg(MIN_LINE_WIDTH_F));
		else if (width > MAX_LINE_WIDTH_F)
			ccLog::Warning(QString("Line width is too big: %1/%2").arg(width).arg(MAX_LINE_WIDTH_F));
	}
	float newWidth = std::max(std::min(width, MAX_LINE_WIDTH_F), MIN_LINE_WIDTH_F);

	if (m_viewportParams.defaultLineWidth != newWidth)
	{
		m_viewportParams.defaultLineWidth = newWidth;
		deprecate3DLayer();
		if (!silent)
		{
			displayNewMessage(QString("New default line width: %1").arg(newWidth),
				ccGLWindowInterface::LOWER_LEFT_MESSAGE,
				false,
				2,
				SCREEN_SIZE_MESSAGE); //DGM HACK: we cheat and use the same 'slot' as the window size
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

int ccGLWindowInterface::getFontPointSize() const
{
	return (m_captureMode.enabled ? FontSizeModifier(getDisplayParameters().defaultFontSize, m_captureMode.zoomFactor) : getDisplayParameters().defaultFontSize) * getDevicePixelRatio();
}

void ccGLWindowInterface::setFontPointSize(int pointSize)
{
	m_font.setPointSize(pointSize);
}

QFont ccGLWindowInterface::getTextDisplayFont() const
{
	QFont font = m_font;
	font.setPointSize(getFontPointSize());
	return font;
}

int ccGLWindowInterface::getLabelFontPointSize() const
{
	return (m_captureMode.enabled ? FontSizeModifier(getDisplayParameters().labelFontSize, m_captureMode.zoomFactor) : getDisplayParameters().labelFontSize) * getDevicePixelRatio();
}

QFont ccGLWindowInterface::getLabelDisplayFont() const
{
	QFont font = m_font;
	font.setPointSize(getLabelFontPointSize());
	return font;
}

void ccGLWindowInterface::glEnableSunLight()
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

void ccGLWindowInterface::glDisableSunLight()
{
	functions()->glDisable(GL_LIGHT0);
}

void ccGLWindowInterface::setSunLight(bool state)
{
	m_sunLightEnabled = state;
	displayNewMessage(	state ? "Sun light ON" : "Sun light OFF",
						ccGLWindowInterface::LOWER_LEFT_MESSAGE,
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

void ccGLWindowInterface::toggleSunLight()
{
	setSunLight(!m_sunLightEnabled);
}

void ccGLWindowInterface::glEnableCustomLight()
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

void ccGLWindowInterface::glDisableCustomLight()
{
	functions()->glDisable(GL_LIGHT1);
}

void ccGLWindowInterface::setCustomLight(bool state)
{
	m_customLightEnabled = state;
	displayNewMessage(	state ? "Custom light ON" : "Custom light OFF",
						ccGLWindowInterface::LOWER_LEFT_MESSAGE,
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

void ccGLWindowInterface::toggleCustomLight()
{
	setCustomLight(!m_customLightEnabled);
}

void ccGLWindowInterface::drawCustomLight()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	ccGL::Color(glFunc, ccColor::yellow);
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

void ccGLWindowInterface::setPivotVisibility(PivotVisibility vis)
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

void ccGLWindowInterface::showPivotSymbol(bool state)
{
	//is the pivot really going to be drawn?
	if (state && !m_pivotSymbolShown && m_viewportParams.objectCenteredView && m_pivotVisibility != PIVOT_HIDE)
	{
		invalidateViewport();
		deprecate3DLayer();
	}

	m_pivotSymbolShown = state;
}

void ccGLWindowInterface::togglePerspective(bool objectCentered)
{
	if (m_viewportParams.objectCenteredView != objectCentered)
		setPerspectiveState(true, objectCentered);
	else
		setPerspectiveState(!m_viewportParams.perspectiveView, objectCentered);
}

double ccGLWindowInterface::computeActualPixelSize() const
{
	double pixelSize = m_viewportParams.computePixelSize(glWidth()); // we now use the width as the driving dimension for scaling

	// but we have to compensate for the aspect ratio is h > w
	double ar = static_cast<double>(glHeight()) / glWidth();
	if (ar > 1.0)
	{
		pixelSize *= ar;
	}

	return pixelSize;

}

void ccGLWindowInterface::setBubbleViewMode(bool state)
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

void ccGLWindowInterface::setPerspectiveState(bool state, bool objectCenteredView)
{
	//previous state
	bool perspectiveWasEnabled = m_viewportParams.perspectiveView;
	bool viewWasObjectCentered = m_viewportParams.objectCenteredView;

	//new state
	m_viewportParams.perspectiveView = state;
	m_viewportParams.objectCenteredView = objectCenteredView;

	if (m_viewportParams.perspectiveView)
	{
		//display message
		displayNewMessage(objectCenteredView ? "Centered perspective ON" : "Viewer-based perspective ON",
			ccGLWindowInterface::LOWER_LEFT_MESSAGE,
			false,
			2,
			PERSPECTIVE_STATE_MESSAGE);
	}
	else
	{
		m_viewportParams.objectCenteredView = true; //object-centered mode is forced for otho. view

		displayNewMessage("Perspective OFF",
			ccGLWindowInterface::LOWER_LEFT_MESSAGE,
			false,
			2,
			PERSPECTIVE_STATE_MESSAGE);
	}

	//Camera center to pivot vector
	CCVector3d cameraCenterToPivot = m_viewportParams.getCameraCenter() - m_viewportParams.getPivotPoint();

	//if we change from object-based to viewer-based visualization, we must
	//'rotate' around the object (or the opposite ;)
	if (viewWasObjectCentered && !m_viewportParams.objectCenteredView)
	{
		m_viewportParams.viewMat.transposed().apply(cameraCenterToPivot); //inverse rotation
	}
	else if (!viewWasObjectCentered && m_viewportParams.objectCenteredView)
	{
		m_viewportParams.viewMat.apply(cameraCenterToPivot);
	}

	setCameraPos(m_viewportParams.getPivotPoint() + cameraCenterToPivot);

	Q_EMIT m_signalEmitter->perspectiveStateChanged();

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

void ccGLWindowInterface::setFov(float fov_deg)
{
	if (CCCoreLib::LessThanEpsilon(fov_deg) || (fov_deg > 180.0f))
	{
		ccLog::Warning("[ccGLWindowInterface::setFov] Invalid FOV value!");
		return;
	}

	//derivation if we are in bubble-view mode
	if (m_bubbleViewModeEnabled)
	{
		setBubbleViewFov(fov_deg);
	}
	else if (m_viewportParams.fov_deg != fov_deg)
	{
		//update param
		m_viewportParams.fov_deg = fov_deg;
		//and camera state
		{
			invalidateViewport();
			invalidateVisualization();
			deprecate3DLayer();

			displayNewMessage(	QString("F.O.V. = %1 deg.").arg(fov_deg, 0, 'f', 1),
								ccGLWindowInterface::LOWER_LEFT_MESSAGE, //DGM HACK: we cheat and use the same 'slot' as the window size
								false,
								2,
								SCREEN_SIZE_MESSAGE);
		}

		Q_EMIT m_signalEmitter->fovChanged(m_viewportParams.fov_deg);
	}
}

float ccGLWindowInterface::getFov() const
{
	return m_bubbleViewModeEnabled ? m_bubbleViewFov_deg : m_viewportParams.fov_deg;
}

void ccGLWindowInterface::setBubbleViewFov(float fov_deg)
{
	if (CCCoreLib::LessThanEpsilon(fov_deg) || (fov_deg > 180.0f))
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
			Q_EMIT m_signalEmitter->fovChanged(m_bubbleViewFov_deg);
		}
	}
}

bool ccGLWindowInterface::setNearClippingPlaneDepth(double depth)
{
	QString message;
	if (std::isnan(depth) || depth <= CCCoreLib::ZERO_TOLERANCE_F)
	{
		if (!std::isnan(m_viewportParams.nearClippingDepth))
		{
			m_viewportParams.nearClippingDepth = std::numeric_limits<double>::quiet_NaN();
			message = QString("Near clipping plane disabled");
		}
		else
		{
			// nothing to do
			return false;
		}
	}
	else
	{
		if (depth < 0.0)
		{
			ccLog::Warning("[ccGLWindowInterface::setNearClippingPlaneDepth] Invalid depth value!");
			return false;
		}

		if (depth > m_viewportParams.farClippingDepth)
		{
			ccLog::Warning(QString("[ccGLWindowInterface::setNearClippingPlaneDepth] near clipping depth (%1) can't be larger than far clipping depth (%2)!").arg(depth).arg(m_viewportParams.farClippingDepth));
			return false;
		}

		if (std::isnan(m_viewportParams.nearClippingDepth) || m_viewportParams.nearClippingDepth != depth)
		{
			//update param
			m_viewportParams.nearClippingDepth = depth;
			message = QString("Near clipping depth = %1").arg(depth);
		}
		else
		{
			// nothing to do
			return false;
		}
	}

	deprecate3DLayer();

	displayNewMessage(	message,
						ccGLWindowInterface::LOWER_LEFT_MESSAGE, //DGM HACK: we cheat and use the same 'slot' as the window size
						false,
						2,
						SCREEN_SIZE_MESSAGE);

	Q_EMIT m_signalEmitter->nearClippingDepthChanged(m_viewportParams.nearClippingDepth);

	return true;
}

bool ccGLWindowInterface::setFarClippingPlaneDepth(double depth)
{
	QString message;
	if (std::isnan(depth) || depth >= 1.0e6)
	{
		if (!std::isnan(m_viewportParams.farClippingDepth))
		{
			m_viewportParams.farClippingDepth = std::numeric_limits<double>::quiet_NaN();
			message = QString("Far clipping plane disabled");
		}
		else
		{
			// nothing to do
			return false;
		}
	}
	else
	{
		if (depth < 0.0)
		{
			ccLog::Warning("[ccGLWindowInterface::setFarClippingPlaneDepth] Invalid depth value!");
			return false;
		}

		if (depth < m_viewportParams.nearClippingDepth)
		{
			ccLog::Warning(QString("[ccGLWindowInterface::setFarClippingPlaneDepth] far clipping depth (%1) can't be smaller than near clipping depth (%2)!").arg(depth).arg(m_viewportParams.nearClippingDepth));
			return false;
		}

		if (std::isnan(m_viewportParams.farClippingDepth) || m_viewportParams.farClippingDepth != depth)
		{
			//update param
			m_viewportParams.farClippingDepth = depth;
			message = QString("Far clipping depth = %1").arg(depth);
		}
		else
		{
			// nothing to do
			return false;
		}
	}

	deprecate3DLayer();

	displayNewMessage(	message,
						ccGLWindowInterface::LOWER_LEFT_MESSAGE, //DGM HACK: we cheat and use the same 'slot' as the window size
						false,
						2,
						SCREEN_SIZE_MESSAGE);

	Q_EMIT m_signalEmitter->farClippingDepthChanged(m_viewportParams.farClippingDepth);

	return true;
}

void ccGLWindowInterface::setViewportParameters(const ccViewportParameters& params)
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

	Q_EMIT m_signalEmitter->baseViewMatChanged(m_viewportParams.viewMat);
	Q_EMIT m_signalEmitter->pivotPointChanged(m_viewportParams.getPivotPoint());
	Q_EMIT m_signalEmitter->cameraPosChanged(m_viewportParams.getCameraCenter());
	Q_EMIT m_signalEmitter->nearClippingDepthChanged(m_viewportParams.nearClippingDepth);
	Q_EMIT m_signalEmitter->farClippingDepthChanged(m_viewportParams.farClippingDepth);
	if (!m_bubbleViewModeEnabled)
	{
		Q_EMIT m_signalEmitter->fovChanged(m_viewportParams.fov_deg);
	}
}

void ccGLWindowInterface::rotateBaseViewMat(const ccGLMatrixd& rotMat)
{
	m_viewportParams.viewMat = rotMat * m_viewportParams.viewMat;

	//we emit the 'baseViewMatChanged' signal
	Q_EMIT m_signalEmitter->baseViewMatChanged(m_viewportParams.viewMat);

	invalidateViewport();
	invalidateVisualization();
	deprecate3DLayer();
}

void ccGLWindowInterface::setupProjectiveViewport(	const ccGLMatrixd& cameraMatrix,
													float fov_deg/*=0.0f*/,
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

void ccGLWindowInterface::aboutToBeRemoved(ccDrawableObject* entity)
{
	ccInteractor* interactor = dynamic_cast<ccInteractor*>(entity);
	if (interactor)
	{
		m_activeItems.erase(interactor);
	}
}

void ccGLWindowInterface::setCustomView(const CCVector3d& forward, const CCVector3d& up, bool forceRedraw/*=true*/)
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

void ccGLWindowInterface::setView(CC_VIEW_ORIENTATION orientation, bool forceRedraw/*=true*/)
{
	bool wasViewerBased = !m_viewportParams.objectCenteredView;
	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView, true);

	m_viewportParams.viewMat = ccGLUtils::GenerateViewMat(orientation);

	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView, false);

	invalidateViewport();
	invalidateVisualization();
	deprecate3DLayer();

	//we emit the 'baseViewMatChanged' signal
	Q_EMIT m_signalEmitter->baseViewMatChanged(m_viewportParams.viewMat);

	if (forceRedraw)
		redraw();
}

bool ccGLWindowInterface::renderToFile(	QString filename,
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
		ccLog::Warning(QString("[Snapshot] Failed to save file '%1'!").arg(filename));
	}

	return success;
}

void ccGLWindowInterface::SetShaderPath(const QString& path)
{
	s_shaderPath = path;
}

QString ccGLWindowInterface::GetShaderPath()
{
	return s_shaderPath;
}

void ccGLWindowInterface::removeFBO()
{
	removeFBOSafe(m_fbo);
	removeFBOSafe(m_fbo2);
}

bool ccGLWindowInterface::initGLFilter(int w, int h, bool silent/*=false*/)
{
	if (!m_activeGLFilter)
	{
		return false;
	}

	doMakeCurrent();

	//correction for HD screens
	const auto devicePixelRatio = getDevicePixelRatio();
	w = static_cast<int>(w * devicePixelRatio);
	h = static_cast<int>(h * devicePixelRatio);

	//we "disconnect" current glFilter, to avoid wrong display/errors
	//if QT tries to redraw window during initialization
	ccGlFilter* _filter = nullptr;
	std::swap(_filter, m_activeGLFilter);

	QString error;
	if (!_filter->init(static_cast<unsigned>(w), static_cast<unsigned>(h), s_shaderPath, error))
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

void ccGLWindowInterface::removeGLFilter()
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

int ccGLWindowInterface::getGlFilterBannerHeight() const
{
	return static_cast<int>((QFontMetrics(getFont()).height() + 2 * CC_GL_FILTER_BANNER_MARGIN) * getDevicePixelRatio());
}

void ccGLWindowInterface::display3DLabel(const QString& str, const CCVector3& pos3D, const ccColor::Rgba* color/*=nullptr*/, const QFont& font/*=QFont()*/)
{
	glColor4ubv_safe<ccQOpenGLFunctions>(functions(), color ? *color : getDisplayParameters().textDefaultCol);
	renderText(pos3D.x, pos3D.y, pos3D.z, str, font);
}

void ccGLWindowInterface::displayText(	QString text,
										int x,
										int y,
										unsigned char align/*=ALIGN_HLEFT|ALIGN_VTOP*/,
										float bkgAlpha/*=0*/,
										const ccColor::Rgba* color/*=nullptr*/,
										const QFont* font/*=nullptr*/)
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	int x2 = x;
	int y2 = y;

	//actual text color
	const ccColor::Rgba& rgba = (color ? *color : getDisplayParameters().textDefaultCol);

	QFont textFont = (font ? *font : getTextDisplayFont());

	QFontMetrics fm(textFont);
	int textDescent = fm.descent();
	int margin = std::max(fm.height() / 4, textDescent + 1);

	QRect rect = fm.boundingRect(text);
	int textPosCorrection = rect.y() + rect.height();
	int textHeight = -rect.y() - textDescent;

	if (align != ALIGN_DEFAULT || bkgAlpha != 0.0f)
	{
		//text alignment
		if (align & ALIGN_HMIDDLE)
			x2 -= rect.width() / 2;
		else if (align & ALIGN_HRIGHT)
			x2 -= rect.width();

		if (align & ALIGN_VMIDDLE)
			y2 -= textHeight / 2;
		else if (align & ALIGN_VBOTTOM)
			y2 -= textHeight;

		//background is not totally transparent
		if (bkgAlpha != 0.0f)
		{
			glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
			glFunc->glEnable(GL_BLEND);

			//inverted color with a bit of transparency
			const ccColor::Rgbaf invertedCol(	(255 - rgba.r) / 255.0f,
												(255 - rgba.g) / 255.0f,
												(255 - rgba.b) / 255.0f,
												bkgAlpha );
			ccGL::Color(glFunc, invertedCol);

			int xB = x2 - glWidth() / 2;
			int yB = y2 - glHeight() / 2;

			glFunc->glMatrixMode(GL_PROJECTION);
			glFunc->glPushMatrix();
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();

			setStandardOrthoCenter();

			glFunc->glBegin(GL_POLYGON);
			glFunc->glVertex2d(xB - margin, yB - margin);
			glFunc->glVertex2d(xB - margin, yB + textHeight + margin);
			glFunc->glVertex2d(xB + rect.width() + margin, yB + textHeight + margin);
			glFunc->glVertex2d(xB + rect.width() + margin, yB - margin);
			glFunc->glEnd();

			glFunc->glMatrixMode(GL_PROJECTION);
			glFunc->glPopMatrix();
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPopMatrix();

			glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT
		}
	}

	// compoensate for the fact that the '0' of the text quad is lower than one would expect
	y2 -= textDescent;

	glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, rgba);
	renderText(x2 + 1, glHeight() - 1 - y2, text, 0, textFont); // x2 + 1 --> empirical
}

CCVector3 ccGLWindowInterface::backprojectPointOnTriangle(	const CCVector2i& P2D,
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
	GLdouble G[3] {	l1 * A3D.x + l2 * B3D.x + l3 * C3D.x,
					l1 * A3D.y + l2 * B3D.y + l3 * C3D.y,
					l1 * A3D.z + l2 * B3D.z + l3 * C3D.z };

	return CCVector3::fromArray(G);

}

void ccGLWindowInterface::cancelScheduledRedraw()
{
	m_scheduledFullRedrawTime = 0;
	m_scheduleTimer.stop();
}

void ccGLWindowInterface::scheduleFullRedraw(unsigned maxDelay_ms)
{
	m_scheduledFullRedrawTime = m_timer.elapsed() + maxDelay_ms;

	if (!m_scheduleTimer.isActive())
	{
		m_scheduleTimer.start(500);
	}
}

ccGLWindowInterface::StereoParams::StereoParams()
	: screenWidth_mm(600)
	, screenDistance_mm(800)
	, eyeSeparation_mm(64)
	, stereoStrength(50)
	, glassType(RED_CYAN)
{}

void ccGLWindowInterface::renderText(int x, int y, const QString & str, uint16_t uniqueID/*=0*/, const QFont& font/*=QFont()*/)
{
	if (m_activeFbo)
	{
		m_activeFbo->start();
	}

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//retrieve the texture
	SharedTexture texture;
	if (uniqueID != 0)
	{
		if (m_uniqueTextures.contains(uniqueID))
		{
			//retrieve the texture
			texture = m_uniqueTextures[uniqueID];
		}
		else
		{
			//register it for later
			texture.reset(new QOpenGLTexture(QOpenGLTexture::Target2D));
			m_uniqueTextures.insert(uniqueID, texture);
		}
	}
	else
	{
		if (m_texturePoolLastIndex < m_texturePool.size())
		{
			//retrieve the texture
			texture = m_texturePool[m_texturePoolLastIndex++];
		}
		else
		{
			texture.reset(new QOpenGLTexture(QOpenGLTexture::Target2D));
			try
			{
				m_texturePool.push_back(texture);
				++m_texturePoolLastIndex;
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory to keep the texture?!
			}
		}
	}
	assert(texture);

	//compute the text bounding rect
	// This adjustment and the change to x & y are to work around a crash with Qt 5.9.
	// At the time I (Andy) could not determine if it is a bug in CC or Qt.
	//		https://bugreports.qt.io/browse/QTBUG-61863
	//		https://github.com/CloudCompare/CloudCompare/issues/543
	QRect textRect = QFontMetrics(font).boundingRect(str).adjusted(-1, -2, 1, 2);
	//ccLog::Print(QString("Texture rect = (%1 ; %2) --> (%3 x %4)").arg(textRect.x()).arg(textRect.y()).arg(textRect.width()).arg(textRect.height()));

	x -= 1;	// magic number!
	y += 3;	// magic number!

	QSize imageSize;
	if (texture->isStorageAllocated())
	{
		if (textRect.width() > texture->width() || textRect.height() > texture->height())
		{
			//we have to enlarge it
			texture->destroy();
			imageSize = textRect.size();
		}
		else
		{
			imageSize = QSize(texture->width(), texture->height());
		}
	}
	else
	{
		imageSize = textRect.size();
	}

	// We create a QImage from the text
	QImage textImage(imageSize.width(), imageSize.height(), QImage::Format::Format_RGBA8888);
	QRect imageRect = textImage.rect();
	//ccLog::Print(QString("Image rect = (%1 ; %2) --> (%3 x %4)").arg(imageRect.x()).arg(imageRect.y()).arg(imageRect.width()).arg(imageRect.height()));

	textImage.fill(Qt::transparent);
	{
		QPainter painter(&textImage);

		float glColor[4];
		glFunc->glGetFloatv(GL_CURRENT_COLOR, glColor);
		QColor color;
		color.setRgbF( glColor[0], glColor[1], glColor[2], glColor[3] );

		painter.setPen( color );
		painter.setFont( font );
		painter.drawText(imageRect, Qt::AlignLeft, str );
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
		glFunc->glOrtho(0, glWidth(), 0, glHeight(), -1, 1);
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glPushMatrix();
		glFunc->glLoadIdentity();
		{
			//move to the right position on the screen
			glFunc->glTranslatef(x, glHeight() - 1 - y, 0);

			glFunc->glEnable(GL_TEXTURE_2D);

			if (texture->height() < textRect.height())
			{
				//we have to re-create it!
				texture->destroy();
			}

			//In order to reduce the time ATI cards take to manage the texture ID generation
			//and switching, we re-use the textures as much as possible.
			//texture->setData(textImage, QOpenGLTexture::DontGenerateMipMaps);
			if (!texture->isStorageAllocated())
			{
				//ccLog::Print(QString("New texture allocated: %1 x %2").arg(imageRect.width()).arg(imageRect.height()));
				texture->setMinificationFilter(QOpenGLTexture::Linear);
				texture->setMagnificationFilter(QOpenGLTexture::Linear);
				texture->setFormat(QOpenGLTexture::RGBA8_UNorm);
				texture->setSize(imageRect.width(), imageRect.height());
				texture->setMipLevels(0);
				texture->allocateStorage();
			}
			texture->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt32_RGBA8_Rev, textImage.bits());
			texture->bind();

			ccGL::Color(glFunc, ccColor::bright); //DGM: warning must be float colors to work properly?!
			glFunc->glBegin(GL_QUADS);
			float ratioW = textRect.width() / static_cast<float>(imageRect.width());
			float ratioH = textRect.height() / static_cast<float>(imageRect.height());
			glFunc->glTexCoord2f(0, ratioH); glFunc->glVertex3i(0, 0, 0);
			glFunc->glTexCoord2f(ratioW, ratioH); glFunc->glVertex3i(textRect.width(), 0, 0);
			glFunc->glTexCoord2f(ratioW, 0); glFunc->glVertex3i(textRect.width(), textRect.height(), 0);
			glFunc->glTexCoord2f(0, 0); glFunc->glVertex3i(0, textRect.height(), 0);
			glFunc->glEnd();

			texture->release();
		}

		glFunc->glMatrixMode(GL_PROJECTION);
		glFunc->glPopMatrix();
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glPopMatrix();

		glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT
	}
}

void ccGLWindowInterface::renderText(double x, double y, double z, const QString& str, const QFont& font/*=QFont()*/)
{
	doMakeCurrent();

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
		Q2D.y = glHeight() - 1 - Q2D.y;
		renderText(Q2D.x, Q2D.y, str, 0, font);
	}
}

void ccGLWindowInterface::logGLError(const char* context) const
{
	if (m_initialized)
	{
		LogGLError(functions()->glGetError(), context);
	}
}

void ccGLWindowInterface::LogGLError(GLenum err, const char* context)
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

void ccGLWindowInterface::toggleAutoRefresh(bool state, int period_ms/*=0*/)
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

QPointF ccGLWindowInterface::toCenteredGLCoordinates(int x, int y) const
{
	return QPointF(x - width() / 2, height() / 2 - y) * getDevicePixelRatio();
}

QPointF ccGLWindowInterface::toCornerGLCoordinates(int x, int y) const
{
	return QPointF(x, height() - 1 - y) * getDevicePixelRatio();
}

void ccGLWindowInterface::removeFBOSafe(ccFrameBufferObject* &fbo)
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

bool ccGLWindowInterface::initFBOSafe(ccFrameBufferObject* &fbo, int w, int h)
{
	//correction for HD screens
	const auto devicePixelRatio = getDevicePixelRatio();
	w = static_cast<int>(w * devicePixelRatio);
	h = static_cast<int>(h * devicePixelRatio);

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

static const GLfloat INVALID_DEPTH = 1.0f;

static const size_t c_depthPickingBufferSize = 9 * sizeof(GLfloat);

bool ccGLWindowInterface::PBOPicking::init()
{
	if (supported && !glBuffer)
	{
		glBuffer = new QOpenGLBuffer(QOpenGLBuffer::PixelPackBuffer);
		if (!glBuffer->create())
		{
			ccLog::Warning("Failed to create picking PBO");
			release();
			supported = false;
			return false;
		}

		glBuffer->setUsagePattern(QOpenGLBuffer::DynamicRead);

		//we need to allocate it the first time
		glBuffer->bind();
		glBuffer->allocate(c_depthPickingBufferSize);
		GLfloat depthPickingBuffer[9];
		for (int i = 0 ; i < 9; ++i)
			depthPickingBuffer[i] = INVALID_DEPTH;
		glBuffer->write(0, depthPickingBuffer, sizeof(GLfloat) * 9);
		glBuffer->release();

		timer.start();
	}

	return true;
}

void ccGLWindowInterface::PBOPicking::release()
{
	if (glBuffer)
	{
		delete glBuffer;
		glBuffer = nullptr;
	}
}

GLfloat ccGLWindowInterface::getGLDepth(int x, int y, bool extendToNeighbors/*=false*/, bool usePBO/*=false*/)
{
	doMakeCurrent();

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	int kernel[2] = { 1, 1 };
	GLfloat depthPickingBuffer[9];

	if (extendToNeighbors)
	{
		if (x > 0 && x + 1 < glWidth())
		{
			kernel[0] = 3;
			--x;
		}
		if (y > 0 && y + 1 < glHeight())
		{
			kernel[1] = 3;
			--y;
		}
	}
	int kernelSize = kernel[0] * kernel[1];

	if (usePBO && m_pickingPBO.supported && !m_pickingPBO.glBuffer)
	{
		if (m_pickingPBO.init())
		{
			ccLog::Print("[ccGLWindow] Succesfully initialized PBO for faster depth picking");
			logGLError("m_pickingPBO.init");
		}
	}

	ccFrameBufferObject* formerFBO = m_activeFbo;
	if (m_fbo && m_activeFbo != m_fbo)
	{
		bindFBO(m_fbo);
	}

	bool bufferRestored = false;
	if (usePBO && m_pickingPBO.glBuffer)
	{
		m_pickingPBO.glBuffer->bind();

		qint64 readTime_ms = m_pickingPBO.timer.elapsed();
		qint64 diff_ms = readTime_ms - m_pickingPBO.lastReadTime_ms;

		if (diff_ms < 100)
		{
			//we can read the previous frame buffer (faster) as there shouldn't be too much differences between the two frames
			if (m_pickingPBO.glBuffer->read(0, depthPickingBuffer, kernelSize * sizeof(GLfloat)))
			{
				bufferRestored = true;
			}
			else
			{
				ccLog::Warning("Failed to read the picking PBO contents. We won't use it anymore");
				m_pickingPBO.glBuffer->release();
				m_pickingPBO.release();
				m_pickingPBO.supported = false;
			}
		}

		m_pickingPBO.lastReadTime_ms = readTime_ms;
	}

	glFunc->glReadPixels(x, y, kernel[0], kernel[1], GL_DEPTH_COMPONENT, GL_FLOAT, usePBO && m_pickingPBO.glBuffer ? nullptr : depthPickingBuffer);

	if (usePBO && m_pickingPBO.glBuffer)
	{
		if (!bufferRestored)
		{
			//wait for the buffer to be ready (slower)
			void* _mappedBuffer = m_pickingPBO.glBuffer->map(QOpenGLBuffer::QOpenGLBuffer::ReadOnly);
			if (_mappedBuffer)
			{
				memcpy(depthPickingBuffer, _mappedBuffer, kernelSize * sizeof(GLfloat));
				m_pickingPBO.glBuffer->unmap();
			}
			else
			{
				ccLog::Warning("Failed to map the picking PBO contents. We won't use it anymore");
				m_pickingPBO.glBuffer->release();
				m_pickingPBO.release();
				m_pickingPBO.supported = false;

				//reset the picking buffer to release things gracefully
				depthPickingBuffer[0] = INVALID_DEPTH;
				kernelSize = 1;
				extendToNeighbors = false;
			}
		}
		if (m_pickingPBO.glBuffer) // may be null if an error occurred when calling glBuffer->map
		{
			m_pickingPBO.glBuffer->release();
		}
	}
	if (m_activeFbo != formerFBO)
	{
		bindFBO(formerFBO);
	}

	logGLError("getGLDepth");

	//by default, we take the center value (= pixel(x,y))
	GLfloat minZ = depthPickingBuffer[(kernelSize + 1) / 2 - 1];

	//if the depth is not defined...
	if (minZ == INVALID_DEPTH && extendToNeighbors)
	{
		//...extend the search to the neighbors
		for (int i = 0; i < kernelSize; ++i)
		{
			minZ = std::min(minZ, depthPickingBuffer[i]);
		}
	}

	return minZ;
}

bool ccGLWindowInterface::getClick3DPos(int x, int y, CCVector3d& P3D, bool usePBO)
{
	y = glHeight() - 1 - y;
	GLfloat glDepth = getGLDepth(x, y, false, usePBO);
	if (glDepth == INVALID_DEPTH)
	{
		return false;
	}

	CCVector3d P2D(x, y, glDepth);

	ccGLCameraParameters camera;
	getGLCameraParameters(camera);
	return camera.unproject(P2D, P3D);
}

void ccGLWindowInterface::lockRotationAxis(bool state, const CCVector3d& axis)
{
	m_rotationAxisLocked = state;
	m_lockedRotationAxis = axis;
	m_lockedRotationAxis.normalize();
}

void ccGLWindowInterface::drawCross()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//force line width
	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(1.0f);

	//cross OpenGL drawing
	glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::lightGrey);
	glFunc->glBegin(GL_LINES);
	glFunc->glVertex3f(0.0f, -CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f);
	glFunc->glVertex3f(0.0f, CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f);
	glFunc->glVertex3f(-CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f, 0.0f);
	glFunc->glVertex3f(CC_DISPLAYED_CENTER_CROSS_LENGTH, 0.0f, 0.0f);
	glFunc->glEnd();

	glFunc->glPopAttrib(); //GL_LINE_BIT
}

float ccGLWindowInterface::computeTrihedronLength() const
{
	return (CC_DISPLAYED_TRIHEDRON_AXES_LENGTH + CC_TRIHEDRON_TEXT_MARGIN + QFontMetrics(m_font).width('X')) * m_captureMode.zoomFactor;
}

void ccGLWindowInterface::drawTrihedron()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	float trihedronEdgeLength = CC_DISPLAYED_TRIHEDRON_AXES_LENGTH * m_captureMode.zoomFactor;
	float trihedronLength = computeTrihedronLength();

	float halfW = glWidth() / 2.0f;
	float halfH = glHeight() / 2.0f;

	float trihedronCenterX = halfW - trihedronLength - 10.0f * m_captureMode.zoomFactor;
	float trihedronCenterY = halfH - trihedronLength - 5.0f * m_captureMode.zoomFactor;

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glTranslatef(trihedronCenterX, -trihedronCenterY, 0.0f);
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
		ccGL::Color(glFunc, ccColor::red);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3d(CC_DISPLAYED_TRIHEDRON_AXES_LENGTH, 0.0, 0.0);
		ccGL::Color(glFunc, ccColor::green);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3d(0.0, CC_DISPLAYED_TRIHEDRON_AXES_LENGTH, 0.0);
		ccGL::Color(glFunc, ccColor::blueCC);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3d(0.0, 0.0, CC_DISPLAYED_TRIHEDRON_AXES_LENGTH);
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

	// now display the X, Y and Z axis labels
	{
		//static const CCVector3d origin(0.0, 0.0, 0.0);
		const CCVector3d tipX(trihedronEdgeLength, 0.0, 0.0);
		const CCVector3d tipY(0.0, trihedronEdgeLength, 0.0);
		const CCVector3d tipZ(0.0, 0.0, trihedronEdgeLength);

		//CCVector3d origin2D = m_viewportParams.viewMat * origin;
		CCVector3d origin2D = m_viewportParams.viewMat.getTranslationAsVec3D();
		CCVector3d tipX2D = m_viewportParams.viewMat * tipX;
		CCVector3d tipY2D = m_viewportParams.viewMat * tipY;
		CCVector3d tipZ2D = m_viewportParams.viewMat * tipZ;

		QRect rectX = QFontMetrics(m_font).boundingRect('X'); //rect X ignores the rendering zoom!

		double radius = (std::max(rectX.width(), rectX.height()) / 2.0 + CC_TRIHEDRON_TEXT_MARGIN) * m_captureMode.zoomFactor;

		CCVector2d toTrihedronOrigin(trihedronCenterX, -trihedronCenterY);
		CCVector2d toCharOrigin = CCVector2d(-(rectX.x() + rectX.width() / 2.0), rectX.y() + rectX.height() / 4.0) * m_captureMode.zoomFactor; // rectX.height() should be divided by 2, but it looks better with 4 !

		QFont textFont = getTextDisplayFont(); //we take rendering zoom into account!

		ccGL::Color(glFunc, ccColor::red);
		{
			CCVector2d dX(tipX2D.x - origin2D.x, tipX2D.y - origin2D.y);
			dX.normalize();
			dX *= radius;
			CCVector2d posX = CCVector2d(tipX2D.x, tipX2D.y) + toTrihedronOrigin + toCharOrigin + dX;
			renderText(static_cast<int>(halfW + posX.x), static_cast<int>(halfH - posX.y), "X", static_cast<uint16_t>(RenderTextReservedIDs::trihedronX), textFont);
		}

		ccGL::Color(glFunc, ccColor::green);
		{
			CCVector2d dY(tipY2D.x - origin2D.x, tipY2D.y - origin2D.y);
			dY.normalize();
			dY *= radius;
			CCVector2d posY = CCVector2d(tipY2D.x, tipY2D.y) + toTrihedronOrigin + toCharOrigin + dY;
			renderText(static_cast<int>(halfW + posY.x), static_cast<int>(halfH - posY.y), "Y", static_cast<uint16_t>(RenderTextReservedIDs::trihedronY), textFont);
		}

		ccGL::Color(glFunc, ccColor::blueCC);
		{
			CCVector2d dZ(tipZ2D.x - origin2D.x, tipZ2D.y - origin2D.y);
			dZ.normalize();
			dZ *= radius;
			CCVector2d posZ = CCVector2d(tipZ2D.x, tipZ2D.y) + toTrihedronOrigin + toCharOrigin + dZ;
			renderText(static_cast<int>(halfW + posZ.x), static_cast<int>(halfH - posZ.y), "Z", static_cast<uint16_t>(RenderTextReservedIDs::trihedronZ), textFont);
		}
	}
}

inline double RoundScale(double equivalentWidth)
{
	//we compute the scale granularity (to avoid width values with a lot of decimals)
	int k = static_cast<int>(std::floor(std::log(equivalentWidth) / std::log(10.0f)));
	double granularity = std::pow(10.0, static_cast<double>(k)) / 2.0;
	//we choose the value closest to equivalentWidth with the right granularity
	return std::floor(std::max(equivalentWidth / granularity, 1.0))*granularity;
}

void ccGLWindowInterface::drawScale(const ccColor::Rgbub& color)
{
	assert(!m_viewportParams.perspectiveView); //a scale is only valid in ortho. mode!

	float scaleMaxW = glWidth() / 4.0f; //25% of screen width

	double pixelSize = computeActualPixelSize();

	//we first compute the width equivalent to 25% of horizontal screen width
	//(this is why it's only valid in orthographic mode !)
	double equivalentWidth = RoundScale(scaleMaxW * pixelSize);

	QFont font = getTextDisplayFont(); //we take rendering zoom into account!
	QFontMetrics fm(font);

	//we deduce the scale drawing width
	float scaleW_pix = static_cast<float>(equivalentWidth / pixelSize);
	float trihedronLength = computeTrihedronLength();
	float dW = 2.0f * trihedronLength + 20.0f * m_captureMode.zoomFactor;
	float dH = std::max(fm.height() * 1.25f, trihedronLength + 5.0f * m_captureMode.zoomFactor);
	float w = glWidth() / 2.0f - dW;
	float h = glHeight() / 2.0f - dH;
	float tick = 3.0f * m_captureMode.zoomFactor;

	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	//force line width
	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(1.0f);

	//scale OpenGL drawing
	glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, color);
	glFunc->glBegin(GL_LINES);
	glFunc->glVertex3f(w - scaleW_pix, -h, 0.0f);
	glFunc->glVertex3f(w, -h, 0.0f);
	glFunc->glVertex3f(w - scaleW_pix, -h - tick, 0.0f);
	glFunc->glVertex3f(w - scaleW_pix, -h + tick, 0.0f);
	glFunc->glVertex3f(w, -h + tick, 0.0f);
	glFunc->glVertex3f(w, -h - tick, 0.0f);
	glFunc->glEnd();

	glFunc->glPopAttrib(); //GL_LINE_BIT

	// display label
	double textEquivalentWidth = RoundScale(scaleMaxW * pixelSize);
	QString text = QString::number(textEquivalentWidth);
	glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, color);
	renderText(	glWidth() - static_cast<int>(scaleW_pix / 2 + dW) - fm.width(text) / 2,
				glHeight() - static_cast<int>(dH / 2) + fm.height() / 3,
				text,
				static_cast<uint16_t>(RenderTextReservedIDs::ScaleLabel),
				font);
}

void ccGLWindowInterface::setInteractionMode(INTERACTION_FLAGS flags)
{
	m_interactionFlags = flags;

	//we need to explicitely enable 'mouse tracking' to track the mouse when no button is clicked
	doSetMouseTracking(flags & (INTERACT_CLICKABLE_ITEMS | INTERACT_SIG_MOUSE_MOVED));

	if ((flags & INTERACT_CLICKABLE_ITEMS) == 0)
	{
		//auto-hide the embedded icons if they are disabled
		m_clickableItemsVisible = false;
	}
}

void ccGLWindowInterface::doDragEnterEvent(QDragEnterEvent* event)
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

void ccGLWindowInterface::doDropEvent(QDropEvent* event)
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

bool ccGLWindowInterface::processEvents(QEvent* evt)
{
	switch (evt->type())
	{

	case QEvent::NativeGesture:
	{
		QNativeGestureEvent* gestEvent = static_cast<QNativeGestureEvent*>(evt);
		Qt::NativeGestureType gestype = gestEvent->gestureType();
		double value = gestEvent->value();
		switch (gestype)
		{
		case Qt::PanNativeGesture:
			break;
		case Qt::ZoomNativeGesture:
#if defined(Q_OS_MAC)
			onWheelEvent(value);
			Q_EMIT m_signalEmitter->mouseWheelRotated(value);
			evt->accept();
#endif
			break;
		case Qt::SmartZoomNativeGesture:
			break;
		case Qt::RotateNativeGesture:
			break;
		case Qt::SwipeNativeGesture:
			break;
		default:
			break;
		}
	}
	break;

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
			Q_EMIT m_signalEmitter->aboutToClose(this);
			evt->accept();
		}
	}
	return true;

	case QEvent::DragEnter:
	{
		doDragEnterEvent(static_cast<QDragEnterEvent*>(evt));
	}
	return true;

	case QEvent::Drop:
	{
		doDropEvent(static_cast<QDropEvent*>(evt));
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
#if !defined(Q_OS_MAC)
					onWheelEvent(pseudo_wheelDelta_deg);
					Q_EMIT m_signalEmitter->mouseWheelRotated(pseudo_wheelDelta_deg);
#endif
				}
				m_touchBaseDist = dist;
				evt->accept();
				return true;
			}
		}
		ccLog::PrintDebug(QString("Touch update (%1 points)").arg(static_cast<QTouchEvent*>(evt)->touchPoints().size()));
	}
	break;

	default:
		// nothing to do
	break;

	}

	return false;
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

void ccGLWindowInterface::startFrameRateTest()
{
	if (s_frameRateTestInProgress)
	{
		ccLog::Error("Framerate test already in progress!");
		return;
	}
	s_frameRateTestInProgress = true;

	//we save the current view matrix
	s_frameRateBackupMat = m_viewportParams.viewMat;

	QObject::connect(&s_frameRateTimer, &QTimer::timeout, asQObject(), [=]() { redraw(); }, Qt::QueuedConnection);

	displayNewMessage(	"[Framerate test in progress]",
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

void ccGLWindowInterface::stopFrameRateTest()
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

bool ccGLWindowInterface::isFrameRateTestInProgress()
{
	return s_frameRateTestInProgress;
}

void ccGLWindowInterface::updateFrameRateTest()
{
	if (s_frameRateTestInProgress)
	{
		s_frameRateElapsedTime_ms = s_frameRateElapsedTimer.elapsed();
		if (++s_frameRateCurrentFrame > FRAMERATE_TEST_MIN_FRAMES && s_frameRateElapsedTime_ms > FRAMERATE_TEST_DURATION_MSEC)
		{
			QTimer::singleShot(0, [&]() { ccGLWindowInterface::stopFrameRateTest(); });
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
		assert(false);
	}
}

void ccGLWindowInterface::doPaintGL()
{
	if (!initPaintGL())
	{
		// something failed, or it is not necessary to proceed
		return;
	}

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

	//context initialization
	CC_DRAW_CONTEXT CONTEXT;
	getContext(CONTEXT);

	//rendering parameters
	RenderingParams renderingParams;
	renderingParams.drawBackground = false;
	renderingParams.draw3DPass = false;
	renderingParams.drawForeground = true;

	//here are all the reasons for which we would like to update the main 3D layer
	if (	!m_fbo
		|| 	m_updateFBO
		||	m_captureMode.enabled
		||	m_currentLODState.inProgress )
	{
		//we must update the FBO (or display without FBO)
		renderingParams.drawBackground = true;
		renderingParams.draw3DPass = true;
	}

	//other rendering options
	renderingParams.useFBO = (m_fbo != nullptr);
	renderingParams.draw3DCross = getDisplayParameters().displayCross;

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
	//1) left or 'mono' pass
	{
		renderingParams.pass = MONO_OR_LEFT_RENDERING_PASS;
		fullRenderingPass(CONTEXT, renderingParams);
#ifdef DEBUG_TIMINGS
		debugTimings.push_back(m_timer.nsecsElapsed());
#endif
	}

	if (m_stereoModeEnabled) //2) Stereo
	{
		renderingParams.pass = RIGHT_RENDERING_PASS;
		fullRenderingPass(CONTEXT, renderingParams);
#ifdef DEBUG_TIMINGS
		debugTimings.push_back(m_timer.nsecsElapsed());
#endif
	}

	swapGLBuffers();

	m_shouldBeRefreshed = false;

	if (m_autoPickPivotAtCenter
		&& !m_mouseMoved
		&& (renderingParams.hasAutoPivotCandidates[0] || (m_stereoModeEnabled && renderingParams.hasAutoPivotCandidates[1]))
		&& !renderingParams.nextLODState.inProgress)
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
	}

	if (isFrameRateTestInProgress())
	{
		updateFrameRateTest();
	}
	else // no frame rate test in progress
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
				QTimer::singleShot(std::max(baseLODRefreshTime_ms - displayTime_ms, Q_INT64_C(0)), [&]() { renderNextLODLevel(); });
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
		debugTimingsMessage += QString("[DT%1 = %2]").arg(i).arg((debugTimings[i] - debugTimings[i - 1]) / 1000);
	}
	debugTimingsMessage += QString("[DT TOTAL = %2]").arg((debugTimings.back() - debugTimings.front()) / 1000);
	ccLog::Print(debugTimingsMessage);
#endif
}

void ccGLWindowInterface::draw3D(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
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
	if (isLODEnabled()
		&& !isFrameRateTestInProgress()
		&& (!m_stereoModeEnabled || m_stereoParams.glassType != StereoParams::OCULUS)
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
		CONTEXT.stereoPassIndex = renderingParams.pass;

		if (!setCustomCameraProjection(renderingParams, modelViewMat, projectionMat))
		{
			//we use the standard modelview matrix
			modelViewMat = getModelViewMatrix();

			//change eye position
			double eyeOffset = renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS ? -1.0 : 1.0;

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
	if (m_autoPickPivotAtCenter
		&& (!m_stereoModeEnabled || renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS))
	{
		CCVector3d P;
		if (getClick3DPos(glWidth() / 2, glHeight() / 2, P, !m_stereoModeEnabled)) //can't use PBO in stereo mode
		{
			renderingParams.autoPivotCandidates[renderingParams.pass] = P;
			renderingParams.hasAutoPivotCandidates[renderingParams.pass] = true;
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
	if (renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS) //only the first pass is meaningful
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
		&& !m_viewportParams.perspectiveView
		&& (!renderingParams.useFBO || !m_activeGLFilter))
	{
		setStandardOrthoCenter();
		drawCross();
	}

	logGLError("ccGLWindow::draw3D");
}

void ccGLWindowInterface::fullRenderingPass(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
{
	//visual traces
	QStringList diagStrings;
	if (m_showDebugTraces)
	{
		diagStrings << QString("Stereo mode %1 (pass %2)").arg(m_stereoModeEnabled ? "ON" : "OFF").arg(renderingParams.pass);
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
		if (m_stereoParams.glassType == StereoParams::NVIDIA_VISION
			|| m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
		{
			if (renderingParams.useFBO && renderingParams.pass == RIGHT_RENDERING_PASS)
			{
				currentFBO = m_fbo2;
			}
		}
		else
		{
			modifiedViewport = prepareOtherStereoGlassType(CONTEXT, renderingParams, currentFBO);
		}
	}

	//if a FBO is activated
	if (currentFBO && renderingParams.useFBO)
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
		GLboolean isStereoEnabled = GL_FALSE;
		glFunc->glGetBooleanv(GL_STEREO, &isStereoEnabled);
		//ccLog::Warning(QString("[fullRenderingPass:%0][NO FBO] Stereo test: %1").arg(renderingParams.pass).arg(isStereoEnabled));

		if (isStereoEnabled)
		{
			if (	m_stereoModeEnabled
				&&	(m_stereoParams.glassType == StereoParams::NVIDIA_VISION || m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
				)
			{
				glFunc->glDrawBuffer(renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS ? GL_BACK_LEFT : GL_BACK_RIGHT);
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
				renderingParams.clearColorLayer = (renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS);
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
			static GLboolean RED[3] { GL_TRUE, GL_FALSE, GL_FALSE };
			static GLboolean BLUE[3] { GL_FALSE, GL_FALSE, GL_TRUE };
			static GLboolean CYAN[3] { GL_FALSE, GL_TRUE, GL_TRUE };

			GLboolean* rgbFilter = nullptr;
			switch (m_stereoParams.glassType)
			{
			case StereoParams::RED_BLUE:
				rgbFilter = (renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS ? RED : BLUE);
				break;

			case StereoParams::BLUE_RED:
				rgbFilter = (renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS ? BLUE : RED);
				break;

			case StereoParams::RED_CYAN:
				rgbFilter = (renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS ? RED : CYAN);
				break;

			case StereoParams::CYAN_RED:
				rgbFilter = (renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS ? CYAN : RED);
				break;

			default:
				assert(false);
			}

			if (rgbFilter)
			{
				glFunc->glColorMask(rgbFilter[0], rgbFilter[1], rgbFilter[2], GL_TRUE);
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
			if (renderingParams.pass == RIGHT_RENDERING_PASS)
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

	glFunc->glFlush();

	//process and/or display the FBO (if any)
	bool oculusMode = (m_stereoModeEnabled && m_stereoParams.glassType == StereoParams::OCULUS);
	if (currentFBO && renderingParams.useFBO)
	{
		//we disable the FBO (if any)
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
				bindFBO(nullptr); //in case the active filter has used a FBO!

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
				GLboolean isStereoEnabled = GL_FALSE;
				glFunc->glGetBooleanv(GL_STEREO, &isStereoEnabled);
				//ccLog::Warning(QString("[fullRenderingPass:%0][FBO] Stereo test: %1").arg(renderingParams.pass).arg(isStereoEnabled));
				if (isStereoEnabled)
				{
					if (	m_stereoModeEnabled
						&&	(m_stereoParams.glassType == StereoParams::NVIDIA_VISION || m_stereoParams.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
						)
					{
						glFunc->glDrawBuffer(renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS ? GL_BACK_LEFT : GL_BACK_RIGHT);
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

	processOtherStereoGlassType(renderingParams);

	/******************/
	/*** FOREGROUND ***/
	/******************/
	if (renderingParams.drawForeground && !oculusMode)
	{
		drawForeground(CONTEXT, renderingParams);
	}

	glFunc->glFlush();
}

void ccGLWindowInterface::drawBackground(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
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
														1.0f );

				glFunc->glClearColor(	backgroundColor.r,
										backgroundColor.g,
										backgroundColor.b,
										backgroundColor.a );

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

void ccGLWindowInterface::drawForeground(CC_DRAW_CONTEXT& CONTEXT, RenderingParams& renderingParams)
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

				auto devicePixelRatio = getDevicePixelRatio();
				QFont newFont;
				newFont.setPointSize(static_cast<int>(newFont.pointSizeF() * getDevicePixelRatio()));

				glColor4ubv_safe<ccQOpenGLFunctions>(glFunc, ccColor::black);
				renderText(	static_cast<int>(10 * devicePixelRatio),
							borderHeight - static_cast<int>((CC_GL_FILTER_BANNER_MARGIN - CC_GL_FILTER_BANNER_MARGIN / 2) * devicePixelRatio),
							QString("[GL filter] ") + m_activeGLFilter->getDescription(),
							static_cast<uint16_t>(RenderTextReservedIDs::GLFilterLabel),
							newFont);

				yStart += borderHeight;
			}

			//current messages (if valid)
			if (!m_messagesToDisplay.empty())
			{
				glColor3ubv_safe<ccQOpenGLFunctions>(glFunc, textCol);

				int ll_currentHeight = glHeight() - 10; //lower left
				int uc_currentHeight = 10; //upper center

				QFont font = getTextDisplayFont();

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
						renderText(10, ll_currentHeight, message.message, textureID, font);
						int messageHeight = QFontMetrics(font).height();
						ll_currentHeight -= (messageHeight * 5) / 4; //add a 25% margin
					}
					break;

					case UPPER_CENTER_MESSAGE:
					{
						QRect rect = QFontMetrics(font).boundingRect(message.message);
						//take the GL filter banner into account!
						int x = (glWidth() - rect.width()) / 2;
						int y = uc_currentHeight + rect.height();
						if (showGLFilterRibbon)
						{
							y += getGlFilterBannerHeight();
						}
						renderText(x, y, message.message, textureID, font);
						uc_currentHeight += (rect.height() * 5) / 4; //add a 25% margin
					}
					break;

					case SCREEN_CENTER_MESSAGE:
					{
						QFont newFont(font); //no need to take zoom into account!
						newFont.setPointSize(static_cast<int>(12 * getDevicePixelRatio()));
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
					float col[3] = { textCol.rgb[0] * intensity,
										textCol.rgb[1] * intensity,
										textCol.rgb[2] * intensity };
					glFunc->glColor3fv(col);
					glFunc->glVertex3f(cx + radius * std::cos(i*alpha), cy + radius * std::sin(i*alpha), 0);
				}
				glFunc->glEnd();

				glFunc->glPopAttrib(); //GL_POINT_BIT | GL_DEPTH_BUFFER_BIT

				yStart += lodIconSize + margin;
			}
		}
	}

	logGLError("ccGLWindow::drawForeground");
}

void ccGLWindowInterface::onItemPickedFast(ccHObject* pickedEntity, int pickedItemIndex, int x, int y)
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

QImage ccGLWindowInterface::renderToImage(	float zoomFactor/*=1.0f*/,
											bool dontScaleFeatures/*=false*/,
											bool renderOverlayItems/*=false*/,
											bool silent/*=false*/)
{
	QImage outputImage;

	if (!m_glExtFuncSupported) //no FBO support?!
	{
		if (isStereo())
		{
			if (!silent)
			{
				ccLog::Error("Direct screen capture without FBO is not supported anymore!");
			}
			return QImage();
		}
		else
		{
			//if no shader or fbo --> we grab the screen directly
			if (m_activeShader)
			{
				if (!silent)
					ccLog::Error("Direct screen capture with shader is not supported!");
			}
			else
			{
				outputImage = doGrabFramebuffer();
				if (outputImage.isNull())
				{
					if (!silent)
						ccLog::Error("Direct screen capture failed! (not enough memory?)");
				}
			}
			return outputImage;
		}
	}

	//otherwise FBOs are supported
	if (!silent)
	{
		ccLog::Print("[Render screen via FBO]");
	}

	doMakeCurrent();

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

		bool success = (fbo->init(glWidth(), glHeight())
			&& fbo->initColor()
			&& fbo->initDepth());
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
	if (m_stereoModeEnabled && !m_stereoParams.isAnaglyph())
	{
		// Screen capture doesn't work with real stereo rendering
		m_stereoModeEnabled = false;
	}

	//disable LOD!
	bool wasLODEnabled = isLODEnabled();
	setLODEnabled(false);

	//enable the FBO
	bindFBO(fbo);
	logGLError("ccGLWindow::renderToFile/FBO start");

	fullRenderingPass(CONTEXT, renderingParams);

	if (m_stereoModeEnabled) //2 nd pass for single display 'stereo' rendering (= anaglyphs)
	{
		renderingParams.pass = RIGHT_RENDERING_PASS;
		fullRenderingPass(CONTEXT, renderingParams);
	}

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

	invalidateViewport();
	invalidateVisualization();
	redraw(true);

	return outputImage;
}

void ccGLWindowInterface::checkScheduledRedraw()
{
	if (m_scheduledFullRedrawTime && m_timer.elapsed() > m_scheduledFullRedrawTime)
	{
		redraw();
	}
}

void ccGLWindowInterface::doPicking()
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

void ccGLWindowInterface::renderNextLODLevel()
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


void ccGLWindowInterface::toggleExclusiveFullScreen(bool state)
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
				doShowFullScreen();
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
				doShowNormal();
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

bool ccGLWindowInterface::initFBO(int w, int h)
{
	doMakeCurrent();

	if (!initFBOSafe(m_fbo, w, h))
	{
		ccLog::Warning("[FBO] Initialization failed!");
		m_alwaysUseFBO = false;
		removeFBOSafe(m_fbo2);
		setLODEnabled(false);
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
			setLODEnabled(false);
			return false;
		}
	}

	deprecate3DLayer();
	return true;
}

void ccGLWindowInterface::processMousePressEvent(QMouseEvent *event)
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
		if ((m_interactionFlags & INTERACT_PAN)
			|| ((QApplication::keyboardModifiers() & Qt::ControlModifier) && (m_interactionFlags & INTERACT_CTRL_PAN))
			)
		{
			setWindowCursor(QCursor(Qt::SizeAllCursor));
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
			setWindowCursor(QCursor(Qt::ClosedHandCursor));
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

void ccGLWindowInterface::processMouseDoubleClickEvent(QMouseEvent *event)
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

void ccGLWindowInterface::processMouseMoveEvent(QMouseEvent *event)
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

			const int devicePixelRatio = getDevicePixelRatio();
			bool inZone = (x * devicePixelRatio * 3 < m_hotZone->topCorner.x() + areaRect.width() * 4   //25% margin
				&& y * devicePixelRatio * 2 < m_hotZone->topCorner.y() + areaRect.height() * 4); //50% margin

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
	setLODEnabled(true);

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
			CCVector3d u(dx * pixSize / m_displayScale.x, -dy * pixSize / m_displayScale.y, 0.0);

			const auto devicePixelRatio = getDevicePixelRatio();
			u *= devicePixelRatio;

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
					setCustomLightPosition(CCVector3f(	m_customLightPos[0] + u.x,
														m_customLightPos[1] + u.y,
														m_customLightPos[2] + u.z) );
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
					&& (QApplication::keyboardModifiers() == Qt::NoModifier
						|| QApplication::keyboardModifiers() == Qt::ControlModifier))
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

			const auto devicePixelRatio = getDevicePixelRatio();
			u *= devicePixelRatio;

			for (auto& activeItem : m_activeItems)
			{
				if (activeItem->move2D(x * devicePixelRatio, y * devicePixelRatio, dx * devicePixelRatio, dy * devicePixelRatio, glWidth(), glHeight()))
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
						rotMat.initFromParameters(CCCoreLib::DegreesToRadians(delta_deg), axis, CCVector3d(0, 0, 0));
					}

					if (std::abs(posDelta.y()) != 0)
					{
						double delta_deg = (posDelta.y() * static_cast<double>(m_bubbleViewFov_deg)) / height();
						//rotation about the local X axis
						ccGLMatrixd rotX;
						rotX.initFromParameters(CCCoreLib::DegreesToRadians(delta_deg), CCVector3d(1, 0, 0), CCVector3d(0, 0, 0));
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
						double angle_rad = 2.0 * M_PI * dx / width();
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
						if (camera.project(m_viewportParams.getPivotPoint(), A2D)
							&& camera.project(m_viewportParams.getPivotPoint() + m_viewportParams.zFar * m_lockedRotationAxis, B2D))
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

void ccGLWindowInterface::processMouseReleaseEvent(QMouseEvent *event)
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
	setWindowCursor(m_defaultCursorShape);

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
		|| (QApplication::keyboardModifiers() & Qt::MetaModifier)
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

void ccGLWindowInterface::processWheelEvent(QWheelEvent* event)
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
		int padDelta = event->delta();
		if (padDelta != 0)
		{
		    float wheelDelta_deg = event->delta() / 8.0f;

		    onWheelEvent(wheelDelta_deg);

		    Q_EMIT m_signalEmitter->mouseWheelRotated(wheelDelta_deg);

		    doRedraw = true;
		}
	}

	if (doRedraw)
	{
		setLODEnabled(true);
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

void ccGLWindowInterface::drawPivot()
{
	if (!m_viewportParams.objectCenteredView
		|| (m_pivotVisibility == PIVOT_HIDE)
		|| (m_pivotVisibility == PIVOT_SHOW_ON_MOVE && !m_pivotSymbolShown))
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

		auto glContext = getOpenGLContext();
		assert(glContext);

		//pivot symbol: 3 circles
		static const ccColor::Rgba RedAlpha(ccColor::redRGB, c_alpha);
		ccGL::Color(glFunc, RedAlpha);
		glDrawUnitCircle(glContext, 0);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(-1.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(1.0f, 0.0f, 0.0f);
		glFunc->glEnd();

		static const ccColor::Rgba GreenAlpha(ccColor::greenRGB, c_alpha);
		ccGL::Color(glFunc, GreenAlpha);
		glDrawUnitCircle(glContext, 1);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(0.0f, -1.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 1.0f, 0.0f);
		glFunc->glEnd();

		static const ccColor::Rgba BlueCCAlpha(ccColor::blueCCRGB, c_alpha);
		ccGL::Color(glFunc, BlueCCAlpha);
		glDrawUnitCircle(glContext, 2);
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

bool ccGLWindowInterface::enableStereoMode(const StereoParams& params)
{
	if (!m_initialized)
	{
		assert(false);
		ccLog::Warning("OpenGL context not initialized");
		return false;
	}

	bool needSecondFBO = false;
	bool needAutoRefresh = false;

	if (params.glassType == StereoParams::NVIDIA_VISION || params.glassType == StereoParams::GENERIC_STEREO_DISPLAY)
	{
		if (!isStereo())
		{
			ccLog::Warning("Wrong 3D window type for Quad Buffered Stereo rendering");
			return false;
		}

		if (!isQuadBufferSupported())
		{
			QMessageBox::critical(asWidget(), "Stereo", "Quad Buffered Stereo not supported");
			return false;
		}

		if (!exclusiveFullScreen())
		{
			ccLog::Warning("3D window should be in exclusive full screen mode!");
			return false;
		}

		needSecondFBO = true;
		needAutoRefresh = false;
	}

	return enableStereoMode(params, needSecondFBO, needAutoRefresh);
}

bool ccGLWindowInterface::enableStereoMode(const StereoParams& params, bool needSecondFBO, bool needAutoRefresh)
{
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

void ccGLWindowInterface::disableStereoMode()
{
	m_stereoModeEnabled = false;

	if (m_fbo2)
	{
		//we don't need it anymore
		removeFBOSafe(m_fbo2);
	}
}

bool ccGLWindowInterface::isQuadBufferSupported() const
{
	// Check with OpenGL
	if (!s_stereoSupported)
	{
		return false;
	}

	// Check with Qt
	QSurfaceFormat surfaceFormat = getSurfaceFormat();
	return (surfaceFormat.stereo() && surfaceFormat.swapBehavior() == QSurfaceFormat::DoubleBuffer);
}

void ccGLWindowInterface::Create(	ccGLWindowInterface*& window,
									QWidget*& widget,
									bool stereoMode/*=false*/,
									bool silentInitialization/*=false*/ )
{
	if (stereoMode)
	{
		ccGLWindowStereo* glWindowStereo = nullptr;
		ccGLWindowStereo::Create(glWindowStereo, widget, silentInitialization);
		window = glWindowStereo;
	}
	else
	{
		ccGLWindow* glWindow = nullptr;
		ccGLWindow::Create(glWindow, widget, silentInitialization);
		window = glWindow;
	}
}

ccGLWindowInterface* ccGLWindowInterface::FromWidget(QWidget* widget)
{
	ccGLWindow* monoWidget = qobject_cast<ccGLWindow*>(widget);
	if (monoWidget)
	{
		return monoWidget;
	}

	ccGLStereoWidget* stereoWidget = qobject_cast<ccGLStereoWidget*>(widget);
	if (stereoWidget)
	{
		return stereoWidget->associatedWindow();
	}

	assert(false);
	return nullptr;
}

ccGLWindowInterface* ccGLWindowInterface::FromEmitter(QObject* object)
{
	ccGLWindowSignalEmitter* emitter = qobject_cast<ccGLWindowSignalEmitter*>(object);
	if (!emitter)
	{
		ccLog::Warning(QString("[ccGLWindowInterface::FromEmitter] Object %1 is not a window signal emitter").arg(object->objectName()));
		assert(false);
		return nullptr;
	}

	return emitter->getAssociatedWindow();
}

ccGLWindowInterface* ccGLWindowInterface::FromQObject(QObject* object)
{
	ccGLWindow* glWindow = qobject_cast<ccGLWindow*>(object);
	if (glWindow)
	{
		return glWindow;
	}
	ccGLWindowStereo* glStereoWindow = qobject_cast<ccGLWindowStereo*>(object);
	if (glStereoWindow)
	{
		return glStereoWindow;
	}

	ccLog::Warning(QString("[ccGLWindowInterface::FromQObject] Object %1 is not a valid GL window").arg(object->objectName()));
	return nullptr;
}


bool ccGLWindowInterface::TestStereoSupport(bool forceRetest/*=false*/)
{
	if (s_stereoChecked && !forceRetest)
	{
		return s_stereoSupported;
	}

	// create a fake window and an associated OpenGL context
	//QWindow* testWindow = new QWindow;
	//testWindow->setSurfaceType(QWindow::OpenGLSurface);

	QOffscreenSurface offSurface;

	QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	format.setStereo(true);
	//testWindow->setFormat(format);
	offSurface.setFormat(format);
	offSurface.create();

	QSharedPointer<QOpenGLContext> context(new QOpenGLContext(&offSurface));
	context->setFormat(format);
	context->setShareContext(QOpenGLContext::globalShareContext());
	if (!context->create())
	{
		ccLog::Error("Failed to create the OpenGL context");
		//testWindow->deleteLater();
		return false;
	}
	if (!context->isValid())
	{
		ccLog::Error("Created OpenGL context is invalid");
		//testWindow->deleteLater();
		return false;
	}
	//context->makeCurrent(testWindow);
	context->makeCurrent(&offSurface);

	ccQOpenGLFunctions* glFunc = context->versionFunctions<ccQOpenGLFunctions>();
	if (!glFunc)
	{
		ccLog::Warning("Failed to retrieve the OpengGL functions");
		//testWindow->deleteLater();
		return false;
	}
	if (!glFunc->initializeOpenGLFunctions()) //DGM: seems to be necessary at least with Qt 5.4
	{
		ccLog::Warning("Failed to initialize the OpengGL functions");
		//testWindow->deleteLater();
		return false;
	}

	GLboolean isStereoEnabled = GL_FALSE;
	glFunc->glGetBooleanv(GL_STEREO, &isStereoEnabled);

	s_stereoSupported = (isStereoEnabled == GL_TRUE);
	s_stereoChecked = true;

	ccLog::Print(QString("Quad Buffered Stereo mode: %1").arg(isStereoEnabled ? "supported" : "not supported"));

	//testWindow->deleteLater();

	return s_stereoSupported;
}

void ccGLWindowInterface::setCustomLightPosition(const CCVector3f& pos)
{
	m_customLightPos[0] = pos.x;
	m_customLightPos[1] = pos.y;
	m_customLightPos[2] = pos.z;
	invalidateViewport();
	deprecate3DLayer();
}
