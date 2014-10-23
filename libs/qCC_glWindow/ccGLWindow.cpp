//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//CCLib
#include <CCConst.h>
#include <CCPlatform.h>

//qCC
#include "ccGLWindow.h"
#include "ccGuiParameters.h"
#include "ccRenderingTools.h"

//qCC_db
#include <ccLog.h>
#include <ccHObject.h>
#include <ccHObjectCaster.h>
#include <ccBBox.h>
#include <cc2DLabel.h>
#include <ccGenericPointCloud.h>
#include <ccGenericMesh.h>
#include <ccTimer.h>
#include <ccSphere.h> //for the pivot symbol
#include <ccPolyline.h>
#include <ccPointCloud.h>
#include <ccColorRampShader.h>
#include <ccClipBox.h>

//CCFbo
#include <ccGlew.h>
#include <ccShader.h>
#include <ccGlFilter.h>
#include <ccFrameBufferObject.h>
#include <ccFBOUtils.h>

//Qt
#include <QtGui>
#include <QWheelEvent>
#include <QElapsedTimer>
#include <QSettings>
#include <QApplication>
#include <QSharedPointer>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

//System
#include <string.h>
#include <math.h>
#include <algorithm>

//Min and max zoom ratio (relative)
const float CC_GL_MAX_ZOOM_RATIO = 1.0e6f;
const float CC_GL_MIN_ZOOM_RATIO = 1.0e-6f;

//Vaious overlay elements dimensions
const double CC_DISPLAYED_PIVOT_RADIUS_PERCENT  = 0.8; //percentage of the smallest screen dimension
const double CC_DISPLAYED_CUSTOM_LIGHT_LENGTH   = 10.0;
const float  CC_DISPLAYED_TRIHEDRON_AXES_LENGTH = 25.0f;
const float  CC_DISPLAYED_CENTER_CROSS_LENGTH   = 10.0f;

//Hot zone (interactors) triggering area
const int CC_HOT_ZONE_TRIGGER_WIDTH  = 270;
const int CC_HOT_ZONE_TRIGGER_HEIGHT = 100;

//Max click duration for enabling picking mode (in ms)
const int CC_MAX_PICKING_CLICK_DURATION_MS = 200;

//invalid GL list index
const GLuint GL_INVALID_LIST_ID = (~0);

//GL filter banner margin (height = 2*margin + current font height)
const int CC_GL_FILTER_BANNER_MARGIN = 5;

/*** Persistent settings ***/

static const char c_ps_groupName[]			= "ccGLWindow";
static const char c_ps_perspectiveView[]	= "perspectiveView";
static const char c_ps_objectMode[]			= "objectCenteredView";
static const char c_ps_sunLight[]			= "sunLightEnabled";
static const char c_ps_customLight[]		= "customLightEnabled";
static const char c_ps_pivotVisibility[]	= "pivotVisibility";

//Unique GL window ID
static int s_GlWindowNumber = 0;

//On some versions of Qt, QGLWidget::renderText seem to need glColorf instead of glColorub!
// See https://bugreports.qt-project.org/browse/QTBUG-6217
inline static void glColor3ubv_safe(const unsigned char* rgb)
{
	//glColor3ubv(rgb);
	glColor3f(	rgb[0] / 255.0f,
				rgb[1] / 255.0f,
				rgb[2] / 255.0f );
}
inline static void glColor4ubv_safe(const unsigned char* rgb)
{
	//glColor4ubv(rgb);
	glColor4f(	rgb[0] / 255.0f,
				rgb[1] / 255.0f,
				rgb[2] / 255.0f,
				rgb[3] / 255.0f );
}

ccGLWindow::ccGLWindow(	QWidget *parent,
						const QGLFormat& format/*=QGLFormat::defaultFormat()*/,
						QGLWidget* shareWidget/*=0*/,
						bool silentInitialization/*=false*/)
	: QGLWidget(format,parent,shareWidget)
	, m_uniqueID(++s_GlWindowNumber) //GL window unique ID
	, m_initialized(false)
	, m_trihedronGLList(GL_INVALID_LIST_ID)
	, m_pivotGLList(GL_INVALID_LIST_ID)
	, m_lastMousePos(-1,-1)
	, m_lastMouseOrientation(1,0,0)
	, m_currentMouseOrientation(1,0,0)
	, m_validModelviewMatrix(false)
	, m_validProjectionMatrix(false)
	, m_glWidth(0)
	, m_glHeight(0)
	, m_lodActivated(false)
	, m_shouldBeRefreshed(false)
	, m_cursorMoved(false)
	, m_unclosable(false)
	, m_interactionMode(TRANSFORM_CAMERA)
	, m_pickingMode(NO_PICKING)
	, m_pickingModeLocked(false)
	, m_lastClickTime_ticks(0)
	, m_sunLightEnabled(true)
	, m_customLightEnabled(false)
	, m_embeddedIconsEnabled(false)
	, m_hotZoneActivated(false)
	, m_activeShader(0)
	, m_shadersEnabled(false)
	, m_fbo(0)
	, m_alwaysUseFBO(false)
	, m_updateFBO(true)
	, m_colorRampShader(0)
	, m_customRenderingShader(0)
	, m_activeGLFilter(0)
	, m_glFiltersEnabled(false)
	, m_winDBRoot(0)
	, m_globalDBRoot(0) //external DB
	, m_font(font())
	, m_pivotVisibility(PIVOT_SHOW_ON_MOVE)
	, m_pivotSymbolShown(false)
	, m_allowRectangularEntityPicking(true)
	, m_rectPickingPoly(0)
	, m_overridenDisplayParametersEnabled(false)
	, m_displayOverlayEntities(true)
	, m_silentInitialization(silentInitialization)
	, m_verticalRotationLocked(false)
	, m_bubbleViewModeEnabled(false)
	, m_bubbleViewFov_deg(90.0f)
{
	//GL window title
	setWindowTitle(QString("3D View %1").arg(m_uniqueID));

	//GL window own DB
	m_winDBRoot = new ccHObject(QString("DB.3DView_%1").arg(m_uniqueID));

	//lights
	m_sunLightEnabled = true;
	m_sunLightPos[0] = 0;
	m_sunLightPos[1] = 1;
	m_sunLightPos[2] = 1;
	m_sunLightPos[3] = 0;

	m_customLightEnabled = false;
	m_customLightPos[0] = 0;
	m_customLightPos[1] = 0;
	m_customLightPos[2] = 0;
	m_customLightPos[3] = 1; //positional light

	//matrices
	m_viewportParams.viewMat.toIdentity();
	memset(m_viewMatd, 0, sizeof(double)*OPENGL_MATRIX_SIZE);
	memset(m_projMatd, 0, sizeof(double)*OPENGL_MATRIX_SIZE);

	//default modes
	setPickingMode(DEFAULT_PICKING);
	setInteractionMode(TRANSFORM_CAMERA);

	//drag & drop handling
	setAcceptDrops(true);

	//embedded icons (point size, etc.)
	enableEmbeddedIcons(true);

	//auto-load previous perspective settings
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		
		//load parameters
		bool perspectiveView	= settings.value(c_ps_perspectiveView,	false				).toBool();
		//DGM: we force object-centered view by default now, as the viewer-based perspective is too dependent
		//on what is displayed (so restoring this parameter at next startup is rarely a good idea)
		bool objectCenteredView	= /*settings.value(c_ps_objectMode,		true				).toBool()*/true;
		m_sunLightEnabled		= settings.value(c_ps_sunLight,			true				).toBool();
		m_customLightEnabled	= settings.value(c_ps_customLight,		false				).toBool();
		int pivotVisibility		= settings.value(c_ps_pivotVisibility,	PIVOT_SHOW_ON_MOVE	).toInt();
		
		settings.endGroup();

		//report current perspective
		if (!m_silentInitialization)
		{
			if (!perspectiveView)
				ccLog::Print("[ccGLWindow] Perspective is off by default");
			else
				ccLog::Print(QString("[ccGLWindow] Perspective is on by default (%1)").arg(objectCenteredView ? "object-centered" : "viewer-centered"));
		}

		//pivot visibility
		switch(pivotVisibility)
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
			displayNewMessage("Warning: custom light is ON",ccGLWindow::LOWER_LEFT_MESSAGE,false,2,CUSTOM_LIGHT_STATE_MESSAGE);
		if (!m_sunLightEnabled)
			displayNewMessage("Warning: sun light is OFF",ccGLWindow::LOWER_LEFT_MESSAGE,false,2,SUN_LIGHT_STATE_MESSAGE);
	}
}

ccGLWindow::~ccGLWindow()
{
	if (m_globalDBRoot)
	{
		//we must unlink entities currently linked to this window
		m_globalDBRoot->removeFromDisplay_recursive(this);
	}

	makeCurrent();
	if (m_trihedronGLList != GL_INVALID_LIST_ID)
	{
		glDeleteLists(m_trihedronGLList,1);
		m_trihedronGLList = GL_INVALID_LIST_ID;
	}
	if (m_pivotGLList != GL_INVALID_LIST_ID)
	{
		glDeleteLists(m_pivotGLList,1);
		m_pivotGLList = GL_INVALID_LIST_ID;
	}

	if (m_winDBRoot)
		delete m_winDBRoot;

	if (m_rectPickingPoly)
		delete m_rectPickingPoly;

	if (m_activeGLFilter)
		delete m_activeGLFilter;

	if (m_colorRampShader)
		delete m_colorRampShader;

	if (m_customRenderingShader)
		delete m_customRenderingShader;

	if (m_activeShader)
		delete m_activeShader;

	if (m_fbo)
		delete m_fbo;
}

const ccGui::ParamStruct& ccGLWindow::getDisplayParameters() const
{
	return m_overridenDisplayParametersEnabled ? m_overridenDisplayParameters : ccGui::Parameters();
}

void ccGLWindow::initializeGL()
{
	if (m_initialized)
		return;

	//we init model view matrix with identity and store it into 'viewMat' and 'm_viewMatd'
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glGetDoublev(GL_MODELVIEW_MATRIX, m_viewportParams.viewMat.data());
	glGetDoublev(GL_MODELVIEW_MATRIX, m_viewMatd);

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_viewportParams.viewMat);

	//we init projection matrix with identity
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glGetDoublev(GL_PROJECTION_MATRIX, m_projMatd);

	//set viewport and visu. as invalid
	invalidateViewport();
	invalidateVisualization();

	//we initialize GLEW
	InitGLEW();

	//OpenGL version
	const char* vendorName = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
	if (!m_silentInitialization)
	{
		ccLog::Print("[3D View %i] GL version: %s",m_uniqueID,glGetString(GL_VERSION));
		ccLog::Print("[3D View %i] Graphic card manufacturer: %s",m_uniqueID,vendorName);
	}

	ccGui::ParamStruct params = getDisplayParameters();

	//VBO support
	if (ccFBOUtils::CheckVBOAvailability())
	{
		if (params.useVBOs && (!vendorName || QString(vendorName).toUpper().startsWith("ATI")))
		{
			if (!m_silentInitialization)
				ccLog::Warning("[3D View %i] VBO support has been disabled as it may not work on %s cards!\nYou can manually activate it in the display settings (at your own risk!)",m_uniqueID,vendorName);
			params.useVBOs = false;
		}
		else if (!m_silentInitialization)
		{
			ccLog::Print("[3D View %i] VBOs available",m_uniqueID);
		}
	}
	else
	{
		params.useVBOs = false;
	}

	//Shaders and other OpenGL extensions
	m_shadersEnabled = ccFBOUtils::CheckShadersAvailability();
	if (!m_shadersEnabled)
	{
		//if no shader, no GL filter!
		if (!m_silentInitialization)
			ccLog::Warning("[3D View %i] Shaders and GL filters unavailable",m_uniqueID);
	}
	else
	{
		if (!m_silentInitialization)
			ccLog::Print("[3D View %i] Shaders available",m_uniqueID);

		m_glFiltersEnabled = ccFBOUtils::CheckFBOAvailability();
		if (m_glFiltersEnabled)
		{
			if (!m_silentInitialization)
				ccLog::Print("[3D View %i] GL filters available",m_uniqueID);
			m_alwaysUseFBO = true;
		}
		else if (!m_silentInitialization)
		{
			ccLog::Warning("[3D View %i] GL filters unavailable (FBO not supported)",m_uniqueID);
		}

		//color ramp shader
		if (!m_colorRampShader)
		{
			//we will update global parameters
			params.colorScaleShaderSupported = false;

			GLint maxBytes = 0;
			glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_COMPONENTS,&maxBytes);

			const GLint minRequiredBytes = ccColorRampShader::MinRequiredBytes();
			if (maxBytes < minRequiredBytes)
			{
				if (!m_silentInitialization)
					ccLog::Warning("[3D View %i] Not enough memory on shader side to use color ramp shader! (max=%i/%i bytes)",m_uniqueID,maxBytes,minRequiredBytes);
			}
			else
			{
				ccColorRampShader* colorRampShader = new ccColorRampShader();
				QString shadersPath = ccGLWindow::getShadersPath();
				QString error;
				if (!colorRampShader->loadProgram(QString(),shadersPath+QString("/ColorRamp/color_ramp.frag"),error))
				{
					if (!m_silentInitialization)
						ccLog::Warning(QString("[3D View %i] Failed to load color ramp shader: '%2'").arg(m_uniqueID).arg(error));
					delete colorRampShader;
					colorRampShader = 0;
				}
				else
				{
					if (!m_silentInitialization)
						ccLog::Print("[3D View %i] Color ramp shader loaded successfully",m_uniqueID);
					m_colorRampShader = colorRampShader;
					params.colorScaleShaderSupported = true;

					//if global parameter is not yet defined
					if (!getDisplayParameters().isInPersistentSettings("colorScaleUseShader"))
					{
						bool shouldUseShader = true;
						if (!vendorName || QString(vendorName).toUpper().startsWith("ATI") || QString(vendorName).toUpper().startsWith("VMWARE"))
						{
							if (!m_silentInitialization)
								ccLog::Warning("[3D View %i] Color ramp shader will remain disabled as it may not work on %s cards!\nYou can manually activate it in the display settings (at your own risk!)",m_uniqueID,vendorName);
							shouldUseShader = false;
						}
						params.colorScaleUseShader = shouldUseShader;
					}
				}
			}
		}
	}

	//apply (potentially) updated parameters;
	setDisplayParameters(params,hasOverridenDisplayParameters());

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
				renderingShader = 0;
			}
			else
			{
				m_customRenderingShader = renderingShader;
			}
			setDisplayParameters(params,hasOverridenDisplayParameters());
		}
	}
#endif

	//transparency off by default
	glDisable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//no global ambient
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,ccColor::night);

	ccGLUtils::CatchGLError("ccGLWindow::initializeGL");

	m_initialized = true;

	if (!m_silentInitialization)
		ccLog::Print("[ccGLWindow] 3D view initialized");
}

void ccGLWindow::resizeGL(int w, int h)
{
	m_glWidth = w;
	m_glHeight = h;

	//DGM --> QGLWidget doc: 'There is no need to call makeCurrent() because this has already been done when this function is called."
	//makeCurrent();

	//update OpenGL viewport
	glViewport(0,0,w,h);

	invalidateViewport();
	invalidateVisualization();

	//filters
	if (m_fbo || m_alwaysUseFBO)
		initFBO(m_glWidth,m_glHeight);
	if (m_activeGLFilter)
		initGLFilter(m_glWidth,m_glHeight);

	//pivot symbol is dependent on the screen size!
	if (m_pivotGLList != GL_INVALID_LIST_ID)
	{
		glDeleteLists(m_pivotGLList,1);
		m_pivotGLList = GL_INVALID_LIST_ID;
	}

	displayNewMessage(QString("New size = %1 * %2 (px)").arg(w).arg(h),
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						SCREEN_SIZE_MESSAGE);

	ccGLUtils::CatchGLError("ccGLWindow::resizeGL");
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

void ccGLWindow::testFrameRate()
{
	if (s_frameRateTestInProgress)
	{
		ccLog::Error("Framerate test already in progress!");
		return;
	}
	s_frameRateTestInProgress = true;

	//we save the current view matrix
	s_frameRateBackupMat = m_viewportParams.viewMat;

	connect(&s_frameRateTimer, SIGNAL(timeout()), this, SLOT(redraw()), Qt::QueuedConnection);

	displayNewMessage("[Framerate test in progress]",
						ccGLWindow::UPPER_CENTER_MESSAGE,
						true,
						3600);

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
	m_validModelviewMatrix = false;

	displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear message in the upper center area
	if (s_frameRateElapsedTime_ms > 0)
	{
		QString message = QString("Framerate: %1 f/s").arg(static_cast<double>(s_frameRateCurrentFrame)*1.0e3/static_cast<double>(s_frameRateElapsedTime_ms),0,'f',3);
		displayNewMessage(message,ccGLWindow::LOWER_LEFT_MESSAGE,true);
		ccLog::Print(message);
	}
	else
	{
		ccLog::Error("An error occurred during framerate test!");
	}

	QApplication::processEvents();

	redraw();
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
	//point size label
	QString psi_label;
	//point size label rect.
	QRect psi_labelRect;

	//! Default margin
	static inline int margin() { return 16; }
	//! Default icon size
	static inline int iconSize() { return 16; }

	HotZone(ccGLWindow* win)
		: textHeight(0)
		, yTextBottomLineShift(0)
		, bbv_label("bubble-view mode")
		, psi_label("default point size")
	{
		//default color ("greenish")
		color[0] = 133;
		color[1] = 193;
		color[2] = 39;

		if (win)
			font = win->font();
		font.setPointSize(12);
		font.setBold(true);

		QFontMetrics metrics(font);
		bbv_labelRect = metrics.boundingRect(bbv_label);
		psi_labelRect = metrics.boundingRect(psi_label);

		textHeight = std::max(psi_labelRect.height(),bbv_labelRect.height()) * 3/4; // --> factor: to recenter the baseline a little
		yTextBottomLineShift = (iconSize()/2) + (textHeight/2);
	}
};
QSharedPointer<HotZone> s_hotZone(0);

void ccGLWindow::drawClickableItems(int xStart0, int& yStart)
{
	if (	!m_hotZoneActivated
		&&	!m_bubbleViewModeEnabled )
	{
		//nothing to do
		return;
	}

	//we init the necessary parameters the first time we need them
	if (!s_hotZone)
		s_hotZone = QSharedPointer<HotZone>(new HotZone(this));

	int halfW = m_glWidth/2;
	int halfH = m_glHeight/2;

	glPushAttrib(GL_COLOR_BUFFER_BIT);
	glEnable(GL_BLEND);

	//draw semi-transparent background
	{
		//total hot zone area size (without margin)
		int psi_totalWidth = 0;
		if (m_hotZoneActivated)
			psi_totalWidth = /*HotZone::margin() + */s_hotZone->psi_labelRect.width() + HotZone::margin() + HotZone::iconSize() + HotZone::margin() + HotZone::iconSize()/* + HotZone::margin()*/;
		int bbv_totalWidth = 0;
		if (m_bubbleViewModeEnabled)
			bbv_totalWidth = /*HotZone::margin() + */s_hotZone->bbv_labelRect.width() + HotZone::margin() + HotZone::iconSize()/* + HotZone::margin()*/;

		int totalWidth = std::max(psi_totalWidth, bbv_totalWidth);

		QPoint minAreaCorner(xStart0 + HotZone::margin(),              yStart + HotZone::margin() + std::min(0, s_hotZone->yTextBottomLineShift - s_hotZone->textHeight));
		QPoint maxAreaCorner(xStart0 + HotZone::margin() + totalWidth, yStart + HotZone::margin() + std::max(HotZone::iconSize(), s_hotZone->yTextBottomLineShift));
		if (m_hotZoneActivated && m_bubbleViewModeEnabled)
			maxAreaCorner.setY(maxAreaCorner.y() + HotZone::iconSize() + HotZone::margin());

		QRect areaRect(	minAreaCorner - QPoint(HotZone::margin(),HotZone::margin())/2,
						maxAreaCorner + QPoint(HotZone::margin(),HotZone::margin())/2 );

		//draw rectangle
		glColor4ub(ccColor::darkGrey[0], ccColor::darkGrey[1], ccColor::darkGrey[2], 210);
		glBegin(GL_QUADS);
		glVertex2i(-halfW+(areaRect.x()),					halfH-(areaRect.y())					);
		glVertex2i(-halfW+(areaRect.x()+areaRect.width()),	halfH-(areaRect.y())					);
		glVertex2i(-halfW+(areaRect.x()+areaRect.width()),	halfH-(areaRect.y()+areaRect.height())	);
		glVertex2i(-halfW+(areaRect.x()),					halfH-(areaRect.y()+areaRect.height())	);
		glEnd();
	}

	if (m_hotZoneActivated)
	{
		yStart += HotZone::margin();
		int xStart = xStart0 + HotZone::margin();
		
		//label
		glColor3ubv_safe(s_hotZone->color);
		renderText(xStart,yStart + s_hotZone->yTextBottomLineShift,s_hotZone->psi_label,s_hotZone->font);

		//icons
		xStart += s_hotZone->psi_labelRect.width() + HotZone::margin();

		//"minus" icon
		{
			static const QPixmap c_psi_minusPix(":/CC/images/ccMinus.png");
			ccGLUtils::DisplayTexture2DPosition(bindTexture(c_psi_minusPix),-halfW+xStart,halfH-(yStart+HotZone::iconSize()),HotZone::iconSize(),HotZone::iconSize());
			m_clickableItems.push_back(ClickableItem(ClickableItem::DECREASE_POINT_SIZE,QRect(xStart,yStart,HotZone::iconSize(),HotZone::iconSize())));
			xStart += HotZone::iconSize();
		}

		//separator
		{
			glColor3ubv(s_hotZone->color);
			glBegin(GL_POINTS);
			glVertex2i(-halfW+xStart+HotZone::margin()/2,halfH-(yStart+HotZone::iconSize()/2));
			glEnd();
			xStart += HotZone::margin();
		}

		//"plus" icon
		{
			static const QPixmap c_psi_plusPix(":/CC/images/ccPlus.png");
			ccGLUtils::DisplayTexture2DPosition(bindTexture(c_psi_plusPix),-halfW+xStart,halfH-(yStart+HotZone::iconSize()),HotZone::iconSize(),HotZone::iconSize());
			m_clickableItems.push_back(ClickableItem(ClickableItem::INCREASE_POINT_SIZE,QRect(xStart,yStart,HotZone::iconSize(),HotZone::iconSize())));
			xStart += HotZone::iconSize();
		}

		yStart += HotZone::iconSize();
	}

	if (m_bubbleViewModeEnabled)
	{
		yStart += HotZone::margin();
		int xStart = xStart0 + HotZone::margin();
		
		//label
		glColor3ubv_safe(s_hotZone->color);
		renderText(xStart,yStart + s_hotZone->yTextBottomLineShift,s_hotZone->bbv_label,s_hotZone->font);
		
		//icon
		xStart += s_hotZone->bbv_labelRect.width() + HotZone::margin();

		//"exit" icon
		{
			static const QPixmap c_bbv_exitPix(":/CC/images/ccExit.png");
			ccGLUtils::DisplayTexture2DPosition(bindTexture(c_bbv_exitPix),-halfW+xStart,halfH-(yStart+HotZone::iconSize()),HotZone::iconSize(),HotZone::iconSize());
			m_clickableItems.push_back(ClickableItem(ClickableItem::LEAVE_BUBBLE_VIEW_MODE,QRect(xStart,yStart,HotZone::iconSize(),HotZone::iconSize())));
			xStart += HotZone::iconSize();
		}

		yStart += HotZone::iconSize();
	}

	glDisable(GL_BLEND);
	glPopAttrib();
}

void ccGLWindow::paintGL()
{
	//context initialization
	CC_DRAW_CONTEXT context;
	getContext(context);

	bool doDraw3D = (!m_fbo || ((m_alwaysUseFBO && m_updateFBO) || m_activeGLFilter || m_captureMode.enabled));

	if (doDraw3D)
	{
		bool doDrawCross = (	!m_captureMode.enabled
							&&	!m_viewportParams.perspectiveView
							&&	!(m_fbo && m_activeGLFilter)
							&&	getDisplayParameters().displayCross );
		draw3D(context,doDrawCross,m_fbo);
		m_updateFBO = false;
	}

	/****************************************/
	/****  PASS: 2D/FOREGROUND/NO LIGHT  ****/
	/****************************************/
	context.flags = CC_DRAW_2D | CC_DRAW_FOREGROUND;
	if (m_interactionMode == TRANSFORM_ENTITY)		
		context.flags |= CC_VIRTUAL_TRANS_ENABLED;

	setStandardOrthoCenter();
	glDisable(GL_DEPTH_TEST);

	GLuint screenTex = 0;
	if (m_fbo)
	{
		if (m_activeGLFilter)
		{
			//we process GL filter
			GLuint depthTex = m_fbo->getDepthTexture();
			GLuint colorTex = m_fbo->getColorTexture(0);
			//minimal set of viewport parameters necessary for GL filters
			ccGlFilter::ViewportParameters parameters;
			parameters.perspectiveMode = m_viewportParams.perspectiveView;
			parameters.zFar = m_viewportParams.zFar;
			parameters.zNear = m_viewportParams.zNear;
			parameters.zoom = m_viewportParams.perspectiveView ? computePerspectiveZoom() : m_viewportParams.zoom; //TODO: doesn't work well with EDL in perspective mode!
			//apply shader
			m_activeGLFilter->shade(depthTex, colorTex, parameters); 

			ccGLUtils::CatchGLError("ccGLWindow::paintGL/glFilter shade");

			//if capture mode is ON: we only want to capture it, not to display it
			if (!m_captureMode.enabled)
				screenTex = m_activeGLFilter->getTexture();
		}
		else if (!m_captureMode.enabled)
		{
			//screenTex = m_fbo->getDepthTexture();
			screenTex = m_fbo->getColorTexture(0);
		}
	}

	//if any, we display texture fullscreen
	if (glIsTexture(screenTex))
	{
		ccGLUtils::DisplayTexture2D(screenTex,m_glWidth,m_glHeight);
		glClear(GL_DEPTH_BUFFER_BIT);
	}

	//we draw 2D entities
	if (m_globalDBRoot)
		m_globalDBRoot->draw(context);
	if (m_winDBRoot)
		m_winDBRoot->draw(context);

	//current displayed scalar field color ramp (if any)
	ccRenderingTools::DrawColorRamp(context);

	m_clickableItems.clear();

	/*** overlay entities ***/
	if (m_displayOverlayEntities)
	{
		//default overlay color
		const unsigned char* textCol = getDisplayParameters().textDefaultCol;

		if (!m_captureMode.enabled || m_captureMode.renderOverlayItems)
		{
			//scale: only in ortho mode
			if (!m_viewportParams.perspectiveView)
				drawScale(textCol);

			//trihedron
			drawTrihedron();
		}

		if (!m_captureMode.enabled)
		{
			int yStart = 0;

			//transparent border at the top of the screen
			if (m_activeGLFilter)
			{
				float w = static_cast<float>(m_glWidth)/2;
				float h = static_cast<float>(m_glHeight)/2;
				int borderHeight = getGlFilterBannerHeight();

				glPushAttrib(GL_COLOR_BUFFER_BIT);
				glEnable(GL_BLEND);

				glColor4f(1.0f,1.0f,0.0f,0.6f);
				glBegin(GL_QUADS);
				glVertex2f( w,h);
				glVertex2f(-w,h);
				glVertex2f(-w,h-static_cast<float>(borderHeight));
				glVertex2f( w,h-static_cast<float>(borderHeight));
				glEnd();

				glPopAttrib();

				glColor3ubv_safe(ccColor::black);
				renderText(	10,
							borderHeight-CC_GL_FILTER_BANNER_MARGIN-CC_GL_FILTER_BANNER_MARGIN/2,
							QString("[GL filter] ")+m_activeGLFilter->getDescription()
							/*,m_font*/ ); //we ignore the custom font size

				yStart += borderHeight;
			}

			//current messages (if valid)
			if (!m_messagesToDisplay.empty())
			{
				int currentTime_sec = ccTimer::Sec();
				//ccLog::Print(QString("[paintGL] Current time: %1 s.").arg(currentTime_sec));

				glColor3ubv_safe(textCol);

				int ll_currentHeight = m_glHeight-10; //lower left
				int uc_currentHeight = 10; //upper center

				std::list<MessageToDisplay>::iterator it = m_messagesToDisplay.begin();
				while (it != m_messagesToDisplay.end())
				{
					//no more valid? we delete the message
					if (it->messageValidity_sec < currentTime_sec)
					{
						it = m_messagesToDisplay.erase(it);
					}
					else
					{
						switch(it->position)
						{
						case LOWER_LEFT_MESSAGE:
							{
								renderText(10, ll_currentHeight, it->message, m_font);
								int messageHeight = QFontMetrics(m_font).height();
								ll_currentHeight -= (messageHeight*5)/4; //add a 25% margin
							}
							break;
						case UPPER_CENTER_MESSAGE:
							{
								QRect rect = QFontMetrics(m_font).boundingRect(it->message);
								//take the GL filter banner into account!
								int x = (m_glWidth-rect.width())/2;
								int y = uc_currentHeight+rect.height();
								if (m_activeGLFilter)
									y += getGlFilterBannerHeight();
								renderText(x, y, it->message,m_font);
								uc_currentHeight += (rect.height()*5)/4; //add a 25% margin
							}
							break;
						case SCREEN_CENTER_MESSAGE:
							{
								QFont newFont(m_font); //no need to take zoom into account!
								newFont.setPointSize(12);
								QRect rect = QFontMetrics(newFont).boundingRect(it->message);
								//only one message supported in the screen center (for the moment ;)
								renderText((m_glWidth-rect.width())/2, (m_glHeight-rect.height())/2, it->message,newFont);
							}
							break;
						}
						++it;
					}
				}
			}

			drawClickableItems(0,yStart);
		}
	}

	ccGLUtils::CatchGLError("ccGLWindow::paintGL");

	m_shouldBeRefreshed = false;

	//For frame rate test
	if (s_frameRateTestInProgress)
	{
		s_frameRateElapsedTime_ms = s_frameRateElapsedTimer.elapsed();
		if (++s_frameRateCurrentFrame > FRAMERATE_TEST_MIN_FRAMES && s_frameRateElapsedTime_ms > FRAMERATE_TEST_DURATION_MSEC)
			stopFrameRateTest();
		else
		{
			//rotate base view matrix
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadMatrixd(m_viewportParams.viewMat.data());
			glRotated(360.0/FRAMERATE_TEST_MIN_FRAMES,0.0,1.0,0.0);
			glGetDoublev(GL_MODELVIEW_MATRIX, m_viewportParams.viewMat.data());
			m_validModelviewMatrix = false;
			glPopMatrix();
		}
	}
}
void ccGLWindow::draw3D(CC_DRAW_CONTEXT& context, bool doDrawCross, ccFrameBufferObject* fbo/*=0*/)
{
	makeCurrent();

	//if a FBO is activated
	if (fbo)
	{
		fbo->start();
		ccGLUtils::CatchGLError("ccGLWindow::paintGL/FBO start");
	}

	setStandardOrthoCenter();
	glDisable(GL_DEPTH_TEST);

	glPointSize(m_viewportParams.defaultPointSize);
	glLineWidth(m_viewportParams.defaultLineWidth);

	//gradient color background
	if (getDisplayParameters().drawBackgroundGradient)
	{
		drawGradientBackground();
		//we clear background
		glClear(GL_DEPTH_BUFFER_BIT);
	}
	else
	{
		const unsigned char* bkgCol = getDisplayParameters().backgroundCol;
		glClearColor(	static_cast<float>(bkgCol[0]) / 255.0f,
						static_cast<float>(bkgCol[1]) / 255.0f,
						static_cast<float>(bkgCol[2]) / 255.0f,
						1.0f );

		//we clear background
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	/****************************************/
	/****  PASS: 2D/BACKGROUND/NO LIGHT  ****/
	/****************************************/
	/*context.flags = CC_DRAW_2D;
	if (m_interactionMode == TRANSFORM_ENTITY)		
		context.flags |= CC_VIRTUAL_TRANS_ENABLED;

	//we draw 2D entities
	if (m_globalDBRoot)
		m_globalDBRoot->draw(context);
	if (m_winDBRoot)
		m_winDBRoot->draw(context);
	//*/

	/****************************************/
	/****  PASS: 3D/BACKGROUND/NO LIGHT  ****/
	/****************************************/
	context.flags = CC_DRAW_3D | CC_DRAW_FOREGROUND;
	if (m_interactionMode == TRANSFORM_ENTITY)		
		context.flags |= CC_VIRTUAL_TRANS_ENABLED;

	glEnable(GL_DEPTH_TEST);

	if (doDrawCross)
		drawCross();

	/****************************************/
	/****    PASS: 3D/FOREGROUND/LIGHT   ****/
	/****************************************/
	if (m_customLightEnabled || m_sunLightEnabled)
		context.flags |= CC_LIGHT_ENABLED;
	if (m_lodActivated)
		context.flags |= CC_LOD_ACTIVATED;

	//we enable absolute sun light (if activated)
	if (m_sunLightEnabled)
		glEnableSunLight();

	//we setup projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(getProjectionMatd());

	//then, the modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(getModelViewMatd());

	//we enable relative custom light (if activated)
	if (m_customLightEnabled)
	{
		glEnableCustomLight();
		if (!m_captureMode.enabled/* && !m_viewportParams.perspectiveView*/)
			//we display it as a litle 3D star
			drawCustomLight();
	}

	//we activate the current shader (if any)
	if (m_activeShader)
		m_activeShader->start();
	
	//color ramp shader for fast dynamic color ramp lookup-up
	if (m_colorRampShader && getDisplayParameters().colorScaleUseShader)
		context.colorRampShader = m_colorRampShader;

	//custom rendering shader (OpenGL 3.3+)
	context.customRenderingShader = m_customRenderingShader;

	//we draw 3D entities
	if (m_globalDBRoot)
	{
		m_globalDBRoot->draw(context);
		if (m_globalDBRoot->getChildrenNumber())
		{
			//draw pivot
			drawPivot();
		}
	}
	if (m_winDBRoot)
		m_winDBRoot->draw(context);

	//for connected items
	emit drawing3D();

	//we disable shader (if any)
	if (m_activeShader)
		m_activeShader->stop();

	//we disable lights
	if (m_sunLightEnabled)
		glDisableSunLight();
	if (m_customLightEnabled)
		glDisableCustomLight();

	//we disable fbo
	if (fbo)
	{
		fbo->stop();
		ccGLUtils::CatchGLError("ccGLWindow::paintGL/FBO stop");
	}
}

void ccGLWindow::dragEnterEvent(QDragEnterEvent *event)
{
	const QMimeData* mimeData = event->mimeData();

	/*//Display all MIME info
	for (unsigned i=0; i<mimeData->formats().size(); ++i)
	{
	QString format = mimeData->formats().at(i);
	ccLog::Print(QString("Drop format: %1").arg(format));
	if (mimeData->hasFormat("FileNameW"))
	{
	QByteArray byteData = mimeData->data(format);
	ccLog::Print(QString("\tdata: %1").arg(QString::fromUtf16((ushort*)byteData.data(), byteData.size() / 2)));
	}
	else
	{
	ccLog::Print(QString("\tdata: %1").arg(QString(mimeData->data(format))));
	}
	}
	//*/

	if (mimeData->hasFormat("text/uri-list"))
		event->acceptProposedAction();
}

void ccGLWindow::dropEvent(QDropEvent *event)
{
	const QMimeData* mimeData = event->mimeData();

	if (mimeData->hasFormat("text/uri-list"))
	{
		QByteArray data = mimeData->data("text/uri-list");
		QStringList fileNames = QUrl::fromPercentEncoding(data).split(QRegExp("\\n+"),QString::SkipEmptyParts);

		for (int i=0; i<fileNames.size(); ++i)
		{
			fileNames[i] = fileNames[i].trimmed();
#if defined(CC_WINDOWS)
			fileNames[i].remove("file:///");
#else
			fileNames[i].remove("file://");
#endif
			//fileNames[i] = QUrl(fileNames[i].trimmed()).toLocalFile(); //toLocalFile removes the end of filenames sometimes!
#ifdef _DEBUG
			ccLog::Print(QString("File dropped: %1").arg(fileNames[i]));
#endif
		}

		if (!fileNames.empty())
			emit filesDropped(fileNames);

		setFocus();

		event->acceptProposedAction();
	}

	/*QString filename("none");
	if (event->mimeData()->hasFormat("FileNameW"))
	{
	QByteArray data = event->mimeData()->data("FileNameW");
	filename = QString::fromUtf16((ushort*)data.data(), data.size() / 2);
	event->acceptProposedAction();
	}
	else if (event->mimeData()->hasFormat("FileName"))
	{
	filename = event->mimeData()->data("FileNameW");
	event->acceptProposedAction();
	}

	ccLog::Print(QString("Drop file(s): %1").arg(filename));
	//*/

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

void ccGLWindow::closeEvent(QCloseEvent *event)
{
	if (m_unclosable)
		event->ignore();
	else
		event->accept();
}

void ccGLWindow::setUnclosable(bool state)
{
	m_unclosable = state;
}

ccHObject* ccGLWindow::getOwnDB()
{
	return m_winDBRoot;
}

void ccGLWindow::addToOwnDB(ccHObject* obj2D, bool noDependency/*=true*/)
{
	assert(obj2D);

	if (m_winDBRoot)
	{
		m_winDBRoot->addChild(obj2D,noDependency ? ccHObject::DP_NONE : ccHObject::DP_PARENT_OF_OTHER);
		obj2D->setDisplay(this);
	}
	else
	{
		ccLog::Error("[ccGLWindow::addToOwnDB] Window has no DB!");
	}
}

void ccGLWindow::removeFromOwnDB(ccHObject* obj2D)
{
	if (m_winDBRoot)
		m_winDBRoot->removeChild(obj2D);
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

	//the user has provided a valid bouding box
	if (aBox)
	{
		zoomedBox = (*aBox);
	}
	else if (m_globalDBRoot) //otherwise we'll take the default one (if possible)
	{
		zoomedBox = m_globalDBRoot->getBB(true, true, this);
	}

	if (!zoomedBox.isValid())
		return;

	//we get the bounding-box diagonal length
	double bbDiag = zoomedBox.getDiagNorm();

	if (bbDiag < ZERO_TOLERANCE)
	{
		ccLog::Warning("[ccGLWindow] Entity/DB has a null bounding-box! Can't zoom in..."); 
		return;
	}

	//we compute the pixel size (in world coordinates)
	{
		int minScreenSize = std::min(m_glWidth,m_glHeight);
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
		assert(currentFov_deg > ZERO_TOLERANCE);
		double d = bbDiag / tan(currentFov_deg * CC_DEG_TO_RAD);

		CCVector3d cameraDir(0,0,-1);
		if (!m_viewportParams.objectCenteredView)
			cameraDir = getCurrentViewDir();

		cameraPos -= cameraDir * d;
	}
	setCameraPos(cameraPos);

	invalidateViewport();
	invalidateVisualization();

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

	if (m_activeShader)
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
			if (!initFBO(m_glWidth,m_glHeight))
			{
				redraw();
				return;
			}
		}

		m_activeGLFilter = filter;
		initGLFilter(m_glWidth,m_glHeight);
	}

	if (!m_activeGLFilter && m_fbo && !m_alwaysUseFBO)
		removeFBO();

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
	}
}

void ccGLWindow::setCameraPos(const CCVector3d& P)
{
	m_viewportParams.cameraCenter = P;
	emit cameraPosChanged(m_viewportParams.cameraCenter);

	invalidateViewport();
	invalidateVisualization();
}

void ccGLWindow::moveCamera(float dx, float dy, float dz)
{
	if (dx != 0 || dy != 0) //camera movement? (dz doesn't count as ot only corresponds to a zoom)
	{
		//feedback for echo mode
		emit cameraDisplaced(dx,dy);
	}

	//current X, Y and Z viewing directions
	//correspond to the 'model view' matrix
	//lines.
	CCVector3d V(dx,dy,dz);
	if (!m_viewportParams.objectCenteredView)
		m_viewportParams.viewMat.transposed().applyRotation(V);

	setCameraPos(m_viewportParams.cameraCenter + V);
}

void ccGLWindow::setPivotPoint(const CCVector3d& P)
{
	m_viewportParams.pivotPoint = P;
	emit pivotPointChanged(m_viewportParams.pivotPoint);

	invalidateViewport();
	invalidateVisualization();
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

void ccGLWindow::drawGradientBackground()
{
	int w = m_glWidth/2+1;
	int h = m_glHeight/2+1;

	const unsigned char* bkgCol = getDisplayParameters().backgroundCol;
	const unsigned char* forCol = getDisplayParameters().textDefaultCol;

	//Gradient "texture" drawing
	glBegin(GL_QUADS);
	//we the user-defined background color for gradient start
	glColor3ubv(bkgCol);
	glVertex2i(-w,h);
	glVertex2i(w,h);
	//and the inverse of points color for gradient end
	glColor3ub(255-forCol[0],255-forCol[1],255-forCol[2]);
	glVertex2i(w,-h);
	glVertex2i(-w,-h);
	glEnd();
}

void ccGLWindow::drawCross()
{
	//cross OpenGL drawing
	glColor3ubv(ccColor::lightGrey);
	glBegin(GL_LINES);
	glVertex3f(0.0,-CC_DISPLAYED_CENTER_CROSS_LENGTH,0.0);
	glVertex3f(0.0,CC_DISPLAYED_CENTER_CROSS_LENGTH,0.0);
	glVertex3f(-CC_DISPLAYED_CENTER_CROSS_LENGTH,0.0,0.0);
	glVertex3f(CC_DISPLAYED_CENTER_CROSS_LENGTH,0.0,0.0);
	glEnd();
}

QFont ccGLWindow::getTextDisplayFont() const
{
	if (!m_captureMode.enabled || m_captureMode.zoomFactor == 1.0f)
		return m_font;

	QFont font = m_font;
	font.setPointSize(static_cast<int>(m_font.pointSize() * m_captureMode.zoomFactor));
	return font;
}

void ccGLWindow::drawScale(const colorType color[] /*= white*/)
{
	assert(!m_viewportParams.perspectiveView); //a scale is only valid in ortho. mode!

	float scaleMaxW = static_cast<float>(m_glWidth) / 4; //25% of screen width
	if (m_viewportParams.zoom < CC_GL_MIN_ZOOM_RATIO)
	{
		assert(false);
		return;
	}

	//we first compute the width equivalent to 25% of horizontal screen width
	//(this is why it's only valid in orthographic mode !)
	float equivalentWidth = scaleMaxW * m_viewportParams.pixelSize / m_viewportParams.zoom;

	//we then compute the scale granularity (to avoid width values with a lot of decimals)
	int k = int(floor(log(static_cast<float>(equivalentWidth))/log(10.0f)));
	float granularity = pow(10.0f,static_cast<float>(k))/2;

	//we choose the value closest to equivalentWidth with the right granularity
	equivalentWidth = floor(std::max(equivalentWidth/granularity,1.0f))*granularity;

	QFont font = getTextDisplayFont(); //we take rendering zoom into account!
	QFontMetrics fm(font);

	//we deduce the scale drawing width
	float scaleW_pix = equivalentWidth / m_viewportParams.pixelSize * m_viewportParams.zoom;
	float trihedronLength = CC_DISPLAYED_TRIHEDRON_AXES_LENGTH * m_captureMode.zoomFactor;
	float dW = 2.0f * trihedronLength + 20.0f;
	float dH = std::max<float>(static_cast<float>(fm.height()) * 1.25f,trihedronLength + 5.0f);
	float w = static_cast<float>(m_glWidth) * 0.5f - dW;
	float h = static_cast<float>(m_glHeight) * 0.5f - dH;
	float tick = 3.0f * m_captureMode.zoomFactor;

	//scale OpenGL drawing
	glColor3ubv(color);
	glBegin(GL_LINES);
	glVertex3f(w-scaleW_pix,-h,0.0);
	glVertex3f(w,-h,0.0);
	glVertex3f(w-scaleW_pix,-h-tick,0.0);
	glVertex3f(w-scaleW_pix,-h+tick,0.0);
	glVertex3f(w,-h+tick,0.0);
	glVertex3f(w,-h-tick,0.0);
	glEnd();

	QString text = QString::number(m_captureMode.enabled ? equivalentWidth/m_captureMode.zoomFactor : equivalentWidth);
	glColor3ubv_safe(color);
	renderText(m_glWidth-static_cast<int>(scaleW_pix/2+dW)-fm.width(text)/2, m_glHeight-static_cast<int>(dH/2)+fm.height()/3, text, font);
}

void ccGLWindow::drawTrihedron()
{
	float trihedronLength = CC_DISPLAYED_TRIHEDRON_AXES_LENGTH * m_captureMode.zoomFactor;

	float w = static_cast<float>(m_glWidth)*0.5f-trihedronLength-10.0f;
	float h = static_cast<float>(m_glHeight)*0.5f-trihedronLength-5.0f;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(w, -h, 0);
	glMultMatrixd(m_viewportParams.viewMat.data());

	if (m_trihedronGLList == GL_INVALID_LIST_ID)
	{
		m_trihedronGLList = glGenLists(1);
		glNewList(m_trihedronGLList, GL_COMPILE);

		glPushAttrib(GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_DEPTH_TEST);

		//trihedron OpenGL drawing
		glBegin(GL_LINES);
		glColor3f(1.0f,0.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f);
		glVertex3f(CC_DISPLAYED_TRIHEDRON_AXES_LENGTH,0.0f,0.0f);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f);
		glVertex3f(0.0f,CC_DISPLAYED_TRIHEDRON_AXES_LENGTH,0.0f);
		glColor3f(0.0f,0.7f,1.0f);
		glVertex3f(0.0f,0.0f,0.0f);
		glVertex3f(0.0f,0.0f,CC_DISPLAYED_TRIHEDRON_AXES_LENGTH);
		glEnd();

		glPopAttrib();

		glEndList();
	}
	else if (m_captureMode.enabled)
	{
		glScalef(m_captureMode.zoomFactor,m_captureMode.zoomFactor,m_captureMode.zoomFactor);
	}

	glCallList(m_trihedronGLList);

	glPopMatrix();
}

CCVector3d ccGLWindow::getRealCameraCenter() const
{
	if (m_viewportParams.perspectiveView)
		return m_viewportParams.cameraCenter;

	ccBBox box = getVisibleObjectsBB();

	return CCVector3d(	m_viewportParams.cameraCenter.x,
						m_viewportParams.cameraCenter.y,
						box.isValid() ? box.getCenter().z : 0 );
}

ccBBox ccGLWindow::getVisibleObjectsBB() const
{
	ccBBox box;

	//compute center of visible objects constellation
	if (m_globalDBRoot)
	{
		//get whole bounding-box
		box = m_globalDBRoot->getBB(true, true, this);
		if (box.isValid())
		{
			//incorporate window own db
			if (m_winDBRoot)
			{
				ccBBox ownBox = m_winDBRoot->getBB(true, true, this);
				if (ownBox.isValid())
				{
					box.add(ownBox.minCorner());
					box.add(ownBox.maxCorner());
				}
			}
		}
	}

	return box;
}

void ccGLWindow::invalidateViewport()
{
	m_validProjectionMatrix = false;
	m_updateFBO = true;
}

void ccGLWindow::recalcProjectionMatrix()
{
	makeCurrent();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	double bbHalfDiag = 1.0;
	CCVector3d bbCenter(0,0,0);

	//compute center of visible objects constellation
	if (m_globalDBRoot)
	{
		//get whole bounding-box
		ccBBox box = getVisibleObjectsBB();
		if (box.isValid())
		{
			//get bbox center
			bbCenter = CCVector3d::fromArray(box.getCenter().u);
			//get half bbox diagonal length
			bbHalfDiag = static_cast<double>(box.getDiagNorm()) / 2;
		}
	}

	//virtual pivot point (i.e. to handle viewer-based mode smoothly)
	CCVector3d pivotPoint = (m_viewportParams.objectCenteredView ? m_viewportParams.pivotPoint : bbCenter);

	//distance between camera and pivot point
	//warning: it's important to get the 'real' center (i.e. with z=bbCenter.z in ortho. view)
	//otherwise we (sometimes largely) overestimate the distance between the camera center
	//and the displayed objects if the camera has been shifted in the Z direction (e.g. after
	//switching from perspective to ortho. view).
	//While the user won't see the difference this has a great influence on GL filters
	//(as normalized depth map values depends on it)
	double CP = (getRealCameraCenter()-pivotPoint).norm();
		
	//distance between pivot point and DB farthest point
	double MP = (bbCenter - pivotPoint).norm() + bbHalfDiag;

	//pivot symbol should always be (potentially) visible in object-based mode
	if (m_pivotSymbolShown && m_viewportParams.objectCenteredView && m_pivotVisibility != PIVOT_HIDE)
	//if (m_viewportParams.objectCenteredView)
	{
		double pivotActualRadius = CC_DISPLAYED_PIVOT_RADIUS_PERCENT * static_cast<double>(std::min(m_glWidth,m_glHeight)) / 2;
		double pivotSymbolScale = pivotActualRadius * computeActualPixelSize();
		MP = std::max<double>(MP,pivotSymbolScale);
	}
	MP *= 1.01; //for round-off issues
	
	if (m_customLightEnabled)
	{
		//distance from custom light to pivot point
		double d = (pivotPoint - CCVector3d::fromArray(m_customLightPos)).norm();
		MP = std::max<double>(MP,d);
	}

	if (m_viewportParams.perspectiveView)
	{
		//we deduce zNear et zFar
		//DGM: the 'zNearCoef' must not be too small, otherwise the loss in accuracy
		//for the detph buffer is too high and the display is jeopardized, especially
		//for entities with big coordinates)
		double zNear = MP * m_viewportParams.zNearCoef;
		//DGM: what was the purpose of this?!
		//if (m_viewportParams.objectCenteredView)
		//	zNear = std::max<double>(CP-MP,zNear);
		double zFar = std::max<double>(CP+MP,1.0);

		//save actual zNear and zFar parameters
		m_viewportParams.zNear = zNear;
		m_viewportParams.zFar = zFar;

		//and aspect ratio
		double ar = static_cast<double>(m_glWidth)/m_glHeight;

		float currentFov_deg = getFov();
		gluPerspective(currentFov_deg,ar,zNear,zFar);
	}
	else
	{
		//max distance (camera to 'farthest' point)
		double maxDist = CP + MP;

		double maxDist_pix = maxDist / m_viewportParams.pixelSize * m_viewportParams.zoom;
		maxDist_pix = std::max<double>(maxDist_pix,1.0);

		double halfW = static_cast<double>(m_glWidth)/2;
		double halfH = static_cast<double>(m_glHeight)/2 * m_viewportParams.orthoAspectRatio;

		//save actual zNear and zFar parameters
		m_viewportParams.zNear = -maxDist_pix;
		m_viewportParams.zFar = maxDist_pix;

		glOrtho(-halfW,halfW,-halfH,halfH,-maxDist_pix,maxDist_pix);
	}

	//we save projection matrix
	glGetDoublev(GL_PROJECTION_MATRIX, m_projMatd);

	m_validProjectionMatrix = true;
}

void ccGLWindow::invalidateVisualization()
{
	m_validModelviewMatrix = false;
	m_updateFBO = true;
}

void ccGLWindow::recalcModelViewMatrix()
{
	makeCurrent();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (m_viewportParams.perspectiveView) //perspective mode
	{
		//for proper aspect ratio handling
		float ar = (m_glHeight != 0 ? float(m_glWidth)/(m_glHeight*m_viewportParams.perspectiveAspectRatio) : 0.0f);
		if (ar < 1.0)
			glScalef(ar,ar,1.0);
	}
	else //ortho. mode
	{
		//apply zoom
		float totalZoom = m_viewportParams.zoom / m_viewportParams.pixelSize;
		glScalef(totalZoom,totalZoom,totalZoom);
	}

	CCVector3d cameraCenter = getRealCameraCenter();

	//apply current camera parameters (see trunk/doc/rendering_pipeline.doc)
	if (m_viewportParams.objectCenteredView)
	{
		//place origin on camera center
		glTranslated(-cameraCenter.x, -cameraCenter.y, -cameraCenter.z);

		//go back to initial origin
		glTranslated(m_viewportParams.pivotPoint.x, m_viewportParams.pivotPoint.y, m_viewportParams.pivotPoint.z);

		//rotation (viewMat is simply a rotation matrix around the pivot here!)
		glMultMatrixd(m_viewportParams.viewMat.data());

		//place origin on pivot point
		glTranslated(-m_viewportParams.pivotPoint.x, -m_viewportParams.pivotPoint.y, -m_viewportParams.pivotPoint.z);
	}
	else
	{
		//rotation (viewMat is the rotation around the camera center here - no pivot)
		glMultMatrixd(m_viewportParams.viewMat.data());

		//place origin on camera center
		glTranslated(-cameraCenter.x, -cameraCenter.y, -cameraCenter.z);
	}		

	//we save visualization matrix
	glGetDoublev(GL_MODELVIEW_MATRIX, m_viewMatd);

	m_validModelviewMatrix = true;
}

const ccGLMatrixd& ccGLWindow::getBaseViewMat()
{
	return m_viewportParams.viewMat;
}

const void ccGLWindow::setBaseViewMat(ccGLMatrixd& mat)
{
	m_viewportParams.viewMat = mat;

	invalidateVisualization();

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_viewportParams.viewMat);
}

const double* ccGLWindow::getModelViewMatd()
{
	if (!m_validModelviewMatrix)
		recalcModelViewMatrix();

	return m_viewMatd;
}

const double* ccGLWindow::getProjectionMatd()
{
	if (!m_validProjectionMatrix)
		recalcProjectionMatrix();

	return m_projMatd;
}

void ccGLWindow::setStandardOrthoCenter()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float halfW = static_cast<float>(m_glWidth)/2;
	float halfH = static_cast<float>(m_glHeight)/2;
	float maxS = std::max(halfW,halfH);
	glOrtho(-halfW,halfW,-halfH,halfH,-maxS,maxS);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void ccGLWindow::setStandardOrthoCorner()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,0,static_cast<double>(m_glWidth),static_cast<double>(m_glHeight),0,1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void ccGLWindow::getContext(CC_DRAW_CONTEXT& context)
{
	//display size
	context.glW = m_glWidth;
	context.glH = m_glHeight;
	context._win = this;
	context.flags = 0;

	const ccGui::ParamStruct& guiParams = getDisplayParameters();

	//decimation options
	context.decimateCloudOnMove = guiParams.decimateCloudOnMove;
	context.decimateMeshOnMove = guiParams.decimateMeshOnMove;

	//scalar field color-bar
	context.sfColorScaleToDisplay = 0;

	//point picking
	double pixSize = computeActualPixelSize();
	context.pickedPointsRadius = static_cast<float>(guiParams.pickedPointsSize * pixSize);
	context.pickedPointsTextShift = static_cast<float>(5 * pixSize); //5 pixels shift

	//text display
	context.dispNumberPrecision = guiParams.displayedNumPrecision;
	context.labelsTransparency = guiParams.labelsTransparency;

	//default materials
	context.defaultMat.name = "default";
	memcpy(context.defaultMat.diffuseFront,guiParams.meshFrontDiff,sizeof(float)*4);
	memcpy(context.defaultMat.diffuseBack,guiParams.meshBackDiff,sizeof(float)*4);
	memcpy(context.defaultMat.ambient,ccColor::bright,sizeof(float)*4);
	memcpy(context.defaultMat.specular,guiParams.meshSpecular,sizeof(float)*4);
	memcpy(context.defaultMat.emission,ccColor::night,sizeof(float)*4);
	context.defaultMat.shininessFront = 30;
	context.defaultMat.shininessBack = 50;
	//default colors
	memcpy(context.pointsDefaultCol,guiParams.pointsDefaultCol,sizeof(unsigned char)*3);
	memcpy(context.textDefaultCol,guiParams.textDefaultCol,sizeof(unsigned char)*3);
	memcpy(context.labelDefaultCol,guiParams.labelCol,sizeof(unsigned char)*3);
	memcpy(context.bbDefaultCol,guiParams.bbDefaultCol,sizeof(unsigned char)*3);

	//default font size
	setFontPointSize(guiParams.defaultFontSize);

	//display acceleration
	context.useVBOs = guiParams.useVBOs;
}

void ccGLWindow::toBeRefreshed()
{
	m_shouldBeRefreshed = true;

	invalidateViewport();
}

void ccGLWindow::refresh()
{
	if (m_shouldBeRefreshed && isVisible())
		redraw();
}

void ccGLWindow::redraw()
{
	m_updateFBO = true;
	updateGL();
}

unsigned ccGLWindow::getTexture(const QImage& image)
{
	makeCurrent();

	//default parameters
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//check that image size is not too big!
	GLint maxTexSize = 0;
	glGetIntegerv(GL_MAX_TEXTURE_SIZE,&maxTexSize);
	int cacheLimitBytes = context()->textureCacheLimit() * 1024; //result in bytes

	if (image.width() <= maxTexSize && image.height() <= maxTexSize && cacheLimitBytes >= image.width()*image.height()*4)
	{
		return bindTexture(image,GL_TEXTURE_2D,GL_RGBA,QGLContext::NoBindOption);
	}
	else
	{
		maxTexSize = std::min(maxTexSize, static_cast<int>(sqrt(static_cast<double>(cacheLimitBytes>>2)))); // ">>2"="/4" because we assume all textures have 4 components
		int width = image.width();
		int height = image.height();
		if (width > height)
		{
			width = maxTexSize;
			height = (width*image.height()) / image.width();
		}
		else
		{
			height = maxTexSize;
			width = (height*image.width()) / image.height();
		}
		ccLog::Warning("[OpenGL] Image is too big to fit in texture cache! Reducing it to (%i x %i) for display!",width,height);

		QImage qImage = image.scaled(width,height,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
		if (qImage.isNull())
		{
			ccLog::Warning("[OpenGL] Not enough memory to perform operation!");
			return GL_INVALID_TEXTURE_ID;
		}

		return bindTexture(qImage,GL_TEXTURE_2D,GL_RGBA,QGLContext::NoBindOption);
	}
}

void ccGLWindow::releaseTexture(unsigned texID)
{
	makeCurrent();
	deleteTexture(texID);
}

CCVector3d ccGLWindow::getCurrentViewDir() const
{
	//view direction is (the opposite of) the 3rd line of the current view matrix
	const double* M = m_viewportParams.viewMat.data();
	CCVector3d axis(-M[2],-M[6],-M[10]);
	axis.normalize();

	return axis;
}

CCVector3d ccGLWindow::getCurrentUpDir() const
{
	//if (m_viewportParams.objectCenteredView)
	//	return CCVector3d(0,1,0);

	//otherwise up direction is the 2nd line of the current view matrix
	const double* M = m_viewportParams.viewMat.data();
	CCVector3d axis(M[1],M[5],M[9]);
	axis.normalize();

	return axis;
}

void ccGLWindow::setInteractionMode(INTERACTION_MODE mode)
{
	m_interactionMode = mode;
}

void ccGLWindow::setPickingMode(PICKING_MODE mode/*=DEFAULT_PICKING*/)
{
	//is the picking mode locked?
	if (m_pickingModeLocked)
	{
		if (mode != m_pickingMode && mode != DEFAULT_PICKING)
			ccLog::Warning("[ccGLWindow::setPickingMode] Picking mode is locked! Can't change it...");
		return;
	}

	switch(mode)
	{
	case DEFAULT_PICKING:
		mode = ENTITY_PICKING;
	case NO_PICKING:
	case ENTITY_PICKING:
		setCursor(QCursor(Qt::ArrowCursor));
		break;
	case TRIANGLE_PICKING:
	case POINT_PICKING:
		setCursor(QCursor(Qt::PointingHandCursor));
		break;
	default:
		break;
	}

	m_pickingMode = mode;
}

void ccGLWindow::enableEmbeddedIcons(bool state)
{
	m_embeddedIconsEnabled = state;
	m_hotZoneActivated = false;
	setMouseTracking(state);
}

CCVector3d ccGLWindow::convertMousePositionToOrientation(int x, int y)
{
	double xc = static_cast<double>(width()/2);
	double yc = static_cast<double>(height()/2);

	GLdouble xp,yp;
	if (m_viewportParams.objectCenteredView)
	{
		//project the current pivot point on screen
		int VP[4];
		getViewportArray(VP);
		GLdouble zp;
		gluProject(m_viewportParams.pivotPoint.x,m_viewportParams.pivotPoint.y,m_viewportParams.pivotPoint.z,getModelViewMatd(),getProjectionMatd(),VP,&xp,&yp,&zp);

		//we set the virtual rotation pivot closer to the actual one (but we always stay in the central part of the screen!)
		xp = std::min<GLdouble>(xp,3*width()/4);
		xp = std::max<GLdouble>(xp,  width()/4);

		yp = std::min<GLdouble>(yp,3*height()/4);
		yp = std::max<GLdouble>(yp,  height()/4);
	}
	else
	{
		xp = static_cast<GLdouble>(xc);
		yp = static_cast<GLdouble>(yc);
	}

	//invert y
	y = height()-1 - y;

	CCVector3d v(x - xp, y - yp, 0);

	v.x = std::max<double>(std::min<double>(v.x/xc,1),-1);
	v.y = std::max<double>(std::min<double>(v.y/yc,1),-1);

	if (m_verticalRotationLocked || m_bubbleViewModeEnabled)
	{
		v.y = 0;
	}

	//square 'radius'
	double d2 = v.x*v.x + v.y*v.y;

	//projection on the unit sphere
	if (d2 > 1)
	{
		double d = sqrt(d2);
		v.x /= d;
		v.y /= d;
	}
	else
	{
		v.z = sqrt(1.0 - d2);
	}

	return v;
}

void ccGLWindow::updateActiveItemsList(int x, int y, bool extendToSelectedLabels/*=false*/)
{
	m_activeItems.clear();

	if (!m_globalDBRoot && !m_winDBRoot)
		return;

	if (m_interactionMode == TRANSFORM_ENTITY) //labels are ignored in 'Interactive Transformation' mode
		return;

	int subID=-1;
	int itemID = startPicking(FAST_PICKING,x,y,2,2,&subID);
	if (itemID < 1)
		return;

	//items can be in local or global DB
	ccHObject* pickedObj = m_globalDBRoot->find(itemID);
	if (!pickedObj && m_winDBRoot)
		pickedObj = m_winDBRoot->find(itemID);
	if (pickedObj)
	{
		if (pickedObj->isA(CC_TYPES::LABEL_2D))
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(pickedObj);
			if (!label->isSelected() || !extendToSelectedLabels)
			{
				//select it?
				//emit entitySelectionChanged(label->getUniqueID());
				//QApplication::processEvents();
				m_activeItems.push_back(label);
				return;
			}
			else
			{
				//we get the other selected labels as well!
				ccHObject::Container labels;
				if (m_globalDBRoot)
					m_globalDBRoot->filterChildren(labels,true,CC_TYPES::LABEL_2D);
				if (m_winDBRoot)
					m_winDBRoot->filterChildren(labels,true,CC_TYPES::LABEL_2D);

				for (ccHObject::Container::iterator it=labels.begin(); it!=labels.end(); ++it)
				{
					if ((*it)->isA(CC_TYPES::LABEL_2D) && (*it)->isVisible()) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
					{
						cc2DLabel* label = static_cast<cc2DLabel*>(*it);
						if (label->isSelected())
						{
							m_activeItems.push_back(label);
						}
					}
				}
			}
		}
		else if (pickedObj->isA(CC_TYPES::CLIPPING_BOX))
		{
			ccClipBox* cbox = static_cast<ccClipBox*>(pickedObj);
			cbox->setActiveComponent(subID);
			cbox->setClickedPoint(x,y,width(),height(),m_viewportParams.viewMat);

			m_activeItems.push_back(cbox);
		}
	}
}

void ccGLWindow::mousePressEvent(QMouseEvent *event)
{
	m_cursorMoved = false;

	if ((event->buttons() & Qt::RightButton)
#ifdef CC_MAC_OS
		|| (QApplication::keyboardModifiers () & Qt::MetaModifier)
#endif
		)
	{
		if (m_interactionMode != SEGMENT_ENTITY) //mouse movement = panning (2D translation)
		{
			m_lastMousePos = event->pos();
			m_lodActivated = true;

			QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
		}

		emit rightButtonClicked(event->x()-width()/2,height()/2-event->y());
	}
	else if (event->buttons() & Qt::LeftButton)
	{
		if (m_interactionMode != SEGMENT_ENTITY) //mouse movement = rotation
		{
			m_lastClickTime_ticks = ccTimer::Msec();

			m_lastMouseOrientation = convertMousePositionToOrientation(event->x(), event->y());
			m_lastMousePos = event->pos();
			m_lodActivated = true;

			QApplication::setOverrideCursor(QCursor(Qt::PointingHandCursor));

			//let's check if the mouse is on a selected item first!
			if (	QApplication::keyboardModifiers () == Qt::NoModifier
				||	QApplication::keyboardModifiers () == Qt::ControlModifier )
			{
				updateActiveItemsList(event->x(), event->y(), true);
			}
		}

		emit leftButtonClicked(event->x()-width()/2,height()/2-event->y());
	}
	else
	{
		event->ignore();
	}
}

void ccGLWindow::mouseMoveEvent(QMouseEvent *event)
{
	if (m_interactionMode == SEGMENT_ENTITY)
	{
		if (event->buttons() != Qt::NoButton || m_alwaysUseFBO) //fast!
			emit mouseMoved(event->x()-width()/2,height()/2-event->y(),event->buttons());
		return;
	}

	const int x = event->x();
	const int y = event->y();

	//no button pressed
	if (event->buttons() == Qt::NoButton)
	{
		if (m_embeddedIconsEnabled)
		{
			bool inZone = (x < CC_HOT_ZONE_TRIGGER_WIDTH && y < CC_HOT_ZONE_TRIGGER_HEIGHT);
			if (inZone != m_hotZoneActivated)
			{
				m_hotZoneActivated = inZone;
				updateGL();
			}
			event->accept();
		}
		else
		{
			event->ignore();
		}

		//don't need to process any further
		return;
	}

	int dx = x - m_lastMousePos.x();
	int dy = y - m_lastMousePos.y();

	if ((event->buttons() & Qt::RightButton)
#ifdef CC_MAC_OS
		|| (QApplication::keyboardModifiers () & Qt::MetaModifier)
#endif
		)
	{
		//displacement vector (in "3D")
		double pixSize = computeActualPixelSize();
		CCVector3d u(static_cast<double>(dx)*pixSize, -static_cast<double>(dy)*pixSize, 0);
		if (!m_viewportParams.perspectiveView)
			u.y *= m_viewportParams.orthoAspectRatio;

		bool entityMovingMode = (m_interactionMode == TRANSFORM_ENTITY) || ((QApplication::keyboardModifiers () & Qt::ControlModifier) && m_customLightEnabled);
		if (entityMovingMode)
		{
			//apply inverse view matrix
			m_viewportParams.viewMat.transposed().applyRotation(u);

			if (m_interactionMode == TRANSFORM_ENTITY)
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
			}
		}
		else //camera moving mode
		{
			if (m_viewportParams.objectCenteredView)
			{
				//inverse displacement in object-based mode
				u = -u;
			}			
			moveCamera(static_cast<float>(u.x),static_cast<float>(u.y),static_cast<float>(u.z));
		}
	}
	else if (event->buttons() & Qt::LeftButton) //rotation
	{
		//specific case: move active item(s)
		if (!m_activeItems.empty())
		{
			//displacement vector (in "3D")
			double pixSize = computeActualPixelSize();
			CCVector3d u(static_cast<double>(dx)*pixSize, -static_cast<double>(dy)*pixSize, 0);
			m_viewportParams.viewMat.transposed().applyRotation(u);

			for (std::list<ccInteractor*>::iterator it=m_activeItems.begin(); it!=m_activeItems.end(); ++it)
			{
				if ((*it)->move2D(x,y,dx,dy,width(),height()) || (*it)->move3D(u))
				{
					invalidateViewport();
					//m_updateFBO = true; //already done by invalidateViewport
				}
			}
		}
		else
		{
			//specific case: rectangular polyline drawing (for rectangular area selection mode)
			if (	m_allowRectangularEntityPicking
				&& (m_pickingMode == ENTITY_PICKING || m_pickingMode == ENTITY_RECT_PICKING)
				&& (m_rectPickingPoly || (QApplication::keyboardModifiers () & Qt::AltModifier)))
			{
				//first time: initialization of the rectangle
				if (!m_rectPickingPoly)
				{
					ccPointCloud* vertices = new ccPointCloud("rect.vertices");
					m_rectPickingPoly = new ccPolyline(vertices);
					m_rectPickingPoly->addChild(vertices);
					if (vertices->reserve(4) && m_rectPickingPoly->addPointIndex(0,4))
					{
						m_rectPickingPoly->setForeground(true);
						m_rectPickingPoly->setColor(ccColor::green);
						m_rectPickingPoly->showColors(true);
						m_rectPickingPoly->set2DMode(true);
						m_rectPickingPoly->setDisplay(this);
						m_rectPickingPoly->setVisible(true);
						CCVector3 A(static_cast<PointCoordinateType>(m_lastMousePos.x()-width()/2),
									static_cast<PointCoordinateType>(height()/2-m_lastMousePos.y()),
									0);
						//we add 4 times the same point (just to fill the cloud!)
						vertices->addPoint(A);
						vertices->addPoint(A);
						vertices->addPoint(A);
						vertices->addPoint(A);
						m_rectPickingPoly->setClosed(true);
						addToOwnDB(m_rectPickingPoly,false);
					}
					else
					{
						ccLog::Warning("[ccGLWindow] Failed to create seleciton polyline! Not enough memory!");
						delete m_rectPickingPoly;
						m_rectPickingPoly = 0;
						vertices = 0;
					}
				}

				if (m_rectPickingPoly)
				{
					CCLib::GenericIndexedCloudPersist* vertices = m_rectPickingPoly->getAssociatedCloud();
					assert(vertices);
					CCVector3* B = const_cast<CCVector3*>(vertices->getPointPersistentPtr(1));
					CCVector3* C = const_cast<CCVector3*>(vertices->getPointPersistentPtr(2));
					CCVector3* D = const_cast<CCVector3*>(vertices->getPointPersistentPtr(3));
					B->x = C->x = static_cast<PointCoordinateType>(event->x() - width()/2);
					C->y = D->y = static_cast<PointCoordinateType>(height()/2 - event->y());
				}
			}
			else if (m_interactionMode != PAN_ONLY) //standard rotation around the current pivot
			{
				m_currentMouseOrientation = convertMousePositionToOrientation(x, y);

				ccGLMatrixd rotMat = ccGLUtils::GenerateGLRotationMatrixFromVectors(m_lastMouseOrientation,m_currentMouseOrientation);
				m_lastMouseOrientation = m_currentMouseOrientation;
				m_updateFBO = true;

				if (m_interactionMode == TRANSFORM_ENTITY)
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

	m_cursorMoved = true;
	m_lastMousePos = event->pos();

	event->accept();

	if (m_interactionMode != TRANSFORM_ENTITY)
		updateGL();
}

bool ccGLWindow::processClickableItems(int x, int y)
{
	ClickableItem::Role clickedItem = ClickableItem::NO_ROLE;
	for (std::vector<ClickableItem>::const_iterator it = m_clickableItems.begin(); it != m_clickableItems.end(); ++it)
	{
		if (it->area.contains(x,y))
		{
			clickedItem = it->role;
			break;
		}
	}

	switch (clickedItem)
	{
	case ClickableItem::NO_ROLE:
		//nothing to do
		break;
	case ClickableItem::INCREASE_POINT_SIZE:
		if (m_viewportParams.defaultPointSize < MAX_POINT_SIZE)
		{
			setPointSize(m_viewportParams.defaultPointSize+1);
			updateGL();
		}
		return true;
	case ClickableItem::DECREASE_POINT_SIZE:
		if (m_viewportParams.defaultPointSize > MIN_POINT_SIZE)
		{
			setPointSize(m_viewportParams.defaultPointSize-1);
			updateGL();
		}
		return true;
	case ClickableItem::LEAVE_BUBBLE_VIEW_MODE:
		{
			setBubbleViewMode(false);
			updateGL();
		}
		return true;
	default:
		//unhandled item?!
		assert(false);
		break;
	}

	return false;
}

void ccGLWindow::mouseReleaseEvent(QMouseEvent *event)
{
	bool cursorHasMoved = m_cursorMoved;
	bool acceptEvent = false;

	//reset to default state
	m_cursorMoved = false;
	m_lodActivated = false;
	QApplication::restoreOverrideCursor();

	if (m_interactionMode == SEGMENT_ENTITY)
	{
		emit buttonReleased();
		return;
	}

	if (m_pivotSymbolShown)
	{
		if (m_pivotVisibility == PIVOT_SHOW_ON_MOVE)
			toBeRefreshed();
		showPivotSymbol(m_pivotVisibility == PIVOT_ALWAYS_SHOW);
	}

	if ((event->button() == Qt::RightButton)
#ifdef CC_MAC_OS
		|| (QApplication::keyboardModifiers () & Qt::MetaModifier)
#endif
		)
	{
		if (!cursorHasMoved)
		{
			//specific case: interaction with item(s)
			updateActiveItemsList(event->x(),event->y(),false);
			if (!m_activeItems.empty())
			{
				ccInteractor* item = m_activeItems.front();
				m_activeItems.clear();
				if (item->acceptClick(event->x(),height()-1-event->y(),Qt::RightButton))
				{
					acceptEvent = true;
					toBeRefreshed();
				}
			}
		}
		else
		{
			acceptEvent = true;
			toBeRefreshed();
		}
	}
	else if (event->button() == Qt::LeftButton)
	{
		if (cursorHasMoved)
		{
			//if a rectangular picking area has been defined
			if (m_rectPickingPoly)
			{
				CCLib::GenericIndexedCloudPersist* vertices = m_rectPickingPoly->getAssociatedCloud();
				assert(vertices);
				const CCVector3* A = vertices->getPointPersistentPtr(0);
				const CCVector3* C = vertices->getPointPersistentPtr(2);

				int pickX = static_cast<int>(A->x+C->x)/2;
				int pickY = static_cast<int>(A->y+C->y)/2;
				int pickW = static_cast<int>(fabs(C->x-A->x));
				int pickH = static_cast<int>(fabs(C->y-A->y));

				removeFromOwnDB(m_rectPickingPoly);
				m_rectPickingPoly = 0;
				delete vertices;
				vertices = 0;

				startPicking(ENTITY_RECT_PICKING, pickX+width()/2, height()/2-pickY, pickW, pickH);
			}

			toBeRefreshed();
			acceptEvent = true;
		}
		else
		{
			//picking?
			if (ccTimer::Msec() < m_lastClickTime_ticks + CC_MAX_PICKING_CLICK_DURATION_MS)
			{
				int x = event->x();
				int y = event->y();

				//specific case: interaction with item(s) such as labels, etc.
				//DGM TODO: to activate only if some items take left clicks into account!
				/*if (!m_activeItems.empty())
				{
					for (std::list<ccInteractor*>::iterator it=m_activeItems.begin(); it!=m_activeItems.end(); ++it)
					if ((*it)->acceptClick(x,y,Qt::LeftButton))
					{
						event->accept();
						redraw();
						return;
					}
				}
				//*/

				//first test if the user has clicked on a particular item on the screen
				if (processClickableItems(x,y))
				{
					acceptEvent = true;
				}
				//otheriwse perform OpenGL picking
				else if (m_pickingMode != NO_PICKING && m_interactionMode != TRANSFORM_ENTITY)
				{
					PICKING_MODE pickingMode = m_pickingMode;

					//shift+click = point/triangle picking
					if (pickingMode == ENTITY_PICKING && (QApplication::keyboardModifiers() & Qt::ShiftModifier))
						pickingMode = AUTO_POINT_PICKING;

					startPicking(pickingMode,event->x(),event->y());

					//we also spread the news (if anyone is interested ;)
					emit leftButtonClicked(event->x(), event->y());

					acceptEvent = true;
				}
			}
		}

		m_activeItems.clear();
	}

	if (acceptEvent)
		event->accept();
	else
		event->ignore();

	refresh();
}

void ccGLWindow::wheelEvent(QWheelEvent* event)
{
	if (m_interactionMode == SEGMENT_ENTITY)
	{
		event->ignore();
		return;
	}

	//see QWheelEvent documentation ("distance that the wheel is rotated, in eighths of a degree")
	float wheelDelta_deg = static_cast<float>(event->delta()) / 8.0f;

	onWheelEvent(wheelDelta_deg);

	emit mouseWheelRotated(wheelDelta_deg);
}

void ccGLWindow::onWheelEvent(float wheelDelta_deg)
{
	//in perspective mode, wheel event corresponds to 'walking'
	if (m_viewportParams.perspectiveView)
	{
		//to zoom in and out we simply change the fov in bubble-view mode!
		if (m_bubbleViewModeEnabled)
		{
			setBubbleViewFov(m_bubbleViewFov_deg - wheelDelta_deg / 3.6); //1 turn = 100 degrees
		}
		else
		{
			//convert degrees in 'constant' walking speed in ... pixels ;)
			static const float c_deg2PixConversion = 1.0f;
			moveCamera(0,0,-(c_deg2PixConversion * wheelDelta_deg) * m_viewportParams.pixelSize);
		}
	}
	else //ortho. mode
	{
		//convert degrees in zoom 'power'
		static const float c_defaultDeg2Zoom = 20.0f;
		float zoomFactor = pow(1.1f,wheelDelta_deg / c_defaultDeg2Zoom);
		updateZoom(zoomFactor);
	}

	redraw();
}

int ccGLWindow::startPicking(PICKING_MODE pickingMode, int centerX, int centerY, int pickWidth, int pickHeight, int* subID/*=0*/)
{
	if (subID)
		*subID = -1;
	if (!m_globalDBRoot && !m_winDBRoot)
		return -1;

	assert(m_interactionMode != TRANSFORM_ENTITY);

	//setup rendering context
	CC_DRAW_CONTEXT context;
	getContext(context);
	unsigned short pickingFlags = CC_DRAW_FOREGROUND;

	switch(pickingMode)
	{
	case ENTITY_PICKING:
	case ENTITY_RECT_PICKING:
		pickingFlags |= CC_DRAW_ENTITY_NAMES;
		break;
	case FAST_PICKING:
		pickingFlags |= CC_DRAW_ENTITY_NAMES;
		pickingFlags |= CC_DRAW_FAST_NAMES_ONLY;
		break;
	case POINT_PICKING:
		pickingFlags |= CC_DRAW_POINT_NAMES;	//automatically push entity names as well!
		break;
	case TRIANGLE_PICKING:
		pickingFlags |= CC_DRAW_TRI_NAMES;		//automatically push entity names as well!
		break;
	case AUTO_POINT_PICKING:
		pickingFlags |= CC_DRAW_POINT_NAMES;	//automatically push entity names as well!
		pickingFlags |= CC_DRAW_TRI_NAMES;
		break;
	default:
		return -1;
	}

	makeCurrent();

	//no need to clear display, we don't draw anything new!
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//setup selection buffers
	memset(m_pickingBuffer,0,sizeof(GLuint)*CC_PICKING_BUFFER_SIZE);
	glSelectBuffer(CC_PICKING_BUFFER_SIZE,m_pickingBuffer);
	glRenderMode(GL_SELECT);
	glInitNames();

	//get viewport
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	//3D objects picking
	{
		context.flags = CC_DRAW_3D | pickingFlags;

		glEnable(GL_DEPTH_TEST);

		//projection matrix
		glMatrixMode(GL_PROJECTION);
		//restrict drawing to the picking area
		glLoadIdentity();
		gluPickMatrix((GLdouble)centerX,(GLdouble)(viewport[3]-centerY),(GLdouble)pickWidth,(GLdouble)pickHeight,viewport);
		glMultMatrixd(getProjectionMatd());

		//model view matrix
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixd(getModelViewMatd());

		//display 3D objects
		if (m_globalDBRoot)
			m_globalDBRoot->draw(context);
		if (m_winDBRoot)
			m_winDBRoot->draw(context);

		ccGLUtils::CatchGLError("ccGLWindow::startPicking.draw(3D)");
	}

	//2D objects picking
	if (pickingMode == ENTITY_PICKING || pickingMode == ENTITY_RECT_PICKING || pickingMode == FAST_PICKING)
	{
		context.flags = CC_DRAW_2D | pickingFlags;

		glDisable(GL_DEPTH_TEST);

		//we must first grab the 2D ortho view projection matrix
		setStandardOrthoCenter();
		glMatrixMode(GL_PROJECTION);
		double orthoProjMatd[OPENGL_MATRIX_SIZE];
		glGetDoublev(GL_PROJECTION_MATRIX, orthoProjMatd);
		//restrict drawing to the picking area
		glLoadIdentity();
		gluPickMatrix((GLdouble)centerX,(GLdouble)(viewport[3]-centerY),(GLdouble)pickWidth,(GLdouble)pickHeight,viewport);
		glMultMatrixd(orthoProjMatd);
		glMatrixMode(GL_MODELVIEW);

		//we display 2D objects
		if (m_globalDBRoot)
			m_globalDBRoot->draw(context);
		if (m_winDBRoot)
			m_winDBRoot->draw(context);

		ccGLUtils::CatchGLError("ccGLWindow::startPicking.draw(2D)");
	}

	glFlush();

	// returning to normal rendering mode
	int hits = glRenderMode(GL_RENDER);

	ccGLUtils::CatchGLError("ccGLWindow::startPicking.render");

	ccLog::PrintDebug("Picking hits: %i",hits);
	if (hits < 0)
	{
		ccLog::Warning("Too many items inside picking zone! Try to zoom in...");
		return -1;
	}

	//process hits
	int selectedID=-1,subSelectedID=-1;
	std::set<int> selectedIDs; //for ENTITY_RECT_PICKING mode only
	{
		GLuint minMinDepth = (~0);
		const GLuint* _selectBuf = m_pickingBuffer;
		for (int i=0; i<hits; ++i)
		{
			const GLuint& n = _selectBuf[0]; //number of names on stack
			if (n) //if we draw anything outside of 'glPushName()... glPopName()' then it will appear here with as an empty set!
			{
				//n should be equal to 1 (CC_DRAW_ENTITY_NAMES mode) or 2 (CC_DRAW_POINT_NAMES/CC_DRAW_TRIANGLES_NAMES modes)!
				assert(n==1 || n==2);
				const GLuint& minDepth = _selectBuf[1];
				//const GLuint& maxDepth = _selectBuf[2];
				const GLuint& currentID = _selectBuf[3];

				if (pickingMode == ENTITY_RECT_PICKING)
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
						subSelectedID = (n>1 ? _selectBuf[4] : -1);
						minMinDepth = minDepth;
					}
				}
			}

			_selectBuf += (3+n);
		}
	}

	if (subID)
		*subID = subSelectedID;

	//standard "entity" picking
	if (pickingMode == ENTITY_PICKING)
	{
		emit entitySelectionChanged(selectedID);
	}
	//rectangular "entity" picking
	else if (pickingMode == ENTITY_RECT_PICKING)
	{
		emit entitiesSelectionChanged(selectedIDs);
	}
	//"3D point" picking
	else if (pickingMode == POINT_PICKING)
	{
		if (selectedID >= 0 && subSelectedID >= 0)
		{
			emit pointPicked(selectedID,(unsigned)subSelectedID,centerX,centerY);
		}
	}
	else if (pickingMode == AUTO_POINT_PICKING)
	{
		if (m_globalDBRoot && selectedID >= 0 && subSelectedID >= 0)
		{
			ccHObject* obj = m_globalDBRoot->find(selectedID);
			if (obj)
			{
				//auto spawn the right label
				cc2DLabel* label = 0;
				if (obj->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					label = new cc2DLabel();
					label->addPoint(ccHObjectCaster::ToGenericPointCloud(obj),subSelectedID);
					obj->addChild(label);
				}
				else if (obj->isKindOf(CC_TYPES::MESH))
				{
					label = new cc2DLabel();
					ccGenericMesh *mesh = ccHObjectCaster::ToGenericMesh(obj);
					ccGenericPointCloud *cloud = mesh->getAssociatedCloud();
					assert(cloud);
					CCLib::TriangleSummitsIndexes *summitsIndexes = mesh->getTriangleIndexes(subSelectedID);
					label->addPoint(cloud,summitsIndexes->i1);
					label->addPoint(cloud,summitsIndexes->i2);
					label->addPoint(cloud,summitsIndexes->i3);
					cloud->addChild(label);
					if (!cloud->isEnabled())
					{
						cloud->setVisible(false);
						cloud->setEnabled(true);
					}
				}

				if (label)
				{
					label->setVisible(true);
					label->setDisplay(obj->getDisplay());
					label->setPosition(	static_cast<float>(centerX+20)/static_cast<float>(width()),
										static_cast<float>(centerY+20)/static_cast<float>(height()) );
					emit newLabel(static_cast<ccHObject*>(label));
					QApplication::processEvents();

					toBeRefreshed();
				}
			}
		}
	}

	return selectedID;
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
		else
		{
			ccLog::WarningDebug("[ccGLWindow::displayNewMessage] Append is forced for custom messages!");
		}
	}
	else
	{
		if (pos == SCREEN_CENTER_MESSAGE)
		{
			ccLog::Warning("[ccGLWindow::displayNewMessage] Append is not supported for center screen messages!"); 
			append = false;
		}
	}

	MessageToDisplay mess;
	mess.message = message;
	mess.messageValidity_sec = ccTimer::Sec()+displayMaxDelay_sec;
	mess.position = pos;
	mess.type = type;
	m_messagesToDisplay.push_back(mess);

	//ccLog::Print(QString("[displayNewMessage] New message valid until %1 s.").arg(mess.messageValidity_sec));
}

void ccGLWindow::setPointSize(float size)
{
	if (size >= static_cast<float>(MIN_POINT_SIZE) && size <= static_cast<float>(MAX_POINT_SIZE))
	{
		m_viewportParams.defaultPointSize = size;
		m_updateFBO = true;
	}
}

void ccGLWindow::setLineWidth(float width)
{
	m_viewportParams.defaultLineWidth = width;
	m_updateFBO = true;
}

void ccGLWindow::setFontPointSize(int pixelSize)
{
	m_font.setPointSize(pixelSize);
}

int ccGLWindow::getFontPointSize() const
{
	return m_captureMode.enabled ? static_cast<int>(m_font.pointSize() * m_captureMode.zoomFactor) : m_font.pointSize();
}

void ccGLWindow::glEnableSunLight()
{
	glLightfv(GL_LIGHT0,GL_DIFFUSE,getDisplayParameters().lightDiffuseColor);
	glLightfv(GL_LIGHT0,GL_AMBIENT,getDisplayParameters().lightAmbientColor);
	glLightfv(GL_LIGHT0,GL_SPECULAR,getDisplayParameters().lightSpecularColor);
	glLightfv(GL_LIGHT0, GL_POSITION, m_sunLightPos);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
	glEnable(GL_LIGHT0);
}

void ccGLWindow::glDisableSunLight()
{
	glDisable(GL_LIGHT0);
}

void ccGLWindow::setSunLight(bool state)
{
	m_sunLightEnabled=state;
	displayNewMessage(state ? "Sun light ON" : "Sun light OFF",
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						SUN_LIGHT_STATE_MESSAGE);
	redraw();

	//save parameter
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		settings.setValue(c_ps_sunLight,	m_sunLightEnabled);
	}
}

void ccGLWindow::toggleSunLight()
{
	setSunLight(!m_sunLightEnabled);
}

void ccGLWindow::glEnableCustomLight()
{
	glLightfv(GL_LIGHT1,GL_DIFFUSE,getDisplayParameters().lightDiffuseColor);
	glLightfv(GL_LIGHT1,GL_AMBIENT,getDisplayParameters().lightAmbientColor);
	glLightfv(GL_LIGHT1,GL_SPECULAR,getDisplayParameters().lightSpecularColor);
	glLightfv(GL_LIGHT1,GL_POSITION,m_customLightPos);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
	glEnable(GL_LIGHT1);
}

void ccGLWindow::glDisableCustomLight()
{
	glDisable(GL_LIGHT1);
}

void ccGLWindow::setCustomLight(bool state)
{
	m_customLightEnabled=state;
	displayNewMessage(state ? "Custom light ON" : "Custom light OFF",
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						CUSTOM_LIGHT_STATE_MESSAGE);

	invalidateViewport();
	redraw();

	//save parameter
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		settings.setValue(c_ps_customLight,	m_customLightEnabled);
	}
}

void ccGLWindow::toggleCustomLight()
{
	setCustomLight(!m_customLightEnabled);
}

void ccGLWindow::drawCustomLight()
{
	glColor3ubv(ccColor::yellow);
	//ensure that the star size is constant (in pixels)
	GLfloat d = static_cast<GLfloat>(CC_DISPLAYED_CUSTOM_LIGHT_LENGTH * computeActualPixelSize());

	glBegin(GL_LINES);
	glVertex3f(m_customLightPos[0]-d, m_customLightPos[1],   m_customLightPos[2]);
	glVertex3f(m_customLightPos[0]+d, m_customLightPos[1],   m_customLightPos[2]);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1]-d, m_customLightPos[2]);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1]+d, m_customLightPos[2]);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1],   m_customLightPos[2]-d);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1],   m_customLightPos[2]+d);
	glEnd();
}

//draw a unit circle in a given plane (0=YZ, 1 = XZ, 2=XY) 
static void glDrawUnitCircle(unsigned char dim, unsigned steps = 64)
{
	double thetaStep = 2.0 * M_PI / static_cast<double>(steps);
	unsigned char dimX = (dim  < 2 ? dim  + 1 : 0);
	unsigned char dimY = (dimX < 2 ? dimX + 1 : 0);

	CCVector3d P(0,0,0);

	glBegin(GL_LINE_LOOP);
	for (unsigned i=0; i<steps; ++i)
	{
		double theta = thetaStep * i;
		P.u[dimX] = cos(theta);
		P.u[dimY] = sin(theta);
		glVertex3dv(P.u);
	}
	glEnd();
}

void ccGLWindow::setPivotVisibility(PivotVisibility vis)
{
	m_pivotVisibility = vis;
	
	//auto-save last pivot visibility settings
	{
		QSettings settings;
		settings.beginGroup(c_ps_groupName);
		settings.setValue(c_ps_pivotVisibility,	vis);
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
	}

	m_pivotSymbolShown = state;
}

void ccGLWindow::drawPivot()
{
	if (!m_viewportParams.objectCenteredView)
		return;

	if (m_pivotVisibility == PIVOT_HIDE ||
		(m_pivotVisibility == PIVOT_SHOW_ON_MOVE && !m_pivotSymbolShown))
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	//place origin on pivot point
	glTranslated(m_viewportParams.pivotPoint.x, m_viewportParams.pivotPoint.y, m_viewportParams.pivotPoint.z);

	//compute actual symbol radius
	double symbolRadius = CC_DISPLAYED_PIVOT_RADIUS_PERCENT * static_cast<double>(std::min(m_glWidth,m_glHeight)) / 2;

	if (m_pivotGLList == GL_INVALID_LIST_ID)
	{
		m_pivotGLList = glGenLists(1);
		glNewList(m_pivotGLList, GL_COMPILE);

		//draw a small sphere
		{
			ccSphere sphere(static_cast<PointCoordinateType>(10.0/symbolRadius));
			sphere.setColor(ccColor::yellow);
			sphere.showColors(true);
			sphere.setVisible(true);
			sphere.setEnabled(true);
			//force lighting for proper sphere display
			glPushAttrib(GL_LIGHTING_BIT);
			glEnableSunLight();
			CC_DRAW_CONTEXT context;
			getContext(context);
			context.flags = CC_DRAW_3D | CC_DRAW_FOREGROUND | CC_LIGHT_ENABLED;
			context._win = 0;
			sphere.draw(context);
			glPopAttrib();
		}

		//draw 3 circles
		glPushAttrib(GL_LINE_BIT);
		glEnable(GL_LINE_SMOOTH);
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		glEnable(GL_BLEND);
		const float c_alpha = 0.6f;
		glLineWidth(2.0f);

		//pivot symbol: 3 circles
		glColor4f(1.0f,0.0f,0.0f,c_alpha);
		glDrawUnitCircle(0);
		glBegin(GL_LINES);
		glVertex3f(-1.0f,0.0f,0.0f);
		glVertex3f( 1.0f,0.0f,0.0f);
		glEnd();

		glColor4f(0.0f,1.0f,0.0f,c_alpha);
		glDrawUnitCircle(1);
		glBegin(GL_LINES);
		glVertex3f(0.0f,-1.0f,0.0f);
		glVertex3f(0.0f, 1.0f,0.0f);
		glEnd();

		glColor4f(0.0f,0.7f,1.0f,c_alpha);
		glDrawUnitCircle(2);
		glBegin(GL_LINES);
		glVertex3f(0.0f,0.0f,-1.0f);
		glVertex3f(0.0f,0.0f, 1.0f);
		glEnd();

		glPopAttrib();
		glPopAttrib();

		glEndList();
	}

	//constant scale
	double scale = symbolRadius * computeActualPixelSize();
	glScaled(scale,scale,scale);
	
	glCallList(m_pivotGLList);

	glPopMatrix();
}

void ccGLWindow::togglePerspective(bool objectCentered)
{
	if (m_viewportParams.objectCenteredView != objectCentered)
		setPerspectiveState(true,objectCentered);
	else
		setPerspectiveState(!m_viewportParams.perspectiveView,objectCentered);
}

double ccGLWindow::computeActualPixelSize() const
{
	if (!m_viewportParams.perspectiveView)
	{
		return static_cast<double>(m_viewportParams.pixelSize) / static_cast<double>(m_viewportParams.zoom);
	}

	int minScreenDim = std::min(m_glWidth,m_glHeight);
	if (minScreenDim <= 0)
		return 1.0;

	//Camera center to pivot vector
	double zoomEquivalentDist = (m_viewportParams.cameraCenter - m_viewportParams.pivotPoint).norm();

	float currentFov_deg = getFov();
	return static_cast<double>(zoomEquivalentDist) * tan(currentFov_deg * static_cast<float>(CC_DEG_TO_RAD)) / static_cast<double>(minScreenDim);
}

float ccGLWindow::computePerspectiveZoom() const
{
	//DGM: in fact it can be useful to compute it even in ortho mode :)
	//if (!m_viewportParams.perspectiveView)
	//	return m_viewportParams.zoom;

	//we compute the zoom equivalent to the corresponding camera position (inverse of above calculus)
	float currentFov_deg = getFov();
	if (currentFov_deg < ZERO_TOLERANCE)
		return 1.0f;

	//Camera center to pivot vector
	double zoomEquivalentDist = (m_viewportParams.cameraCenter - m_viewportParams.pivotPoint).norm();
	if (zoomEquivalentDist < ZERO_TOLERANCE)
		return 1.0f;
	
	float screenSize = static_cast<float>(std::min(m_glWidth,m_glHeight)) * m_viewportParams.pixelSize; //see how pixelSize is computed!
	return screenSize / static_cast<float>(zoomEquivalentDist * tan(currentFov_deg * CC_DEG_TO_RAD));
}

void ccGLWindow::setBubbleViewMode(bool state)
{
	//Backup the camera center before entering this mode!
	bool bubbleViewModeWasEnabled = m_bubbleViewModeEnabled;
	if (!m_bubbleViewModeEnabled && state)
		m_preBubbleViewParameters = m_viewportParams;

	if (state)
	{
		//bubble-view mode = viewer-based perspective mode
		//setPerspectiveState must be called first as it
		//automatically deactivates bubble-view mode!
		setPerspectiveState(true,false);

		m_bubbleViewModeEnabled = true;

		//when entering this mode, we reset the f.o.v.
		m_bubbleViewFov_deg = 0; //to trick the signal emission mechanism
		setBubbleViewFov(90.0f);
	}
	else if (bubbleViewModeWasEnabled)
	{
		m_bubbleViewModeEnabled = false;
		setPerspectiveState(m_preBubbleViewParameters.perspectiveView,m_preBubbleViewParameters.objectCenteredView);

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
			float currentFov_deg = getFov();
			assert(currentFov_deg > ZERO_TOLERANCE);
			double screenSize = static_cast<double>(std::min(m_glWidth,m_glHeight))*m_viewportParams.pixelSize; //see how pixelSize is computed!
			PC.z = screenSize / (m_viewportParams.zoom*tan(currentFov_deg*CC_DEG_TO_RAD));
		}

		//display message
		displayNewMessage(objectCenteredView ?	"Centered perspective ON" : "Viewer-based perspective ON",
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
		m_viewportParams.viewMat.transposed().apply(PC); //inverse rotation
	else if (!viewWasObjectCentered && m_viewportParams.objectCenteredView)
		m_viewportParams.viewMat.apply(PC);

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
}

void ccGLWindow::setAspectRatio(float ar)
{
	if (ar < 0)
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
		}
	}
}

void ccGLWindow::setFov(float fov_deg)
{
	if (fov_deg < ZERO_TOLERANCE || fov_deg > 180.0f)
	{
		ccLog::Warning("[ccGLWindow::setFov] Invalid FOV value!");
		return;
	}

	//derivation if we are in bubble-view mode
	if (m_bubbleViewModeEnabled)
	{
		setBubbleViewFov(fov_deg);
		return;
	}

	if (m_viewportParams.fov != fov_deg)
	{
		//update param
		m_viewportParams.fov = fov_deg;
		//and camera state (if perspective view is 'on')
		if (m_viewportParams.perspectiveView)
		{
			invalidateViewport();
			invalidateVisualization();
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
	if (fov_deg < ZERO_TOLERANCE || fov_deg > 180.0f)
		return;

	if (fov_deg != m_bubbleViewFov_deg)
	{
		m_bubbleViewFov_deg = fov_deg;

		if (m_bubbleViewModeEnabled)
		{
			invalidateViewport();
			invalidateVisualization();
			emit fovChanged(m_bubbleViewFov_deg);
		}
	}
}

void ccGLWindow::setZNearCoef(double coef)
{
	if (coef <= 0)
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
			invalidateViewport();
			invalidateVisualization();
		}
	}
}

void ccGLWindow::setViewportParameters(const ccViewportParameters& params)
{
	ccViewportParameters oldParams = m_viewportParams;
	m_viewportParams = params;

	invalidateViewport();
	invalidateVisualization();

	emit baseViewMatChanged(m_viewportParams.viewMat);
	emit pivotPointChanged(m_viewportParams.pivotPoint);
	emit cameraPosChanged(m_viewportParams.cameraCenter);
	emit fovChanged(m_viewportParams.fov);
}

void ccGLWindow::getViewportArray(int vpArray[])
{
	makeCurrent();
	glGetIntegerv(GL_VIEWPORT, vpArray);
}

void ccGLWindow::rotateBaseViewMat(const ccGLMatrixd& rotMat)
{
	m_viewportParams.viewMat = rotMat * m_viewportParams.viewMat;

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_viewportParams.viewMat);

	invalidateVisualization();
}

void ccGLWindow::updateZoom(float zoomFactor)
{
	//no 'zoom' in viewer based perspective
	assert(!m_viewportParams.perspectiveView);

	if (zoomFactor>0.0 && zoomFactor!=1.0)
		setZoom(m_viewportParams.zoom*zoomFactor);
}

void ccGLWindow::setupProjectiveViewport(	const ccGLMatrixd& cameraMatrix,
											float fov_deg/*=0.0f*/,
											float ar/*=1.0f*/,
											bool viewerBasedPerspective/*=true*/,
											bool bubbleViewMode/*=false*/)
{
	//perspective (viewer-based by default)
	if (bubbleViewMode)
		setBubbleViewMode(true);
	else
		setPerspectiveState(true,!viewerBasedPerspective);

	//field of view (= OpenGL 'fovy' but in degrees)
	if (fov_deg > 0)
		setFov(fov_deg);

	//aspect ratio
	setAspectRatio(ar);

	//set the camera matrix 'translation' as OpenGL camera center
	CCVector3d T = cameraMatrix.getTranslationAsVec3D();
	setCameraPos(T);
	if (viewerBasedPerspective)
		setPivotPoint(T);

	//apply orientation matrix
	ccGLMatrixd trans = cameraMatrix;
	trans.clearTranslation();
	trans.invert();
	setBaseViewMat(trans);

	redraw();
}

void ccGLWindow::setCustomView(const CCVector3d& forward, const CCVector3d& up, bool forceRedraw/*=true*/)
{
	makeCurrent();

	bool wasViewerBased = !m_viewportParams.objectCenteredView;
	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView,true);

	ccGLMatrixd viewMat = ccGLMatrixd::FromViewDirAndUpDir(forward,up);
	setBaseViewMat(viewMat);

	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView,false);

	if (forceRedraw)
		redraw();
}

void ccGLWindow::setView(CC_VIEW_ORIENTATION orientation, bool forceRedraw/*=true*/)
{
	makeCurrent();

	bool wasViewerBased = !m_viewportParams.objectCenteredView;
	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView,true);

	m_viewportParams.viewMat = ccGLUtils::GenerateViewMat(orientation);

	if (wasViewerBased)
		setPerspectiveState(m_viewportParams.perspectiveView,false);

	invalidateVisualization();

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_viewportParams.viewMat);

	if (forceRedraw)
		redraw();
}

bool ccGLWindow::renderToFile(	const char* filename,
								float zoomFactor/*=1.0*/,
								bool dontScaleFeatures/*=false*/,
								bool renderOverlayItems/*=false*/)
{
	if (!filename || zoomFactor<1e-2)
		return false;

	//current window size (in pixels)
	int Wp = static_cast<int>(width() * zoomFactor);
	int Hp = static_cast<int>(height() * zoomFactor);

	QImage output(Wp,Hp,QImage::Format_ARGB32);
	GLubyte* data = output.bits();
	if (!data)
	{
		ccLog::Error("[ccGLWindow::renderToFile] Not enough memory!");
		return false;
	}

	m_glWidth = Wp;
	m_glHeight = Hp;

	//we activate 'capture' mode
	m_captureMode.enabled = true;
	m_captureMode.zoomFactor = zoomFactor;
	m_captureMode.renderOverlayItems = renderOverlayItems;

	//current viewport parameters backup
	float _defaultPointSize = m_viewportParams.defaultPointSize;
	float _defaultLineWidth = m_viewportParams.defaultLineWidth;
	//current display parameters backup
	//bool displayParametersWereOverriden = m_overridenDisplayParametersEnabled;
	//ccGui::ParamStruct displayParams = getDisplayParameters();
	//int _fontSize = displayParams.defaultFontSize;

	if (!dontScaleFeatures)
	{
		//we update point size (for point clouds)
		setPointSize(_defaultPointSize*zoomFactor);
		//we update line width (for bounding-boxes, etc.)
		setLineWidth(_defaultLineWidth*zoomFactor);
		//we update font size (for text display)
		//displayParams.defaultFontSize = static_cast<int>(static_cast<float>(_fontSize) * zoomFactor);
	}

	//setDisplayParameters(displayParams,true);

	bool result = false;
	if (m_fbo)
	{
		ccLog::Print("[Render screen via FBO]");

		ccFrameBufferObject* fbo = 0;
		ccGlFilter* filter = 0;
		if (zoomFactor == 1.0f)
		{
			fbo = m_fbo;
			filter = m_activeGLFilter;
		}
		else
		{
			fbo = new ccFrameBufferObject();
			bool success = false;
			if (fbo->init(Wp,Hp))
				if (fbo->initTexture(0,GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE))
					success = fbo->initDepth(GL_CLAMP_TO_BORDER,GL_DEPTH_COMPONENT32,GL_NEAREST,GL_TEXTURE_2D);
			if (!success)
			{
				ccLog::Error("[FBO] Initialization failed! (not enough memory?)");
				delete fbo;
				fbo = 0;
			}
		}

		if (fbo)
		{
			makeCurrent();

			//update viewport
			glViewport(0,0,Wp,Hp);

			if (m_activeGLFilter && !filter)
			{
				QString shadersPath = ccGLWindow::getShadersPath();

				QString error;
				if (!m_activeGLFilter->init(Wp,Hp,shadersPath,error))
				{
					ccLog::Error(QString("[GL Filter] GL filter can't be used during rendering: %1").arg(error));
				}
				else
				{
					filter = m_activeGLFilter;
				}
			}

			//updateZoom(zoomFactor);

			CC_DRAW_CONTEXT context;
			getContext(context);
			context.glW = Wp;
			context.glH = Hp;
			context.renderZoom = zoomFactor;

			draw3D(context,false,fbo);

			context.flags = CC_DRAW_2D | CC_DRAW_FOREGROUND;
			if (m_interactionMode == TRANSFORM_ENTITY)		
				context.flags |= CC_VIRTUAL_TRANS_ENABLED;

			//setStandardOrthoCenter();
			{
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				float halfW = float(Wp)*0.5f;
				float halfH = float(Hp)*0.5f;
				float maxS = std::max(halfW,halfH);
				glOrtho(-halfW,halfW,-halfH,halfH,-maxS,maxS);
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
			}

			glDisable(GL_DEPTH_TEST);

			if (filter)
			{
				//we process GL filter
				GLuint depthTex = fbo->getDepthTexture();
				GLuint colorTex = fbo->getColorTexture(0);
				//minimal set of viewport parameters necessary for GL filters
				ccGlFilter::ViewportParameters parameters;
				parameters.perspectiveMode = m_viewportParams.perspectiveView;
				parameters.zFar = m_viewportParams.zFar;
				parameters.zNear = m_viewportParams.zNear;
				parameters.zoom = m_viewportParams.perspectiveView ? computePerspectiveZoom() : m_viewportParams.zoom; //TODO: doesn't work well with EDL in perspective mode!
				//apply shader
				filter->shade(depthTex, colorTex, parameters);

				ccGLUtils::CatchGLError("ccGLWindow::renderToFile/glFilter shade");

				//if render mode is ON: we only want to capture it, not to display it
				fbo->start();
				ccGLUtils::DisplayTexture2D(filter->getTexture(),context.glW,context.glH);
				//glClear(GL_DEPTH_BUFFER_BIT);
				fbo->stop();
			}

			fbo->start();

			//WARNING: THIS IS A ***FRACKING*** TRICK!!!
			//we must trick Qt painter that the widget has actually
			//been resized, otherwise the 'renderText' won't work!
			QRect backupRect = geometry();
			QRect& ncrect = const_cast<QRect&>(geometry());
			ncrect.setWidth(Wp);
			ncrect.setHeight(Hp);

			//we draw 2D entities (mainly for the color ramp!)
			if (m_globalDBRoot)
				m_globalDBRoot->draw(context);
			if (m_winDBRoot)
				m_winDBRoot->draw(context);

			//For tests
			//displayText("BOTTOM_LEFT",10,10);
			//displayText("MIDDLE_LEFT",10,Hp/2-10);
			//displayText("BOTTOM_RIGHT",Wp-100,10);
			//displayText("MIDDLE_DOWN",Wp/2,10);
			//displayText("TOP_RIGHT",Wp-100,Hp-10);
			//displayText("MIDDLE_RIGHT",Wp-100,Hp/2-10);
			//displayText("TOP_LEFT",10,Hp-10);
			//displayText("MIDDLE_UP",Wp/2,Hp-10);
			//displayText("MIDDLE",Wp/2,Hp/2);

			//current displayed scalar field color ramp (if any)
			ccRenderingTools::DrawColorRamp(context);

			if (m_displayOverlayEntities && m_captureMode.renderOverlayItems)
			{
				//scale: only in ortho mode
				if (!m_viewportParams.perspectiveView)
					drawScale(getDisplayParameters().textDefaultCol);

				//trihedron
				drawTrihedron();
			}

			//don't forget to restore the right 'rect' or the widget will be broken!
			ncrect = backupRect;

			//read from fbo
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			//to avoid memory issues, we read line by line
			for (int i=0; i<Hp; ++i)
				glReadPixels(0,i,Wp,1,GL_BGRA,GL_UNSIGNED_BYTE,data+(Hp-1-i)*Wp*4);
			glReadBuffer(GL_NONE);

			fbo->stop();

			if (m_fbo != fbo)
				delete fbo;
			fbo = 0;

			output.save(filename);

			ccGLUtils::CatchGLError("ccGLWindow::renderToFile");

			if (m_activeGLFilter)
				initGLFilter(width(),height());

			//resizeGL(width(),height());
			glViewport(0,0,width(),height());

			//updateZoom(1.0/zoomFactor);
			result = true;
		}

		//resizeGL(width(),height());
	}
	else if (m_activeShader)
	{
		ccLog::Error("Screen capture with shader not supported!");
	}
	//if no shader or fbo --> we grab screen directly
	else
	{
		ccLog::Print("[Render screen via QT pixmap]");

		QPixmap capture = renderPixmap(Wp,Hp);
		if (capture.width()>0 && capture.height()>0)
		{
			capture.save(filename);
			result = true;
		}
		else
		{
			ccLog::Error("Direct screen capture failed! (not enough memory?)");
		}
	}

	//resizeGL(width(),height());
	m_glWidth = width();
	m_glHeight = height();

	//we restore viewport parameters
	setPointSize(_defaultPointSize);
	setLineWidth(_defaultLineWidth);
	m_captureMode.enabled = false;
	m_captureMode.zoomFactor = 1.0f;

	if (result)
		ccLog::Print("[Snapshot] File '%s' saved! (%i x %i pixels)",filename,Wp,Hp);

	return true;
}

void ccGLWindow::removeFBO()
{
	//we "disconnect" current FBO, to avoid wrong display/errors
	//if QT tries to redraw window during object destruction
	ccFrameBufferObject* _fbo = m_fbo;
	m_fbo = 0;

	if (_fbo)
		delete _fbo;
}

bool ccGLWindow::initFBO(int w, int h)
{
	//we "disconnect" current FBO, to avoid wrong display/errors
	//if QT tries to redraw window during initialization
	ccFrameBufferObject* _fbo = m_fbo;
	m_fbo = 0;

	if (!_fbo)
		_fbo = new ccFrameBufferObject();

	bool success = false;
	if (_fbo->init(w,h))
	{
		if (_fbo->initTexture(0,GL_RGBA,GL_RGBA,GL_FLOAT))
			success = _fbo->initDepth(GL_CLAMP_TO_BORDER,GL_DEPTH_COMPONENT32,GL_NEAREST,GL_TEXTURE_2D);
	}

	if (!success)
	{
		ccLog::Warning("[FBO] Initialization failed!");
		delete _fbo;
		_fbo = 0;
		m_alwaysUseFBO = false;
		return false;
	}

	//ccLog::Print("[FBO] Initialized");

	m_fbo = _fbo;
	m_updateFBO = true;

	return true;
}

void ccGLWindow::removeGLFilter()
{
	//we "disconnect" current glFilter, to avoid wrong display/errors
	//if QT tries to redraw window during object destruction
	ccGlFilter* _filter = 0;
	std::swap(_filter,m_activeGLFilter);

	if (_filter)
		delete _filter;
}

bool ccGLWindow::initGLFilter(int w, int h)
{
	if (!m_activeGLFilter)
		return false;

	//we "disconnect" current glFilter, to avoid wrong display/errors
	//if QT tries to redraw window during initialization
	ccGlFilter* _filter = 0;
	std::swap(_filter,m_activeGLFilter);

	QString shadersPath = ccGLWindow::getShadersPath();

	QString error;
	if (!_filter->init(w,h,shadersPath,error))
	{
		ccLog::Warning(QString("[GL Filter] Initialization failed: ")+error.trimmed());
		return false;
	}

	ccLog::Print("[GL Filter] Filter initialized");

	m_activeGLFilter = _filter;

	return true;
}

int ccGLWindow::getGlFilterBannerHeight() const
{
	return QFontMetrics(font()).height() + 2*CC_GL_FILTER_BANNER_MARGIN;
}

bool ccGLWindow::supportOpenGLVersion(unsigned openGLVersionFlag)
{
	return (format().openGLVersionFlags() & openGLVersionFlag);
}

void ccGLWindow::display3DLabel(const QString& str, const CCVector3& pos3D, const unsigned char* rgb/*=0*/, const QFont& font/*=QFont()*/)
{
	glColor3ubv_safe(rgb ? rgb : getDisplayParameters().textDefaultCol);

	renderText(pos3D.x, pos3D.y, pos3D.z, str, font);
}

void ccGLWindow::displayText(	QString text,
								int x,
								int y,
								unsigned char align/*=ALIGN_HLEFT|ALIGN_VTOP*/,
								float bkgAlpha/*=0*/,
								const unsigned char* rgbColor/*=0*/,
								const QFont* font/*=0*/)
{
	makeCurrent();

	int x2 = x;
	int y2 = m_glHeight - 1 - y;

	//actual text color
	const unsigned char* col = (rgbColor ? rgbColor : getDisplayParameters().textDefaultCol);

	QFont textFont = (font ? *font : m_font);

	QFontMetrics fm(textFont);
	int margin = fm.height()/4;

	if (align != ALIGN_DEFAULT || bkgAlpha != 0)
	{
		QRect rect = fm.boundingRect(text);

		//text alignment
		if (align & ALIGN_HMIDDLE)
			x2 -= rect.width()/2;
		else if (align & ALIGN_HRIGHT)
			x2 -= rect.width();
		if (align & ALIGN_VMIDDLE)
			y2 += rect.height()/2;
		else if (align & ALIGN_VBOTTOM)
			y2 += rect.height();

		//background is not totally transparent
		if (bkgAlpha != 0)
		{
			glPushAttrib(GL_COLOR_BUFFER_BIT);
			glEnable(GL_BLEND);

			//inverted color with a bit of transparency
			const float invertedCol[4] = {	1.0f-static_cast<float>(col[0])/255.0f,
											1.0f-static_cast<float>(col[0])/255.0f,
											1.0f-static_cast<float>(col[0])/255.0f,
											bkgAlpha };
			glColor4fv(invertedCol);

			int xB = x2 - m_glWidth/2;
			int yB = m_glHeight/2 - y2;
			yB += margin/2; //empirical compensation

			glBegin(GL_POLYGON);
			glVertex2d(xB - margin, yB - margin);
			glVertex2d(xB - margin, yB + rect.height() + margin/2);
			glVertex2d(xB + rect.width() + margin, yB + rect.height() + margin/2); 
			glVertex2d(xB + rect.width() + margin, yB - margin); 
			glEnd();
			glPopAttrib();
		}
	}

	if (align & ALIGN_VBOTTOM)
		y2 -= margin; //empirical compensation
	else if (align & ALIGN_VMIDDLE)
		y2 -= margin/2; //empirical compensation
	
	glColor3ubv_safe(col);
	renderText(x2, y2, text, textFont);
}

QString ccGLWindow::getShadersPath()
{
#if defined(Q_OS_MAC)
	// shaders are in the bundle
	QString  path = QCoreApplication::applicationDirPath();
	path.remove( "MacOS" );
	return path + "Shaders";
#else
	return QApplication::applicationDirPath() + "/shaders";
#endif
}

//! Loads all available OpenGL extensions
bool ccGLWindow::InitGLEW()
{
#ifdef USE_GLEW
	
	// GLEW initialization
	if (!ccFBOUtils::InitGLEW())
	{
		ccLog::Warning("[Glew] An error occurred while initializing OpenGL extensions!");
		return false;
	}
	else
	{
		ccLog::Print("[Glew] Initialized!");
		return true;
	}

#else

	return false;

#endif
}
