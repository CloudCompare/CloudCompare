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
#include <cc2DLabel.h>
#include <ccColorRampShader.h>
#include <ccHObjectCaster.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccSubMesh.h>

//CCFbo
#include <ccFrameBufferObject.h>
#include <ccGlFilter.h>

//Qt
#include <QCoreApplication>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QOpenGLBuffer>

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
constexpr int ccGLWindowInterface::CC_GL_FILTER_BANNER_MARGIN;
constexpr double ccGLWindowInterface::CC_DISPLAYED_PIVOT_RADIUS_PERCENT;

//Vaious overlay elements dimensions
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

ccGLWindowInterface::HotZone::HotZone(ccGLWindowInterface* win)
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
{
	if (win)
	{
		font = win->getFont();
		int retinaScale = win->getDevicePixelRatio();
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
	fs_totalWidth = /*margin() + */fs_labelRect.width() + margin + iconSize/* + margin*/;

	textHeight = std::max(psi_labelRect.height(), bbv_labelRect.height());
	textHeight = std::max(lsi_labelRect.height(), textHeight);
	textHeight = std::max(fs_labelRect.height(), textHeight);
	textHeight = (3 * textHeight) / 4; // --> factor: to recenter the baseline a little
	yTextBottomLineShift = (iconSize / 2) + (textHeight / 2);
}

QRect ccGLWindowInterface::HotZone::rect(bool clickableItemsVisible, bool bubbleViewModeEnabled, bool fullScreenEnabled) const
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
	, m_signalEmitter(new ccGLWindowSignalEmitter(parent))
{
	//start internal timer
	m_timer.start();

	QString windowTitle = QString("3D View %1").arg(m_uniqueID);

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

bool ccGLWindowInterface::setLODEnabled(bool state, bool autoDisable/*=false*/)
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

void ccGLWindowInterface::drawClickableItems(int xStart0, int& yStart)
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

float ccGLWindowInterface::computeTrihedronLength() const
{
	return (CC_DISPLAYED_TRIHEDRON_AXES_LENGTH + CC_TRIHEDRON_TEXT_MARGIN) * m_captureMode.zoomFactor + QFontMetrics(getTextDisplayFont()).width('X');
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
	const int retinaScale = getDevicePixelRatio();
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

	setLODEnabled(true, true);
	m_currentLODState.level = 0;

	redraw();
}

void ccGLWindowInterface::startPicking(PickingParameters& params)
{
	//correction for HD screens
	const int retinaScale = getDevicePixelRatio();
	params.centerX *= retinaScale;
	params.centerY *= retinaScale;

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

				toBeRefreshed();
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
	return m_font;
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
	return m_viewportParams.computePixelSize(glHeight() <= glWidth() ? glWidth() : (glWidth() * glWidth()) / glHeight());
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

void ccGLWindowInterface::setGLCameraAspectRatio(float ar)
{
	if (ar < 0.0f)
	{
		ccLog::Warning("[ccGLWindowInterface::setGLCameraAspectRatio] Invalid AR value!");
		return;
	}

	if (m_viewportParams.cameraAspectRatio != ar)
	{
		//update parameter
		m_viewportParams.cameraAspectRatio = ar;

		//and camera state
		invalidateViewport();
		invalidateVisualization();
		deprecate3DLayer();
	}
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
	setGLCameraAspectRatio(ar);

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
	const int retinaScale = getDevicePixelRatio();
	w *= retinaScale;
	h *= retinaScale;

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
	return QFontMetrics(getFont()).height() + 2 * CC_GL_FILTER_BANNER_MARGIN;
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

	QFont textFont = (font ? *font : m_font);

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

void ccGLWindowInterface::renderText(int x, int y, const QString & str, uint16_t uniqueID/*=0*/, const QFont & font/*=QFont()*/)
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
	const int retinaScale = getDevicePixelRatio();
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
		m_pickingPBO.glBuffer->release();
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

void ccGLWindowInterface::drawTrihedron()
{
	ccQOpenGLFunctions* glFunc = functions();
	assert(glFunc);

	QFont textFont = getTextDisplayFont(); //we take rendering zoom into account!
	QFontMetrics fm(textFont);
	QRect rectX = fm.boundingRect('X');
	float trihedronEdgeLength = CC_DISPLAYED_TRIHEDRON_AXES_LENGTH * m_captureMode.zoomFactor;
	float trihedronLength = trihedronEdgeLength + CC_TRIHEDRON_TEXT_MARGIN * m_captureMode.zoomFactor + rectX.width();

	float halfW = glWidth() / 2.0f;
	float halfH = glHeight() / 2.0f;

	float trihedronCenterX = halfW - trihedronLength - 10.0f;
	float trihedronCenterY = halfH - trihedronLength - 5.0f;

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

		double radius = std::max(rectX.width(), rectX.height()) / 2.0 + CC_TRIHEDRON_TEXT_MARGIN * m_captureMode.zoomFactor;

		CCVector2d toTrihedronOrigin(trihedronCenterX, -trihedronCenterY);
		CCVector2d toCharOrigin(-(rectX.x() + rectX.width() / 2.0), rectX.y() + rectX.height() / 4.0); // rectX.height() should be divided by 2, but it looks better with 4 !

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
	//DGM: in 'capture mode', we have to fall back to the case 'render zoom = 1' (otherwise we might not get the exact same aspect)
	double equivalentWidth = RoundScale(scaleMaxW * pixelSize / (m_captureMode.enabled ? m_captureMode.zoomFactor : 1.0f));

	QFont font = getTextDisplayFont(); //we take rendering zoom into account!
	QFontMetrics fm(font);

	//we deduce the scale drawing width
	float scaleW_pix = static_cast<float>(equivalentWidth / pixelSize);
	if (m_captureMode.enabled)
	{
		//we can now safely apply the rendering zoom
		scaleW_pix *= m_captureMode.zoomFactor;
	}
	float trihedronLength = computeTrihedronLength();
	float dW = 2.0f * trihedronLength + 20.0f;
	float dH = std::max(fm.height() * 1.25f, trihedronLength + 5.0f);
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

void ccGLWindowInterface::Create(	ccGLWindowInterface*& window,
									QWidget*& widget,
									bool stereoMode/*=false*/,
									bool silentInitialization/*=false*/ )
{
	QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	format.setStereo(stereoMode);

	ccGLWindow* glWindow = new ccGLWindow(&format, nullptr, silentInitialization);
	window = glWindow;

#ifdef CC_GL_WINDOW_USE_QWINDOW
	widget = new ccGLWidget(glWindow);
#else
	widget = glWindow;
#endif
}

ccGLWindowInterface* ccGLWindowInterface::FromWidget(QWidget* widget)
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

bool ccGLWindowInterface::SupportStereo()
{
#ifdef CC_GL_WINDOW_USE_QWINDOW
	return true;
#else
	return false;
#endif
}
