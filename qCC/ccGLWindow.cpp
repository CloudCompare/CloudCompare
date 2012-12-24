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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2228                                                              $
//$LastChangedDate:: 2012-07-27 17:38:19 +0200 (ven., 27 juil. 2012)       $
//**************************************************************************
//

//CCLib
#include <CCConst.h>

//qCC
#include "ccGLWindow.h"
#include "ccGuiParameters.h"
#include "ccConsole.h"
#include "ccRenderingTools.h"
#ifdef CC_USE_DB_ROOT_AS_SCENE_GRAPH
//not compatible with ccViewer!
#include "db_tree/ccDBRoot.h"
#endif

//qCC_db
#include <ccHObject.h>
#include <ccBBox.h>
#include <ccCalibratedImage.h>
#include <cc2DLabel.h>
#include <ccGenericPointCloud.h>
#include <ccTimer.h>

//CCFbo
#include <ccShader.h>
#include <ccGlFilter.h>
#include <ccFrameBufferObject.h>

//QT
#include <QtGui>
#include <QWheelEvent>
#include <QElapsedTimer>
#include <QSettings>

//System
#include <math.h>
#include <algorithm>

#define CC_GL_MAX_ZOOM_RATIO 1.0e6f
#define CC_GL_MIN_ZOOM_RATIO 1.0e-6f

#define CC_DISPLAYED_TRIHEDRON_AXES_LENGTH 25.0f
#define CC_DISPLAYED_CUSTOM_LIGHT_LENGTH 10.0f
#define CC_DISPLAYED_CENTER_CROSS_LENGTH 10.0f

//Unique GL window ID
static int s_GlWindowNumber = 0;

ccGLWindow::ccGLWindow(QWidget *parent, const QGLFormat& format, QGLWidget* shareWidget /*=0*/)
	: QGLWidget(format,parent,shareWidget)
	, m_uniqueID(++s_GlWindowNumber) //GL window unique ID
	, m_initialized(false)
	, m_lastMousePos(-1,-1)
	, m_lastMouseOrientation(1,0,0)
	, m_currentMouseOrientation(1,0,0)
	, m_glWidth(0)
	, m_glHeight(0)
	, m_captureMode(false)
	, m_lodActivated(false)
	, m_shouldBeRefreshed(false)
	, m_cursorMoved(false)
	, m_unclosable(false)
	, m_interactionMode(TRANSFORM_CAMERA)
	, m_pickingMode(NO_PICKING)
	, m_captureModeZoomFactor(1.0f)
	, m_pivotPointBackup(0.0f)
	, m_validProjectionMatrix(false)
	, m_validModelviewMatrix(false)
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
	, m_activeGLFilter(0)
	, m_glFiltersEnabled(false)
	, m_winDBRoot(0)
	, m_globalDBRoot(0) //external DB
	, m_font(font())
{
	//GL window title
	setWindowTitle(QString("3D View %1").arg(m_uniqueID));

	//GL window own DB
	m_winDBRoot = new ccHObject(QString("DB.3DView_%1").arg(m_uniqueID));

	//default font point size
	setFontPointSize(10);

	//screen panning
	m_params.screenPan[0]=m_params.screenPan[1]=0;

	//lights
	m_sunLightEnabled = true;
	m_sunLightPos[0] = 0.0f;
	m_sunLightPos[1] = 1.0f;
	m_sunLightPos[2] = 1.0f;
	m_sunLightPos[3] = 0.0f;

	m_customLightEnabled = false;
	m_customLightPos[0] = 0.0f;
	m_customLightPos[1] = 0.0f;
	m_customLightPos[2] = 0.0f;
	m_customLightPos[3] = 1.0f;

	//matrices
	m_params.baseViewMat.toIdentity();
	//m_viewMat.toZero();
	memset(m_viewMatd,	0, sizeof(double)*16);
	//m_projMat.toZero();
	memset(m_projMatd,	0, sizeof(double)*16);

	//default modes
	setPickingMode(ENTITY_PICKING);
	setInteractionMode(TRANSFORM_CAMERA);

	//Drag & drop handling
	setAcceptDrops(true);

	//Embedded icons (point size, etc.)
	enableEmbeddedIcons(true);

	//auto-load last perspective settings
	{
		QSettings settings;
		settings.beginGroup("ccGLWindow");
		//load parameters
		bool perspectiveView = settings.value("perspectiveView", false).toBool();
		bool objectCenteredPerspective = settings.value("objectCenteredPerspective", true).toBool();
		m_sunLightEnabled = settings.value("sunLightEnabled", true).toBool();
		m_customLightEnabled = settings.value("customLightEnabled", false).toBool();
		settings.endGroup();

		if (!perspectiveView)
			ccLog::Print("[ccGLWindow] Persective is off by default");
		else
			ccLog::Print(QString("[ccGLWindow] Persective is on by default (%1)").arg(objectCenteredPerspective ? "object-centered" : "viewer-centered"));

		//apply saved parameters
		setPerspectiveState(perspectiveView, objectCenteredPerspective);
		if (m_customLightEnabled)
			displayNewMessage("Warning: custom light is ON",ccGLWindow::LOWER_LEFT_MESSAGE,false,2,CUSTOM_LIGHT_STATE_MESSAGE);
		if (!m_sunLightEnabled)
			displayNewMessage("Warning: sun light is OFF",ccGLWindow::LOWER_LEFT_MESSAGE,false,2,SUN_LIGHT_STATE_MESSAGE);
	}
}

ccGLWindow::~ccGLWindow()
{
	if (m_winDBRoot)
		delete m_winDBRoot;

	if (m_activeGLFilter)
		delete m_activeGLFilter;

	if (m_activeShader)
		delete m_activeShader;

	if (m_fbo)
		delete m_fbo;

	if (m_vbo.enabled)
	{
		makeCurrent();
		glDeleteBuffersARB(1, &m_vbo.idVert);
		glDeleteBuffersARB(1, &m_vbo.idInd);
	}
	m_vbo.clear();
}

void ccGLWindow::initializeGL()
{
	if (m_initialized)
		return;

	//we init model view matrix with identity and store it into 'baseViewMat' and 'm_viewMatd'
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glGetFloatv(GL_MODELVIEW_MATRIX, m_params.baseViewMat.data());
	glGetDoublev(GL_MODELVIEW_MATRIX, m_viewMatd);

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_params.baseViewMat);

	//we init projection matrix with identity and push it on the stack
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glGetFloatv(GL_PROJECTION_MATRIX, m_projMat.data());
	glGetDoublev(GL_PROJECTION_MATRIX, m_projMatd);

	//set viewport and visu. as invalid
	invalidateViewport();
	invalidateVisualization();

	//we initialize GLEW
	ccGLUtils::InitGLEW();

	//OpenGL version
	ccConsole::Print("[3D View %i] GL version: %s",m_uniqueID,glGetString(GL_VERSION));

	//GL extensions test
	m_shadersEnabled  = ccGLUtils::CheckShadersAvailability();
	if (!m_shadersEnabled)
	{
		//if no shader, no GL filter!
		ccConsole::Warning("[3D View %i] Shaders and GL filters unavailable",m_uniqueID);
	}
	else
	{
		ccConsole::Print("[3D View %i] Shaders available",m_uniqueID);

		m_glFiltersEnabled = ccGLUtils::CheckFBOAvailability();
		if (m_glFiltersEnabled)
		{
			ccConsole::Print("[3D View %i] GL filters available",m_uniqueID);
			m_alwaysUseFBO = true;
		}
		else
		{
			ccConsole::Warning("[3D View %i] GL filters unavailable (FBO not supported)",m_uniqueID);
		}
	}

	//we don't use it yet
	m_vbo.init();
	/*m_vbo.enabled = ccGLUtils::CheckVBOAvailability();
	if (m_vbo.enabled)
	{
	glGenBuffersARB(1, &m_vbo.idVert);
	glGenBuffersARB(1, &m_vbo.idInd);
	}
	//*/

	//transparency off
	glDisable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	ccGLUtils::CatchGLError("ccGLWindow::initializeGL");

	m_initialized = true;

	ccLog::Print("[ccGLWindow] 3D view initialized");
}

static bool s_resizeGLInitSuccess=true;
void ccGLWindow::resizeGL(int w, int h)
{
	//TRACE
	//ccConsole::Print("[resizeGL] (%i,%i) (px)",w,h);

	s_resizeGLInitSuccess = true;

	//if (!m_captureMode)
	{
		m_glWidth = w;
		m_glHeight = h;
	}

	//on redefinit le viewport (nécessaire si la fenêtre a été redimensionnée)
	glViewport(0,0,w,h);

	//la matrice de projection n'est plus valide
	invalidateViewport();
	if (m_params.perspectiveView) //for proper aspect ratio handling!
		invalidateVisualization();

	//filters
	if (m_fbo || m_alwaysUseFBO)
		s_resizeGLInitSuccess &= initFBO(m_glWidth,m_glHeight);
	if (m_activeGLFilter)
		s_resizeGLInitSuccess &= initGLFilter(m_glWidth,m_glHeight);

	displayNewMessage(QString("New size = %1 * %2 (px)").arg(w).arg(h),
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						SCREEN_SIZE_MESSAGE);

	s_resizeGLInitSuccess &= !ccGLUtils::CatchGLError("ccGLWindow::resizeGL");
}

//Framerate test
static const int FRAMERATE_TEST_DURATION_MSEC = 10000;
static const unsigned FRAMERATE_TEST_MIN_FRAMES = 50;
static bool s_frameRateTestInProgress = false;
static ccGLMatrix s_frameRateBackupMat;
static QTimer s_frameRateTimer;
static QElapsedTimer s_frameRateElapsedTimer;
static int s_frameRateElapsedTime_ms = -1; //i.e. not initialized
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
	s_frameRateBackupMat = m_params.baseViewMat;

	//s_frameRateTimer.setParent(this);
	connect(&s_frameRateTimer, SIGNAL(timeout()), this, SLOT(redraw()), Qt::QueuedConnection);

	displayNewMessage("[Framerate test in progress]",
						ccGLWindow::UPPER_CENTER_MESSAGE,
						true,
						3600);

	//let's start
	s_frameRateCurrentFrame = 0;
	s_frameRateElapsedTime_ms = -1;
	s_frameRateElapsedTimer.start();
	s_frameRateTimer.start(0);
};

void ccGLWindow::stopFrameRateTest()
{
	if (s_frameRateTestInProgress)
	{
		s_frameRateTimer.stop();
		s_frameRateTimer.disconnect();
		QApplication::processEvents();
	}
	s_frameRateTestInProgress=false;

	//we restore the original view mat
	m_params.baseViewMat = s_frameRateBackupMat;
	m_validModelviewMatrix = false;

	displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear message in the upper center area
	if (s_frameRateElapsedTime_ms>0)
	{
		QString message = QString("Framerate: %1 f/s").arg((double)s_frameRateCurrentFrame*1.0e3/(double)s_frameRateElapsedTime_ms,0,'f',3);
		displayNewMessage(message,ccGLWindow::LOWER_LEFT_MESSAGE,true);
		ccConsole::Print(message);
	}
	else
	{
		ccLog::Error("An error occured during framerate test!");
	}

	redraw();
}

void ccGLWindow::paintGL()
{
	//context initialization
	CC_DRAW_CONTEXT context;
	getContext(context);

	bool doDraw3D = (!m_fbo || ((m_alwaysUseFBO && m_updateFBO) || m_activeGLFilter || m_captureMode));

	if (doDraw3D)
	{
		bool doDrawCross = (!m_captureMode && !m_params.perspectiveView && !(m_fbo && m_activeGLFilter) && ccGui::Parameters().displayCross);
		draw3D(context,doDrawCross,m_fbo);
		m_updateFBO=false;
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
			m_activeGLFilter->shade(depthTex, colorTex, (m_params.perspectiveView && !m_params.objectCenteredPerspective ? 1.0 : m_params.zoom));

			ccGLUtils::CatchGLError("ccGLWindow::paintGL/glFilter shade");

			//if capture mode is ON: we only want to capture it, not to display it
			if (!m_captureMode)
				screenTex = m_activeGLFilter->getTexture();
		}
		else if (!m_captureMode)
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
	m_winDBRoot->draw(context);

	//current displayed scalar field color ramp (if any)
	ccRenderingTools::DrawColorRamp(context);

	//overlay entities
	if (!m_captureMode)
	{
		//default overlay color
		const unsigned char* textCol = ccGui::Parameters().textDefaultCol;

		//transparent border at the bottom of the screen
		if (m_activeGLFilter)
		{
			float w = (float)m_glWidth*0.5f;
			float h = (float)m_glHeight*0.5f;
			int borderWidth = 2*CC_DISPLAYED_TRIHEDRON_AXES_LENGTH+10;

			glPushAttrib(GL_COLOR_BUFFER_BIT);
			glEnable(GL_BLEND);

			glColor4f(1.0f,1.0f,0.0f,0.6f);
			glBegin(GL_QUADS);
			glVertex2f(-w,-h+(float)borderWidth);
			glVertex2f(w,-h+(float)borderWidth);
			glVertex2f(w,-h);
			glVertex2f(-w,-h);
			glEnd();

			glPopAttrib();

			textCol = ccColor::black;
			//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
			//glColor3ubv(textCol);
			glColor3f((float)textCol[0]/(float)MAX_COLOR_COMP,(float)textCol[1]/(float)MAX_COLOR_COMP,(float)textCol[2]/(float)MAX_COLOR_COMP);
			renderText(10, m_glHeight-(float)borderWidth+15,QString("[GL filter] ")+m_activeGLFilter->getName()/*,m_font*/); //we ignore the custom font size
		}

		//trihedral
		drawAxis();

		//scale
		if (!m_params.perspectiveView)
			//if fbo --> override color
			drawScale(m_fbo && m_activeGLFilter ? ccColor::black : textCol);

		//current messages (if valid)
		if (!m_messagesToDisplay.empty())
		{
			int currentTime_sec = ccTimer::Sec();
			//ccLog::Print(QString("[paintGL] Current time: %1 s.").arg(currentTime_sec));

			//if fbo --> override color
			//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
			//glColor3ubv(m_fbo && m_activeGLFilter ? ccColor::black : textCol);
			const unsigned char* col = (m_fbo && m_activeGLFilter ? ccColor::black : textCol);
			glColor3f((float)col[0]/(float)MAX_COLOR_COMP,(float)col[1]/(float)MAX_COLOR_COMP,(float)col[2]/(float)MAX_COLOR_COMP);

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
							renderText(10, ll_currentHeight, it->message,m_font);
							int messageHeight = QFontMetrics(m_font).height();
							ll_currentHeight -= (messageHeight*5)/4; //add a 25% margin
						}
						break;
					case UPPER_CENTER_MESSAGE:
						{
							QRect rect = QFontMetrics(m_font).boundingRect(it->message);
							renderText((m_glWidth-rect.width())/2, uc_currentHeight+rect.height(), it->message,m_font);
							uc_currentHeight += (rect.height()*5)/4; //add a 25% margin
						}
						break;
					case SCREEN_CENTER_MESSAGE:
						{
							QFont newFont(m_font);
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

		if (m_hotZoneActivated)
		{
			QPixmap plusPix(":/CC/Other/images/other/plus.png");
			QPixmap minusPix(":/CC/Other/images/other/minus.png");
			GLuint plusTex = bindTexture(plusPix);
			GLuint minusTex = bindTexture(minusPix);

			//display parameters
			const int c_iconSize = 16;
			const int c_margin = 16;
			const int c_alphaChannel = 200;

			QFont newFont(m_font);
			newFont.setPointSize(12);
			QFontMetrics newMetrics(newFont);

			glPushAttrib(GL_COLOR_BUFFER_BIT);
			glEnable(GL_BLEND);

			//label
			//QFontMetrics::width("Point size");
			QString label("Point size");
			QRect rect = QFontMetrics(newFont).boundingRect(label);
			//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
			//glColor4ub(133,193,39,c_alphaChannel);
			glColor4f(0.52f,0.76f,0.15f,(float)c_alphaChannel/(float)MAX_COLOR_COMP);
			renderText(c_margin,(c_margin+c_iconSize/2)+(rect.height()/2)*2/3,label,newFont); // --> 2/3 to compensate the effect of the upper case letter (P)

			//icons
			int halfW = (m_glWidth>>1);
			int halfH = (m_glHeight>>1);

			int xStart = c_margin+rect.width()+c_margin;
			int yStart = c_margin;

			//"minus"
			ccGLUtils::DisplayTexture2DPosition(minusTex,-halfW+xStart,halfH-(yStart+c_iconSize),c_iconSize,c_iconSize,c_alphaChannel);
			m_hotZoneMinusIconROI[0]=xStart;
			m_hotZoneMinusIconROI[1]=yStart;
			m_hotZoneMinusIconROI[2]=xStart+c_iconSize;
			m_hotZoneMinusIconROI[3]=yStart+c_iconSize;
			xStart += c_iconSize;

			//separator
			glColor4ub(133,193,39,c_alphaChannel);
			glBegin(GL_POINTS);
			glVertex2i(-halfW+xStart+c_margin/2,halfH-(yStart+c_iconSize/2));
			glEnd();
			xStart += c_margin;

			//"plus"
			ccGLUtils::DisplayTexture2DPosition(plusTex,-halfW+xStart,halfH-(yStart+c_iconSize),c_iconSize,c_iconSize,c_alphaChannel);
			m_hotZonePlusIconROI[0]=xStart;
			m_hotZonePlusIconROI[1]=m_hotZoneMinusIconROI[1];
			m_hotZonePlusIconROI[2]=xStart+c_iconSize;
			m_hotZonePlusIconROI[3]=m_hotZoneMinusIconROI[3];
			xStart += c_iconSize;

			glDisable(GL_BLEND);
			glPopAttrib();
		}
	}

	ccGLUtils::CatchGLError("ccGLWindow::paintGL");

	m_shouldBeRefreshed=false;

	//For frame rate test
	if (s_frameRateTestInProgress)
	{
		s_frameRateElapsedTime_ms = s_frameRateElapsedTimer.elapsed();
		if (++s_frameRateCurrentFrame > FRAMERATE_TEST_MIN_FRAMES && s_frameRateElapsedTime_ms > FRAMERATE_TEST_DURATION_MSEC)
			stopFrameRateTest();
		else
		{
			//rotate base view matrtix
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadMatrixf(m_params.baseViewMat.data());
			glRotated(360.0/(double)FRAMERATE_TEST_MIN_FRAMES,0.0,1.0,0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, m_params.baseViewMat.data());
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

	glPointSize(m_params.defaultPointSize);
	glLineWidth(m_params.defaultLineWidth);

	//gradient color background
	if (ccGui::Parameters().drawBackgroundGradient)
	{
		drawGradientBackground();
		//we clear background
		glClear(GL_DEPTH_BUFFER_BIT);
	}
	else
	{
		const unsigned char* bkgCol = ccGui::Parameters().backgroundCol;
		glClearColor((float)bkgCol[0] / 255.0,
			(float)bkgCol[1] / 255.0,
			(float)bkgCol[2] / 255.0,
			1.0);

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
		if (!m_captureMode/* && !m_params.perspectiveView*/)
			//we display it as a litle 3D star
			displayCustomLight();
	}

	//we activate current shader (if activated)
	if (m_activeShader)
		m_activeShader->start();

	//we draw 3D entities
	if (m_globalDBRoot)
		m_globalDBRoot->draw(context);
	m_winDBRoot->draw(context);

	//for connected items
	emit drawing3D();

	//we disable shader
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
	for (unsigned i=0;i<mimeData->formats().size();++i)
	{
	QString format = mimeData->formats().at(i);
	ccConsole::Print(QString("Drop format: %1").arg(format));
	if (mimeData->hasFormat("FileNameW"))
	{
	QByteArray byteData = mimeData->data(format);
	ccConsole::Print(QString("\tdata: %1").arg(QString::fromUtf16((ushort*)byteData.data(), byteData.size() / 2)));
	}
	else
	{
	ccConsole::Print(QString("\tdata: %1").arg(QString(mimeData->data(format))));
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

		for (int i=0;i<fileNames.size();++i)
		{
			fileNames[i] = fileNames[i].trimmed();
#if defined(_WIN32) || defined(WIN32)
			fileNames[i].remove("file:///");
#else
			fileNames[i].remove("file://");
#endif
			//fileNames[i] = QUrl(fileNames[i].trimmed()).toLocalFile(); //toLocalFile removes the end of filenames sometimes!
#ifdef _DEBUG
			ccConsole::Print(QString("File dropped: %1").arg(fileNames[i]));
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

	ccConsole::Print(QString("Drop file(s): %1").arg(filename));
	//*/

	event->ignore();
}

float ccGLWindow::computeTotalZoom() const
{
	return m_params.zoom*m_params.globalZoom;
}

bool ccGLWindow::getPerspectiveState(bool& objectCentered) const
{
	objectCentered = m_params.objectCenteredPerspective;
	return m_params.perspectiveView;
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

void ccGLWindow::addToOwnDB(ccHObject* obj2D)
{
	assert(obj2D);

	m_winDBRoot->addChild(obj2D,false);
	obj2D->setDisplay(this);
}

void ccGLWindow::removeFromOwnDB(ccHObject* obj2D)
{
	m_winDBRoot->removeChild(obj2D);
}

void ccGLWindow::zoomGlobal()
{
	updateConstellationCenterAndZoom();
}

void ccGLWindow::updateConstellationCenterAndZoom(const ccBBox* aBox/*=0*/)
{
	m_params.screenPan[0] = m_params.screenPan[1] = 0;
	setZoom(1.0);

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

	//calcul du zoom 1:1
	//on récupère la diagonale de la bounding box de toutes les listes affichées
	float maxD = zoomedBox.getDiagNorm();

	if (maxD == 0)
		m_params.globalZoom = 1.0f;
	else
		m_params.globalZoom = float(std::max(m_glWidth,m_glHeight))/maxD;

	//pivot point
	CCVector3 c = zoomedBox.getCenter();
	setPivotPoint(c.x,c.y,c.z);

	if (m_params.perspectiveView)
		setPerspectiveState(true, m_params.objectCenteredPerspective);

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
		ccConsole::Warning("[ccGLWindow::setShader] Shader ignored (not supported)");
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
		ccConsole::Warning("[ccGLWindow::setGlFilter] GL filter ignored (not supported)");
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
	if (value < CC_GL_MIN_ZOOM_RATIO)
		value = CC_GL_MIN_ZOOM_RATIO;
	else if (value > CC_GL_MAX_ZOOM_RATIO)
		value = CC_GL_MAX_ZOOM_RATIO;

	if (m_params.zoom != value)
	{
		invalidateViewport();
		invalidateVisualization();
		m_params.zoom = value;
	}
}

void ccGLWindow::setPivotPoint(float x, float y, float z)
{
	m_params.pivotPoint = CCVector3(x,y,z);
	emit pivotPointChanged(m_params.pivotPoint);
	m_pivotPointBackup = m_params.pivotPoint;

	if (!m_params.perspectiveView)
	{
		invalidateViewport();
		invalidateVisualization();
	}
	else
	{
		setPerspectiveState(true, m_params.objectCenteredPerspective);
	}
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
	int w = (m_glWidth>>1)+1;
	int h = (m_glHeight>>1)+1;

	const unsigned char* bkgCol = ccGui::Parameters().backgroundCol;
	const unsigned char* forCol = ccGui::Parameters().pointsDefaultCol;

	//Gradient "texture" drawing
	glBegin(GL_QUADS);
	//we the user-defined background color for gradient start
	glColor3ubv(bkgCol);
	glVertex2f(-w,h);
	glVertex2f(w,h);
	//and the inverse of points color for gradient end
	glColor3ub(255-forCol[0],255-forCol[1],255-forCol[2]);
	glVertex2f(w,-h);
	glVertex2f(-w,-h);
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

void ccGLWindow::drawScale(const colorType color[] /*= white*/)
{
	float scaleMaxW = float(m_glWidth)*0.25; //25% de l'écran
	float totalZoom = computeTotalZoom();
	if (totalZoom==0.0)
		return;

	//we first compute the width equivalent to 25% of horizontal screen width
	//(this is why it's only valid in orthographic mode !)
	float equivalentWidth = scaleMaxW/totalZoom;

	//we then compute the scale granularity (to avoid width values with a lot of decimals)
	int k = int(floor(log((float)equivalentWidth)/log((float)10.0)));
	float granularity =pow((float)10.0,(float)k)*0.5;

	//we choose the value closest to equivalentWidth with the right granularity
	equivalentWidth = floor(std::max(equivalentWidth/granularity,1.0f))*granularity;

	//we deduce the scale drawing width
	float scaleW = equivalentWidth*totalZoom;
	float dW = 2*CC_DISPLAYED_TRIHEDRON_AXES_LENGTH+20.0;
	float w = float(m_glWidth)*0.5-dW;
	float h = float(m_glHeight)*0.5-10.0-QFontMetrics(m_font).height();

	//scale OpenGL drawing
	glColor3ubv(color);
	glBegin(GL_LINES);
	glVertex3f(w-scaleW,-h,0.0);
	glVertex3f(w,-h,0.0);
	glVertex3f(w-scaleW,-h-3,0.0);
	glVertex3f(w-scaleW,-h+3,0.0);
	glVertex3f(w,-h+3,0.0);
	glVertex3f(w,-h-3,0.0);
	glEnd();

	QString text = QString::number(equivalentWidth);
	//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
	glColor3f((float)color[0]/(float)MAX_COLOR_COMP,(float)color[1]/(float)MAX_COLOR_COMP,(float)color[2]/(float)MAX_COLOR_COMP);
	renderText(m_glWidth-int(scaleW*0.5+dW)-QFontMetrics(m_font).width(text)/2, m_glHeight-10, text, m_font);
}

void ccGLWindow::drawAxis()
{
	float w = float(m_glWidth)*0.5-CC_DISPLAYED_TRIHEDRON_AXES_LENGTH-10.0;
	float h = float(m_glHeight)*0.5-CC_DISPLAYED_TRIHEDRON_AXES_LENGTH-5.0;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(w, -h, 0.0);
	glMultMatrixf(m_params.baseViewMat.data());

	glPushAttrib(GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_DEPTH_TEST);

	//trihedra OpenGL drawing
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

	glPopMatrix();
}

void ccGLWindow::invalidateViewport()
{
	m_validProjectionMatrix=false;
	m_updateFBO=true;
}

void ccGLWindow::recalcProjectionMatrix()
{
	makeCurrent();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	float maxD = 1.0;
	CCVector3 center(0.0);

	//compute center of viewed objects constellation
	if (m_globalDBRoot)
	{
		//get whole bounding-box
		ccBBox box = m_globalDBRoot->getBB(true, true, this);
		//get bbox center
		center = box.getCenter();
		//get bbox diagonal length
		maxD = box.getDiagNorm()*0.5;
	}

	if (m_params.perspectiveView)
	{
		//we compute distance between camera and this center point
		CCVector3 cameraPos = computeCameraPos();
		float distToC = (cameraPos-center).norm();

		//we deduce zNear et zFar
		//DGM: we clip zNear just after 0 (not to close,
		//otherwise it can cause a very strange behavior
		//when looking at objects with large coordinates)
		float zNear = std::max(distToC-maxD,maxD*1e-3f);
		float zFar = std::max(distToC+maxD,1.0f);

		float ar = float(m_glWidth)/float(m_glHeight);

		gluPerspective(m_params.fov,ar,zNear,zFar);
	}
	else
	{
		//distance from bbox center to pivot point
		float d = (center-m_params.pivotPoint).norm();
		maxD += d;

		if (m_customLightEnabled)
		{
			//distance from pivot point to custom light display
			d = CCVector3::vdistance(m_params.pivotPoint.u,m_customLightPos);
			maxD = std::max(maxD,d);
		}

		maxD *= computeTotalZoom();
		maxD = std::max(maxD,1.0f);

		float halfW = float(m_glWidth)*0.5f;
		float halfH = float(m_glHeight)*0.5f;

		glOrtho(-halfW,halfW,-halfH,halfH,-maxD,maxD);
	}

	//we save projection matrix
	//glGetFloatv(GL_PROJECTION_MATRIX, m_projMat.data());
	glGetDoublev(GL_PROJECTION_MATRIX, m_projMatd);

	m_validProjectionMatrix = true;
}

void ccGLWindow::invalidateVisualization()
{
	m_validModelviewMatrix=false;
	m_updateFBO=true;
}

void ccGLWindow::recalcModelViewMatrix()
{
	makeCurrent();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (m_params.perspectiveView)
	{
		//for proper aspect ratio handling
		float ar = float(m_glWidth)/(float(m_glHeight)*m_params.aspectRatio);
		if (ar<1.0)
			glScalef(ar,ar,1.0);

		//on applique le panning
		if (m_params.objectCenteredPerspective)
			glTranslatef(-float(m_params.screenPan[0]), -float(m_params.screenPan[1]), 0);

		//rotation
		glMultMatrixf(m_params.baseViewMat.data());

		//get real camera position
		CCVector3 cameraPos = computeCameraPos();
		glTranslatef(-cameraPos.x, -cameraPos.y, -cameraPos.z);
	}
	else
	{
		//on fait rentrer le tout dans le cube de vision [-1:1]
		float totalZoom = computeTotalZoom();
		glScalef(totalZoom,totalZoom,totalZoom);

		//on applique le panning
		glTranslatef(-m_params.screenPan[0], -m_params.screenPan[1], 0);

		//rotation
		glMultMatrixf(m_params.baseViewMat.data());

		//par défaut on vise le centre des nuages de points
		glTranslatef(-m_params.pivotPoint.x, -m_params.pivotPoint.y, -m_params.pivotPoint.z);
	}

	//we save visualization matrix
	//glGetFloatv(GL_MODELVIEW_MATRIX, m_viewMat.data());
	glGetDoublev(GL_MODELVIEW_MATRIX, m_viewMatd);

	m_validModelviewMatrix=true;
}

const ccGLMatrix& ccGLWindow::getBaseModelViewMat()
{
	return m_params.baseViewMat;
}

const void ccGLWindow::setBaseModelViewMat(ccGLMatrix& mat)
{
	m_params.baseViewMat = mat;

	invalidateVisualization();

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_params.baseViewMat);
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
	float halfW = float(m_glWidth)*0.5;
	float halfH = float(m_glHeight)*0.5;
	float maxS = std::max(halfW,halfH);
	glOrtho(-halfW,halfW,-halfH,halfH,-maxS,maxS);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void ccGLWindow::setStandardOrthoCorner()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,0,float(m_glWidth),float(m_glHeight),0,1);
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

	const ccGui::ParamStruct& guiParams = ccGui::Parameters();

	//decimation options
	context.decimateCloudOnMove = guiParams.decimateCloudOnMove;
	context.decimateMeshOnMove = guiParams.decimateMeshOnMove;

	//scalar field colorbar
	context.sfColorScaleToDisplay = 0;

	//point picking
	context.pickedPointsSize = guiParams.pickedPointsSize;
	context.pickedPointsStartIndex = guiParams.pickedPointsStartIndex;
	context.pickedPointsTextShift = 5.0 / computeTotalZoom();

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
	memcpy(context.bbDefaultCol,guiParams.bbDefaultCol,sizeof(unsigned char)*3);

	//default font size
	setFontPointSize(guiParams.defaultFontSize);

	//VBO
	context.vbo = m_vbo;
}

void ccGLWindow::toBeRefreshed()
{
	m_shouldBeRefreshed=true;

	invalidateViewport();
}

void ccGLWindow::refresh()
{
	if (m_shouldBeRefreshed && isVisible())
		redraw();
}

void ccGLWindow::redraw()
{
	m_updateFBO=true;
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
	GLint maxTexSize=0;
	glGetIntegerv(GL_MAX_TEXTURE_SIZE,&maxTexSize);
	int cacheLimit = context()->textureCacheLimit()*1024;

	if (image.width() <= maxTexSize && image.height() <= maxTexSize && cacheLimit > image.width()*image.height()*4)
	{
		return bindTexture(image,GL_TEXTURE_2D,GL_RGBA,QGLContext::NoBindOption);
	}
	else
	{
		maxTexSize = std::min(maxTexSize, (int)sqrt((double)(cacheLimit>>2))); // ">>2"="/4" because we assume all textures have 4 components
		int width = image.width();
		int height = image.height();
		if (width>height)
		{
			width = maxTexSize;
			height = (int)((float)width*(float)image.height()/(float)image.width());
		}
		else
		{
			height = maxTexSize;
			width = (int)((float)height*(float)image.width()/(float)image.height());
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

CCVector3 ccGLWindow::getBaseViewMatDir() const
{
	const float* M = m_params.baseViewMat.data();
	return CCVector3(M[2],M[6],M[10]);
}

void ccGLWindow::setInteractionMode(INTERACTION_MODE mode)
{
	m_interactionMode = mode;
}

void ccGLWindow::setPickingMode(PICKING_MODE mode/*=DEFAULT_PICKING*/)
{
	switch(mode)
	{
	case DEFAULT_PICKING:
		mode = ENTITY_PICKING;
	case NO_PICKING:
	case ENTITY_PICKING:
	case TRIANGLE_PICKING:
		setCursor(QCursor(Qt::ArrowCursor));
		break;
	case POINT_PICKING:
		setCursor(QCursor(Qt::PointingHandCursor));
		break;
	}

	m_pickingMode = mode;
}

void ccGLWindow::enableEmbeddedIcons(bool state)
{
	m_embeddedIconsEnabled	= state;
	m_hotZoneActivated		= false;
	setMouseTracking(state);
}

void tbPointToVector(int x, int y, int width, int height, CCVector3& v)
{
	//position dans le plan
	v.x = float(2.0 * std::max(std::min(x,width-1),-width+1) - width) / (float)width;
	v.y = float(height - 2.0 * std::max(std::min(y,height-1),-height+1)) / (float)height;

	double d = v.x*v.x + v.y*v.y;

	//projection sur la sphère centrée au centre de la fenêtre
	if (d > 1.0)
	{
		PointCoordinateType a = (PointCoordinateType)(1.0 / sqrt(d));

		v.x *= a;
		v.y *= a;
		v.z = (PointCoordinateType)0.0;
	}
	else
	{
		v.z = (PointCoordinateType)(sqrt(1.0 - d));
	}
}

void ccGLWindow::updateActiveLabelsList(int x, int y, bool extendToSelectedLabels/*=false*/)
{
	m_activeLabels.clear();

	if (!m_globalDBRoot)
		return;

	int labelID = startPicking(x,y,LABELS_PICKING);
	if (labelID<1)
		return;

	//labels can be in local or global DB
	ccHObject* labelObj = m_globalDBRoot->find(labelID);
	if (!labelObj)
		labelObj = m_winDBRoot->find(labelID);
	if (labelObj && labelObj->isA(CC_2D_LABEL))
	{
		cc2DLabel* label = static_cast<cc2DLabel*>(labelObj);
		if (!label->isSelected() || !extendToSelectedLabels)
		{
			//select it?
			//emit entitySelectionChanged(label->getUniqueID());
			//QApplication::processEvents();
			m_activeLabels.resize(1);
			m_activeLabels.back() = label;
			return;
		}
		else
		{
			//we get the other selected labels as well!
			ccHObject::Container labels;
			m_globalDBRoot->filterChildren(labels,true,CC_2D_LABEL);
			m_winDBRoot->filterChildren(labels,true,CC_2D_LABEL);

			for (ccHObject::Container::iterator it=labels.begin(); it!=labels.end(); ++it)
				if ((*it)->isVisible())
				{
					cc2DLabel* label = static_cast<cc2DLabel*>(*it);
					if (label->isSelected())
					{
						m_activeLabels.resize(m_activeLabels.size()+1);
						m_activeLabels.back() = label;
					}
				}
		}
	}
}

void ccGLWindow::mousePressEvent(QMouseEvent *event)
{
	m_cursorMoved = false;

	if ((event->buttons() & Qt::RightButton)
#ifdef __APPLE__
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

		emit rightButtonClicked(event->x()-(width()>>1),(height()>>1)-event->y());
	}
	else if (event->buttons() & Qt::LeftButton)
	{
		if (m_interactionMode != SEGMENT_ENTITY) //mouse movement = rotation
		{
			tbPointToVector(event->x(), event->y(), width(), height(), m_lastMouseOrientation);
			m_lastMousePos = event->pos();

			m_lastClickTime_ticks = ccTimer::Msec();
			m_lodActivated = true;

			QApplication::setOverrideCursor(QCursor(Qt::PointingHandCursor));

			//let's check if the mouse is on a selected label first!
			updateActiveLabelsList(event->x(), event->y(), true);
		}

		emit leftButtonClicked(event->x()-(width()>>1),(height()>>1)-event->y());
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
		if (event->buttons()!=Qt::NoButton || m_alwaysUseFBO) //fast!
			emit mouseMoved(event->x()-(width()>>1),(height()>>1)-event->y(),event->buttons());
		return;
	}

	int x = event->x();
	int y = event->y();

	//no button pressed
	if (event->buttons()==Qt::NoButton)
	{
		if (m_embeddedIconsEnabled)
		{
			if (x < 200 && y < 100)
			{
				if (!m_hotZoneActivated)
				{
					m_hotZoneActivated = true;
					updateGL();
				}
			}
			else
			{
				if (m_hotZoneActivated)
				{
					m_hotZoneActivated = false;
					updateGL();
				}
			}
			event->accept();
		}
		else
		{
			event->ignore();
		}

		return;
	}

	int dx = x - m_lastMousePos.x();
	int dy = y - m_lastMousePos.y();
	m_lastMousePos = event->pos();

	if ((event->buttons() & Qt::RightButton)
#ifdef __APPLE__
		|| (QApplication::keyboardModifiers () & Qt::MetaModifier)
#endif
		)
	{
		if (!m_params.perspectiveView || m_params.objectCenteredPerspective)
		{
			bool movingMode = (m_interactionMode == TRANSFORM_ENTITY) || ((QApplication::keyboardModifiers () & Qt::ControlModifier) && m_customLightEnabled);
			if (movingMode)
			{
				//displacement vector projected on screen
				float totalZoom = computeTotalZoom();
				CCVector3 u((float)dx/totalZoom,-(float)dy/totalZoom,0);

				ccGLMatrix inverseBaseViewMat = m_params.baseViewMat;
				inverseBaseViewMat.transpose();
				inverseBaseViewMat.applyRotation(u);

				if (m_interactionMode == TRANSFORM_ENTITY)
				{
					emit translation(u);
				}
				else if (m_customLightEnabled)
				{
					CCVector3::vadd(m_customLightPos,u.u,m_customLightPos);
					invalidateViewport();
				}
			}
			else
			{

				float ddx = -float(dx)/m_params.zoom;
				float ddy = float(dy)/m_params.zoom;

				updateScreenPan(ddx,ddy);

				//feedback
				emit panChanged(ddx,ddy);
			}
		}
	}
	else if (event->buttons() & Qt::LeftButton) //rotation
	{
		//specific case: move label(s)
		if (!m_activeLabels.empty())
		{
			for (std::vector<cc2DLabel*>::iterator it=m_activeLabels.begin(); it!=m_activeLabels.end(); ++it)
				(*it)->move((float)dx/(float)width(),(float)dy/(float)height());
		}
		else
		{
			tbPointToVector(x, y, width(), height(), m_currentMouseOrientation);

			ccGLMatrix rotMat = ccGLUtils::GenerateGLRotationMatrixFromVectors(m_lastMouseOrientation.u,m_currentMouseOrientation.u);
			m_lastMouseOrientation = m_currentMouseOrientation;
			m_updateFBO=true;

			if (m_interactionMode == TRANSFORM_ENTITY)
			{
				rotMat = m_params.baseViewMat.transposed() * rotMat * m_params.baseViewMat;

				emit rotation(rotMat);
			}
			else
			{
				rotateViewMat(rotMat);

				QApplication::changeOverrideCursor(QCursor(Qt::ClosedHandCursor));

				//feedback
				emit viewMatRotated(rotMat);
			}
		}
	}

	m_cursorMoved = true;

	event->accept();

	if (m_interactionMode != TRANSFORM_ENTITY)
		updateGL();
}

void ccGLWindow::mouseReleaseEvent(QMouseEvent *event)
{
	if (m_interactionMode == SEGMENT_ENTITY)
	{
		emit buttonReleased();
		return;
	}

	m_lodActivated = false;
	QApplication::restoreOverrideCursor();

#ifndef __APPLE__
	if (event->button() == Qt::RightButton)
#endif
#ifdef __APPLE__
	  if ((event->button() == Qt::RightButton) || (QApplication::keyboardModifiers () & Qt::MetaModifier))
#endif
	{
		if (!m_cursorMoved)
		{
			//specific case: interaction with label(s)
			updateActiveLabelsList(event->x(),event->y(),false);
			if (!m_activeLabels.empty())
			{
				cc2DLabel* label = m_activeLabels[0];
				m_activeLabels.clear();
				if (label->acceptClick(event->x(),height()-1-event->y(),Qt::RightButton))
				{
					event->accept();
					redraw();
					return;
				}
			}
		}
		else
		{
			redraw();
			event->accept();
		}
	}
	else if (event->button() == Qt::LeftButton)
	{
		if (m_cursorMoved)
		{
			redraw();
			event->accept();
		}
		else
		{
			//picking = click < 1/5 s.
			if (ccTimer::Msec()-m_lastClickTime_ticks < 200)
			{
				int x = event->x();
				int y = event->y();

				//specific case: interaction with label(s)
				//DGM TODO: to activate when labels will take left clicks into account!
				/*if (!m_activeLabels.empty())
				{
				for (std::vector<cc2DLabel*>::iterator it=m_activeLabels.begin(); it!=m_activeLabels.end(); ++it)
				if ((*it)->acceptClick(x,y,Qt::LeftButton))
				{
				event->accept();
				redraw();
				return;
				}
				}
				//*/

				bool hotZoneClick = false;
				if (m_hotZoneActivated)
				{
					if (y >= m_hotZoneMinusIconROI[1] && y <= m_hotZoneMinusIconROI[3])
					{
						if (x >= m_hotZoneMinusIconROI[0] && x <= m_hotZoneMinusIconROI[2])
						{
							if (m_params.defaultPointSize>1)
							{
								setPointSize(m_params.defaultPointSize-1);
								updateGL();
							}
							hotZoneClick=true;
						}
						else if (x >= m_hotZonePlusIconROI[0] && x <= m_hotZonePlusIconROI[2])
						{
							if (m_params.defaultPointSize<10)
							{
								setPointSize(m_params.defaultPointSize+1);
								updateGL();
							}
							hotZoneClick=true;
						}
					}

					event->accept();
				}

				if (!hotZoneClick && m_pickingMode != NO_PICKING)
				{
					PICKING_MODE pickingMode = m_pickingMode;
					if (pickingMode == ENTITY_PICKING && QApplication::keyboardModifiers() & Qt::ShiftModifier)
						pickingMode = AUTO_POINT_PICKING;

					startPicking(event->x(),event->y(),pickingMode);

					emit leftButtonClicked(event->x(), event->y());

					event->accept();
				}
				else
				{
					event->ignore();
				}
			}
			else
			{
				event->ignore();
			}
		}
		m_activeLabels.clear();
	}
	else
	{
		event->ignore();
	}

	m_cursorMoved = false;
}

void ccGLWindow::wheelEvent(QWheelEvent* event)
{
	if (m_interactionMode == SEGMENT_ENTITY)
	{
		event->ignore();
		return;
	}

	float defaultLength = (m_params.perspectiveView && !m_params.objectCenteredPerspective ? 240.0f : 120.0f);
	float zoomFactor = pow(1.1f,float(event->delta())/defaultLength);

	updateZoom(zoomFactor);

	//Feedback
	emit zoomChanged(zoomFactor);

	redraw();
}

int ccGLWindow::startPicking(int cursorX, int cursorY, PICKING_MODE pickingMode)
{
	if (!m_globalDBRoot)
		return -1;

	//setup rendering context
	CC_DRAW_CONTEXT context;
	getContext(context);
	unsigned short pickingFlags = CC_DRAW_FOREGROUND;
	if (m_interactionMode == TRANSFORM_ENTITY)		
		pickingFlags |= CC_VIRTUAL_TRANS_ENABLED;

	switch(pickingMode)
	{
	case ENTITY_PICKING:
	case LABELS_PICKING:
		pickingFlags |= CC_DRAW_NAMES;
		break;
	case POINT_PICKING:
	case AUTO_POINT_PICKING:
		pickingFlags |= CC_DRAW_NAMES;
		pickingFlags |= CC_DRAW_POINT_NAMES;
		break;
	case TRIANGLE_PICKING:
		pickingFlags |= CC_DRAW_NAMES;
		pickingFlags |= CC_DRAW_TRI_NAMES;
	case NO_PICKING:
	default:
		return -1;
	}

	makeCurrent();

	//no need to clear display, we don't draw anything new!
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//selection buffer for selecting 3D entities with mouse
	memset(m_pickingBuffer,0,sizeof(GLuint)*CC_PICKING_BUFFER_SIZE);
	glSelectBuffer(CC_PICKING_BUFFER_SIZE,m_pickingBuffer);
	glRenderMode(GL_SELECT);
	glInitNames();

	//get viewport
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	//3D objects picking
	if (pickingMode != LABELS_PICKING)
	{
		context.flags = CC_DRAW_3D | pickingFlags;

		//mode selection
		glEnable(GL_DEPTH_TEST);

		//projection matrix
		glMatrixMode(GL_PROJECTION);
		//restrict drawing to a small region around the cursor (5x5 pixels)
		glLoadIdentity();
		gluPickMatrix((float)cursorX,(float)(viewport[3]-cursorY),5,5,viewport);
		glMultMatrixd(getProjectionMatd());

		//model view matrix
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixd(getModelViewMatd());

		//we display 3D objects
		m_globalDBRoot->draw(context);

		ccGLUtils::CatchGLError("ccGLWindow::startPicking.draw(3D)");
	}

	//2D objects picking
	if (pickingMode == ENTITY_PICKING || pickingMode == LABELS_PICKING)
	{
		context.flags = CC_DRAW_2D | pickingFlags;

		//Warning we must reset properly the projection matrix
		setStandardOrthoCenter();
		glMatrixMode(GL_PROJECTION);
		double orthoProjMatd[16];
		glGetDoublev(GL_PROJECTION_MATRIX, orthoProjMatd);
		glLoadIdentity();
		gluPickMatrix((float)cursorX,(float)(viewport[3]-cursorY),5,5,viewport);
		glMultMatrixd(orthoProjMatd);
		glMatrixMode(GL_MODELVIEW);

		glDisable(GL_DEPTH_TEST);

		//we display 2D objects
		m_globalDBRoot->draw(context);
		m_winDBRoot->draw(context);

		ccGLUtils::CatchGLError("ccGLWindow::startPicking.draw(2D)");
	}

	glFlush();

	// returning to normal rendering mode
	int hits = glRenderMode(GL_RENDER);
	ccConsole::PrintDebug("hits:%i",hits);

	ccGLUtils::CatchGLError("ccGLWindow::startPicking.render");

	if (hits<0)
	{
		ccConsole::Warning("Too many items inside picking zone! Try to zoom in...");
		return -1;
	}

	int selectedID=-1,pointID=-1;
	processHits(hits,selectedID,pointID);

	//standard "entity" picking
	if (pickingMode == ENTITY_PICKING)
	{
		emit entitySelectionChanged(selectedID);
		m_updateFBO=true;
	}

	//"3D point" picking
	else if (pickingMode == POINT_PICKING)
	{
		if (selectedID>=0 && pointID>=0)
		{
			emit pointPicked(selectedID,(unsigned)pointID,cursorX,cursorY);
			//TODO: m_updateFBO=true;?
		}
	}
	else if (pickingMode == AUTO_POINT_PICKING)
	{
		if (m_globalDBRoot && selectedID>=0 && pointID>=0)
		{
			//auto spawn the corresponding label
			ccHObject* obj = m_globalDBRoot->find(selectedID);
			if (obj && obj->isKindOf(CC_POINT_CLOUD))
			{
				cc2DLabel* label = new cc2DLabel();
				label->addPoint(static_cast<ccGenericPointCloud*>(obj),pointID);
				label->setVisible(true);
				label->setDisplay(obj->getDisplay());
				label->setPosition((float)(cursorX+20)/(float)width(),(float)(cursorY+20)/(float)height());
				obj->addChild(label,true);
				emit newLabel(static_cast<ccHObject*>(label));
				QApplication::processEvents();
				redraw();
			}
		}
	}

	return selectedID;
}


void ccGLWindow::processHits(GLint hits, int& entID, int& subCompID)
{
	entID = -1;		//-1 means "nothing selected"
	subCompID = -1; //-1 means "nothing selected"

	if (hits<1)
		return;

	GLuint minMinDepth = 0;
	const GLuint* _selectBuf = m_pickingBuffer;
	for (int i=0;i<hits;++i)
	{
		const GLuint& n = _selectBuf[0]; //number of names on stack: should be 1 (CC_DRAW_NAMES mode) or 2 (CC_DRAW_POINT_NAMES mode)!
		if (n>0) //strangely, we get sometimes empty sets?!
		{
			assert(n==1 || n==2);
			const GLuint& minDepth = _selectBuf[1];//(GLfloat)_selectBuf[1]/(GLfloat)0xffffffff;
			//const GLuint& maxDepth = _selectBuf[2];

			//if there is multiple hits, we keep only the nearest
			if (i == 0 || minDepth < minMinDepth)
			{
				entID = _selectBuf[3];
				if (n>1)
					subCompID = _selectBuf[4];
				minMinDepth = minDepth;
			}
		}

		_selectBuf += (3+n);
	}
}

void ccGLWindow::displayNewMessage(const QString& message,
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
			ccLog::Warning("[ccGLWindow::displayNewMessage] Append is forced for custom messages!");
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
	m_params.defaultPointSize = size;
	m_updateFBO=true;
}

void ccGLWindow::setLineWidth(float width)
{
	m_params.defaultLineWidth = width;
	m_updateFBO=true;
}

void ccGLWindow::setFontPointSize(int pixelSize)
{
	m_font.setPointSize(pixelSize);
}

int ccGLWindow::getFontPointSize() const
{
	return m_font.pointSize();
}

void ccGLWindow::glEnableSunLight()
{
	glLightfv(GL_LIGHT0,GL_DIFFUSE,ccGui::Parameters().lightDiffuseColor);
	glLightfv(GL_LIGHT0,GL_AMBIENT,ccGui::Parameters().lightAmbientColor);
	glLightfv(GL_LIGHT0,GL_SPECULAR,ccGui::Parameters().lightSpecularColor);
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
		QSettings().setValue("ccGLWindow/sunLightEnabled", m_sunLightEnabled);
	}
}

void ccGLWindow::toggleSunLight()
{
	setSunLight(!m_sunLightEnabled);
}

void ccGLWindow::glEnableCustomLight()
{
	glLightfv(GL_LIGHT1,GL_DIFFUSE,ccGui::Parameters().lightDiffuseColor);
	glLightfv(GL_LIGHT1,GL_AMBIENT,ccGui::Parameters().lightAmbientColor);
	glLightfv(GL_LIGHT1,GL_SPECULAR,ccGui::Parameters().lightSpecularColor);
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
		QSettings().setValue("ccGLWindow/customLightEnabled", m_customLightEnabled);
	}
}

void ccGLWindow::toggleCustomLight()
{
	setCustomLight(!m_customLightEnabled);
}

void ccGLWindow::displayCustomLight()
{
	glColor3ubv(ccColor::yellow);
	//we must assure that the star size is constant
	GLfloat d = CC_DISPLAYED_CUSTOM_LIGHT_LENGTH/computeTotalZoom();

	glBegin(GL_LINES);
	glVertex3f(m_customLightPos[0]-d, m_customLightPos[1],   m_customLightPos[2]);
	glVertex3f(m_customLightPos[0]+d, m_customLightPos[1],   m_customLightPos[2]);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1]-d, m_customLightPos[2]);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1]+d, m_customLightPos[2]);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1],   m_customLightPos[2]-d);
	glVertex3f(m_customLightPos[0],   m_customLightPos[1],   m_customLightPos[2]+d);
	glEnd();
}

void ccGLWindow::togglePerspective(bool objectCentered)
{
	if (m_params.objectCenteredPerspective != objectCentered)
		setPerspectiveState(true,objectCentered);
	else
		setPerspectiveState(!m_params.perspectiveView,objectCentered);
}

void ccGLWindow::setPerspectiveState(bool state, bool objectCenteredPerspective)
{
	m_params.perspectiveView=state;

	if (m_params.perspectiveView)
	{
		m_params.objectCenteredPerspective = objectCenteredPerspective;
		if (m_params.objectCenteredPerspective)
		{
			m_params.pivotPoint = m_pivotPointBackup;
			emit pivotPointChanged(m_params.pivotPoint);

			//centered perspective
			displayNewMessage("Centered perspective ON",
								ccGLWindow::LOWER_LEFT_MESSAGE,
								false,
								2,
								PERSPECTIVE_STATE_MESSAGE);
		}
		else
		{
			//viewer based perspective
			displayNewMessage("Viewer-based perspective ON",
								ccGLWindow::LOWER_LEFT_MESSAGE,
								false,
								2,
								PERSPECTIVE_STATE_MESSAGE);
			setZoom(1.0);
		}
	}
	else
	{
		m_params.pivotPoint = m_pivotPointBackup;
		emit pivotPointChanged(m_params.pivotPoint);
		m_params.aspectRatio = 1.0;

		displayNewMessage("Perspective OFF",
							ccGLWindow::LOWER_LEFT_MESSAGE,
							false,
							2,
							PERSPECTIVE_STATE_MESSAGE);
	}

	//auto-save last perspective settings
	{
		QSettings settings;
		settings.beginGroup("ccGLWindow");
		//write parameters
		settings.setValue("perspectiveView", m_params.perspectiveView);
		settings.setValue("objectCenteredPerspective", m_params.objectCenteredPerspective);
		settings.endGroup();
	}

	invalidateViewport();
	invalidateVisualization();
}

void ccGLWindow::setFov(float fov)
{
	if (m_params.fov == fov)
		return;

	//update param
	m_params.fov = fov;
	//and camera state (if perspective view is 'on')
	if (m_params.perspectiveView)
		setPerspectiveState(true, m_params.objectCenteredPerspective);
}

void ccGLWindow::setViewportParameters(const ccViewportParameters& params)
{
	ccViewportParameters oldParams = m_params;
	m_params = params;

	invalidateViewport();
	invalidateVisualization();

	emit baseViewMatChanged(m_params.baseViewMat);
	emit pivotPointChanged(m_params.pivotPoint);
	emit zoomChanged(m_params.zoom/oldParams.zoom);
	emit panChanged(m_params.screenPan[0]-oldParams.screenPan[0],m_params.screenPan[1]-oldParams.screenPan[1]);
}

void ccGLWindow::applyImageViewport(ccCalibratedImage* theImage)
{
	if (!theImage)
		return;

	//field of view (= OpenGL 'fovy' in radians)
	setFov(theImage->getFov());

	const ccGLMatrix& trans = theImage->getCameraMatrix();

	//"centre" de la caméra
	//m_pivotPointBackup = m_params.pivotPoint;
	m_params.pivotPoint = CCVector3(trans.inverse().getTranslation());
	emit pivotPointChanged(m_params.pivotPoint);

	//aspect ratio
	m_params.aspectRatio = float(theImage->getW())/float(theImage->getH());

	//on déduit la matrice de rotation à partir du vecteur orientation et de l'angle
	setView(CC_TOP_VIEW,false);
	m_params.baseViewMat=trans;
	memset(m_params.baseViewMat.getTranslation(),0,sizeof(float)*3);

	setPerspectiveState(true,false);

	displayNewMessage("Viewport applied (viewer-based perspective ON)",
						ccGLWindow::LOWER_LEFT_MESSAGE,
						false,
						2,
						PERSPECTIVE_STATE_MESSAGE);

	redraw();
}

CCVector3 ccGLWindow::computeCameraPos() const
{
	if (m_params.objectCenteredPerspective)
	{
		//we handle zoom in object-centered perspective mode by moving the camera along the viewing axis
		float globalZoomEquivalentDist = (std::min(m_glWidth,m_glHeight))/(tan(m_params.fov*CC_DEG_TO_RAD)*m_params.globalZoom);

		return m_params.pivotPoint + getBaseViewMatDir()*(globalZoomEquivalentDist/m_params.zoom);
	}

	//default
	return m_params.pivotPoint;
}

void ccGLWindow::getViewportArray(int array[])
{
	makeCurrent();
	//glMatrixMode(GL_PROJECTION);
	//glLoadMatrixd(getProjectionMatd());
	glGetIntegerv(GL_VIEWPORT, array);
}

void ccGLWindow::rotateViewMat(const ccGLMatrix& rotMat)
{
	m_params.baseViewMat = rotMat * m_params.baseViewMat;

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_params.baseViewMat);

	invalidateVisualization();
}

void ccGLWindow::updateZoom(float zoomFactor)
{
	//viewer based perspective
	if (m_params.perspectiveView && !m_params.objectCenteredPerspective)
	{
		float globalZoomEquivalentDist = std::min(m_glWidth,m_glHeight)/(tan(m_params.fov*CC_DEG_TO_RAD)*m_params.globalZoom);
		m_params.pivotPoint += getBaseViewMatDir()*(globalZoomEquivalentDist*(1.0-zoomFactor));

		emit pivotPointChanged(m_params.pivotPoint);

		invalidateViewport();
		invalidateVisualization();
	}
	else
	{
		if (zoomFactor>0.0 && zoomFactor!=1.0)
			setZoom(m_params.zoom*zoomFactor);
	}
}

void ccGLWindow::setScreenPan(float tx, float ty)
{
	m_params.screenPan[0] = tx;
	m_params.screenPan[1] = ty;

	invalidateVisualization();
}

void ccGLWindow::updateScreenPan(float dx, float dy)
{
	setScreenPan(m_params.screenPan[0] + dx/m_params.globalZoom,
		m_params.screenPan[1] + dy/m_params.globalZoom);
}

void ccGLWindow::setView(CC_VIEW_ORIENTATION orientation, bool forceRedraw/*=true*/)
{
	makeCurrent();

	m_params.baseViewMat = ccGLUtils::GenerateViewMat(orientation);

	invalidateVisualization();

	//we emit the 'baseViewMatChanged' signal
	emit baseViewMatChanged(m_params.baseViewMat);

	if (forceRedraw)
		redraw();
}

bool ccGLWindow::renderToFile(const char* filename, float zoomFactor/*=1.0*/, bool dontScaleFeatures/*=false*/)
{
	if (!filename || zoomFactor<1e-2)
		return false;

	//taille de la fenêtre courante en pixels
	int Wp = (int) ((float)width() * zoomFactor);
	int Hp = (int) ((float)height() * zoomFactor);

	QImage output(Wp,Hp,QImage::Format_ARGB32);
	GLubyte* data = output.bits();
	if (!data)
	{
		ccConsole::Error("[ccGLWindow::renderToFile] Not enough memory!");
		return false;
	}

	//we activate 'capture' mode
	m_captureModeZoomFactor = zoomFactor;
	m_captureMode = true;

	//current display parameters backup
	float _defaultPointSize = m_params.defaultPointSize;
	float _defaultLineWidth = m_params.defaultLineWidth;
	int _fontSize = getFontPointSize();

	if (!dontScaleFeatures)
	{
		//we update point size (for point clouds)
		setPointSize(_defaultPointSize*zoomFactor);
		//we update line width (for bounding-boxes, etc.)
		setLineWidth(_defaultLineWidth*zoomFactor);
		//we update font size (for text display)
		setFontPointSize((int)((float)_fontSize * zoomFactor));
	}

	bool result = false;
	if (m_fbo)
	{
		ccConsole::Print("[Render screen via FBO]");
		m_captureModeZoomFactor = 1.0f;

		ccFrameBufferObject* fbo = 0;
		ccGlFilter* filter = 0;
		if (zoomFactor == 1.0)
		{
			fbo = m_fbo;
			filter = m_activeGLFilter;
		}
		else
		{
			fbo = new ccFrameBufferObject();
			bool success=false;
			if (fbo->init(Wp,Hp))
				if (fbo->initTexture(0,GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE))
					success = fbo->initDepth(GL_CLAMP_TO_BORDER,GL_DEPTH_COMPONENT32,GL_NEAREST,GL_TEXTURE_2D);
			if (!success)
			{
				ccConsole::Error("[FBO] Initialization failed! (not enough memory?)");
				delete fbo;
				fbo=0;
			}
		}

		if (fbo)
		{
			makeCurrent();

			//on redefinit le viewport
			glViewport(0,0,Wp,Hp);

			if (m_activeGLFilter && !filter)
			{
				QString shadersPath = QApplication::applicationDirPath() + QString("/shaders");
				if (!m_activeGLFilter->init(Wp,Hp,qPrintable(shadersPath)))
				{
					ccConsole::Error("[GL Filter] GL filter can't be used during rendering (not enough memory)!");
				}
				else
				{
					filter = m_activeGLFilter;
				}
			}

			//updateZoom(zoomFactor);

			CC_DRAW_CONTEXT context;
			getContext(context);
			//context.glW = Wp;
			//context.glH = Hp;

			draw3D(context,false,fbo);

			context.flags = CC_DRAW_2D | CC_DRAW_FOREGROUND;
			if (m_interactionMode == TRANSFORM_ENTITY)		
				context.flags |= CC_VIRTUAL_TRANS_ENABLED;

			setStandardOrthoCenter();
			glDisable(GL_DEPTH_TEST);
			//As renderPixmap (or QGLPixelBuffer or QGLRenderBuffer) doesn't seem to handle shaders and/or fbo in their
			//own rendering context, we simply get the existing fbo texture and save it.
			//GLuint tex = (m_activeGLFilter ? m_activeGLFilter->getTexture() : fbo->getColorTexture(0));
			//result = ccGLUtils::SaveTextureToFile(filename,tex,Wp,Hp);

			if (filter)
			{
				//we process GL filter
				GLuint depthTex = fbo->getDepthTexture();
				GLuint colorTex = fbo->getColorTexture(0);
				filter->shade(depthTex, colorTex, zoomFactor*(m_params.perspectiveView ? 1.0 : m_params.zoom));

				ccGLUtils::CatchGLError("ccGLWindow::renderToFile/glFilter shade");

				//if render mode is ON: we only want to capture it, not to display it
				fbo->start();
				ccGLUtils::DisplayTexture2D(filter->getTexture(),context.glW,context.glH);
				//glClear(GL_DEPTH_BUFFER_BIT);
				fbo->stop();
			}

			fbo->start();

			//add color ramp!
			ccRenderingTools::DrawColorRamp(context);

			//read from fbo
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			//to avoid memory issues, we read line by line
			for (int i=0;i<Hp;++i)
				glReadPixels(0,i,Wp,1,GL_BGRA,GL_UNSIGNED_BYTE,data+(Hp-1-i)*Wp*4);
			glReadBuffer(GL_NONE);

			fbo->stop();

			if (m_fbo != fbo)
				delete fbo;
			fbo=0;

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
		ccConsole::Error("Screen capture with shader not supported!");
	}
	//if no shader or fbo --> we grab screen directly
	else
	{
		ccConsole::Print("[Render screen via QT pixmap]");

		QPixmap capture = renderPixmap(Wp,Hp);
		if (capture.width()>0 && capture.height()>0)
		{
			capture.save(filename);
			result = true;
		}
		else
		{
			ccConsole::Error("Direct screen capture failed! (not enough memory?)");
			/*ccConsole::Warning("Direct screen capture failed! Let's try with a frame buffer object");

			resizeGL(Wp,Hp);
			if (!initFBO(Wp,Hp))
			ccConsole::Warning("Couldn't create FBO ... screen capture aborted");
			else
			{
			result = ccGLUtils::SaveTextureToFile(filename,m_fbo->getColorTexture(0),Wp,Hp);
			if (!m_alwaysUseFBO)
			removeFBO();
			redraw();
			}
			//*/
		}
	}

	//resizeGL(width(),height());

	//we restore display parameters
	setPointSize(_defaultPointSize);
	setLineWidth(_defaultLineWidth);
	setFontPointSize(_fontSize);
	m_captureModeZoomFactor = 1.0f;
	m_captureMode = false;

	if (result)
		ccConsole::Print("[Snapshot] File '%s' saved! (%i x %i pixels)",filename,Wp,Hp);

	return true;
}

void ccGLWindow::removeFBO()
{
	//we "disconnect" current FBO, to avoid wrong display/errors
	//if QT tries to redraw window during object destruction
	ccFrameBufferObject* _fbo = m_fbo;
	m_fbo=0;

	if (_fbo)
		delete _fbo;
}

bool ccGLWindow::initFBO(int w, int h)
{
	//we "disconnect" current FBO, to avoid wrong display/errors
	//if QT tries to redraw window during initialization
	ccFrameBufferObject* _fbo = m_fbo;
	m_fbo=0;

	if (!_fbo)
		_fbo = new ccFrameBufferObject();

	bool success=false;
	if (_fbo->init(w,h))
	{
		if (_fbo->initTexture(0,GL_RGBA,GL_RGBA,GL_FLOAT))
			success = _fbo->initDepth(GL_CLAMP_TO_BORDER,GL_DEPTH_COMPONENT32,GL_NEAREST,GL_TEXTURE_2D);
	}

	if (!success)
	{
		ccConsole::Warning("[FBO] Initialization failed!");
		delete _fbo;
		_fbo=0;
		m_alwaysUseFBO=false;
		return false;
	}

	//ccConsole::Print("[FBO] Initialized");

	m_fbo = _fbo;
	m_updateFBO = true;

	return true;
}

void ccGLWindow::removeGLFilter()
{
	//we "disconnect" current glFilter, to avoid wrong display/errors
	//if QT tries to redraw window during object destruction
	ccGlFilter* _filter = m_activeGLFilter;
	m_activeGLFilter=0;

	if (_filter)
		delete _filter;
}

bool ccGLWindow::initGLFilter(int w, int h)
{
	if (!m_activeGLFilter)
		return false;

	//we "disconnect" current glFilter, to avoid wrong display/errors
	//if QT tries to redraw window during initialization
	ccGlFilter* _filter = m_activeGLFilter;
	m_activeGLFilter=0;

	QString shadersPath = QApplication::applicationDirPath() + QString("/shaders");
	//ccConsole::Print(QString("Shaders path: %1").arg(shadersPath));

	if (!_filter->init(w,h,qPrintable(shadersPath)))
	{
		ccConsole::Warning("[GL Filter] Initialization failed!");
		return false;
	}

	ccConsole::Print("[GL Filter] Filter initialized");

	m_activeGLFilter = _filter;

	return true;
}

bool ccGLWindow::supportOpenGLVersion(unsigned openGLVersionFlag)
{
	return (format().openGLVersionFlags() & openGLVersionFlag);
}

void ccGLWindow::display3DLabel(const QString& str, const CCVector3& pos3D, const unsigned char* rgb/*=0*/, const QFont& font/*=QFont()*/)
{
	//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
	//glColor3ubv(rgb ? rgb : ccGui::Parameters().textDefaultCol);
	const unsigned char* col = rgb ? rgb : ccGui::Parameters().textDefaultCol;
	glColor3f((float)col[0]/(float)MAX_COLOR_COMP,(float)col[1]/(float)MAX_COLOR_COMP,(float)col[2]/(float)MAX_COLOR_COMP);

	renderText(pos3D.x, pos3D.y, pos3D.z, str, font);
}

void ccGLWindow::displayText(QString text, int x, int y, bool alignRight/*=false*/, const unsigned char* rgbColor/*=0*/, const QFont& font/*=QFont()*/)
{
	makeCurrent();

	int x2 = x;
	int y2 = m_glHeight-1-y;

	if (m_captureModeZoomFactor != 1.0f)
	{
		x2 = (int)(m_captureModeZoomFactor * (float)x2);
		y2 = (int)(m_captureModeZoomFactor * (float)y2);
	}

	if (alignRight)
	{
		QFontMetrics fm(font);
		x2 -= fm.width(text);
		y2 += fm.height()/2;
	}

	//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
	const unsigned char* col = (rgbColor ? rgbColor : ccGui::Parameters().textDefaultCol);
	//glColor3ubv(col);
	glColor3f((float)col[0]/(float)MAX_COLOR_COMP,(float)col[1]/(float)MAX_COLOR_COMP,(float)col[2]/(float)MAX_COLOR_COMP);
	renderText(x2, y2, text, font);
}
