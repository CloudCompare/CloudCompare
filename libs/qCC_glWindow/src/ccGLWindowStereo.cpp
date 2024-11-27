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
#include "ccGLWindowStereo.h"

//qCC_db
#include <ccHObject.h>

//CCFbo
#include <ccFrameBufferObject.h>

//Qt
#include <QMessageBox>
#include <QResizeEvent>
#include <QOpenGLPaintDevice>

#ifdef CC_OCULUS_SUPPORT
#include "oculus/ccOculus.h"
static OculusHMD s_oculus;
#endif //CC_OCULUS_SUPPORT

ccGLWindowStereo::ccGLWindowStereo(	QSurfaceFormat* format/*=nullptr*/,
									QWindow* parent/*=nullptr*/,
									bool silentInitialization/*=false*/)
	: QWindow(parent)
	, ccGLWindowInterface(this, silentInitialization)
	, m_context(nullptr)
	, m_parentWidget(nullptr)
{
	setSurfaceType(QWindow::OpenGLSurface);

	m_format = format ? *format : requestedFormat();

	//default picking mode
	setPickingMode(DEFAULT_PICKING);

	//default interaction mode
	setInteractionMode(MODE_TRANSFORM_CAMERA);

	//signal/slot connections
	connect(m_signalEmitter, &ccGLWindowSignalEmitter::itemPickedFast, this, &ccGLWindowStereo::onItemPickedFastSlot, Qt::DirectConnection);
	connect(&m_scheduleTimer, &QTimer::timeout, [&]() { checkScheduledRedraw(); });
	connect(&m_autoRefreshTimer, &QTimer::timeout, this, [&]() { update(); });
	connect(&m_deferredPickingTimer, &QTimer::timeout, this, [&]() { doPicking(); });

	QString windowTitle = QString("3D View Stereo %1").arg(m_uniqueID);
	setWindowTitle(windowTitle);
	setObjectName(windowTitle);
}

ccGLWindowStereo::~ccGLWindowStereo()
{
	//disable the stereo mode (mainly to release the Oculus FBO ;)
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
		//drag & drop handling
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
	if (	!m_stereoModeEnabled
		||	m_stereoParams.glassType != StereoParams::OCULUS
#ifdef CC_OCULUS_SUPPORT
		||	s_oculus.mirror.texture
#endif
		)
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

bool ccGLWindowStereo::prepareOtherStereoGlassType(	CC_DRAW_CONTEXT& CONTEXT,
												RenderingParams& renderingParams,
												ccFrameBufferObject*& currentFBO)
{
#ifdef CC_OCULUS_SUPPORT
	if (m_stereoParams.glassType == StereoParams::OCULUS && s_oculus.session)
	{
		renderingParams.useFBO = true;
		renderingParams.drawBackground = (CONTEXT.currentLODLevel == 0);
		renderingParams.draw3DPass = true;
		currentFBO = s_oculus.fbo;

		if (renderingParams.pass == MONO_OR_LEFT_RENDERING_PASS)
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

			//s_oculus.fbo->setDrawBuffer(renderingParams.pass);
			const ovrRecti& vp = s_oculus.layer.Viewport[renderingParams.pass];
			setGLViewport(vp.Pos.x, vp.Pos.y, vp.Size.w, vp.Size.h);
			CONTEXT.glW = vp.Size.w;
			CONTEXT.glH = vp.Size.h;

			bindFBO(nullptr);
		}

		return true;
	}
#endif //CC_OCULUS_SUPPORT
	return false;
}

void ccGLWindowStereo::processOtherStereoGlassType(RenderingParams& renderingParams)
{
#ifdef CC_OCULUS_SUPPORT
	if (	m_stereoModeEnabled
		&&	m_stereoParams.glassType == StereoParams::OCULUS
		&&	s_oculus.session
		&&	renderingParams.pass == RIGHT_RENDERING_PASS)
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
}

bool ccGLWindowStereo::setCustomCameraProjection(RenderingParams& renderingParams, ccGLMatrixd& modelViewMat, ccGLMatrixd& projectionMat)
{
#ifdef CC_OCULUS_SUPPORT
	if (m_stereoParams.glassType == StereoParams::OCULUS && s_oculus.session)
	{
		//we use the standard modelview matrix
		modelViewMat = getModelViewMatrix();

		if (s_oculus.hasLastOVRPos)
		{
			s_oculus.layer.RenderPose[renderingParams.pass].Position;
			OVR::Quatf q(s_oculus.layer.RenderPose[renderingParams.pass].Orientation);
			OVR::Matrix4f sensorRot(q);

			ccGLMatrixd sensorMat = FromOVRMat(sensorRot);
			const ovrVector3f& P = s_oculus.layer.RenderPose[renderingParams.pass].Position;
			sensorMat.setTranslation(sensorMat.getTranslationAsVec3D() + CCVector3d(P.x, P.y, P.z));

			modelViewMat = sensorMat.inverse() * modelViewMat;
		}

		//projection matrix
		m_viewportParams.zNear = 0.001;
		m_viewportParams.zFar = 1000.0;
		OVR::Matrix4f proj = ovrMatrix4f_Projection(s_oculus.layer.Fov[renderingParams.pass],
													static_cast<float>(m_viewportParams.zNear),
													static_cast<float>(m_viewportParams.zFar),
													ovrProjection_ClipRangeOpenGL);
		projectionMat = FromOVRMat(proj);

		return true;
	}
#endif //CC_OCULUS_SUPPORT
	return false;
}

bool ccGLWindowStereo::enableStereoMode(const StereoParams& params)
{
	if (params.glassType == StereoParams::OCULUS)
	{
#ifdef CC_OCULUS_SUPPORT
		if (!s_oculus.session)
		{
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
			//ovr_ConfigureTracking(s_oculus.session,
			//						/*requested = */ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position,
			//						/*required  = */ovrTrackingCap_Orientation );

			//reset tracking
			s_oculus.hasLastOVRPos = false;
			ovr_RecenterTrackingOrigin(s_oculus.session);
		}

		displayNewMessage("Look into your headset", ccGLWindowStereo::SCREEN_CENTER_MESSAGE, false, 3600);
		//force the screen update before we freeze it! (see doPaintGL)
		update();

		return ccGLWindowInterface::enableStereoMode(params, false, true);

#else //no CC_OCULUS_SUPPORT

		QMessageBox::critical(asWidget(), "Oculus", "The Oculus device is not supported by this version\n(use the 'Stereo' version)");
		return false;
#endif
	}
	else
	{
		return ccGLWindowInterface::enableStereoMode(params);
	}
}

void ccGLWindowStereo::disableStereoMode()
{
#ifdef CC_OCULUS_SUPPORT
	if (m_stereoModeEnabled)
	{
		if (m_stereoParams.glassType == StereoParams::OCULUS)
		{
			toggleAutoRefresh(false);
			displayNewMessage(QString(), ccGLWindowStereo::SCREEN_CENTER_MESSAGE, false);

			if (s_oculus.session)
			{
				if (m_glExtFuncSupported)
				{
					s_oculus.releaseMirrorTexture(m_glExtFunc);
				}

				s_oculus.stop(false);
			}
		}
	}
#endif

	ccGLWindowInterface::disableStereoMode();
}

void ccGLWindowStereo::Create(ccGLWindowStereo*& window, QWidget*& widget, bool silentInitialization/*=false*/)
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
