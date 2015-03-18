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

#include "ccGLRenderingThread.h"
#include "ccGLWindow.h"

//CC_FBO_LIB
#include <ccFrameBufferObject.h>

//qCC_db
#include <ccLog.h>

ccGLRenderingThread::ccGLRenderingThread(ccGLWindow* associatedWindow)
	: QThread(associatedWindow)
	, m_associatedWindow(associatedWindow)
	, m_context(0)
	, m_fbo(0)
#ifdef USE_RENDERING_THREAD
	, m_renderingReady(0)
	, m_isValid(0)
	, m_renderingFBO(0)
	, m_fakeSurface(0)
#endif
{
#ifdef USE_RENDERING_THREAD
	// Set up the QGLContext to use for rendering in this thread. It is sharing
	// memory space with the GL context of the scene graph. This constructor is called
	// during updatePaintNode, so we are currently on the scene graph thread with the
	// scene graph's OpenGL context current.
	if (m_associatedWindow)
	{
		QOpenGLContext* context = m_associatedWindow->context()->contextHandle();

		m_context = new QOpenGLContext();
		m_context->setShareContext(context);
		m_context->setFormat(context->format());
		m_context->moveToThread(this);

		// We need a non-visible surface to make current in the other thread
		// and QWindows must be created and managed on the GUI thread.
		m_fakeSurface = new QOffscreenSurface();
		m_fakeSurface->setFormat(context->format());
		m_fakeSurface->create();
	}
#endif
}

ccGLRenderingThread::~ccGLRenderingThread()
{
	kill();
	
#ifdef USE_RENDERING_THREAD
	if (m_fakeSurface)
	{
		delete m_fakeSurface;
		m_fakeSurface = 0;
	}
#endif
}

void ccGLRenderingThread::kill(unsigned timeout_ms/*=1000*/)
{
#ifdef USE_RENDERING_THREAD
	if (isRunning() && !wait(timeout_ms))
	{
		terminate();
		wait(timeout_ms);
	}
	m_renderingReady.store(0);
#endif
}

bool ccGLRenderingThread::init(QSize size)
{
	m_size = size;
	m_isValid.store(1);
	return true;
}

bool ccGLRenderingThread::isValid() const
{
#ifdef USE_RENDERING_THREAD
	return m_isValid.load() != 0;
#else
	return false;
#endif
}

bool ccGLRenderingThread::isReady() const
{
#ifdef USE_RENDERING_THREAD
	return m_renderingReady.load() != 0;
#else
	return false;
#endif
}

GLuint ccGLRenderingThread::useTexture()
{
	GLuint tex = 0;
#ifdef USE_RENDERING_THREAD
	if (isReady())
	{
		if (m_fbo)
			tex = m_fbo->getColorTexture(0);
		else if (m_renderingFBO)
			tex = m_renderingFBO->texture();
		else
			assert(false);
		
		m_renderingReady.store(0);
	}
#endif

	return tex;
}

void ccGLRenderingThread::run()
{
#ifdef USE_RENDERING_THREAD
	m_renderingReady.store(0);
	if (	m_isValid.load() == 0
		||	!m_associatedWindow
		||	!m_context
		)
	{
		return;
	}

	m_timer.restart();

	if (!m_context->isValid())
		m_context->create();
	m_context->makeCurrent(m_fakeSurface);

	assert(QGLContext::areSharing(m_associatedWindow->context(), QGLContext::fromOpenGLContext(m_context)));

	if (!m_fbo || QSize(m_fbo->width(), m_fbo->height()) != m_size)
	{
		if (m_fbo)
			delete m_fbo;

		m_fbo = new ccFrameBufferObject();
		if (	!m_fbo->init(m_size.width(), m_size.height())
			||	!m_fbo->initTexture(0, GL_RGBA, GL_RGBA, GL_FLOAT)
			||	!m_fbo->initDepth(GL_CLAMP_TO_BORDER, GL_DEPTH_COMPONENT32, GL_NEAREST, GL_TEXTURE_2D))
		{
			delete m_fbo;
			m_fbo = 0;
			ccLog::Warning("[ccGLRenderingThread] Failed to init offline FBO!");
			m_isValid.store(0);
			return;
		}
	}
	//if (!m_renderingFBO || m_renderingFBO->size() != m_size)
	//{
	//	if (m_renderingFBO)
	//		delete m_renderingFBO;

	//	m_renderingFBO = new QOpenGLFramebufferObject(m_size, QOpenGLFramebufferObject::Depth);
	//	if (m_renderingFBO->size() != m_size)
	//	{
	//		delete m_renderingFBO;
	//		m_renderingFBO = 0;
	//		ccLog::Warning("[ccGLRenderingThread] Failed to init offline FBO!");
	//		m_isValid.store(0);
	//		return;
	//	}
	//}

	//m_renderingFBO->bind();
	m_fbo->start();
	//render
	bool drawCross = m_associatedWindow->crossShouldBeDrawn();
	CC_DRAW_CONTEXT CONTEXT;
	m_associatedWindow->getContext(CONTEXT);

	m_associatedWindow->draw3D(CONTEXT, drawCross, QGLContext::fromOpenGLContext(m_context));
	glFlush();

	//m_renderingFBO->release();
	m_fbo->stop();

	qint64 duration_ms = m_timer.elapsed();
	if (duration_ms < 300)
		wait(300 - duration_ms);

	m_renderingReady.store(1);
	emit ready();
#endif
}
