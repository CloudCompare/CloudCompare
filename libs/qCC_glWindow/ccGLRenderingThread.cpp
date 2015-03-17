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

//qCC_db
#include <ccLog.h>

ccGLRenderingThread::ccGLRenderingThread(ccGLWindow* associatedWindow)
	: QThread(associatedWindow)
	, m_associatedWindow(associatedWindow)
	, m_context(0)
#ifdef USE_RENDERING_THREAD
	, m_renderingInProgress(0)
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
	if (m_associatedWindow && associatedWindow->getOpenGLContext())
	{
		QOpenGLContext* context = associatedWindow->getOpenGLContext();

		m_context = new QOpenGLContext(m_associatedWindow);
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
	m_renderingInProgress.store(0);
#endif
}

bool ccGLRenderingThread::init(QSize size)
{
#ifdef USE_RENDERING_THREAD
	m_isValid.store(0);
	m_renderingReady.store(0);

	if (!m_context)
		return false;

	if (m_renderingFBO && m_renderingFBO->size() != size)
	{
		kill();
		delete m_renderingFBO;
		m_renderingFBO = 0;
	}
	
	//m_associatedWindow->makeCurrent();
	m_context->makeCurrent(m_fakeSurface);
	m_renderingFBO = new QOpenGLFramebufferObject(size,QOpenGLFramebufferObject::Depth);
	if (m_renderingFBO->size() != size)
	{
		delete m_renderingFBO;
		m_renderingFBO = 0;
		ccLog::Warning("[ccGLRenderingThread] Failed to init offline FBO!");
		return false;
	}

	m_isValid.store(1);
	return true;
#else
	return false;
#endif
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
		if (m_renderingFBO)
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
		||	!m_renderingFBO
		||	!m_context
		)
	{
		return;
	}

	m_renderingInProgress.store(1);

	//render
	bool drawCross = m_associatedWindow->crossShouldBeDrawn();
	CC_DRAW_CONTEXT context;
	m_associatedWindow->getContext(context);

	if (!m_context->isValid())
		m_context->create();
	m_context->makeCurrent(m_fakeSurface);

	m_renderingFBO->bind();

	m_associatedWindow->draw3D(context,drawCross,QGLContext::fromOpenGLContext(m_context));
	glFlush();

	m_renderingFBO->bindDefault();

	m_renderingInProgress.store(0);
	m_renderingReady.store(1);
	emit ready();
#endif
}
