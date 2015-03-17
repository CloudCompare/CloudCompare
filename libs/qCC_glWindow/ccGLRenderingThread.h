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

#ifndef CC_GL_RENDERING_THREAD_HEADER
#define CC_GL_RENDERING_THREAD_HEADER

//Always first
#include <ccIncludeGL.h>

//Qt
#include <QThread>

#if (QT_VERSION >= QT_VERSION_CHECK(5, 4, 0))
#define USE_RENDERING_THREAD
#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QGLContext>
#include <QAtomicInt>
#include <QOpenGLContext>
#else
class QOpenGLFramebufferObject; 
class QOpenGLContext;
#endif

class ccGLWindow;

//! Rendering thread for offline rendering
class ccGLRenderingThread : public QThread
{
	Q_OBJECT

public:
	//! Default constructor
	ccGLRenderingThread(ccGLWindow* associatedWindow);

	//! Destructor
	virtual ~ccGLRenderingThread();

	//! Inits FBO with the right size
	/** The right OpenGL context should be enabled.
	**/
	bool init(QSize size);

	//! Returns whether the thread is valid/initialized
	bool isValid() const;
	//! Returns whether a new render result is ready
	bool isReady() const;

	//! Returns the FBO texture (and cancels the 'ready' state)
	GLuint useTexture();

signals:

	//! Signal emitted when a new rendering is ready
	void ready();

protected:

	//! Kills the current rendering sequence
	void kill(unsigned timeout_ms = 1000);

	//inherited from QThread
	virtual void run();

	//! Associated GL window
	ccGLWindow* m_associatedWindow;

	//! Associated context
	QOpenGLContext* m_context;

	//! Rendering size
	QSize m_size;

#ifdef USE_RENDERING_THREAD
	QAtomicInt m_renderingReady;
	QAtomicInt m_isValid;
	QOpenGLFramebufferObject* m_renderingFBO;
    QOffscreenSurface* m_fakeSurface;
#endif
};

#endif //CC_GL_RENDERING_THREAD_HEADER
