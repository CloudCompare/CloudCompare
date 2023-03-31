#pragma once
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

#include "qCC_glWindow.h"

//local
#include "ccGLWindowInterface.h"

#ifdef CC_GL_WINDOW_USE_QWINDOW
#include <QHBoxLayout>
#include <QWidget>
#include <QWindow>
#else
#include <QOpenGLWidget>
#endif

#ifdef CC_GL_WINDOW_USE_QWINDOW
class QOpenGLPaintDevice;
using ccGLWindowParent = QWindow;
#else
using ccGLWindowParent = QOpenGLWidget;
#endif

//! OpenGL 3D view
class CCGLWINDOW_LIB_API ccGLWindow : public ccGLWindowParent, public ccGLWindowInterface
{
	Q_OBJECT

public:

	//! Default constructor
	ccGLWindow(QSurfaceFormat* format = nullptr, ccGLWindowParent* parent = nullptr, bool silentInitialization = false);

	//! Destructor
	~ccGLWindow() override;

	//inherited from ccGLWindowInterface
	inline qreal getDevicePixelRatio() const override { return devicePixelRatio(); }
	inline QFont getFont() const override { return font(); }
	inline QOpenGLContext* getOpenGLContext() const override { return context(); }
	inline void setWindowCursor(const QCursor& cursor) override { setCursor(cursor); }
	inline void doMakeCurrent() override { makeCurrent();  }
	inline QObject* asQObject() override { return this; }
	inline const QObject* asQObject() const override { return this; }
	inline QString getWindowTitle() const override { return windowTitle(); }
	inline void doGrabMouse() override { grabMouse(); }
	inline void doReleaseMouse() override { releaseMouse(); }
	inline QPoint doMapFromGlobal(const QPoint& P) const override { return mapFromGlobal(P); }
	inline void doShowMaximized() override { showMaximized(); }
	inline void doResize(int w, int h) override { resize(w, h); }
	inline void doResize(const QSize& size) override { resize(size); }

#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! Returns the parent widget
	QWidget* parentWidget() const { return m_parentWidget; }

	//! Sets 'parent' widget
	void setParentWidget(QWidget* widget);

	//! Returns the font
	inline const QFont& font() const { return m_font; }

	//shortcuts
	void setWindowTitle(QString title) { setTitle(title); }
	QString windowTitle() const { return title(); }
#endif

	//inherited from ccGenericGLDisplay
	void refresh(bool only2D = false) override;
#ifdef CC_GL_WINDOW_USE_QWINDOW
	inline QWidget* asWidget() override { return m_parentWidget; }
#else
	inline QWidget* asWidget() override { return this; }
#endif
	inline QSize getScreenSize() const override { return size(); }

	//inherited from ccGLWindowInterface
	void setInteractionMode(INTERACTION_FLAGS flags) override;
	QImage renderToImage(	float zoomFactor = 1.0f,
							bool dontScaleFeatures = false,
							bool renderOverlayItems = false,
							bool silent = false) override;
	inline int qtWidth() const override { return ccGLWindow::width(); }
	inline int qtHeight() const override { return ccGLWindow::height(); }
	inline QSize qtSize() const override { return ccGLWindowParent::size(); }
	void toggleExclusiveFullScreen(bool state) override;
	bool enableStereoMode(const StereoParams& params) override;
	void disableStereoMode() override;

public:

	//inherited from ccGenericGLDisplay
	void redraw(bool only2D = false, bool resetLOD = true) override;
	void startFrameRateTest() override;
	void requestUpdate() override;
	
#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! For compatibility with the QOpenGLWidget version
	inline void update() { paintGL(); }

	void grabMouse();
	void releaseMouse();
#endif

	static void Create(ccGLWindow*& window, QWidget*& widget, bool stereoMode = false, bool silentInitialization = false);

	static ccGLWindow* FromWidget(QWidget* widget);

protected:

#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! Updates the display
	void paintGL();
#endif

	//! Stops frame rate test
	void stopFrameRateTest();

protected: //rendering

	//! Sets the OpenGL viewport
	void setGLViewport(const QRect& rect);
	//! Sets the OpenGL viewport (shortut)
	inline void setGLViewport(int x, int y, int w, int h) { setGLViewport(QRect(x, y, w, h)); }

	//inherited from ccGLWindowInterface
	inline ccQOpenGLFunctions* functions() const override { return context() ? context()->versionFunctions<ccQOpenGLFunctions>() : nullptr; }

#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! Returns the context (if any)
	inline QOpenGLContext* context() const { return m_context; }
#endif

	//reimplemented from QOpenGLWidget
	//Because QOpenGLWidget::makeCurrent silently binds the widget's own FBO,
	//we need to automatically bind our own afterwards!
	//(Sadly QOpenGLWidget::makeCurrentmakeCurrent is not virtual)
	void makeCurrent();

	//! Rendering params
	struct RenderingParams;

	//! Full rendering pass (drawBackground + draw3D + drawForeground)
	void fullRenderingPass(CC_DRAW_CONTEXT& context, RenderingParams& params);

	//! Draws the background layer
	/** Background + 2D background objects
	**/
	void drawBackground(CC_DRAW_CONTEXT& context, RenderingParams& params);

	//! Draws the main 3D layer
	void draw3D(CC_DRAW_CONTEXT& context, RenderingParams& params);

	//! Draws the foreground layer
	/** 2D foreground objects / text
	**/
	void drawForeground(CC_DRAW_CONTEXT& context, RenderingParams& params);

	//! Renders the next L.O.D. level
	void renderNextLODLevel();

protected: //other methods

	//! Reacts to the itemPickedFast signal
	Q_SLOT void onItemPickedFast(ccHObject* pickedEntity, int pickedItemIndex, int x, int y);

	//! Checks for scheduled redraw
	Q_SLOT void checkScheduledRedraw();

	//! Performs standard picking at the last clicked mouse position (see m_lastMousePos)
	Q_SLOT void doPicking();

	//inherited from ccGLWindowInterface
	int width() const override { return ccGLWindowParent::width(); }
	int height() const override { return ccGLWindowParent::height(); }
	QSize size() const override { return ccGLWindowParent::size(); }
	GLuint defaultQtFBO() const override;
	bool initFBO(int w, int h) override;

	//events handling
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseDoubleClickEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	bool event(QEvent* evt) override;

	bool initialize();

	//! OpenGL KHR debug log
	void handleLoggedMessage(const QOpenGLDebugMessage&);


#ifdef CC_GL_WINDOW_USE_QWINDOW
	void resizeGL(int w, int h);
	virtual void dragEnterEvent(QDragEnterEvent* event);
	virtual void dropEvent(QDropEvent* event);
#else
	void initializeGL() override { initialize(); }
	void resizeGL(int w, int h) override;
	void paintGL() override;
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dropEvent(QDropEvent* event) override;
#endif

	//! Draws pivot point symbol in 3D
	void drawPivot();

	// Releases all textures, GL lists, etc.
	void uninitializeGL();

protected: //members

#ifdef CC_GL_WINDOW_USE_QWINDOW

	//! Associated OpenGL context
	QOpenGLContext* m_context;

	//! OpenGL device
	QOpenGLPaintDevice* m_device;

	//! Format
	QSurfaceFormat m_format;

	//! Associated widget (we use the WidgetContainer mechanism)
	QWidget* m_parentWidget;

#endif
};

#ifdef CC_GL_WINDOW_USE_QWINDOW

//! Container widget for ccGLWindow
class CCGLWINDOW_LIB_API ccGLWidget : public QWidget
{
	Q_OBJECT

public:

	ccGLWidget(ccGLWindow* window, QWidget* parent = nullptr)
		: QWidget(parent)
	{
		setLayout(new QHBoxLayout);
		layout()->setContentsMargins(0, 0, 0, 0);

		if (window)
		{
			setAssociatedWindow(window);
		}
	}

	virtual ~ccGLWidget()
	{
		if (m_associatedWindow)
		{
			m_associatedWindow->setParent(nullptr);
			m_associatedWindow->close();
		}
	}

	inline ccGLWindow* associatedWindow() const { return m_associatedWindow; }

	void setAssociatedWindow(ccGLWindow* window)
	{
		if (window)
		{
			assert(layout() && layout()->count() == 0);
			QWidget* container = QWidget::createWindowContainer(window, this);
			layout()->addWidget(container);

			m_associatedWindow = window;
			m_associatedWindow->setParentWidget(container);
		}
		else
		{
			assert(false);
		}
	}

protected:

	ccGLWindow* m_associatedWindow;
};

#endif

