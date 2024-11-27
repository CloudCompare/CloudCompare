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

#include <QHBoxLayout>
#include <QWidget>
#include <QWindow>

class QOpenGLPaintDevice;

//! OpenGL 3D view
class CCGLWINDOW_LIB_API ccGLWindowStereo : public QWindow, public ccGLWindowInterface
{
	Q_OBJECT

public:

	//! Default constructor
	ccGLWindowStereo(QSurfaceFormat* format = nullptr, QWindow* parent = nullptr, bool silentInitialization = false);

	//! Destructor
	~ccGLWindowStereo() override;

	//inherited from ccGLWindowInterface
	inline qreal getDevicePixelRatio() const override { return devicePixelRatio(); }
	inline QFont getFont() const override { return font(); }
	inline QOpenGLContext* getOpenGLContext() const override { return context(); }
	inline void setWindowCursor(const QCursor& cursor) override { setCursor(cursor); }
	void doMakeCurrent() override;
	inline QObject* asQObject() override { return this; }
	inline const QObject* asQObject() const override { return this; }
	inline QString getWindowTitle() const override { return windowTitle(); }
	inline void doGrabMouse() override { grabMouse(); }
	inline void doReleaseMouse() override { releaseMouse(); }
	inline QPoint doMapFromGlobal(const QPoint& P) const override { return mapFromGlobal(P); }
	inline void doShowMaximized() override { showMaximized(); }
	inline void doResize(int w, int h) override { resize(w, h); }
	inline void doResize(const QSize& size) override { resize(size); }
	inline QImage doGrabFramebuffer() override { return {}; }
	inline bool isStereo() const override { return true; }

	//! Returns the parent widget
	QWidget* parentWidget() const { return m_parentWidget; }

	//! Sets 'parent' widget
	void setParentWidget(QWidget* widget);

	//! Returns the font
	inline const QFont& font() const { return m_font; }

	//shortcuts
	void setWindowTitle(QString title) { setTitle(title); }
	QString windowTitle() const { return title(); }
	inline QWidget* asWidget() override { return m_parentWidget; }
	inline QSize getScreenSize() const override { return size(); }

	//inherited from ccGLWindowInterface
	inline int qtWidth() const override { return QWindow::width(); }
	inline int qtHeight() const override { return QWindow::height(); }
	inline QSize qtSize() const override { return QWindow::size(); }
	bool enableStereoMode(const StereoParams& params) override;
	void disableStereoMode() override;

	//inherited from ccGenericGLDisplay
	void requestUpdate() override;
	
	//! For compatibility with the QOpenGLWidget version
	inline void update() { doPaintGL(); }

	void grabMouse();
	void releaseMouse();

	//! Creates an instance
	static void Create(ccGLWindowStereo*& window, QWidget*& widget, bool silentInitialization = false);

	//! Casts a widget to a ccGLWindowStereo instance (if possible)
	static ccGLWindowStereo* FromWidget(QWidget* widget);

protected: //rendering

	//inherited from ccGLWindowInterface
	inline ccQOpenGLFunctions* functions() const override { return context() ? context()->versionFunctions<ccQOpenGLFunctions>() : nullptr; }
	inline QSurfaceFormat getSurfaceFormat() const override { return format(); }
	void doSetMouseTracking(bool enable) override;
	void doShowFullScreen() override { showFullScreen(); }
	void doShowNormal() override { showNormal(); }
	bool prepareOtherStereoGlassType(CC_DRAW_CONTEXT& context, RenderingParams& params, ccFrameBufferObject*& currentFBO) override;
	void processOtherStereoGlassType(RenderingParams& renderingParams) override;
	bool setCustomCameraProjection(RenderingParams& params, ccGLMatrixd& modelViewMat, ccGLMatrixd& projectionMat) override;
	bool initPaintGL() override;
	void swapGLBuffers() override;

	//! Returns the context (if any)
	inline QOpenGLContext* context() const { return m_context; }

	//! Don't call makeCurrent() on this instance, as the FBO would not properly be managed
	/** Use doMakeCurrent() instaad.
	**/
	void makeCurrent() = delete;

protected: //other methods

	//! Reacts to the itemPickedFast signal (shortcut)
	Q_SLOT void onItemPickedFastSlot(ccHObject* pickedEntity, int pickedItemIndex, int x, int y) { onItemPickedFast(pickedEntity, pickedItemIndex, x, y); }

	//inherited from ccGLWindowInterface
	int width() const override { return QWindow::width(); }
	int height() const override { return QWindow::height(); }
	QSize size() const override { return QWindow::size(); }
	GLuint defaultQtFBO() const override;

	//events handling
	void mousePressEvent(QMouseEvent* event) override { processMousePressEvent(event); }
	void mouseMoveEvent(QMouseEvent* event) override { processMouseMoveEvent(event); }
	void mouseDoubleClickEvent(QMouseEvent* event) override { processMouseDoubleClickEvent(event); }
	void mouseReleaseEvent(QMouseEvent* event) override { processMouseReleaseEvent(event); }
	void wheelEvent(QWheelEvent* event) override { processWheelEvent(event); }
	bool event(QEvent* evt) override;

	bool preInitialize(bool &firstTime) override;
	bool postInitialize(bool firstTime) override;

	void resizeGL(int w, int h);
	virtual void dragEnterEvent(QDragEnterEvent* event) { doDragEnterEvent(event); }
	virtual void dropEvent(QDropEvent* event) { doDropEvent(event); }

protected: //members

	//! Associated OpenGL context
	QOpenGLContext* m_context;

	//! Format
	QSurfaceFormat m_format;

	//! Associated widget (we use the WidgetContainer mechanism)
	QWidget* m_parentWidget;
};

//! Container widget for ccGLWindow
class CCGLWINDOW_LIB_API ccGLStereoWidget : public QWidget
{
	Q_OBJECT

public:

	ccGLStereoWidget(ccGLWindowStereo* window, QWidget* parent = nullptr)
		: QWidget(parent)
		, m_associatedWindow(nullptr)
	{
		setLayout(new QHBoxLayout);
		layout()->setContentsMargins(0, 0, 0, 0);

		if (window)
		{
			setAssociatedWindow(window);
		}
	}

	virtual ~ccGLStereoWidget()
	{
		if (m_associatedWindow)
		{
			m_associatedWindow->setParent(nullptr);
			m_associatedWindow->close();
		}
	}

	inline ccGLWindowStereo* associatedWindow() const { return m_associatedWindow; }

	void setAssociatedWindow(ccGLWindowStereo* window)
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

	ccGLWindowStereo* m_associatedWindow;
};
