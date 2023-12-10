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

// Qt
#include <QOpenGLWidget>

//! OpenGL 3D view
class CCGLWINDOW_LIB_API ccGLWindow : public QOpenGLWidget, public ccGLWindowInterface
{
	Q_OBJECT

public:

	//! Default constructor
	ccGLWindow(QSurfaceFormat* format = nullptr, QOpenGLWidget* parent = nullptr, bool silentInitialization = false);

	//! Destructor
	~ccGLWindow() override;

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
	inline QImage doGrabFramebuffer() override { return grabFramebuffer(); }
	inline bool isStereo() const override { return false; }

	inline QWidget* asWidget() override { return this; }
	inline QSize getScreenSize() const override { return size(); }

	//inherited from ccGLWindowInterface
	inline int qtWidth() const override { return QOpenGLWidget::width(); }
	inline int qtHeight() const override { return QOpenGLWidget::height(); }
	inline QSize qtSize() const override { return QOpenGLWidget::size(); }
	bool enableStereoMode(const StereoParams& params) override;

	//inherited from ccGenericGLDisplay
	void requestUpdate() override;

	//! Creates an instance
	static void Create(ccGLWindow*& window, QWidget*& widget, bool silentInitialization = false);

	//! Casts a widget to a ccGLWindow instance (if possible)
	static ccGLWindow* FromWidget(QWidget* widget);

protected: //rendering

	//inherited from ccGLWindowInterface
	inline ccQOpenGLFunctions* functions() const override { return context() ? context()->versionFunctions<ccQOpenGLFunctions>() : nullptr; }
	inline QSurfaceFormat getSurfaceFormat() const override { return format(); }
	inline void doSetMouseTracking(bool enable) override { setMouseTracking(true); }
	inline void doShowFullScreen() override { showFullScreen(); }
	inline void doShowNormal() override { showNormal(); }
	
	//! Don't call makeCurrent() on this instance, as the FBO would not properly be managed
	/** Use doMakeCurrent() instaad.
	**/
	void makeCurrent() = delete;

protected: //other methods

	//! Reacts to the itemPickedFast signal (shortcut)
	Q_SLOT void onItemPickedFastSlot(ccHObject* pickedEntity, int pickedItemIndex, int x, int y) { onItemPickedFast(pickedEntity, pickedItemIndex, x, y); }

	//inherited from ccGLWindowInterface
	int width() const override { return QOpenGLWidget::width(); }
	int height() const override { return QOpenGLWidget::height(); }
	QSize size() const override { return QOpenGLWidget::size(); }
	GLuint defaultQtFBO() const override;

	//events handling
	void mousePressEvent(QMouseEvent* event) override { processMousePressEvent(event); }
	void mouseMoveEvent(QMouseEvent* event) override { processMouseMoveEvent(event); }
	void mouseDoubleClickEvent(QMouseEvent* event) override { processMouseDoubleClickEvent(event); }
	void mouseReleaseEvent(QMouseEvent* event) override { processMouseReleaseEvent(event); }
	void wheelEvent(QWheelEvent* event) override { processWheelEvent(event); }
	bool event(QEvent* evt) override;

	void initializeGL() override { initialize(); }
	void resizeGL(int w, int h) override { onResizeGL(w, h); }
	void paintGL() override { doPaintGL(); }
	void dragEnterEvent(QDragEnterEvent* event) override { doDragEnterEvent(event); }
	void dropEvent(QDropEvent* event) override { doDropEvent(event); }
};
