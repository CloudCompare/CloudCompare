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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#ifndef CC_GL_WIDGET_HEADER
#define CC_GL_WIDGET_HEADER

//Local
#include <ccGLWindow.h>

//Qt
#include <QWidget>

#ifdef CC_GL_WINDOW_USE_QWINDOW

//Qt
#include <QHBoxLayout>

//System
#include <assert.h>

//! Container widget for ccGLWindow
class ccGLWidget : public QWidget
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

	static ccGLWindow* FromWidget(const QWidget* widget)
	{
		const ccGLWidget* myWidget = qobject_cast<const ccGLWidget*>(widget);
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

inline void CreateGLWindow(ccGLWindow*& window, QWidget*& widget, bool stereoMode = false, bool silentInitialization = false)
{
	QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	format.setStereo(stereoMode);

	window = new ccGLWindow(&format, nullptr, silentInitialization);

#ifdef CC_GL_WINDOW_USE_QWINDOW
	widget = new ccGLWidget(window);
#else
	widget = window;
#endif
}

inline ccGLWindow* GLWindowFromWidget(QWidget* widget)
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

#endif //CC_GL_WIDGET_HEADER
