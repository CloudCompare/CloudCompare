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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#ifndef CC_GL_WIDGET_HEADER
#define CC_GL_WIDGET_HEADER

//Local
#include <ccGLWindow.h>

//Qt
#include <QWidget>

//System
#include <assert.h>

//! Container widget for ccGLWindow
class ccGLWidget : public QWidget
{
	Q_OBJECT

public:

	static ccGLWidget* Create(bool stereoMode = false, bool silentInitialization = false)
	{
		QSurfaceFormat format = QSurfaceFormat::defaultFormat();
		format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
		format.setStereo(stereoMode);

		ccGLWindow* glWindow = new ccGLWindow(&format, 0, silentInitialization);

		return new ccGLWidget(glWindow);
	}

	ccGLWidget(ccGLWindow* window, QWidget* parent = 0)
		: QWidget(parent)
		, m_layout(0)
	{
		m_layout = new QHBoxLayout(this);
		m_layout->setContentsMargins(0, 0, 0, 0);

		if (window)
		{
			setAssociatedWindow(window);
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
			return 0;
		}
	}

	void setAssociatedWindow(ccGLWindow* window)
	{
		if (window)
		{
			assert(m_layout->count() == 0);
			QWidget* container = QWidget::createWindowContainer(window);
			m_layout->addWidget(container);

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
	QHBoxLayout* m_layout;
};

#endif //CC_GL_WIDGET_HEADER
