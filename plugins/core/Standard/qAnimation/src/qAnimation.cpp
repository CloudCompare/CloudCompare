//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#             COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015              #
//#                                                                        #
//##########################################################################

#include "qAnimation.h"

//Local
#include "qAnimationDlg.h"
#include "ExtendedViewport.h"

//qCC_db
#include <cc2DViewportObject.h>
#include <ccPolyline.h>
#include <ccPointCloud.h>

//qCC_gl
#include <ccGLWindowInterface.h>

//Qt
#include <QtGui>
#include <QMainWindow>

typedef std::vector<ExtendedViewport> ViewPortList;

static ViewPortList GetSelectedViewPorts( const ccHObject::Container &selectedEntities )
{
	ViewPortList viewports;
	
	for ( ccHObject *object : selectedEntities )
	{
		if ( object->getClassID() == static_cast<CC_CLASS_ENUM>(CC_TYPES::VIEWPORT_2D_OBJECT) )
		{
			viewports.push_back( static_cast<cc2DViewportObject*>(object) );
		}
	}
	
	return viewports;
}

qAnimation::qAnimation(QObject* parent)
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qAnimation/info.json" )
	, m_action( nullptr )
{
}

void qAnimation::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	ViewPortList viewports = GetSelectedViewPorts( selectedEntities );
	
	if ( viewports.size() >= 2 )
	{
		m_action->setEnabled( true );
		m_action->setToolTip( getDescription() );
	}
	else
	{
		m_action->setEnabled( false );
		m_action->setToolTip( tr( "%1\nAt least 2 viewports must be selected.").arg( getDescription() ) );
	}
}

QList<QAction *> qAnimation::getActions()
{
	//default action (if it has not been already created, it's the moment to do it)
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());

		connect(m_action, &QAction::triggered, this, &qAnimation::doAction);
	}

	return QList<QAction *>{ m_action };
}

//what to do when clicked.
void qAnimation::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> purely internal check)
	if (nullptr == m_app)
	{
		assert(false);
		return;
	}

	//get active GL window
	ccGLWindowInterface* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		m_app->dispToConsole("No active 3D view!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	// backup the view parameters before running the plugin
	ExtendedViewportParameters evp(glWindow->getViewportParameters());
	evp.customLightEnabled = glWindow->customLightEnabled();
	evp.customLightPos = glWindow->getCustomLightPosition();

	ViewPortList viewports = GetSelectedViewPorts( m_app->getSelectedEntities() );

	Q_ASSERT( viewports.size() >= 2 ); // action will not be active unless we have at least 2 viewports

	m_app->dispToConsole(QString("[qAnimation] Selected viewports: %1").arg(viewports.size()));

	qAnimationDlg videoDlg(glWindow, m_app->getMainWindow());
	
	if (!videoDlg.init(viewports))
	{
		m_app->dispToConsole("Failed to initialize the plugin dialog (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	
	videoDlg.exec();

	// restore the view parameters after running the plugin
	glWindow->setViewportParameters(evp.params);
	glWindow->setCustomLight(evp.customLightEnabled);
	glWindow->setCustomLightPosition(evp.customLightPos);
	glWindow->redraw();

	//Export trajectory (for debug)
	if (videoDlg.exportTrajectoryOnExit())
	{
		ccPolyline* trajectory = videoDlg.getTrajectory();
		if (trajectory)
		{
			trajectory->setTempColor(ccColor::red);
			trajectory->setWidth(2);
			//trajectory->prepareDisplayForRefresh();

			getMainAppInterface()->addToDB(trajectory);
		}
	}
}
