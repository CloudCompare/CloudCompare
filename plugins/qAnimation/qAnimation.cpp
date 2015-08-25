//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#         COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015				   #
//#                                                                        #
//##########################################################################

#include "qAnimation.h"

//Local
#include "ViewInterpolate.h"
#include "VideoStepItem.h"
#include "qAnimationDlg.h"

//Qt
#include <QtGui>
#include <QApplication>
#include <QListWidget>

//qCC_db
#include "qCC_db.h"

//system
#include <vector>

//Default constructor: should mainly be used to initialize
//actions (pointers) and other members
qAnimation::qAnimation(QObject* parent/*=0*/)
    : QObject(parent)
    , m_action(0)
{
}

//This method should enable or disable each plugin action
//depending on the currently selected entities ('selectedEntities').
//For example: if none of the selected entities is a cloud, and your
//plugin deals only with clouds, call 'm_action->setEnabled(false)'
void qAnimation::onNewSelection(const ccHObject::Container& selectedEntities)
{
    if (m_action)
        m_action->setEnabled( !selectedEntities.empty() );
}

//This method returns all 'actions' of your plugin.
//It will be called only once, when plugin is loaded.
void qAnimation::getActions(QActionGroup& group)
{
    //default action (if it has not been already created, it's the moment to do it)
    if (!m_action)
    {
        //here we use the default plugin name, description and icon,
        //but each action can have its own!
        m_action = new QAction(getName(),this);
        m_action->setToolTip(getDescription());
        m_action->setIcon(getIcon());
        //connect appropriate signal
        connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
    }

    group.addAction(m_action);
}

//what to do when clicked.
void qAnimation::doAction()
{
    //m_app should have already been initialized by CC when plugin is loaded!
    //(--> pure internal check)
    assert(m_app);
    if (!m_app)
        return;

    /*** HERE STARTS THE ACTION ***/

    //This is how you can output messages
    m_app->dispToConsole("[qAnimation] Starting Animation plugin!",ccMainAppInterface::STD_CONSOLE_MESSAGE); //a standard message is displayed in the console

    const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

    std::vector<cc2DViewportObject*> selectedViews;
	try
	{
		for ( ccHObject::Container::const_iterator entity_iterator = selectedEntities.begin(); entity_iterator != selectedEntities.end() ; ++entity_iterator )
		{
			if ( (*(entity_iterator))->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT )
			{
				selectedViews.push_back ( static_cast<cc2DViewportObject*>(*entity_iterator) );
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		m_app->dispToConsole("Not enough memory!");
		return;
	}

    if ( selectedViews.size() < 2 )
    {
        m_app->dispToConsole("Animation plugin requires at least two selected viewports to function!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
        return;
    }

    m_app->dispToConsole( QString("[qAnimation] Loaded %1 viewports").arg(selectedViews.size()) );

	//get active GL window
    ccGLWindow* glWindow( m_app->getActiveGLWindow() );
    if ( !glWindow )
    {
        m_app->dispToConsole("No active 3D view!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
        return;
    }

    std::vector<VideoStepItem> videoSteps;
	{
		try
		{
			videoSteps.reserve( selectedViews.size() );
		}
		catch (const std::bad_alloc&)
		{
			m_app->dispToConsole("Not enough memory!");
			return;
		}

		for (size_t i=0 ; i<selectedViews.size()-1; ++i )
		{
			videoSteps.push_back( VideoStepItem(selectedViews[i], selectedViews[i+1]) );
		}
	}

    qAnimationDlg videoDlg ( videoSteps, glWindow );
    if ( !videoDlg.exec() )
    {
        return;
    }

    /*** HERE ENDS THE ACTION ***/
}

//This method should return the plugin icon (it will be used mainly
//if your plugin as several actions in which case CC will create a
//dedicated sub-menu entry with this icon.
QIcon qAnimation::getIcon() const
{
    //open qAnimation.qrc (text file), update the "prefix" and the
    //icon(s) filename(s). Then save it with the right name (yourPlugin.qrc).
    //(eventually, remove the original qAnimation.qrc file!)
    return QIcon(":/CC/plugin/qAnimation/animation.png");
}

#ifndef CC_QT5
//Don't forget to replace 'qAnimation' by your own plugin class name here also!
Q_EXPORT_PLUGIN2(qAnimation,qAnimation);
#endif
