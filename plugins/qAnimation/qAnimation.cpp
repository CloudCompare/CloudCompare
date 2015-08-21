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

//Qt
#include <QtGui>
#include <QApplication>
#include <QListWidget>

//standard includes
#include <vector>
#include <sstream>

//qCC_db
#include "qCC_db.h"

//Local
#include "qAnimationDlg.h"

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


//static information
static double g_fps             =    25.0;
static double g_default_wait    =    1.0;

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

    const ccHObject::Container& selected_entities = m_app->getSelectedEntities();

    std::vector < cc2DViewportObject * > selected_views;

    for ( ccHObject::Container::const_iterator entity_iterator = selected_entities.begin(); entity_iterator != selected_entities.end() ; ++entity_iterator )
    {
        if ( (*(entity_iterator))->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT )
        {
            selected_views.push_back ( dynamic_cast < cc2DViewportObject * > ( *(entity_iterator) ) );
        }
    }

    if ( selected_views.size() < 2 )
    {
        //m_app->dispToConsole("[qAnimation] Warning: Animation plugin requires at least two selected viewports to function!",ccMainAppInterface::WRN_CONSOLE_MESSAGE); //a warning message is displayed in the console
        m_app->dispToConsole("Animation plugin requires at least two selected viewports to function!",ccMainAppInterface::ERR_CONSOLE_MESSAGE); //an error message is displayed in the console AND an error box will pop-up!
        return;
    }

    std::stringstream loaded_message;
    loaded_message << "[qAnimation] Loaded " << selected_views.size() << " viewports.";

    m_app->dispToConsole( QString::fromStdString( loaded_message.str() ) );

    std::vector < VideoStepItem > video_steps;
    video_steps.reserve ( selected_views.size() );

    for ( unsigned int i = 0 ; i < selected_views.size() ; ++i )
    {
        VideoStepItem temp_item;
        temp_item.interpolator.setView1 ( selected_views[i] );
        temp_item.interpolator.setView2 ( selected_views[i+1] );
        temp_item.time_to_run = g_default_wait;
        temp_item.fps = g_fps;
        video_steps.push_back( temp_item );
    }

    ccGLWindow * main_window ( m_app->getActiveGLWindow() );

    if ( !main_window )
    {
        m_app->dispToConsole("Animation plugin can't access window object.",ccMainAppInterface::ERR_CONSOLE_MESSAGE); //an error message is displayed in the console AND an error box will pop-up!
        return;
    }

    qAnimationDlg videoDlg ( video_steps, main_window );

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
