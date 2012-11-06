//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qDummy                      #
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
//#               COPYRIGHT: XXX                                           #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//

#include <QtGui>

//First: replace all occurences of 'qDummyPlugin' by your own plugin class name in this file!
#include "qDummyPlugin.h"

//This method should return your plugin description
void qDummyPlugin::getDescription(ccPluginDescription& desc)
{
    //Your plugin name
    strcpy(desc.name,"Dummy Plugin");
    //A short description of your plugin action
    strcpy(desc.menuName,"Dummy Action");
    //Tells whether your plugin provides its own icon (see 'getIcon' below)
    //Otherwise, a default plugin icon is displayed
    desc.hasAnIcon=false;
    //Your plugin version
    desc.version=1;
}

//This method should returns whether the plugin should be enabled or not,
//regarding the currently selected entities ('selectedEntities').
//For example: if none of the selected entities is a cloud, and your
//plugin deals only with clouds -> return false;
bool qDummyPlugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
    return true;
}

//This is where all the action takes place. This method is called when
//the user clicks on the plugin icon. To do this, you can use any of the
//currently selected entities (selectedEntities). The second parameters let
//you request for UI modifications (at the end of the process - such as a
//refresh of the 3D windows, or the objects browser, etc. - see the top part
//of 'ccStdPluginInterface.h'). Eventually, provided as input are a progress
//bar (progressCb) that you can freely init, display and update during the
//process, and the parent widget (if you wish to display a dialog relatively
//to it, etc.). It should be theoretically qCC main window.
int qDummyPlugin::doAction(ccHObject::Container& selectedEntities,
                            unsigned& uiModificationFlags,
                            ccProgressDialog* progressCb,
                            QWidget* parent)
{
    //by default, no modification is necessary (update these flags if necessary)
    uiModificationFlags=0;

    /*** HERE STARTS THE MAIN PLUGIN ACTION ***/

    //put your code here
    //--> you may want to start by asking parameters (with a custom dialog, etc.)
    //and then do your stuffs...

    /*** HERE ENDS THE MAIN PLUGIN ACTION ***/

    //default return code: '1' = success
    return 1;
    //otherwise, define you own return codes (<0 or >1) and put the corresponding
    //error messages in 'getErrorMessage' (see below)
}

//This method should returns your custom error messages (they will be
//displayed in the main qCC interface). This lets you manage your own
//error codes in 'doAction' (apart from '0' and '1' which are reserved).
QString qDummyPlugin::getErrorMessage(int errorCode/*, LANGUAGE lang*/)
{
    QString errorMsg;
    switch(errorCode)
    {
        //Example
        /*case 2:
            errorMsg=QString("Input cloud is too small!");
            break;
        **/
        default:
            errorMsg=QString("Undefined error!");
            break;
    }
    return errorMsg;
}

//This method should return the plugin icon (it will be displayed
//in the plugin toolbar) if you have declared that your plugin
//provides its own icon in the 'getDescription' method (see above).
QIcon qDummyPlugin::getIcon() const
{
    return QIcon();
}

//Don't forget to replace 'qDummyPlugin' by your own plugin class name here also!
Q_EXPORT_PLUGIN2(qDummyPlugin,qDummyPlugin);
