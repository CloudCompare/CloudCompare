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

#ifndef Q_DUMMY_PLUGIN_HEADER
#define Q_DUMMY_PLUGIN_HEADER

#include <QObject>

#include "../ccStdPluginInterface.h"

//! Dummy qCC plugin
/** Replace the 'qDummyPlugin' name by you own plugin class name
    and then check 'qDummyPlugin.cpp' for more directions (you
    have to fill in the blank methods (especially the 'doAction'
    method). Of course, in this header file, you may add you own
    attributes and methods.
**/
class qDummyPlugin : public QObject, public ccStdPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(ccStdPluginInterface)

public:

    //inherited from ccPluginInterface
    void getDescription(ccPluginDescription& desc);
    QIcon getIcon() const;

    //inherited from ccStdPluginInterface
	bool onNewSelection(const ccHObject::Container& selectedEntities);
    int doAction(ccHObject::Container& selectedEntities,
                unsigned& uiModificationFlags,
                ccProgressDialog* progressCb=NULL,
                QWidget* parent=NULL);
    QString getErrorMessage(int errorCode/*, LANGUAGE lang*/);
};

#endif
