//##########################################################################
//#                                                                        #
//#                 CLOUDCOMPARE PLUGIN: qRANSAC_SD                        #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1631                                                              $
//$LastChangedDate:: 2010-08-25 07:21:40 +0200 (mer., 25 août 2010)       $
//**************************************************************************
//

#ifndef Q_RANSAC_SD_PLUGIN_HEADER
#define Q_RANSAC_SD_PLUGIN_HEADER

//Qt
#include <QObject>

#include "../ccStdPluginInterface.h"

//! Wrapper to Schnabel et al. library for automatic shape detection in point cloud
/** "Efficient RANSAC for Point-Cloud Shape Detection", Ruwen Schnabel, Roland Wahl, 
	and Reinhard Klein, in Computer Graphics Forum (June 2007), 26:2(214-226)
	http://cg.cs.uni-bonn.de/en/publications/paper-details/schnabel-2007-efficient/
**/
class qRansacSDPlugin : public QObject, public ccStdPluginInterface
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
