//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCV                        #
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
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//

#ifndef Q_PCV_PLUGIN_HEADER
#define Q_PCV_PLUGIN_HEADER

#include <QObject>

#include "../ccStdPluginInterface.h"

//! Wrapper to the ShadeVis algorithm for computing Ambient Occlusion on meshes and point clouds
/** "Visibility based methods and assessment for detail-recovery", M. Tarini, P. Cignoni, R. Scopigno
	Proc. of Visualization 2003, October 19-24, Seattle, USA.
	http://vcg.sourceforge.net/index.php/ShadeVis
**/
class qPCVPlugin : public QObject, public ccStdPluginInterface
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
