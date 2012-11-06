//##########################################################################
//#                                                                        #
//#                  CLOUDCOMPARE PLUGIN: qPoissonRecon                    #
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
//$Rev:: 1743                                                              $
//$LastChangedDate:: 2010-12-03 16:51:30 +0100 (ven., 03 déc. 2010)       $
//**************************************************************************
//

#ifndef Q_POISSON_RECON_PLUGIN_HEADER
#define Q_POISSON_RECON_PLUGIN_HEADER

//Qt
#include <QObject>

#include "../ccStdPluginInterface.h"

//! Wrapper to the "Poisson Surface Reconstruction (Version 2)" algorithm
/** "Poisson Surface Reconstruction", M. Kazhdan, M. Bolitho, and H. Hoppe
	Symposium on Geometry Processing (June 2006), pages 61--70
	http://www.cs.jhu.edu/~misha/Code/PoissonRecon/
**/
class qPoissonReconPlugin : public QObject, public ccStdPluginInterface
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
