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

#ifndef Q_POISSON_RECON_PLUGIN_HEADER
#define Q_POISSON_RECON_PLUGIN_HEADER

//Qt
#include <QObject>

#include "../ccStdPluginInterface.h"

//! Wrapper to the "Poisson Surface Reconstruction (Version 3)" algorithm
/** "Poisson Surface Reconstruction", M. Kazhdan, M. Bolitho, and H. Hoppe
	Symposium on Geometry Processing (June 2006), pages 61--70
	http://www.cs.jhu.edu/~misha/Code/PoissonRecon/
**/
class qPoissonRecon : public QObject, public ccStdPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(ccStdPluginInterface)

public:

	//! Default constructor
	qPoissonRecon(QObject* parent=0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "PoissonReconstruction"; }
	virtual QString getDescription() const { return "3D Mesh Poisson Reconstruction (Kazhdan et al.)"; }
	virtual QIcon getIcon() const;

    //inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities);
    virtual void getActions(QActionGroup& group);

protected slots:

	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Associated action
	QAction* m_action;

};

#endif
