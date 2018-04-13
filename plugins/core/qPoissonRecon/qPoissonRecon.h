//##########################################################################
//#                                                                        #
//#                  CLOUDCOMPARE PLUGIN: qPoissonRecon                    #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#ifndef Q_POISSON_RECON_PLUGIN_HEADER
#define Q_POISSON_RECON_PLUGIN_HEADER

#include "ccStdPluginInterface.h"

//! Wrapper to the "Poisson Surface Reconstruction (Version 9)" algorithm
/** "Poisson Surface Reconstruction", M. Kazhdan, M. Bolitho, and H. Hoppe
	Symposium on Geometry Processing (June 2006), pages 61--70
	http://www.cs.jhu.edu/~misha/Code/PoissonRecon/
**/
class qPoissonRecon : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qPoissonRecon" FILE "info.json")

public:

	//! Default constructor
	explicit qPoissonRecon(QObject* parent = nullptr);
	
	virtual ~qPoissonRecon() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;

protected slots:

	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Associated action
	QAction* m_action;

};

#endif
