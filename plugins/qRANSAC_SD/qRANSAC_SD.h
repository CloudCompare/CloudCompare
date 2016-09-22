//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qRANSAC_SD                     #
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

#ifndef Q_RANSAC_SD_PLUGIN_HEADER
#define Q_RANSAC_SD_PLUGIN_HEADER

#include "../ccStdPluginInterface.h"

//! Wrapper to Schnabel et al. library for automatic shape detection in point cloud
/** "Efficient RANSAC for Point-Cloud Shape Detection", Ruwen Schnabel, Roland Wahl, 
	and Reinhard Klein, in Computer Graphics Forum (June 2007), 26:2(214-226)
	http://cg.cs.uni-bonn.de/en/publications/paper-details/schnabel-2007-efficient/
**/
class qRansacSD : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qRansacSD")

public:

	//! Default constructor
	explicit qRansacSD(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "RANSAC Shape Detection"; }
	virtual QString getDescription() const { return "Efficient RANSAC for Point-Cloud Shape Detection (Schnabel et al 2007)"; }
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
