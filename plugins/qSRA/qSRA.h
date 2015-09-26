//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#ifndef Q_SRA_PLUGIN_HEADER
#define Q_SRA_PLUGIN_HEADER

#include "../ccStdPluginInterface.h"

class ccPointCloud;
class ccPolyline;

//! Surface of Revolution Analysis plugin
class qSRA : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
#ifdef CC_QT5
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qSRA")
#endif

public:

	//! Default constructor
	explicit qSRA(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "Surface of Revolution Analysis"; }
	virtual QString getDescription() const { return "Generates a distance map between a point cloud and a surface of revolution"; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

protected slots:

	//! Loads profile from a dedicated file
	void loadProfile() const;

	//! Computes cloud-to-profile radial distances
	void computeCloud2ProfileRadialDist() const;

	//! Projects the cloud distances into a 2D grid
	void projectCloudDistsInGrid() const;

protected:

	//! Projects the cloud distances into a 2D grid (needs the revolution profile)
	void doProjectCloudDistsInGrid(ccPointCloud* cloud, ccPolyline* polyline) const;

	//! Computes cloud-to-profile radial distances
	bool doComputeRadialDists(ccPointCloud* cloud, ccPolyline* polyline) const;

	//! Associated action
	QAction* m_doLoadProfile;
	//! Associated action
	QAction* m_doCompareCloudToProfile;
	//! Associated action
	QAction* m_doProjectCloudDists;
};

#endif //Q_SRA_PLUGIN_HEADER
