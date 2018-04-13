//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#ifndef Q_SRA_PLUGIN_HEADER
#define Q_SRA_PLUGIN_HEADER

#include "ccStdPluginInterface.h"

class ccPointCloud;
class ccPolyline;

//! Surface of Revolution Analysis plugin
class qSRA : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qSRA" FILE "info.json")

public:

	//! Default constructor
	explicit qSRA(QObject* parent = nullptr);
	
	virtual ~qSRA() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;

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
