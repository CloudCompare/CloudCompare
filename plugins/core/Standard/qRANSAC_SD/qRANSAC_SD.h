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

#include "ccStdPluginInterface.h"


//! Wrapper to Schnabel et al. library for automatic shape detection in point cloud
/** "Efficient RANSAC for Point-Cloud Shape Detection", Ruwen Schnabel, Roland Wahl, 
	and Reinhard Klein, in Computer Graphics Forum (June 2007), 26:2(214-226)
	http://cg.cs.uni-bonn.de/en/publications/paper-details/schnabel-2007-efficient/
**/
class qRansacSD : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qRansacSD" FILE "info.json")

public:
	enum RANSAC_PRIMITIVE_TYPES
	{
		RPT_PLANE = 0,
		RPT_SPHERE = 1,
		RPT_CYLINDER = 2,
		RPT_CONE = 3,
		RPT_TORUS = 4,
	};

	struct RansacParams
	{
		float epsilon;	// distance threshold
		float bitmapEpsilon; //bitmap resolution
		unsigned supportPoints;	// this is the minimal numer of points required for a primitive
		float   maxNormalDev_deg;	// maximal normal deviation from ideal shape (in degrees)
		float   probability;	// probability that no better candidate was overlooked during sampling
		bool randomColor; // should the resulting detected shapes sub point cloud be colored randomly
		bool primEnabled[5]; //RANSAC_PRIMITIVE_TYPES
		RansacParams() : epsilon(0.005f), bitmapEpsilon(0.001f), supportPoints(500), maxNormalDev_deg(25.0f), probability(0.01f), randomColor(true)
		{
			primEnabled[RPT_PLANE] = true;
			primEnabled[RPT_SPHERE] = true;
			primEnabled[RPT_CYLINDER] = true;
			primEnabled[RPT_CONE] = false;
			primEnabled[RPT_TORUS] = false;
		};

		RansacParams(float scale) : epsilon(0.005f * scale), bitmapEpsilon(0.001f * scale), supportPoints(500), maxNormalDev_deg(25.0f), probability(0.01f), randomColor(true)
		{
			primEnabled[RPT_PLANE] = true;
			primEnabled[RPT_SPHERE] = true;
			primEnabled[RPT_CYLINDER] = true;
			primEnabled[RPT_CONE] = false;
			primEnabled[RPT_TORUS] = false;
		};

	};
	//! Default constructor
	explicit qRansacSD(QObject* parent = nullptr);

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;

	static ccHObject* executeRANSAC(ccPointCloud* ccPC, const RansacParams& params, bool silent = false);
protected slots:

	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Associated action
	QAction* m_action;
};

#endif
