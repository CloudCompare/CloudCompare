//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qTreeIso                          #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper if you find this tool helpful                #
//#                                                                                     #
//#        Xi, Z.; Hopkinson, C. 3D Graph-Based Individual-Tree Isolation (Treeiso)     #
//#        from Terrestrial Laser Scanning Point Clouds. Remote Sens. 2022, 14, 6116.   #
//#        https://doi.org/10.3390/rs14236116                                           #
//#                                                                                     #
//#		   Our work relies on the cut-pursuit algorithm, please also consider citing:   #
//#        Landrieu, L.; Obozinski, G. Cut Pursuit: Fast Algorithms to Learn Piecewise  #
//#        Constant Functions on General Weighted Graphs. SIAM J. Imaging Sci.          #
//#        2017, 10, 1724–1766.                                                         #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#                  Artemis Lab, Department of Geography & Environment                 #
//#                            University of Lethbridge, Canada                         #
//#                                                                                     #
//#                                                                                     #
//#                           Zhouxin Xi and Chris Hopkinson;                           #
//#                    truebelief2010@gmail.com; c.hopkinson@uleth.ca                   #
//#                                                                                     #
//#######################################################################################


#pragma once

// A Matlab version shared via:
// https://github.com/truebelief/artemis_treeiso

#ifndef Q_TREEISO_PLUGIN_HEADER
#define Q_TREEISO_PLUGIN_HEADER

#include "ccStdPluginInterface.h"

//TreeIso
#include <TreeIso.h>
//Dialog
#include "ccTreeIsoDlg.h"


class qTreeIso : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qTreeISO" FILE "../info.json" )

public:

	//! Default constructor
	explicit qTreeIso(QObject* parent = nullptr);

	virtual ~qTreeIso() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;
	void init_segs();
	void intermediate_segs();
	void final_segs();
	//ccPointCloud* m_pc;

protected:

	//! Slot called when associated ation is triggered
	void doAction();
	bool init();

	TreeIso* m_treeiso;
	QProgressDialog* m_progress_dlg;
	ccTreeIsoDlg* m_treeiso_dlg;


	//! Associated action
	QAction* m_action;
	bool m_has_initialized =false;

	struct Parameters
	{
		float reg_strength1 = 1.0; //lambda1
		int min_nn1 = 5; //K1:key parameter
		int reg_strength2 = 20; //lambda2:key parameter
		int min_nn2 = 20; //K2:key parameter
		float edge_strength2 = 1;

		float rel_height_length_ratio = 0.5; //rho
		float vertical_weight = 0.5; //w:key parameter

		float decimate_res1 = 0.05;
		float decimate_res2 = 0.1;

		float max_gap = 2.0;

		std::string fname;
	};

	Parameters m_treeiso_parameters;
};

#endif
