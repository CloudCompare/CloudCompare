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

#ifndef QTreeIso_PLUGIN_COMMANDS_HEADER
#define QTreeIso_PLUGIN_COMMANDS_HEADER

//CloudCompare
#include "ccCommandLineInterface.h"

//Local
#include "ccTreeIsoDlg.h"
#include "TreeIso.h"

static const char COMMAND_TREEISO[] = "TREEISO";

static const char COMMAND_LAMBDA1[] = "LAMBDA1";
static const char COMMAND_K1[] = "K1";
static const char COMMAND_DECIMATE_RESOLUTION1[] = "DECIMATE_RESOLUTION1";


static const char COMMAND_LAMBDA2[] = "LAMBDA2";
static const char COMMAND_K2[] = "K2";
static const char COMMAND_MAX_GAP[] = "MAX_GAP";
static const char COMMAND_DECIMATE_RESOLUTION2[] = "DECIMATE_RESOLUTION2";

static const char COMMAND_RHO[] = "RHO";
static const char COMMAND_VERTICAL_OVERLAP_WEIGHT[] = "VERTICAL_OVERLAP_WEIGHT";



struct CommandTreeIso : public ccCommandLineInterface::Command
{
	CommandTreeIso() : ccCommandLineInterface::Command("TREEISO", COMMAND_TREEISO) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[TreeIso]");

		if (cmd.clouds().empty()) 
		{
			cmd.error("No cloud loaded");
			return false;
		}

		//initial parameters
		float reg_strength1 = 1.0; //lambda1
		int min_nn1 = 5; //K1:key parameter
		float decimate_res1 = 0.05;

		int reg_strength2 = 20; //lambda2:key parameter
		int min_nn2 = 20; //K2:key parameter
		float decimate_res2 = 0.1;
		float max_gap = 2.0;

		float rel_height_length_ratio = 0.5; //rho
		float vertical_weight = 0.5; //w:key parameter


		bool try_init_seg=false;
		bool try_intermediate_seg = false;
		bool try_final_seg = false;

		while (!cmd.arguments().empty())
		{
			const QString& ARGUMENT = cmd.arguments().front();
			if ((ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_LAMBDA1)))
			{
				try_init_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				reg_strength1 = cmd.arguments().takeFirst().toFloat(&convert);
				if ((!convert) & (reg_strength1 <=0.0))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_LAMBDA1));
				}
				cmd.print(QString("lambda1 (Regularization strength for initial segmentation) set: %1").arg(reg_strength1));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_K1)) {
				try_init_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				min_nn1 = cmd.arguments().takeFirst().toInt(&convert);
				if ((!convert) & (min_nn1<3))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_K1));
				}
				cmd.print(QString("K1 (Nearest neighbors to search for initial segmentation) set: %1").arg(min_nn1));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_DECIMATE_RESOLUTION1)) {
				try_init_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				decimate_res1 = cmd.arguments().takeFirst().toFloat(&convert);
				if ((!convert) & (decimate_res1 < 0.001))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_DECIMATE_RESOLUTION1));
				}
				cmd.print(QString("Decimated resolution (in m) for initial segmentation set: %1").arg(decimate_res1));
			}


			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_LAMBDA2)) {
				try_intermediate_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				reg_strength2 = cmd.arguments().takeFirst().toFloat(&convert);
				if ((!convert) & (reg_strength2 <= 0.0))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_LAMBDA2));
				}
				cmd.print(QString("lambda2 (Regularization strength for intermediate segmentation) set: %1").arg(reg_strength2));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_K2)) {
				try_intermediate_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				min_nn2 = cmd.arguments().takeFirst().toInt(&convert);
				if ((!convert) & (min_nn2 < 3))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_K2));
				}
				cmd.print(QString("K2 (Nearest neighbors to search for intermediate segmentation) set: %1").arg(min_nn2));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_MAX_GAP)) {
				try_intermediate_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				max_gap = cmd.arguments().takeFirst().toFloat(&convert);
				if ((!convert) & (max_gap <= 0.0001))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_MAX_GAP));
				}
				cmd.print(QString("Maximum point gap (in m) for intermediate segmentation set: %1").arg(max_gap));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_DECIMATE_RESOLUTION2)) {
				try_intermediate_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				decimate_res2 = cmd.arguments().takeFirst().toFloat(&convert);
				if ((!convert) & (decimate_res2 < 0.001))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_DECIMATE_RESOLUTION2));
				}
				cmd.print(QString("Decimated resolution (in m) for intermediate segmentation set: %1").arg(decimate_res2));
			}

			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_RHO)) {
				try_final_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				rel_height_length_ratio = cmd.arguments().takeFirst().toFloat(&convert);
				if ((!convert) & (rel_height_length_ratio < 0.001))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_RHO));
				}
				cmd.print(QString("Relative height to length ratio (used to detect non-stems for final segmentation) set: %1").arg(rel_height_length_ratio));

			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VERTICAL_OVERLAP_WEIGHT)) {
				try_final_seg = true;
				cmd.arguments().pop_front();
				bool convert = false;

				vertical_weight = cmd.arguments().takeFirst().toFloat(&convert);
				if ((!convert) & (vertical_weight < 0.001))
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_VERTICAL_OVERLAP_WEIGHT));
				}
				cmd.print(QString("Vertical overlapping ratio weight for final segmentation set: %1").arg(vertical_weight));
			}

			else
			{
				cmd.print("Parameters All Set");
				break;
			}
		}

		TreeIso* m_treeiso = new TreeIso();

		for (CLCloudDesc& desc : cmd.clouds())
		{
			ccPointCloud* pc = desc.pc;

			//Convert CC point cloud to treeIso type
			unsigned count = pc->size();
			if (count == 0)
			{
				cmd.print(QString("Cloud %1 is empty").arg(pc->getName()));
				continue;
			}

			if (try_init_seg)
			{
				if (!m_treeiso->init_seg_pcd(pc, min_nn1, reg_strength1, decimate_res1))
				{
					return cmd.error("Failed to finish initial segmentation due to unknown reasons.");
				}
			}
			if (try_intermediate_seg)
			{
				if (!m_treeiso->intermediate_seg_pcd(pc, min_nn2, reg_strength2, decimate_res2,max_gap))
				{
					return cmd.error("Failed to finish intermediate segmentation due to unknown reasons.");
				}
			}
			if (try_final_seg)
			{
				if (!m_treeiso->final_seg_pcd(pc, min_nn2, rel_height_length_ratio, vertical_weight))
				{
					return cmd.error("Failed to finish final segmentation due to unknown reasons.");
				}
			}
		}

		return true;
	}
};

#endif //QTREEISO_PLUGIN_COMMANDS_HEADER
