#pragma once

//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
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
//#        Please cite the following paper, If you use this plugin in your work.        #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

// A mex version for programming in Matlab is at File Exchange of Mathworks website:
// http://www.mathworks.com/matlabcentral/fileexchange/58139-csf--ground-filtering-of-point-cloud-based-on-cloth-simulation

//CloudCompare
#include "ccCommandLineInterface.h"

//Local
#include "ccCSFDlg.h"
#include "CSF.h"

static const char COMMAND_CSF[] = "CSF"; 
static const char COMMAND_CSF_SCENE[] = "SCENES";
static const char COMMAND_CSF_SCENE_SLOPE[] = "SLOPE";
static const char COMMAND_CSF_SCENE_RELIEF[] = "RELIEF";
static const char COMMAND_CSF_SCENE_FLAT[] = "FLAT";
static const char COMMAND_CSF_PROC_SLOPE[] = "PROC_SLOPE"; 
static const char COMMAND_CSF_CLOTH_RESOLUTION[] = "CLOTH_RESOLUTION";
static const char COMMAND_CSF_MAX_ITERATION[] = "MAX_ITERATION";
static const char COMMAND_CSF_CLASS_THRESHOLD[] = "CLASS_THRESHOLD";
static const char COMMAND_CSF_EXPORT_GROUND[] = "EXPORT_GROUND";
static const char COMMAND_CSF_EXPORT_OFFGROUND[] = "EXPORT_OFFGROUND";

struct CommandCSF : public ccCommandLineInterface::Command
{
	CommandCSF() : ccCommandLineInterface::Command("CSF", COMMAND_CSF) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[CSF]");

		if (cmd.clouds().empty()) 
		{
			cmd.error("No cloud loaded");
			return false;
		}

		//initial parameters
		bool csfPostprocessing = false;
		double clothResolution = 2.0;
		double classThreshold = 0.5;
		int csfRigidness = 2;
		int maxIteration = 500;
		bool exportGround = false;
		bool exportOffground = false;

		while (!cmd.arguments().empty())
		{
			const QString& ARGUMENT = cmd.arguments().front();
			if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_CSF_SCENE))
			{
				cmd.arguments().pop_front();
				bool conv = false;
				QString scene = cmd.arguments().takeFirst();
				if (scene == COMMAND_CSF_SCENE_SLOPE)
				{
					cmd.print("Set scene to steep slope");
					csfRigidness = 1;
				}
				else if (scene == COMMAND_CSF_SCENE_RELIEF)
				{
					cmd.print("Set scene to relief");
					csfRigidness = 2;
				}
				else if (scene == COMMAND_CSF_SCENE_FLAT)
				{
					cmd.print("Set scene to flat");
					csfRigidness = 3;
				}
				else
				{
					cmd.error("Unknown scene parameter. Defaulting to relief");
					csfRigidness = 2;
				}
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_CSF_PROC_SLOPE)) 
			{
				cmd.arguments().pop_front();
				cmd.print("Slope processing turned on");
				csfPostprocessing = true;
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_CSF_CLOTH_RESOLUTION))
			{
				cmd.arguments().pop_front();
				bool conv = false;
				clothResolution = cmd.arguments().takeFirst().toDouble(&conv);
				if (!conv)
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_CSF_CLOTH_RESOLUTION));
				}
				cmd.print(QString("Custom cloth resulution set: %1").arg(clothResolution));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_CSF_MAX_ITERATION))
			{
				cmd.arguments().pop_front();
				bool conv = false;
				maxIteration = cmd.arguments().takeFirst().toInt(&conv);
				if (!conv)
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_CSF_MAX_ITERATION));
				}
				cmd.print(QString("Custom max iteration set: %1").arg(maxIteration));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_CSF_CLASS_THRESHOLD))
			{
				cmd.arguments().pop_front();
				bool conv = false;
				classThreshold = cmd.arguments().takeFirst().toDouble(&conv);
				if (!conv)
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_CSF_CLASS_THRESHOLD));
				}
				cmd.print(QString("Custom class threshold set: %1").arg(classThreshold));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_CSF_EXPORT_GROUND))
			{
				cmd.arguments().pop_front();
				cmd.print("Ground will be exported");
				exportGround = true;
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_CSF_EXPORT_OFFGROUND))
			{
				cmd.arguments().pop_front();
				cmd.print("Off-ground will be exported");
				exportOffground = true;
			}
			else
			{
				cmd.print("Set all parameters");
				break;
			}
		}

		//setup parameters
		CSF::Parameters csfParams;
		{
			csfParams.smoothSlope = csfPostprocessing;
			csfParams.class_threshold = classThreshold;
			csfParams.cloth_resolution = clothResolution;
			csfParams.rigidness = csfRigidness;
			csfParams.iterations = maxIteration;
		}

		std::vector<CLCloudDesc> newClouds;

		for (CLCloudDesc& desc : cmd.clouds())
		{
			ccPointCloud* pc = desc.pc;

			//Convert CC point cloud to CSF type
			unsigned count = pc->size();
			if (count == 0)
			{
				cmd.print(QString("Cloud %1 is empty").arg(pc->getName()));
				continue;
			}

			ccPointCloud* groundCloud = nullptr;
			ccPointCloud* offGroundCloud = nullptr;
			ccMesh* clothMesh = nullptr;

			if (!CSF::Apply(pc,
							csfParams,
							groundCloud,
							offGroundCloud,
							false,
							clothMesh,
							nullptr))
			{
				return cmd.error("Not enough memory");
			}

			//store ground subset
			if (groundCloud)
			{
				groundCloud->setName(desc.basename + QString("_ground_points"));
				//add cloud to the pool of new clouds
				CLCloudDesc groundDesc(groundCloud, desc.basename + QString("_ground_points"), desc.path, -1);
				newClouds.push_back(groundDesc);

				if (exportGround)
				{
					QString errorStr = cmd.exportEntity(groundDesc, QString(), nullptr, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
					if (!errorStr.isEmpty())
					{
						cmd.error(errorStr);
					}
				}
			}

			//store off-ground subset
			if (offGroundCloud)
			{
				offGroundCloud->setName(desc.basename + QString("_offground_points"));
				CLCloudDesc offgroundDesc(offGroundCloud, desc.basename + QString("_offground_points"), desc.path, -1);
				newClouds.push_back(offgroundDesc);
				if (exportOffground)
				{
					QString errorStr = cmd.exportEntity(offgroundDesc, QString(), nullptr, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
					if (!errorStr.isEmpty())
					{
						cmd.error(errorStr);
					}
				}
			}
		}

		// replace the original clouds by the new ones
		cmd.removeClouds();
		cmd.clouds() = newClouds;

		return true;
	}
};
