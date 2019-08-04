//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#ifndef CANUPO_PLUGIN_COMMANDS_HEADER
#define CANUPO_PLUGIN_COMMANDS_HEADER

//CloudCompare
#include "ccCommandLineInterface.h"

//Local
#include "qCanupoProcess.h"

static const char COMMAND_CANUPO_CALSSIFY[] = "CANUPO_CLASSIFY";
static const char COMMAND_CANUPO_CONFIDENCE[] = "USE_CONFIDENCE";

struct CommandCanupoClassif : public ccCommandLineInterface::Command
{
	CommandCanupoClassif() : ccCommandLineInterface::Command("Canupo Classify", COMMAND_CANUPO_CALSSIFY) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[CANUPO]");
		if (cmd.arguments().empty())
		{
			return cmd.error(QString("Missing parameter: classifier filename (.prm) after \"-%1\"").arg(COMMAND_CANUPO_CALSSIFY));
		}

		qCanupoProcess::ClassifyParams params;
		params.confidenceThreshold = 0.0;
		params.generateAdditionalSF = false;
		params.generateRoughnessSF = false;
		params.samplingDist = 0.0;
		params.useActiveSFForConfidence = false;
		
		QString classifierFilename;

		//optional parameter
		while (!cmd.arguments().empty())
		{
			QString argument = cmd.arguments().front();
			if (ccCommandLineInterface::IsCommand(argument, COMMAND_CANUPO_CONFIDENCE))
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();

				if (cmd.arguments().empty())
				{
					return cmd.error(QString("Missing parameter: confidence threshold after '%1'").arg(COMMAND_CANUPO_CONFIDENCE));
				}

				bool ok;
				params.confidenceThreshold = cmd.arguments().takeFirst().toDouble(&ok);
				if (!ok || params.confidenceThreshold < 0.0)
				{
					return cmd.error(QString("Invalid parameter: confidence threshold after '%1'").arg(COMMAND_CANUPO_CONFIDENCE));
				}

				cmd.print(QString("Confidence threshold set to %1").arg(params.confidenceThreshold));
			}
			else
			{
				//we assume the parameter is the classifier filename
				classifierFilename = argument;
				cmd.arguments().pop_front();
				break;
			}
		}

		if (classifierFilename.isEmpty())
		{
			return cmd.error("Classifier name not set");
		}

		if (cmd.clouds().empty())
		{
			return cmd.error("Need at least At least one cloud must be loaded");
		}

		for (CLCloudDesc& desc : cmd.clouds())
		{
			CorePointDescSet corePointsDescriptors; //core point descriptors
			ccPointCloud* realCorePoints = desc.pc;
			CCLib::GenericIndexedCloudPersist* corePoints = realCorePoints;

			//has the current cloud an active SF?
			int currentSFIndex = desc.pc->getCurrentDisplayedScalarFieldIndex();
			if (currentSFIndex >= 0)
			{
				desc.pc->setCurrentDisplayedScalarField(currentSFIndex);
				params.useActiveSFForConfidence = true;
			}
			else
			{
				params.useActiveSFForConfidence = false;
			}

			if (qCanupoProcess::Classify(classifierFilename, params, desc.pc, corePoints, corePointsDescriptors, realCorePoints, nullptr, nullptr, cmd.silentMode()))
			{
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc, "CLASSIFIED");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
			else
			{
				//process failed
				break;
			}
		}

		return true;
	}
};

#endif //CANUPO_PLUGIN_COMMANDS_HEADER
