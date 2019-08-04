#include "PCVCommand.h"
#include "PCV.h"
#include "qPCV.h"

//qCC_db
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

constexpr char COMMAND_PCV[] = "PCV";
constexpr char COMMAND_PCV_N_RAYS[] = "N_RAYS";
constexpr char COMMAND_PCV_IS_CLOSED[] = "IS_CLOSED";
constexpr char COMMAND_PCV_180[] = "180";
constexpr char COMMAND_PCV_RESOLUTION[] = "RESOLUTION";

PCVCommand::PCVCommand() :
	Command("PCV", COMMAND_PCV)
{
}

bool PCVCommand::process(ccCommandLineInterface& cmd)
{
	cmd.print("[PCV]");

	// Initialize to match PCV::Launch defaults
	unsigned nRays = 256;
	bool meshIsClosed = false;
	bool mode360 = true;
	unsigned resolution = 1024;

	while (!cmd.arguments().empty())
	{
		const QString& arg = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_IS_CLOSED))
		{
			cmd.arguments().pop_front();
			meshIsClosed = true;
		}

		// PCV::Launch mode360 defaults to true. To make absence / presence of command map to false / true,
		// the command is named "180," and is the inverse of mode360.
		else if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_180))
		{
			cmd.arguments().pop_front();
			mode360 = false;
		}
		else if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_N_RAYS))
		{
			cmd.arguments().pop_front();
			bool conversionOk = false;
			nRays = cmd.arguments().takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_PCV_N_RAYS));
			}
		}
		else if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_RESOLUTION))
		{
			cmd.arguments().pop_front();
			bool conversionOk = false;
			resolution = cmd.arguments().takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_PCV_RESOLUTION));
			}
		}
		else
		{
			break;
		}
	}

	if (cmd.meshes().empty())
	{
		return cmd.error(
			qPCV::tr("No mesh is available. The CLI PCV implementation currently only supports a single mesh."));
	}

	ccGenericPointCloud* meshCloud = cmd.meshes()[0].mesh->getAssociatedCloud();
	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(meshCloud);
	ccGenericMesh* mesh = cmd.meshes()[0].mesh;

	ccProgressDialog pcvProgressCb(true);
	pcvProgressCb.setAutoClose(false);

	PCV::Launch(nRays, pc, mesh, meshIsClosed, mode360, resolution, resolution,
	            &pcvProgressCb, QString(mesh->getName()));

	cmd.meshes()[0].basename += QString("_PCV");

	return true;
}

void qPCV::registerCommands(ccCommandLineInterface* cmd)
{
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCVCommand));
}
