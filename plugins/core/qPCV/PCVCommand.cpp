#include "PCVCommand.h"
#include "PCV.h"
#include "qPCV.h"

//qCC_db
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

static const char COMMAND_PCV[] = "PCV";

PCVCommand::PCVCommand() :
	Command("PCV", COMMAND_PCV)
{
}

bool PCVCommand::process(ccCommandLineInterface& cmd)
{
	static int n_rays = 256;
	static bool meshIsClosed = true;
	static bool mode360 = true;
	static unsigned resolution = 1024;

	cmd.print("[PCV]");
	if (cmd.meshes().empty())
	{
		return cmd.error(
			QString("No mesh is available. The CLI PCV implementation currently only supports a single mesh."));
	}

	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(cmd.meshes()[0].mesh->getAssociatedCloud());
	ccGenericMesh* mesh = cmd.meshes()[0].mesh;

	int point_count = pc->size();

	cmd.print(QString("Size is %1").arg(point_count));

	ccProgressDialog pcvProgressCb(true);
	pcvProgressCb.setAutoClose(false);

	PCV::Launch(n_rays, pc, mesh, meshIsClosed, mode360, resolution, resolution,
	            &pcvProgressCb, QString(mesh->getName()));

	cmd.meshes()[0].basename += QString("_PCV");

	return true;
}

void qPCV::registerCommands(ccCommandLineInterface* cmd)
{
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCVCommand));
}
