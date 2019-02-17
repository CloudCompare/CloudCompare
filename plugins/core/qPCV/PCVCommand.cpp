#include "PCVCommand.h"
#include "PCV.h"
#include "qPCV.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccColorScalesManager.h>

static const char COMMAND_PCV[] = "PCV";

PCVCommand::PCVCommand() :
	ccCommandLineInterface::Command("PCV", COMMAND_PCV) {
}

bool PCVCommand::process(ccCommandLineInterface &cmd) {
	static int n_rays = 256;
	static bool mode360 = true;

	cmd.print("[PCV]");
	if (cmd.meshes().empty()) {
		return cmd.error(QObject::tr("No point cloud or mesh available. Be sure to open or generate one first!"));
	}

	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(cmd.meshes()[0].mesh->getAssociatedCloud());
	ccGenericMesh* mesh = cmd.meshes()[0].mesh;

	int point_count = pc->size();

	cmd.print(QObject::tr("Size is %1").arg(point_count));

	PCV::Launch(n_rays, pc, mesh, mode360);

	cmd.meshes()[0].basename += QObject::tr("_PCV");

	return true;
}

void qPCV::registerCommands(ccCommandLineInterface *cmd) {
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCVCommand));
}