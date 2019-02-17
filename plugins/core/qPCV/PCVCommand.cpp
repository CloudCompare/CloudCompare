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
	static int s_resSpinBoxValue = 1024;
	static bool mode360 = true;

	cmd.print("[PCV]");
	if (cmd.meshes().empty()) {
		return cmd.error(QObject::tr("No point cloud or mesh available. Be sure to open or generate one first!"));
	}

	ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(cmd.meshes()[0].mesh);

	ccGenericPointCloud* pc = mesh->getAssociatedCloud();

	unsigned point_count = pc->size();

	PCV::Launch(n_rays, pc);

	cmd.exportEntity(pc, "PCV");

	return true;
}

void qPCV::registerCommands(ccCommandLineInterface *cmd) {
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCVCommand));
}