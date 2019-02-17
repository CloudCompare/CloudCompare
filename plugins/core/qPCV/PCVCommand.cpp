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

#include <QProgressBar>

static const char COMMAND_PCV[] = "PCV";

PCVCommand::PCVCommand() :
	ccCommandLineInterface::Command("PCV", COMMAND_PCV) {
}

bool PCVCommand::process(ccCommandLineInterface &cmd) {
	static int n_rays = 256;
	static bool meshIsClosed = true;
	static bool mode360 = true;
	static unsigned resolution = 1024;

	cmd.print("[PCV]");
	if (cmd.meshes().empty()) {
		return cmd.error(QObject::tr("No mesh is available. The CLI PCV impementation currently only supports a single mesh."));
	}

	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(cmd.meshes()[0].mesh->getAssociatedCloud());
	ccGenericMesh* mesh = cmd.meshes()[0].mesh;

	int point_count = pc->size();

	cmd.print(QObject::tr("Size is %1").arg(point_count));

	ccProgressDialog pcvProgressCb(true);
	pcvProgressCb.setAutoClose(false);

	PCV::Launch(n_rays, pc, mesh, meshIsClosed, mode360, resolution, resolution,
		&pcvProgressCb, QString(mesh->getName()));

	cmd.meshes()[0].basename += QObject::tr("_PCV");

	return true;
}

void qPCV::registerCommands(ccCommandLineInterface *cmd) {
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCVCommand));
}