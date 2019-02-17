#include "PCVCommand.h"
#include "PCV.h"
#include "qPCV.h"

static const char COMMAND_PCV[] = "PCV";

PCVCommand::PCVCommand() :
	ccCommandLineInterface::Command("PCV", COMMAND_PCV) {
}

bool PCVCommand::process(ccCommandLineInterface &cmd) {
	cmd.print("[PCV]");
	if (cmd.meshes().empty()) {
		return cmd.error(QObject::tr("No point cloud or mesh available. Be sure to open or generate one first!"));
	}

	return true;
}

void qPCV::registerCommands(ccCommandLineInterface *cmd) {
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCVCommand));
}