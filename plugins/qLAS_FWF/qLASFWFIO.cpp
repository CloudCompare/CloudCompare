//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qLasFWFIO                      #
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
//#                         COPYRIGHT: CNRS / OSUR                         #
//#                                                                        #
//##########################################################################

#include "qLASFWFIO.h"

//local
#include "Filter/LASFWFFilter.h"
#include "../ccCommandLineInterface.h"

//Qt
#include <QtPlugin>

FileIOFilter::Shared qLASFWFIO::getFilter()
{
	return FileIOFilter::Shared(new LASFWFFilter);
}

static const char COMMAND_LOAD_FWF[] = "FWF_O";

struct CommandLoadLASFWF : public ccCommandLineInterface::Command
{
	CommandLoadLASFWF() : ccCommandLineInterface::Command("Load FWF", COMMAND_LOAD_FWF) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[LOADING LAS]");
		if (cmd.arguments().empty())
		{
			return cmd.error(QString("Missing parameter: filename after \"-%1\"").arg(COMMAND_LOAD_FWF));
		}

		//open specified file
		QString filename(cmd.arguments().takeFirst());
		cmd.print(QString("Opening file: '%1'").arg(filename));

		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		FileIOFilter::Shared filter = FileIOFilter::GetFilter(LASFWFFilter::GetFileFilter(), true);
		if (!filter)
		{
			return cmd.error("Internal error: failed to load the LAS FWF I/O filter");
		}
		
		return cmd.importFile(filename, filter);
	}
};

void qLASFWFIO::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}

	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandLoadLASFWF));
}
