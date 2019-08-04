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
#include "qLASFWFIOCommands.h"

//Qt
#include <QtPlugin>

qLASFWFIO::qLASFWFIO(QObject *parent)
    : QObject(parent)
    , ccIOPluginInterface(":/CC/plugin/qLASFWFIO/info.json")
{
}

ccIOPluginInterface::FilterList qLASFWFIO::getFilters()
{
	return { FileIOFilter::Shared(new LASFWFFilter) };
}

void qLASFWFIO::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}

	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandLoadLASFWF));
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandSaveLASFWF));
}
