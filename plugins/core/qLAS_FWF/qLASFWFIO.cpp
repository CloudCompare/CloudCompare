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
	, ccIOFilterPluginInterface(":/CC/plugin/qLASFWFIO/info.json")
{
}

FileIOFilter::Shared qLASFWFIO::getFilter()
{
	return FileIOFilter::Shared(new LASFWFFilter);
}

void qLASFWFIO::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}

	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandLoadLASFWF));
}
