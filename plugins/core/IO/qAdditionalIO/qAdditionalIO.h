#ifndef Q_ADDITIONAL_IO_PLUGIN_HEADER
#define Q_ADDITIONAL_IO_PLUGIN_HEADER

//##########################################################################
//#                                                                        #
//#         CLOUDCOMPARE PLUGIN: qAdditionalIO                             #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccIOPluginInterface.h"

//! Additional I/O Formats
class qAdditionalIO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccIOPluginInterface)
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qAdditionalIO" FILE "info.json")

public:
	explicit qAdditionalIO( QObject* parent = nullptr );
	
	~qAdditionalIO() override = default;

	void registerCommands(ccCommandLineInterface* cmd) override;
	
	// inherited from ccIOPluginInterface
	FilterList getFilters() override;
};

#endif
