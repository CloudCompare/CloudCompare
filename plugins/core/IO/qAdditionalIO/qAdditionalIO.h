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

#include "ccIOFilterPluginInterface.h"

//! Additional I/O Formats
class qAdditionalIO : public QObject, public ccIOFilterPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccIOFilterPluginInterface)
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qAdditionalIO" FILE "info.json")

public:
	explicit qAdditionalIO( QObject* parent = nullptr );
	
	virtual ~qAdditionalIO() = default;

	virtual void registerCommands(ccCommandLineInterface* cmd) override;
	
	// inherited from ccIOFilterPluginInterface
	QVector<FileIOFilter::Shared> getFilters() override;
};

#endif
