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

#ifndef Q_LAS_FWF_IO_PLUGIN_HEADER
#define Q_LAS_FWF_IO_PLUGIN_HEADER

//Qt
#include <QObject>

#include "ccIOFilterPluginInterface.h"

class qLASFWFIO : public QObject, public ccIOFilterPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccIOFilterPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qLAS_FWF_IO" FILE "info.json")

public:

	//! Default constructor
	qLASFWFIO(QObject* parent = nullptr);

	//inherited from ccPluginInterface
	virtual void registerCommands(ccCommandLineInterface* cmd) override;

	//inherited from ccIOFilterPluginInterface
	FileIOFilter::Shared getFilter() override;
};

#endif //Q_LAS_FWF_IO_PLUGIN_HEADER
