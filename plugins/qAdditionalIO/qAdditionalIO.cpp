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

#include "qAdditionalIO.h"

#include "BundlerCommand.h"

#include "BundlerFilter.h"
#include "IcmFilter.h"
#include "PNFilter.h"
#include "PovFilter.h"
#include "PVFilter.h"
#include "SalomeHydroFilter.h"
#include "SinusxFilter.h"
#include "SoiFilter.h"


qAdditionalIOPlugin::qAdditionalIOPlugin( QObject* parent )
	: QObject( parent )
{
}

QString qAdditionalIOPlugin::getName() const
{
	return QStringLiteral( "Additional I/O" );
}

QString qAdditionalIOPlugin::getDescription() const
{
	return QStringLiteral( "This plugin adds some less frequently used I/O formats to CloudCompare:"
						   "<ul>"
						   "<li>Snavely's Bundler output (*.out)</li>"
						   "<li>Clouds + calibrated images [meta][ascii] (*.icm)</li>"
						   "<li>Point + Normal cloud (*.pn)</li>"
						   "<li>Clouds + sensor info. [meta][ascii] (*.pov)</li>"
						   "<li>Point + Value cloud (*.pv)</li>"
						   "<li>Salome Hydro polylines (*.poly)</li>"
						   "<li>SinusX curve (*.sx)</li>"
						   "<li>Mensi Soisic cloud (*.soi)</li>"
						   "</ul>");
}

ccPluginInterface::ContactList qAdditionalIOPlugin::getAuthors() const
{
	return ccPluginInterface::ContactList{
		Contact{ "Daniel Girardeau-Montaut", "daniel.girardeau@gmail.com" },
	};
}

ccPluginInterface::ContactList qAdditionalIOPlugin::getMaintainers() const
{
	return ccPluginInterface::ContactList{
		Contact{ "Daniel Girardeau-Montaut", "daniel.girardeau@gmail.com" },
		Contact{ "Andy Maloney", "asmaloney@gmail.com" },
	};
}

void qAdditionalIOPlugin::registerCommands( ccCommandLineInterface *cmd )
{
	cmd->registerCommand( ccCommandLineInterface::Command::Shared( new BundlerCommand ) );	
}

QVector<FileIOFilter::Shared> qAdditionalIOPlugin::getFilters()
{
	return QVector<FileIOFilter::Shared>{
		FileIOFilter::Shared( new BundlerFilter ),
		FileIOFilter::Shared( new IcmFilter ),
		FileIOFilter::Shared( new PNFilter ),
		FileIOFilter::Shared( new PovFilter ),
		FileIOFilter::Shared( new PVFilter ),
		FileIOFilter::Shared( new SalomeHydroFilter ),
		FileIOFilter::Shared( new SinusxFilter ),
		FileIOFilter::Shared( new SoiFilter ),
	};
}

