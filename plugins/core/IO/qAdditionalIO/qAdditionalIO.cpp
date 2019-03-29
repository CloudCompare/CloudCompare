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


qAdditionalIO::qAdditionalIO( QObject* parent ) :
	QObject( parent )
  , ccIOPluginInterface( ":/CC/plugin/qAdditionalIO/info.json" )
{
}

void qAdditionalIO::registerCommands( ccCommandLineInterface *cmd )
{
	cmd->registerCommand( ccCommandLineInterface::Command::Shared( new BundlerCommand ) );	
}

ccIOPluginInterface::FilterList qAdditionalIO::getFilters()
{
	return {
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

