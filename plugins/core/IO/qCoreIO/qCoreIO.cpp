//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "qCoreIO.h"

#include "HeightProfileFilter.h"
#include "MAFilter.h"
#include "MascaretFilter.h"
#include "ObjFilter.h"
#include "OFFFilter.h"
#include "PDMSFilter.h"
#include "PTXFilter.h"
#include "SimpleBinFilter.h"
#include "STLFilter.h"
#include "VTKFilter.h"


qCoreIO::qCoreIO( QObject *parent ) :
	QObject( parent ),
	ccIOPluginInterface( ":/CC/plugin/CoreIO/info.json" )
{
}

void qCoreIO::registerCommands( ccCommandLineInterface *inCmdLine )
{
	Q_UNUSED( inCmdLine );
}

ccIOPluginInterface::FilterList qCoreIO::getFilters()
{
	return {
		FileIOFilter::Shared( new PTXFilter ),
		FileIOFilter::Shared( new SimpleBinFilter ),
		FileIOFilter::Shared( new ObjFilter ),
		FileIOFilter::Shared( new VTKFilter ),
		FileIOFilter::Shared( new STLFilter ),
		FileIOFilter::Shared( new OFFFilter ),
		FileIOFilter::Shared( new PDMSFilter ),
		FileIOFilter::Shared( new MAFilter ),
		FileIOFilter::Shared( new MascaretFilter ),
		FileIOFilter::Shared( new HeightProfileFilter ),
	};
}
