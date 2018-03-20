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

#include "filters/IcmFilter.h"
#include "filters/PNFilter.h"
#include "filters/PovFilter.h"
#include "filters/PVFilter.h"
#include "filters/SoiFilter.h"


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
	return QStringLiteral( "This plugin adds some lesser-used I/O formats to CloudCompare: ICM, PN, POV, PV, and SOI." );
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

QVector<FileIOFilter::Shared> qAdditionalIOPlugin::getFilters()
{
	return QVector<FileIOFilter::Shared>{
		FileIOFilter::Shared( new IcmFilter ),
		FileIOFilter::Shared( new PNFilter ),
		FileIOFilter::Shared( new PovFilter ),
		FileIOFilter::Shared( new PVFilter ),
		FileIOFilter::Shared( new SoiFilter ),
	};
}

