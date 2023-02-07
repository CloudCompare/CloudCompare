//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
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
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################

#include "LasPlugin.h"

#include "LasIOFilter.h"
#include "LasVlr.h"

LasPlugin::LasPlugin(QObject* parent)
    : QObject(parent)
    , ccIOPluginInterface(":/CC/plugin/LAS-IO/info.json")
{
	qRegisterMetaType<LasVlr>();
	qRegisterMetaTypeStreamOperators<LasVlr>("LasVlr");

	QMetaType::registerConverter(&LasVlr::toString);
}

ccIOPluginInterface::FilterList LasPlugin::getFilters()
{
	return {
	    FileIOFilter::Shared(new LasIOFilter),
	};
}
