//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#pragma once

#include <QVector>

//qCC_io
#include <FileIOFilter.h>

#include "ccDefaultPluginInterface.h"

//! I/O filter plugin interface
/** Version 1.3
**/
class ccIOPluginInterface : public ccDefaultPluginInterface
{
public:
	using FilterList = QVector<FileIOFilter::Shared>;
	
public:
	ccIOPluginInterface( const QString &resourcePath = QString() ) :
		ccDefaultPluginInterface( resourcePath )
	{
	}
	
	~ccIOPluginInterface() override = default;
	
	//inherited from ccPluginInterface
	CC_PLUGIN_TYPE getType() const override { return CC_IO_FILTER_PLUGIN; }

	//! Returns a list of I/O filter instances
	virtual FilterList getFilters() { return FilterList{}; }
};

Q_DECLARE_INTERFACE(ccIOPluginInterface,
                    "cccorp.cloudcompare.ccIOPluginInterface/1.3")
