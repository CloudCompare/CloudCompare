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

#ifndef CC_IO_FILTER_PLUGIN_INTERFACE_HEADER
#define CC_IO_FILTER_PLUGIN_INTERFACE_HEADER

#include <QVector>

//qCC_io
#include <FileIOFilter.h>

#include "ccDefaultPluginInterface.h"

//! I/O filter plugin interface
/** Version 1.1
**/
class ccIOFilterPluginInterface : public ccDefaultPluginInterface
{
public:
	ccIOFilterPluginInterface( const QString &resourcePath = QString() ) :
		ccDefaultPluginInterface( resourcePath )
	{
	}
	
	virtual ~ccIOFilterPluginInterface() {}
	
	//inherited from ccPluginInterface
	virtual CC_PLUGIN_TYPE getType() const { return CC_IO_FILTER_PLUGIN; }

	//! Returns an I/O filter instance
	/** Either getFilter or getFilters should be reimplemented by the child class
		(depending on the number of I/O filters managed by the plugin)
	**/
	virtual FileIOFilter::Shared getFilter() { return FileIOFilter::Shared(nullptr); }

	//! Returns a list of I/O filter instances
	/** Either getFilter or getFilters should be reimplemented by the child class
		(depending on the number of I/O filters managed by the plugin)
	**/
	virtual QVector<FileIOFilter::Shared> getFilters() { return QVector<FileIOFilter::Shared>{ getFilter() }; }
};

Q_DECLARE_INTERFACE(ccIOFilterPluginInterface,
                    "edf.rd.CloudCompare.ccIOFilterPluginInterface/1.1")

#endif //CC_IO_FILTER_PLUGIN_INTERFACE_HEADER
