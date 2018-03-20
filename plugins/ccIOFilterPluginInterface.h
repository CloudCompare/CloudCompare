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

#include "ccPluginInterface.h"

//qCC_io
#include <FileIOFilter.h>

//! I/O filter plugin interface
/** Version 1.1
**/
class ccIOFilterPluginInterface : public ccPluginInterface
{
public:
	//inherited from ccPluginInterface
	virtual CC_PLUGIN_TYPE getType() const { return CC_IO_FILTER_PLUGIN; }

	//! Returns a list of I/O filter instances
	virtual QVector<FileIOFilter::Shared> getFilters() = 0;
};

Q_DECLARE_INTERFACE(ccIOFilterPluginInterface,
                    "edf.rd.CloudCompare.ccIOFilterPluginInterface/1.1")

#endif //CC_IO_FILTER_PLUGIN_INTERFACE_HEADER
