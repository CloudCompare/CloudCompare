//##########################################################################
//#                                                                        #
//#                  CLOUDCOMPARE PLUGIN: qCSVMatrixIO                     #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "qCSVMatrixIO.h"

//local
#include "CSVMatrixFilter.h"

//Qt
#include <QtPlugin>

FileIOFilter::Shared qCSVMatrixIO::getFilter()
{
	return FileIOFilter::Shared(new CSVMatrixFilter);
}
