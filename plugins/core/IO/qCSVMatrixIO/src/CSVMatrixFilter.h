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

#ifndef CC_CSV_MATRIX_FILTER_HEADER
#define CC_CSV_MATRIX_FILTER_HEADER

//qCC_io
#include <FileIOFilter.h>

//! CSV matrix I/O filter
class CSVMatrixFilter : public FileIOFilter
{
public:
	CSVMatrixFilter();

	//inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
};

#endif //CC_CSV_MATRIX_FILTER_HEADER
