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

#ifndef CC_PCD_FILTER_HEADER
#define CC_PCD_FILTER_HEADER

#include "FileIOFilter.h"
#include "InputMemoryFile.h"

class ccPointCloud;

//! PCD point cloud I/O filter
class PCDFilter : public FileIOFilter
{
public:

	//inherited from FileIOFilter
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, CCVector3d* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename);

protected:

	//! (Binary) PCD file header
	struct PCDHeader
	{
		QString version; //0.7
		std::vector<QString> fields; // x y z corrected_int Scalar_field
		std::vector<size_t> size;// 4 4 4 4 4
		std::vector<QString> type;// F F F F F
		std::vector<size_t> count;// 1 1 1 1 1 multiplicity of each field
		size_t width;// 4318440
		size_t height; //1
		std::vector<float> viewpoint;// 0 0 0 1 0 0 0
		size_t points;// 4318440
		QString data;// binary_compressed
		qint64 data_position;
		size_t pointStride;
		size_t lineCount;
	};

	CC_FILE_ERROR loadFileBinaryMemMap(QString filename, ccHObject& container, const PCDHeader& header);

	static int ReadScalarFieldMemMap(const QString& fieldname, const InputMemoryFile& mem_file, const PCDHeader& header, ccScalarField& field, size_t count = 0);

	static int ReadRGBMemMap(const InputMemoryFile& mem_file, const PCDHeader& header, ccPointCloud& cloud);

	static int ReadNormalsMemMap(const InputMemoryFile& mem_file, const PCDHeader& header, ccPointCloud& cloud);

	/*** HELPERS FOR ACCESSING HEADER INFOS ***/

	CC_FILE_ERROR readFileHeader(QString filename, PCDHeader& header);

	static int GetIDOfField(const QString& fieldName, const PCDFilter::PCDHeader& header);

	static size_t GetSizeOfField(int fieldID, const PCDFilter::PCDHeader& header);

	static size_t GetOffsetOfField(int fieldID, const PCDFilter::PCDHeader& header);
};

#endif
