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
//
//*********************** Last revision of this file ***********************
//$Author:: lpenasa                                                        $
//$Rev:: 2224                                                              $
//$LastChangedDate:: 2012-07-25 19:13:23 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//
#ifndef CC_PCD_FILTER_HEADER
#define CC_PCD_FILTER_HEADER

#include "FileIOFilter.h"
#include "InputMemoryFile.h"


class ccPointCloud;


//! PCD point cloud I/O filter
class PCDFilter : public FileIOFilter
{
public:

    struct PCDHeader
    {
        std::string version; //0.7
        std::vector<std::string> fields; // x y z corrected_int Scalar_field
        std::vector<size_t> size;// 4 4 4 4 4
        std::vector<char> type;// F F F F F
        std::vector<size_t> count;// 1 1 1 1 1 multiplicity of each field
        size_t width;// 4318440
        size_t height; //1
        std::vector<float> viewpoint;// 0 0 0 1 0 0 0
        size_t points;// 4318440
        std::string data;// binary_compressed
        unsigned int data_position;

    };


    //inherited from FileIOFilter
    virtual CC_FILE_ERROR loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, double* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const char* filename);

protected:
    CC_FILE_ERROR loadFileBinaryMemMap(const char* filename, ccHObject& container, PCDHeader &header);

    int readScalarFieldMemMap(const std::string fieldname, const InputMemoryFile & mem_file, const PCDHeader header, ccScalarField &field , const int count = 0);

    int readRGBMemMap(const InputMemoryFile & mem_file, const PCDHeader &header, ccPointCloud &cloud);

    int readNormalsMemMap(const InputMemoryFile & mem_file, const PCDHeader &header, ccPointCloud &cloud);

    //// HELPERS FOR ACCESSING HEADER INFOS
    CC_FILE_ERROR readFileHeader(const char * filename, PCDHeader &header);

    int getIDOfField(const std::string fieldname, const PCDHeader header);

    int getSizeOfField(const std::string fieldname, const PCDHeader header);

    int getOffsetOfField(const std::string fieldname, const PCDHeader header);
};

#endif
