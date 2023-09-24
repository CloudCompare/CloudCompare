#pragma once

//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                        COPYRIGHT: Luca Penasa                          #
//#                                                                        #
//##########################################################################

//Local
#include "PCLCloud.h"

//qCC_db
#include <ccPointCloud.h>

//qCC_io
#include <FileIOFilter.h>

class ccGLMatrixd;

//! PCL to CC cloud converter
class pcl2cc
{
public:

	//! Converts a PCL point cloud to a ccPointCloud
	static ccPointCloud* Convert(	const PCLCloud& pclCloud,
									ccGLMatrixd* _transform = nullptr,
									FileIOFilter::LoadParameters* _loadParameters = nullptr );

public: // other related utility functions

	static bool CopyXYZ(const PCLCloud& pclCloud,
						ccPointCloud& ccCloud,
						uint8_t coordinateType,
						ccGLMatrixd* _transform = nullptr,
						FileIOFilter::LoadParameters* _loadParameters = nullptr);
	static bool CopyNormals(const PCLCloud& pclCloud, ccPointCloud& ccCloud);
	static bool CopyRGB(const PCLCloud& pclCloud, ccPointCloud& ccCloud);
	static bool CopyScalarField(const PCLCloud& pclCloud, const std::string& sfName, ccPointCloud& ccCloud, bool overwriteIfExist = true);

};
