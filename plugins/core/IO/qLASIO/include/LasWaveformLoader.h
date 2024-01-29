#pragma once

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

// Qt
#include <QFileInfo>
#include <QString>
// qCC_db
#include <ccPointCloud.h>
// LASzip
#include <laszip/laszip_api.h>

struct LasWaveformLoader
{
	LasWaveformLoader(const laszip_header_struct& laszipHeader,
	                  const QString&              lasFilename,
	                  ccPointCloud&               pointCloud);

	void loadWaveform(ccPointCloud& pointCloud, const laszip_point& currentPoint) const;

	uint64_t                       fwfDataCount{0};
	uint64_t                       fwfDataOffset{0};
	bool                           isPointFormatExtended{false};
	ccPointCloud::FWFDescriptorSet descriptors;
};
