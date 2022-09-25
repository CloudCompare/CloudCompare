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

#include <QFileInfo>
#include <QString>

#include <ccPointCloud.h>

#include <laszip/laszip_api.h>

struct LasWaveformLoader
{
    LasWaveformLoader(const laszip_header_struct &laszipHeader,
                      const QString &lasFilename,
                      ccPointCloud &pointCloud);

    void loadWaveform(ccPointCloud &pointCloud, const laszip_point &currentPoint) const;

    unsigned int fwfDataCount{0};
    unsigned int fwfDataOffset{0};
    bool isPointFormatExtended{false};
    ccPointCloud::FWFDescriptorSet descriptors;
};
