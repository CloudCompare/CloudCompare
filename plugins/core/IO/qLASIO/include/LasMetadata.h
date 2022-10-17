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

struct laszip_header;
class ccPointCloud;


#include "LasExtraScalarField.h"
#include "LasDetails.h"
#include "LasVlr.h"

#include <CCGeom.h>

#include <laszip/laszip_api.h>



namespace LasMetadata
{
const char X_SCALE[] = "LAS.scale.x";
const char Y_SCALE[] = "LAS.scale.y";
const char Z_SCALE[] = "LAS.scale.z";
const char X_OFFSET[] = "LAS.offset.x";
const char Y_OFFSET[] = "LAS.offset.y";
const char Z_OFFSET[] = "LAS.offset.z";
const char VERSION_MAJOR[] = "LAS.version.major";
const char VERSION_MINOR[] = "LAS.version.minor";
const char POINT_FORMAT[] = "LAS.point_format";
const char GLOBAL_ENCODING[] = "LAS.global_encoding";
const char PROJECT_UUID[] = "LAS.project_uuid";
const char SYSTEM_IDENTIFIER[] = "LAS.system_identifier";
const char VLRS[] = "LAS.vlrs";
const char EXTRA_FIELDS[] = "LAS.extra_fields";


void SaveMetadataInto(const laszip_header &header, ccPointCloud &pointCloud, std::vector<LasExtraScalarField> extraScalarFields);
bool LoadVlrs(const ccPointCloud &pointCloud, LasVlr &vlr);
bool LoadScalesFrom(const ccPointCloud &pointCloud, CCVector3d &scales);
bool LoadOffsetsFrom(const ccPointCloud &pointCloud, CCVector3d &offsets);
bool LoadLasVersionFrom(const ccPointCloud &pointCloud, LasDetails::LasVersion &version);
}

