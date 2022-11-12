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

#include "LasDetails.h"
#include "LasExtraScalarField.h"
#include "LasVlr.h"

// CCCoreLib
#include <CCGeom.h>
// LASzip
#include <laszip/laszip_api.h>

namespace LasMetadata
{
	constexpr const char X_SCALE[]           = "LAS.scale.x";
	constexpr const char Y_SCALE[]           = "LAS.scale.y";
	constexpr const char Z_SCALE[]           = "LAS.scale.z";
	constexpr const char X_OFFSET[]          = "LAS.offset.x";
	constexpr const char Y_OFFSET[]          = "LAS.offset.y";
	constexpr const char Z_OFFSET[]          = "LAS.offset.z";
	constexpr const char VERSION_MAJOR[]     = "LAS.version.major";
	constexpr const char VERSION_MINOR[]     = "LAS.version.minor";
	constexpr const char POINT_FORMAT[]      = "LAS.point_format";
	constexpr const char GLOBAL_ENCODING[]   = "LAS.global_encoding";
	constexpr const char PROJECT_UUID[]      = "LAS.project_uuid";
	constexpr const char SYSTEM_IDENTIFIER[] = "LAS.system_identifier";
	constexpr const char VLRS[]              = "LAS.vlrs";
	constexpr const char EXTRA_FIELDS[]      = "LAS.extra_fields";

	void SaveMetadataInto(const laszip_header& header, ccPointCloud& pointCloud, const std::vector<LasExtraScalarField>& extraScalarFields);
	bool LoadVlrs(const ccPointCloud& pointCloud, LasVlr& vlr);
	bool LoadScaleFrom(const ccPointCloud& pointCloud, CCVector3d& scale);
	bool LoadOffsetFrom(const ccPointCloud& pointCloud, CCVector3d& offset);
	bool LoadLasVersionFrom(const ccPointCloud& pointCloud, LasDetails::LasVersion& version);
	bool LoadGlobalEncoding(const ccPointCloud& pointCloud, uint16_t& outGlobalEncoding);
	bool LoadProjectUUID(const ccPointCloud& pointCloud, laszip_header& header);
} // namespace LasMetadata
