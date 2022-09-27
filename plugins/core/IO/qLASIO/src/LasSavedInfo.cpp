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

#include "LasSavedInfo.h"

#include <QtGlobal>

#include <algorithm>
#include <cstring>

LasSavedInfo::LasSavedInfo(const laszip_header &header)
    : fileSourceId(header.file_source_ID), guidData1(header.project_ID_GUID_data_1),
      guidData2(header.project_ID_GUID_data_2), guidData3(header.project_ID_GUID_data_3),
      versionMinor(header.version_minor), pointFormat(header.point_data_format),
      xScale(header.x_scale_factor), yScale(header.y_scale_factor), zScale(header.z_scale_factor)
{
    strncpy(guidData4, header.project_ID_GUID_data_4, 8);
    strncpy(systemIdentifier, header.system_identifier, 32);
    const auto vlrShouldBeCopied = [](const laszip_vlr_struct &vlr)
    { return !LasDetails::IsLaszipVlr(vlr) && !LasDetails::IsExtraBytesVlr(vlr); };

    numVlrs =
        std::count_if(header.vlrs, header.vlrs + header.number_of_variable_length_records, vlrShouldBeCopied);

    if (numVlrs > 0)
    {
        vlrs = new laszip_vlr_struct[numVlrs];
        laszip_U32 j{0};
        for (laszip_U32 i{0}; i < header.number_of_variable_length_records; ++i)
        {
            if (vlrShouldBeCopied(header.vlrs[i]))
            {
                LasDetails::CloneVlrInto(header.vlrs[i], vlrs[j]);
                j++;
            }
        }
    }
}

LasSavedInfo &LasSavedInfo::operator=(LasSavedInfo rhs)
{
    LasSavedInfo::Swap(*this, rhs);
    return *this;
}

LasSavedInfo::LasSavedInfo(const LasSavedInfo &rhs)
    : fileSourceId(rhs.fileSourceId), guidData1(rhs.guidData1), guidData2(rhs.guidData2),
      guidData3(rhs.guidData3), versionMinor(rhs.versionMinor), pointFormat(rhs.pointFormat),
      xScale(rhs.xScale), yScale(rhs.yScale), zScale(rhs.zScale), numVlrs(rhs.numVlrs),
      extraScalarFields(rhs.extraScalarFields)
{

    strncpy(guidData4, rhs.guidData4, 8);
    strncpy(systemIdentifier, rhs.systemIdentifier, 32);
    if (numVlrs > 0)
    {
        vlrs = new laszip_vlr_struct[numVlrs];
        for (laszip_U32 i{0}; i < numVlrs; ++i)
        {
            LasDetails::CloneVlrInto(rhs.vlrs[i], vlrs[i]);
        }
    }
}

LasSavedInfo::~LasSavedInfo() noexcept
{
    delete[] vlrs;
}

void LasSavedInfo::Swap(LasSavedInfo &lhs, LasSavedInfo &rhs) noexcept
{
    std::swap(lhs.fileSourceId, rhs.fileSourceId);
    std::swap(lhs.guidData1, rhs.guidData1);
    std::swap(lhs.guidData2, rhs.guidData2);
    std::swap(lhs.guidData3, rhs.guidData3);
    std::swap<laszip_CHAR, LasSavedInfo::GUID_DATA_4_SIZE>(lhs.guidData4, rhs.guidData4);
    std::swap(lhs.versionMinor, rhs.versionMinor);
    std::swap(lhs.pointFormat, rhs.pointFormat);
    std::swap<laszip_CHAR, LasSavedInfo::SYSTEM_IDENTIFIER_SIZE>(lhs.systemIdentifier, rhs.systemIdentifier);
    std::swap(lhs.xScale, rhs.xScale);
    std::swap(lhs.yScale, rhs.yScale);
    std::swap(lhs.zScale, rhs.zScale);
    std::swap(lhs.numVlrs, rhs.numVlrs);
    std::swap(lhs.vlrs, rhs.vlrs);
    std::swap(lhs.extraScalarFields, rhs.extraScalarFields);
}
