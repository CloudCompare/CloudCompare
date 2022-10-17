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

#include "LasVlr.h"
#include "LasMetadata.h"

#include <QtGlobal>

#include <ccPointCloud.h>

#include <algorithm>
#include <cstring>

LasVlr::LasVlr(const laszip_header &header)
{
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

LasVlr &LasVlr::operator=(LasVlr rhs)
{
    LasVlr::Swap(*this, rhs);
    return *this;
}

LasVlr::LasVlr(const LasVlr &rhs)
    : numVlrs(rhs.numVlrs),
      extraScalarFields(rhs.extraScalarFields)
{

    if (numVlrs > 0)
    {
        vlrs = new laszip_vlr_struct[numVlrs];
        for (laszip_U32 i{0}; i < numVlrs; ++i)
        {
            LasDetails::CloneVlrInto(rhs.vlrs[i], vlrs[i]);
        }
    }
}

LasVlr::~LasVlr() noexcept
{
    delete[] vlrs;
}

void LasVlr::Swap(LasVlr &lhs, LasVlr &rhs) noexcept
{
    std::swap(lhs.numVlrs, rhs.numVlrs);
    std::swap(lhs.vlrs, rhs.vlrs);
    std::swap(lhs.extraScalarFields, rhs.extraScalarFields);
}

