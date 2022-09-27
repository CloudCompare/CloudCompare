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

#include "LasDetails.h"
#include "LasExtraScalarField.h"

#include <laszip/laszip_api.h>

#include <QMetaType>

/// Holds Meta-Information about the original file that we want to save
/// to restore them when writing
struct LasSavedInfo
{
    static constexpr size_t GUID_DATA_4_SIZE = 8;
    static constexpr size_t SYSTEM_IDENTIFIER_SIZE = 32;

    LasSavedInfo() = default;

    explicit LasSavedInfo(const laszip_header &header);

    LasSavedInfo(const LasSavedInfo &rhs);
    LasSavedInfo &operator=(LasSavedInfo rhs);
    static void Swap(LasSavedInfo &lhs, LasSavedInfo &rhs) noexcept;

    virtual ~LasSavedInfo() noexcept;

    laszip_U16 fileSourceId{0};
    laszip_U32 guidData1{0};
    laszip_U16 guidData2{0};
    laszip_U16 guidData3{0};
    laszip_CHAR guidData4[GUID_DATA_4_SIZE]{0};
    laszip_U8 versionMinor{0};
    laszip_U8 pointFormat{0};
    laszip_CHAR systemIdentifier[SYSTEM_IDENTIFIER_SIZE]{0};
    double xScale{0.0};
    double yScale{0.0};
    double zScale{0.0};

    laszip_U32 numVlrs{0};
    laszip_vlr_struct *vlrs{nullptr};
    std::vector<LasExtraScalarField> extraScalarFields{};
};

Q_DECLARE_METATYPE(LasSavedInfo);
