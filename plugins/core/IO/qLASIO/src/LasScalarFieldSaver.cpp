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

#include "LasScalarFieldSaver.h"

#include <laszip/laszip_api.h>

#include <ccScalarField.h>

LasScalarFieldSaver::LasScalarFieldSaver(std::vector<LasScalarField> &&standardFields,
                                         std::vector<LasExtraScalarField> &&extraFields)
    : m_standardFields(standardFields), m_extraFields(extraFields)
{
}

void LasScalarFieldSaver::handleScalarFields(size_t pointIndex, laszip_point &point)
{
    for (const LasScalarField &field : m_standardFields)
    {
        Q_ASSERT_X(field.sf != nullptr, __func__, "LasScalarField has a null ptr to ccScalarField");
        ScalarType value = field.sf->getValue(pointIndex);
        value = std::max(field.range.min, value);
        value = std::min(field.range.max, value);
        switch (field.id)
        {
        case LasScalarField::Intensity:
            if (field.sf)
            {
                point.intensity = static_cast<laszip_U16>(value);
            }
            break;
        case LasScalarField::ReturnNumber:
            if (field.sf)
            {
                point.return_number = static_cast<laszip_U8>(value);
            }
            break;
        case LasScalarField::NumberOfReturns:
            if (field.sf)
            {
                point.number_of_returns = static_cast<laszip_U8>(value);
            }
            break;
        case LasScalarField::ScanDirectionFlag:
            if (field.sf)
            {
                point.scan_direction_flag = (static_cast<laszip_U8>(value) > 0);
            }
            break;
        case LasScalarField::EdgeOfFlightLine:
            if (field.sf)
            {
                point.edge_of_flight_line = (static_cast<laszip_U8>(value) > 0);
            }
            break;
        case LasScalarField::Classification:
            if (field.sf)
            {
                point.classification = static_cast<laszip_U8>(value);
            }
        case LasScalarField::SyntheticFlag:
            if (field.sf)
            {
                point.synthetic_flag = (static_cast<laszip_U8>(value) > 0);
            }
            break;
        case LasScalarField::KeypointFlag:
            if (field.sf)
            {
                point.keypoint_flag = (static_cast<laszip_U8>(value) > 0);
            }
            break;
        case LasScalarField::WithheldFlag:
            if (field.sf)
            {
                point.withheld_flag = (static_cast<laszip_U8>(value) > 0);
            }
            break;
        case LasScalarField::ScanAngleRank:
            if (field.sf)
            {
                point.extended_scan_angle = static_cast<laszip_I16>(value);
            }
        case LasScalarField::UserData:
            if (field.sf)
            {
                point.user_data = static_cast<laszip_U8>(value);
            }
            break;
        case LasScalarField::PointSourceId:
            if (field.sf)
            {
                point.point_source_ID = static_cast<laszip_U16>(value);
            }
            break;
        case LasScalarField::GpsTime:
            if (field.sf)
            {
                point.gps_time = static_cast<laszip_F64>(value) + field.sf->getGlobalShift();
            }
            break;
        case LasScalarField::ExtendedScanAngle:
            if (field.sf)
            {
                point.extended_scan_angle = static_cast<laszip_I16>(value / SCAN_ANGLE_SCALE);
            }
            break;
        case LasScalarField::ExtendedScannerChannel:
            if (field.sf)
            {
                point.extended_scanner_channel = static_cast<laszip_U8>(value);
            }
            break;
        case LasScalarField::OverlapFlag:
            if (field.sf)
            {
                point.extended_classification_flags |= (static_cast<laszip_U8>(value) > 0) ? (1u << 5) : 0;
            }
            break;
        case LasScalarField::ExtendedClassification:
            if (field.sf)
            {
                point.extended_classification = static_cast<laszip_U8>(value);
            }
            break;
        case LasScalarField::ExtendedReturnNumber:
            if (field.sf)
            {
                point.extended_return_number = static_cast<laszip_U16>(value);
            }
            break;
        case LasScalarField::ExtendedNumberOfReturns:
            if (field.sf)
            {
                point.extended_number_of_returns = static_cast<laszip_U16>(value);
            }
            break;
        case LasScalarField::NearInfrared:
            if (field.sf)
            {
                point.rgb[3] = static_cast<laszip_U16>(value);
            }
            break;
        }
    }
}

void LasScalarFieldSaver::handleExtraFields(size_t pointIndex, laszip_point &point)
{
    if (point.num_extra_bytes == 0 || point.extra_bytes == nullptr)
    {
        return;
    }

    ScalarType values[MAX_ELEMENTS_IN_EXTRA_ARRAY_DIM] = {0.0};

    for (const LasExtraScalarField &extraField : m_extraFields)
    {
        laszip_U8 *dataStart = point.extra_bytes + extraField.byteOffset;

        Q_ASSERT(extraField.byteOffset < static_cast<unsigned int>(point.num_extra_bytes));
        Q_ASSERT(extraField.byteOffset + extraField.byteSize() <=
                 static_cast<unsigned int>(point.num_extra_bytes));


        for (size_t elemIndex{0}; elemIndex < extraField.numElements(); ++elemIndex)
        {
            values[elemIndex] = (*extraField.scalarFields[elemIndex])[pointIndex];
        }

        if (extraField.scaleIsRelevant() || extraField.offsetIsRelevant())
        {
            for (size_t elemIndex{0}; elemIndex < extraField.numElements(); ++elemIndex)
            {
                values[elemIndex] =
                    (values[elemIndex] - extraField.offsets[elemIndex]) / extraField.scales[elemIndex];
            }
        }

        // Notice that we rely on implicit fallthrough
        switch (extraField.type)
        {
        case LasExtraScalarField::u8_3:
            WriteScalarValueAs<uint8_t>(values[2], dataStart + 2);
        case LasExtraScalarField::u8_2:
            WriteScalarValueAs<uint8_t>(values[1], dataStart + 1);
        case LasExtraScalarField::u8:
            WriteScalarValueAs<uint8_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::u16_3:
            WriteScalarValueAs<uint16_t>(values[2], dataStart + 4);
        case LasExtraScalarField::u16_2:
            WriteScalarValueAs<uint16_t>(values[1], dataStart + 2);
        case LasExtraScalarField::u16:
            WriteScalarValueAs<uint16_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::u32_3:
            WriteScalarValueAs<uint32_t>(values[2], dataStart + 8);
        case LasExtraScalarField::u32_2:
            WriteScalarValueAs<uint32_t>(values[1], dataStart + 4);
        case LasExtraScalarField::u32:
            WriteScalarValueAs<uint32_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::u64_3:
            WriteScalarValueAs<uint64_t>(values[2], dataStart + 16);
        case LasExtraScalarField::u64_2:
            WriteScalarValueAs<uint64_t>(values[1], dataStart + 8);
        case LasExtraScalarField::u64:
            WriteScalarValueAs<uint64_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::i8_3:
            WriteScalarValueAs<int8_t>(values[2], dataStart + 2);
        case LasExtraScalarField::i8_2:
            WriteScalarValueAs<int8_t>(values[1], dataStart + 1);
        case LasExtraScalarField::i8:
            WriteScalarValueAs<int8_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::i16_3:
            WriteScalarValueAs<int16_t>(values[2], dataStart + 4);
        case LasExtraScalarField::i16_2:
            WriteScalarValueAs<int16_t>(values[1], dataStart + 2);
        case LasExtraScalarField::i16:
            WriteScalarValueAs<int16_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::i32_3:
            WriteScalarValueAs<int32_t>(values[2], dataStart + 8);
        case LasExtraScalarField::i32_2:
            WriteScalarValueAs<int32_t>(values[1], dataStart + 4);
        case LasExtraScalarField::i32:
            WriteScalarValueAs<int32_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::i64_3:
            WriteScalarValueAs<int64_t>(values[2], dataStart + 16);
        case LasExtraScalarField::i64_2:
            WriteScalarValueAs<int64_t>(values[1], dataStart + 8);
        case LasExtraScalarField::i64:
            WriteScalarValueAs<int64_t>(values[0], dataStart);
            break;
        case LasExtraScalarField::f32_3:
            WriteScalarValueAs<float>(values[2], dataStart + 8);
        case LasExtraScalarField::f32_2:
            WriteScalarValueAs<float>(values[1], dataStart + 4);
        case LasExtraScalarField::f32:
            WriteScalarValueAs<float>(values[0], dataStart);
            break;
        case LasExtraScalarField::f64_3:
            WriteScalarValueAs<double>(values[2], dataStart + 16);
        case LasExtraScalarField::f64_2:
            WriteScalarValueAs<double>(values[1], dataStart + 8);
        case LasExtraScalarField::f64:
            WriteScalarValueAs<double>(values[0], dataStart);
            break;
        case LasExtraScalarField::Undocumented:
        case LasExtraScalarField::Invalid:
            break;
        }
    }
}
