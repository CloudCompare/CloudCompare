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

#include "LasScalarFieldLoader.h"

#include <laszip/laszip_api.h>

#include <ccPointCloud.h>
#include <ccScalarField.h>

#include <utility>

// TODO take by move
LasScalarFieldLoader::LasScalarFieldLoader(std::vector<LasScalarField>& standardScalarFields,
                                           std::vector<LasExtraScalarField>& extraScalarFields,
                                           ccPointCloud &pointCloud)
    : m_standardFields(standardScalarFields), m_extraScalarFields(extraScalarFields)
{
    createScalarFieldsForExtraBytes(pointCloud);
}

CC_FILE_ERROR LasScalarFieldLoader::handleScalarFields(ccPointCloud &pointCloud,
                                                       const laszip_point &currentPoint)
{
    CC_FILE_ERROR error = CC_FERR_NO_ERROR;
    for (LasScalarField &lasScalarField : m_standardFields)
    {
        switch (lasScalarField.id)
        {
        case LasScalarField::Intensity:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.intensity);
            break;
        case LasScalarField::ReturnNumber:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.return_number);
            break;
        case LasScalarField::NumberOfReturns:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.number_of_returns);
            break;
        case LasScalarField::ScanDirectionFlag:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.scan_direction_flag);
            break;
        case LasScalarField::EdgeOfFlightLine:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.edge_of_flight_line);
            break;
        case LasScalarField::Classification:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.classification);
            break;
        case LasScalarField::SyntheticFlag:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.synthetic_flag);
            break;
        case LasScalarField::KeypointFlag:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.keypoint_flag);
            break;
        case LasScalarField::WithheldFlag:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.withheld_flag);
            break;
        case LasScalarField::ScanAngleRank:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.scan_angle_rank);
            break;
        case LasScalarField::UserData:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.user_data);
            break;
        case LasScalarField::PointSourceId:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.point_source_ID);
            break;
        case LasScalarField::GpsTime:
            error = handleGpsTime(lasScalarField, pointCloud, currentPoint.gps_time);
            break;
        case LasScalarField::ExtendedScanAngle:
            error = handleScalarField(
                lasScalarField, pointCloud, currentPoint.extended_scan_angle * SCAN_ANGLE_SCALE);
            break;
        case LasScalarField::ExtendedScannerChannel:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.extended_scanner_channel);
            break;
        case LasScalarField::OverlapFlag:
            error =
                handleScalarField(lasScalarField, pointCloud, currentPoint.extended_classification_flags & 8);
            break;
        case LasScalarField::ExtendedClassification:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.extended_classification);
            break;
        case LasScalarField::ExtendedReturnNumber:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.extended_return_number);
            break;
        case LasScalarField::ExtendedNumberOfReturns:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.extended_number_of_returns);
            break;
        case LasScalarField::NearInfrared:
            error = handleScalarField(lasScalarField, pointCloud, currentPoint.rgb[3]);
            break;
        }

        if (error != CC_FERR_NO_ERROR)
        {
            return error;
        }
    }

    return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LasScalarFieldLoader::handleRGBValue(ccPointCloud &pointCloud, const laszip_point &currentPoint)
{
    if (!pointCloud.hasColors() && currentPoint.rgb[0] != 0)
    {
        if (!pointCloud.reserveTheRGBTable())
        {
            return CC_FERR_NOT_ENOUGH_MEMORY;
        }
        if ((currentPoint.rgb[0] | currentPoint.rgb[1] | currentPoint.rgb[2]) > 255)
        {
            colorCompShift = 8;
        }
        for (unsigned int j{0}; j < pointCloud.size() - 1; ++j)
        {
            auto red = static_cast<ColorCompType>(0);
            auto green = static_cast<ColorCompType>(0);
            auto blue = static_cast<ColorCompType>(0);
            pointCloud.addColor(ccColor::Rgb(red, green, blue));
        }
    }

    if (pointCloud.hasColors())
    {
        auto red = static_cast<ColorCompType>(currentPoint.rgb[0] >> colorCompShift);
        auto green = static_cast<ColorCompType>(currentPoint.rgb[1] >> colorCompShift);
        auto blue = static_cast<ColorCompType>(currentPoint.rgb[2] >> colorCompShift);
        pointCloud.addColor(ccColor::Rgb(red, green, blue));
    }
    return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LasScalarFieldLoader::handleExtraScalarFields(ccPointCloud &pointCloud,
                                                            const laszip_point &currentPoint)
{
    if (currentPoint.num_extra_bytes <= 0 || currentPoint.extra_bytes == nullptr)
    {
        return CC_FERR_NO_ERROR;
    }

    for (const LasExtraScalarField &extraField : m_extraScalarFields)
    {
        if (extraField.byteOffset + extraField.byteSize() >
            static_cast<unsigned int>(currentPoint.num_extra_bytes))
        {
            Q_ASSERT(false);
            return CC_FERR_READING;
        }

        laszip_U8 *dataStart = currentPoint.extra_bytes + extraField.byteOffset;
        parseRawValues(extraField, dataStart);
        switch (extraField.kind())
        {
        case LasExtraScalarField::Unsigned:
            handleOptionsFor(extraField, rawValues.unsignedValues);
            break;
        case LasExtraScalarField::Signed:
            handleOptionsFor(extraField, rawValues.signedValues);
            break;
        case LasExtraScalarField::Floating:
            handleOptionsFor(extraField, rawValues.floatingValues);
            break;
        }
    }
    return CC_FERR_NO_ERROR;
}

template <typename T>
CC_FILE_ERROR
LasScalarFieldLoader::handleScalarField(LasScalarField &sfInfo, ccPointCloud &pointCloud, T currentValue)
{
    if (!sfInfo.sf && currentValue != T{})
    {
        auto newSf = new ccScalarField(sfInfo.name());
        sfInfo.sf = newSf;
        pointCloud.addScalarField(newSf);
        if (!newSf->reserveSafe(pointCloud.capacity()))
        {
            return CC_FERR_NOT_ENOUGH_MEMORY;
        }
        // addScalarField resizes the point scalarField
        for (unsigned int j{0}; j < newSf->size() - 1; ++j)
        {
            newSf->setValue(j, static_cast<ScalarType>(T{}));
        }
    }

    if (sfInfo.sf)
    {
        sfInfo.sf->addElement(static_cast<ScalarType>(currentValue));
    }
    return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR
LasScalarFieldLoader::handleGpsTime(LasScalarField &sfInfo, ccPointCloud &pointCloud, double currentValue)
{
    if (!sfInfo.sf && currentValue != 0.0)
    {
        auto newSf = new ccScalarField(sfInfo.name());
        sfInfo.sf = newSf;
        pointCloud.addScalarField(newSf);
        if (!newSf->reserveSafe(pointCloud.capacity()))
        {
            return CC_FERR_NOT_ENOUGH_MEMORY;
        }

        newSf->setGlobalShift(currentValue);

        // addScalarField resizes the point scalarField
        for (unsigned int j{0}; j < newSf->size() - 1; ++j)
        {
            newSf->setValue(j, static_cast<ScalarType>(0.0));
        }
    }

    if (sfInfo.sf)
    {
        sfInfo.sf->addElement(static_cast<ScalarType>(currentValue - sfInfo.sf->getGlobalShift()));
    }
    return CC_FERR_NO_ERROR;
}

bool LasScalarFieldLoader::createScalarFieldsForExtraBytes(ccPointCloud &pointCloud)
{
    char name[50];
    for (LasExtraScalarField &extraField : m_extraScalarFields)
    {
        switch (extraField.numElements())
        {
        case 1:
            if (pointCloud.getScalarFieldIndexByName(extraField.name) != -1)
            {
                sprintf(name, "%s (Extra)", extraField.name);
                extraField.scalarFields[0] = new ccScalarField(name);
                extraField.ccName = name;
            }
            else
            {
                extraField.scalarFields[0] = new ccScalarField(extraField.name);
            }

            if (!extraField.scalarFields[0]->reserveSafe(pointCloud.capacity()))
            {
                return false;
            }
            pointCloud.addScalarField(extraField.scalarFields[0]);
            break;
        case 2:
        case 3:
            for (unsigned int dimIndex{0}; dimIndex < extraField.numElements(); ++dimIndex)
            {
                sprintf(name, "%s [%d]", extraField.name, dimIndex);
                extraField.scalarFields[dimIndex] = new ccScalarField(name);
                if (!extraField.scalarFields[dimIndex]->reserveSafe(pointCloud.capacity()))
                {
                    return false;
                }
                pointCloud.addScalarField(extraField.scalarFields[dimIndex]);
            }
            break;
        }
        if (extraField.offsetIsRelevant())
        {
            for (unsigned int i = 0; i < extraField.numElements(); ++i)
            {
                extraField.scalarFields[i]->setGlobalShift(extraField.offsets[i]);
            }
        }
    }
    return true;
}

template <typename T> ScalarType LasScalarFieldLoader::ParseValueOfType(uint8_t *source)
{
    return static_cast<ScalarType>(*reinterpret_cast<T *>(source));
}

template <typename T, typename V> V LasScalarFieldLoader::ParseValueOfTypeAs(const uint8_t *source)
{
    return static_cast<V>(*reinterpret_cast<const T *>(source));
}

void LasScalarFieldLoader::parseRawValues(const LasExtraScalarField &extraField, uint8_t *dataStart)
{
    switch (extraField.type)
    {
    case LasExtraScalarField::Invalid:
    case LasExtraScalarField::Undocumented:
        break;
    case LasExtraScalarField::u8_3:
        rawValues.unsignedValues[2] = ParseValueOfTypeAs<uint8_t, uint64_t>(dataStart + 2);
    case LasExtraScalarField::u8_2:
        rawValues.unsignedValues[1] = ParseValueOfTypeAs<uint8_t, uint64_t>(dataStart + 1);
    case LasExtraScalarField::u8:
        rawValues.unsignedValues[0] = ParseValueOfTypeAs<uint8_t, uint64_t>(dataStart);
        break;
    case LasExtraScalarField::u16_3:
        rawValues.unsignedValues[2] = ParseValueOfTypeAs<uint16_t, uint64_t>(dataStart + 4);
    case LasExtraScalarField::u16_2:
        rawValues.unsignedValues[1] = ParseValueOfTypeAs<uint16_t, uint64_t>(dataStart + 2);
    case LasExtraScalarField::u16:
        rawValues.unsignedValues[0] = ParseValueOfTypeAs<uint16_t, uint64_t>(dataStart);
        break;
    case LasExtraScalarField::u32_3:
        rawValues.unsignedValues[2] = ParseValueOfTypeAs<uint32_t, uint64_t>(dataStart + 8);
    case LasExtraScalarField::u32_2:
        rawValues.unsignedValues[1] = ParseValueOfTypeAs<uint32_t, uint64_t>(dataStart + 4);
    case LasExtraScalarField::u32:
        rawValues.unsignedValues[0] = ParseValueOfTypeAs<uint32_t, uint64_t>(dataStart);
        break;
    case LasExtraScalarField::u64_3:
        rawValues.unsignedValues[2] = ParseValueOfTypeAs<uint64_t, uint64_t>(dataStart + 16);
    case LasExtraScalarField::u64_2:
        rawValues.unsignedValues[1] = ParseValueOfTypeAs<uint64_t, uint64_t>(dataStart + 8);
    case LasExtraScalarField::u64:
        rawValues.unsignedValues[0] = ParseValueOfTypeAs<uint64_t, uint64_t>(dataStart);
        break;
    case LasExtraScalarField::i8_3:
        rawValues.signedValues[2] = ParseValueOfTypeAs<int64_t, int64_t>(dataStart + 2);
    case LasExtraScalarField::i8_2:
        rawValues.signedValues[1] = ParseValueOfTypeAs<int8_t, int64_t>(dataStart + 1);
    case LasExtraScalarField::i8:
        rawValues.signedValues[0] = ParseValueOfTypeAs<int8_t, int64_t>(dataStart);
        break;
    case LasExtraScalarField::i16_3:
        rawValues.signedValues[2] = ParseValueOfTypeAs<int16_t, int64_t>(dataStart + 4);
    case LasExtraScalarField::i16_2:
        rawValues.signedValues[1] = ParseValueOfTypeAs<int16_t, int64_t>(dataStart + 2);
    case LasExtraScalarField::i16:
        rawValues.signedValues[0] = ParseValueOfTypeAs<int16_t, int64_t>(dataStart);
        break;
    case LasExtraScalarField::i32_3:
        rawValues.signedValues[2] = ParseValueOfTypeAs<int32_t, int64_t>(dataStart + 8);
    case LasExtraScalarField::i32_2:
        rawValues.signedValues[1] = ParseValueOfTypeAs<int32_t, int64_t>(dataStart + 4);
    case LasExtraScalarField::i32:
        rawValues.signedValues[0] = ParseValueOfTypeAs<int32_t, int64_t>(dataStart);
        break;
    case LasExtraScalarField::i64_3:
        rawValues.signedValues[2] = ParseValueOfTypeAs<int64_t, int64_t>(dataStart + 16);
    case LasExtraScalarField::i64_2:
        rawValues.signedValues[1] = ParseValueOfTypeAs<int64_t, int64_t>(dataStart + 8);
    case LasExtraScalarField::i64:
        rawValues.signedValues[0] = ParseValueOfTypeAs<int64_t, int64_t>(dataStart);
        break;
    case LasExtraScalarField::f32_3:
        rawValues.floatingValues[2] = ParseValueOfTypeAs<float, double>(dataStart + 8);
    case LasExtraScalarField::f32_2:
        rawValues.floatingValues[1] = ParseValueOfTypeAs<float, double>(dataStart + 4);
    case LasExtraScalarField::f32:
        rawValues.floatingValues[0] = ParseValueOfTypeAs<float, double>(dataStart);
        break;
    case LasExtraScalarField::f64_3:
        rawValues.floatingValues[2] = ParseValueOfTypeAs<double, double>(dataStart + 16);
    case LasExtraScalarField::f64_2:
        rawValues.floatingValues[1] = ParseValueOfTypeAs<double, double>(dataStart + 8);
    case LasExtraScalarField::f64:
        rawValues.floatingValues[0] = ParseValueOfTypeAs<double, double>(dataStart);
        break;
    }
}

template <typename T>
void LasScalarFieldLoader::handleOptionsFor(const LasExtraScalarField &extraField, T values[3])
{
    for (unsigned int dimIndex = 0; dimIndex < extraField.numElements(); ++dimIndex)
    {
        if (extraField.noDataIsRelevant())
        {
            auto noDataValue =
                ParseValueOfTypeAs<T, T>(static_cast<const uint8_t *>(extraField.noData[dimIndex]));
            if (noDataValue == values[dimIndex])
            {
                extraField.scalarFields[dimIndex]->addElement(ccScalarField::NaN());
            }
        }
        else if (extraField.scaleIsRelevant())
        {
            double scaledValue =
                (values[dimIndex] * extraField.scales[dimIndex]) + (extraField.offsets[dimIndex]);
            extraField.scalarFields[dimIndex]->addElement(static_cast<ScalarType>(scaledValue));
        }
        else
        {
            extraField.scalarFields[dimIndex]->addElement(static_cast<ScalarType>(values[dimIndex]));
        }
    }
}
