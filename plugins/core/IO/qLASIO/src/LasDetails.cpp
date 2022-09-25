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

#include <laszip/laszip_api.h>

#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

#include <QDataStream>

#include <cstring>
#include <stdexcept>
#include <vector>

const char *AvailableVersions[3] = {"1.2", "1.3", "1.4"};

static const std::vector<unsigned int> PointFormatForV1_2 = {0, 1, 2, 3};
static const std::vector<unsigned int> PointFormatForV1_3 = {0, 1, 2, 3, 4, 5};
static const std::vector<unsigned int> PointFormatForV1_4 = {6, 7, 8, 9, 10};

const std::vector<unsigned int> *PointFormatsAvailableForVersion(const char *version)
{
    if (version == nullptr)
    {
        return nullptr;
    }
    if (strcmp(version, "1.2") == 0)
    {
        return &PointFormatForV1_2;
    }

    if (strcmp(version, "1.3") == 0)
    {
        return &PointFormatForV1_3;
    }

    if (strcmp(version, "1.4") == 0)
    {
        return &PointFormatForV1_4;
    }
    return nullptr;
}

static constexpr bool IsPointFormatExtended(unsigned int pointFormat)
{
    return pointFormat >= 6;
}

uint16_t PointFormatSize(unsigned int pointFormat)
{
    switch (pointFormat)
    {
    case 0:
        return 20;
    case 1:
        return 28;
    case 2:
        return 26;
    case 3:
        return 34;
    case 4:
        return 28 + 29;
    case 5:
        return 34 + 29;
    case 6:
        return 30;
    case 7:
        return 36;
    case 8:
        return 38;
    case 9:
        return 30 + 29;
    case 10:
        return 38 + 29;
    default:
        Q_ASSERT(false);
        return 0;
    }
}

uint16_t HeaderSize(unsigned int versionMinor)
{
    switch (versionMinor)
    {
    case 2:
        return 227;
    case 3:
        return 227 + 8;
    case 4:
        return 375;
    default:
        return 227;
    }
}

constexpr const char *LasScalarField::NameFromId(LasScalarField::Id id)
{
    switch (id)
    {
    case Intensity:
        return LasNames::Intensity;
    case ReturnNumber:
        return LasNames::ReturnNumber;
    case NumberOfReturns:
        return LasNames::NumberOfReturns;
    case ScanDirectionFlag:
        return LasNames::ScanDirectionFlag;
    case EdgeOfFlightLine:
        return LasNames::EdgeOfFlightLine;
    case Classification:
        return LasNames::Classification;
    case SyntheticFlag:
        return LasNames::SyntheticFlag;
    case KeypointFlag:
        return LasNames::KeypointFlag;
    case WithheldFlag:
        return LasNames::WithheldFlag;
    case ScanAngleRank:
        return LasNames::ScanAngleRank;
    case UserData:
        return LasNames::UserData;
    case PointSourceId:
        return LasNames::PointSourceId;
    case GpsTime:
        return LasNames::GpsTime;
    case ExtendedScanAngle:
        return LasNames::ScanAngle;
    case ExtendedScannerChannel:
        return LasNames::ScannerChannel;
    case OverlapFlag:
        return LasNames::OverlapFlag;
    case ExtendedClassification:
        return LasNames::Classification;
    case ExtendedReturnNumber:
        return LasNames::ReturnNumber;
    case ExtendedNumberOfReturns:
        return LasNames::NumberOfReturns;
    case NearInfrared:
        return LasNames::NearInfrared;
    }
    throw std::logic_error("unhandled id");
}

LasScalarField::Id LasScalarField::IdFromName(const char *name, unsigned int targetPointFormat)
{
    bool isExtended = IsPointFormatExtended(targetPointFormat);
    if (strcmp(name, LasNames::Intensity) == 0)
    {
        return LasScalarField::Id::Intensity;
    }

    if (strcmp(name, LasNames::ReturnNumber) == 0)
    {
        if (!isExtended)
        {
            return LasScalarField::Id::ReturnNumber;
        }
        else
        {
            return LasScalarField::Id::ExtendedReturnNumber;
        }
    }

    if (strcmp(name, LasNames::NumberOfReturns) == 0)
    {
        if (!isExtended)
        {
            return LasScalarField::Id::NumberOfReturns;
        }
        else
        {
            return LasScalarField::Id::ExtendedNumberOfReturns;
        }
    }

    if (strcmp(name, LasNames::ScanDirectionFlag) == 0)
    {
        return LasScalarField::Id::ScanDirectionFlag;
    }

    if (strcmp(name, LasNames::EdgeOfFlightLine) == 0)
    {
        return LasScalarField::Id::EdgeOfFlightLine;
    }

    if (strcmp(name, LasNames::Classification) == 0)
    {
        if (!isExtended)
        {
            return LasScalarField::Id::Classification;
        }
        else
        {
            return LasScalarField::Id::ExtendedClassification;
        }
    }

    if (strcmp(name, LasNames::SyntheticFlag) == 0)
    {
        return LasScalarField::Id::SyntheticFlag;
    }

    if (strcmp(name, LasNames::KeypointFlag) == 0)
    {
        return LasScalarField::Id::KeypointFlag;
    }

    if (strcmp(name, LasNames::WithheldFlag) == 0)
    {
        return LasScalarField::Id::KeypointFlag;
    }

    if (strcmp(name, LasNames::ScanAngleRank) == 0)
    {
        return LasScalarField::Id::ScanAngleRank;
    }

    if (strcmp(name, LasNames::UserData) == 0)
    {
        return LasScalarField::Id::UserData;
    }

    if (strcmp(name, LasNames::PointSourceId) == 0)
    {
        return LasScalarField::Id::PointSourceId;
    }

    if (strcmp(name, LasNames::GpsTime) == 0)
    {
        return LasScalarField::Id::GpsTime;
    }

    if (strcmp(name, LasNames::ScanAngle) == 0)
    {
        return LasScalarField::Id::ExtendedScanAngle;
    }

    if (strcmp(name, LasNames::ScannerChannel) == 0)
    {
        return LasScalarField::Id::ExtendedScannerChannel;
    }

    if (strcmp(name, LasNames::OverlapFlag) == 0)
    {
        return LasScalarField::Id::OverlapFlag;
    }

    if (strcmp(name, LasNames::NearInfrared) == 0)
    {
        return LasScalarField::Id::NearInfrared;
    }

    ccLog::Warning("Unhandled Name %s", name);
    throw std::logic_error("Unknown name");
}

LasScalarField::Range LasScalarField::ValueRange(LasScalarField::Id id)
{
    switch (id)
    {
    case Intensity:
        return Range::ForType<uint16_t>();
    case ReturnNumber:
        return Range::ForBitSize(3);
    case NumberOfReturns:
        return Range::ForBitSize(3);
    case ScanDirectionFlag:
        return Range::ForBitSize(1);
    case EdgeOfFlightLine:
        return Range::ForBitSize(1);
    case Classification:
        return Range::ForBitSize(5);
    case SyntheticFlag:
        return Range::ForBitSize(1);
    case KeypointFlag:
        return Range::ForBitSize(1);
    case WithheldFlag:
        return Range::ForBitSize(1);
    case ScanAngleRank:
        //  The real range is Range(-90, 90);
        // but we will allow the full range
        return Range::ForType<int8_t>();
    case UserData:
        return Range::ForType<uint8_t>();
    case PointSourceId:
        return Range::ForType<uint16_t>();
    case GpsTime:
        return Range(std::numeric_limits<ScalarType>::lowest(), std::numeric_limits<ScalarType>::max());
    case ExtendedScanAngle:
        return Range(-30'000.0, 30'000.0);
    case ExtendedScannerChannel:
        return Range::ForBitSize(2);
    case OverlapFlag:
        return Range::ForBitSize(1);
    case ExtendedClassification:
        return Range::ForType<uint8_t>();
    case ExtendedReturnNumber:
        return Range::ForBitSize(4);
    case ExtendedNumberOfReturns:
        return Range::ForBitSize(4);
    case NearInfrared:
        return Range::ForType<uint16_t>();
    }

    Q_ASSERT_X(false, __FUNCTION__, "Unhandled las scalar field range");
    return Range::ForType<ScalarType>();
}

LasScalarField::LasScalarField(LasScalarField::Id id, ccScalarField *sf)
    : id(id), sf(sf), range(LasScalarField::ValueRange(id))
{
}

const char *LasScalarField::name() const
{
    return LasScalarField::NameFromId(id);
}

std::vector<LasScalarField> LasScalarFieldForPointFormat(unsigned int pointFormatId)
{
    std::vector<LasScalarField> scalarFields;
    scalarFields.reserve(16);

    if (pointFormatId >= 0 && pointFormatId <= 5)
    {
        scalarFields.emplace_back(LasScalarField::Id::Intensity);
        scalarFields.emplace_back(LasScalarField::Id::ReturnNumber);
        scalarFields.emplace_back(LasScalarField::Id::NumberOfReturns);
        scalarFields.emplace_back(LasScalarField::Id::ScanDirectionFlag);
        scalarFields.emplace_back(LasScalarField::Id::EdgeOfFlightLine);
        scalarFields.emplace_back(LasScalarField::Id::Classification);
        scalarFields.emplace_back(LasScalarField::Id::SyntheticFlag);
        scalarFields.emplace_back(LasScalarField::Id::KeypointFlag);
        scalarFields.emplace_back(LasScalarField::Id::WithheldFlag);
        scalarFields.emplace_back(LasScalarField::Id::ScanAngleRank);
        scalarFields.emplace_back(LasScalarField::Id::UserData);
        scalarFields.emplace_back(LasScalarField::Id::PointSourceId);
    }
    else if (pointFormatId >= 6 && pointFormatId <= 10)
    {
        scalarFields.emplace_back(LasScalarField::Id::Intensity);
        scalarFields.emplace_back(LasScalarField::Id::ExtendedReturnNumber);
        scalarFields.emplace_back(LasScalarField::Id::ExtendedNumberOfReturns);
        scalarFields.emplace_back(LasScalarField::Id::ExtendedScannerChannel);
        scalarFields.emplace_back(LasScalarField::Id::ScanDirectionFlag);
        scalarFields.emplace_back(LasScalarField::Id::EdgeOfFlightLine);
        scalarFields.emplace_back(LasScalarField::Id::ExtendedClassification);
        scalarFields.emplace_back(LasScalarField::Id::SyntheticFlag);
        scalarFields.emplace_back(LasScalarField::Id::KeypointFlag);
        scalarFields.emplace_back(LasScalarField::Id::WithheldFlag);
        scalarFields.emplace_back(LasScalarField::Id::OverlapFlag);
        scalarFields.emplace_back(LasScalarField::Id::ExtendedScanAngle);
        scalarFields.emplace_back(LasScalarField::Id::UserData);
        scalarFields.emplace_back(LasScalarField::Id::PointSourceId);
    }

    if (HasGpsTime(pointFormatId))
    {
        scalarFields.emplace_back(LasScalarField::Id::GpsTime);
    }

    if (HasNearInfrared(pointFormatId))
    {
        scalarFields.emplace_back(LasScalarField::Id::NearInfrared);
    }

    return scalarFields;
}

bool IsLaszipVlr(const laszip_vlr_struct &vlr)
{
    if (strcmp(vlr.user_id, "Laszip encoded") == 0 && vlr.record_id == 22204)
    {
        return true;
    }
    return false;
}

bool IsExtraBytesVlr(const laszip_vlr_struct &vlr)
{
    if (strcmp(vlr.user_id, "LASF_Spec") == 0 && vlr.record_id == 4)
    {
        return true;
    }
    return false;
}

unsigned int SizeOfVlrs(const laszip_vlr_struct *vlrs, unsigned int numVlrs)
{
    return std::accumulate(vlrs,
                           vlrs + numVlrs,
                           0,
                           [](laszip_U32 size, const laszip_vlr_struct &vlr)
                           { return vlr.record_length_after_header + LAS_VLR_HEADER_SIZE + size; });
}

LasExtraScalarField::LasExtraScalarField(QDataStream &dataStream)
{
    dataStream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    uint8_t dataType;
    dataStream.skipRawData(2);
    dataStream >> dataType >> options;
    dataStream.readRawData(name, 32);
    dataStream.skipRawData(4);
    dataStream.readRawData(reinterpret_cast<char *>(noData), 3 * 8);
    dataStream.readRawData(reinterpret_cast<char *>(mins), 3 * 8);
    dataStream.readRawData(reinterpret_cast<char *>(maxs), 3 * 8);
    dataStream >> scales[0] >> scales[1] >> scales[2];
    dataStream >> offsets[0] >> offsets[1] >> offsets[2];
    dataStream.readRawData(reinterpret_cast<char *>(description), 32);

    type = DataTypeFromValue(dataType);
}

void LasExtraScalarField::writeTo(QDataStream &dataStream) const
{
    dataStream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    uint8_t emptyByte{0};
    dataStream << emptyByte << emptyByte;
    dataStream << typeCode() << options;
    dataStream.writeRawData(name, 32);
    dataStream << emptyByte << emptyByte << emptyByte << emptyByte;
    dataStream.writeRawData(reinterpret_cast<const char *>(noData), 3 * 8);
    dataStream.writeRawData(reinterpret_cast<const char *>(mins), 3 * 8);
    dataStream.writeRawData(reinterpret_cast<const char *>(maxs), 3 * 8);
    dataStream << scales[0] << scales[1] << scales[2];
    dataStream << offsets[0] << offsets[1] << offsets[2];
    dataStream.writeRawData(reinterpret_cast<const char *>(description), 32);
}

LasExtraScalarField::DataType LasExtraScalarField::DataTypeFromValue(uint8_t value)
{
    switch (value)
    {
    case 0:
        return DataType::Undocumented;
    case 1:
        return DataType::u8;
    case 3:
        return DataType::u16;
    case 5:
        return DataType::u32;
    case 7:
        return DataType::u64;
    case 2:
        return DataType::i8;
    case 4:
        return DataType::i16;
    case 6:
        return DataType::i32;
    case 8:
        return DataType::i64;
    case 9:
        return DataType::f32;
    case 10:
        return DataType::f64;
    // Array types (2 elements)
    case 11:
        return DataType::u8_2;
    case 13:
        return DataType::u16_2;
    case 15:
        return DataType::u32_2;
    case 17:
        return DataType::u64_2;
    case 12:
        return DataType::i8_2;
    case 14:
        return DataType::i16_2;
    case 16:
        return DataType::i32_2;
    case 18:
        return DataType::i64_2;
    case 19:
        return DataType::f32_2;
    case 20:
        return DataType::f64_2;
    // Array types (3 elements)
    case 21:
        return DataType::u8_3;
    case 23:
        return DataType::u16_3;
    case 25:
        return DataType::u32_3;
    case 27:
        return DataType::u64_3;
    case 22:
        return DataType::i8_3;
    case 24:
        return DataType::i16_3;
    case 26:
        return DataType::i32_3;
    case 28:
        return DataType::i64_3;
    case 29:
        return DataType::f32_3;
    case 30:
        return DataType::f64_3;
    default:
        return DataType::Invalid;
    }
}

unsigned int LasExtraScalarField::elementSize() const
{
    switch (type)
    {
    case Undocumented:
    case u8_3:
    case u8_2:
    case u8:
        return sizeof(uint8_t);
    case u16_3:
    case u16_2:
    case u16:
        return sizeof(uint16_t);
    case u32_3:
    case u32_2:
    case u32:
        return sizeof(uint32_t);
    case u64_3:
    case u64_2:
    case u64:
        return sizeof(uint64_t);
    case i8_3:
    case i8_2:
    case i8:
        return sizeof(int8_t);
    case i16_3:
    case i16_2:
    case i16:
        return sizeof(int16_t);
    case i32_3:
    case i32_2:
    case i32:
        return sizeof(int32_t);
    case i64_3:
    case i64_2:
    case i64:
        return sizeof(int64_t);
    case f32_3:
    case f32_2:
    case f32:
        return sizeof(float);
    case f64_3:
    case f64_2:
    case f64:
        return sizeof(double);
    case Invalid:
        return 0;
    }

    Q_ASSERT_X(false, "elementSize", "Unhandled data type");
    return 0;
}

unsigned int LasExtraScalarField::byteSize() const
{
    return elementSize() * numElements();
}

unsigned int LasExtraScalarField::numElements() const
{
    switch (type)
    {
    case u8:
    case i8:
    case u16:
    case i16:
    case u32:
    case i32:
    case f32:
    case u64:
    case i64:
    case f64:
        return 1;
    case u8_2:
    case i8_2:
    case u16_2:
    case i16_2:
    case u32_2:
    case i32_2:
    case f32_2:
    case u64_2:
    case i64_2:
    case f64_2:
        return 2;
    case u8_3:
    case i8_3:
    case i16_3:
    case u16_3:
    case u32_3:
    case i32_3:
    case f32_3:
    case u64_3:
    case i64_3:
    case f64_3:
        return 3;
    case Undocumented:
        return options;
    case Invalid:
        Q_ASSERT(false);
        return 0;
    }
    Q_ASSERT_X(false, "numElements", "Unhandled data type");
    return 0;
}

std::vector<LasExtraScalarField>
LasExtraScalarField::ParseExtraScalarFields(const laszip_header &laszipHeader)
{
    auto *extraBytesVlr = std::find_if(laszipHeader.vlrs,
                                       laszipHeader.vlrs + laszipHeader.number_of_variable_length_records,
                                       IsExtraBytesVlr);
    if (extraBytesVlr < laszipHeader.vlrs + laszipHeader.number_of_variable_length_records)
    {
        return LasExtraScalarField::ParseExtraScalarFields(*extraBytesVlr);
    }
    return {};
}

std::vector<LasExtraScalarField>
LasExtraScalarField::ParseExtraScalarFields(const laszip_vlr_struct &extraBytesVlr)
{
    if (!IsExtraBytesVlr(extraBytesVlr))
    {
        return {};
    }

    std::vector<LasExtraScalarField> info;
    QByteArray data(reinterpret_cast<char *>(extraBytesVlr.data), extraBytesVlr.record_length_after_header);
    QDataStream dataStream(data);

    int numExtraFields = extraBytesVlr.record_length_after_header / 192;

    unsigned int byteOffset{0};
    for (int j{0}; j < numExtraFields; ++j)
    {
        LasExtraScalarField ebInfo(dataStream);
        ebInfo.byteOffset = byteOffset;

        if (ebInfo.type != DataType::Undocumented && ebInfo.type != ebInfo.DataType::Invalid)
        {
            info.push_back(ebInfo);
        }
        else
        {
            ccLog::Warning("Undocumented or invalid Extra Bytes are not supported");
        }

        byteOffset += ebInfo.byteSize();
        ccLog::Print("[LAS] Extra Bytes: Name: '%s', Type: %s -> Size %d, Offset %d",
                     ebInfo.name,
                     ebInfo.typeName(),
                     ebInfo.byteSize(),
                     ebInfo.byteOffset);
    }
    return info;
}

bool LasExtraScalarField::noDataIsRelevant() const
{
    return options & 1;
}

bool LasExtraScalarField::minIsRelevant() const
{
    return options & 2;
}

bool LasExtraScalarField::maxIsRelevant() const
{
    return options & 4;
}

bool LasExtraScalarField::scaleIsRelevant() const
{
    return options & 8;
}

bool LasExtraScalarField::offsetIsRelevant() const
{
    return options & 16;
}

LasExtraScalarField::Kind LasExtraScalarField::kind() const
{
    switch (type)
    {
    case LasExtraScalarField::Undocumented:
    case LasExtraScalarField::Invalid:
    case LasExtraScalarField::u8:
    case LasExtraScalarField::u16:
    case LasExtraScalarField::u32:
    case LasExtraScalarField::u64:
    case LasExtraScalarField::u8_2:
    case LasExtraScalarField::u16_2:
    case LasExtraScalarField::u32_2:
    case LasExtraScalarField::u64_2:
    case LasExtraScalarField::u8_3:
    case LasExtraScalarField::u16_3:
    case LasExtraScalarField::u32_3:
    case LasExtraScalarField::u64_3:
        return Unsigned;
    case LasExtraScalarField::i8:
    case LasExtraScalarField::i16:
    case LasExtraScalarField::i32:
    case LasExtraScalarField::i64:
    case LasExtraScalarField::i8_2:
    case LasExtraScalarField::i16_2:
    case LasExtraScalarField::i32_2:
    case LasExtraScalarField::i64_2:
    case LasExtraScalarField::i8_3:
    case LasExtraScalarField::i16_3:
    case LasExtraScalarField::i32_3:
    case LasExtraScalarField::i64_3:
        return Signed;
    case LasExtraScalarField::f32:
    case LasExtraScalarField::f64:
    case LasExtraScalarField::f32_2:
    case LasExtraScalarField::f64_2:
    case LasExtraScalarField::f32_3:
    case LasExtraScalarField::f64_3:
        return Floating;
    }
    return Unsigned;
}

void LasExtraScalarField::InitExtraBytesVlr(laszip_vlr_struct &vlr,
                                            const vector<LasExtraScalarField> &extraFields)
{
    strcpy(vlr.user_id, "LASF_Spec");
    vlr.record_id = 4;
    vlr.record_length_after_header = 192 * static_cast<laszip_U16>(extraFields.size());
    std::fill(vlr.description, vlr.description + 32, 0);
    vlr.data = new laszip_U8[vlr.record_length_after_header];

    QByteArray byteArray;
    byteArray.resize(vlr.record_length_after_header);
    QDataStream dataStream(&byteArray, QIODevice::WriteOnly);
    for (const LasExtraScalarField &extraScalarField : extraFields)
    {
        extraScalarField.writeTo(dataStream);
    }
    Q_ASSERT(byteArray.size() == vlr.record_length_after_header);
    std::copy(byteArray.begin(), byteArray.end(), vlr.data);
}

uint8_t LasExtraScalarField::typeCode() const
{
    return static_cast<uint8_t>(type);
}

const char *LasExtraScalarField::typeName() const
{
    switch (type)
    {
    case Undocumented:
        return "Undocumented";
    case u8:
        return "u8";
    case u16:
        return "u16";
    case u32:
        return "u32";
    case u64:
        return "u64";
    case i8:
        return "i8";
    case i16:
        return "i16";
    case i32:
        return "i32";
    case i64:
        return "i64";
    case f32:
        return "f32";
    case f64:
        return "f64";
    case u8_2:
        return "u8[2]";
    case u16_2:
        return "u16[2]";
    case u32_2:
        return "u32[2]";
    case u64_2:
        return "u64[2]";
    case i8_2:
        return "i8[2]";
    case i16_2:
        return "i16[2]";
    case i32_2:
        return "i32[2]";
    case i64_2:
        return "i64[2]";
    case f32_2:
        return "f32[2]";
    case f64_2:
        return "f64[2]";
    case u8_3:
        return "u8[3]";
    case u16_3:
        return "u16[3]";
    case u32_3:
        return "u32[3]";
    case u64_3:
        return "u64[3]";
    case i8_3:
        return "i8[3]";
    case i16_3:
        return "i16[3]";
    case i32_3:
        return "i32[3]";
    case i64_3:
        return "i64[3]";
    case f32_3:
        return "f32[3]";
    case f64_3:
        return "f64[3]";
    case Invalid:
        return "Invalid";
    }
    return "";
}

unsigned int
LasExtraScalarField::TotalExtraBytesSize(const std::vector<LasExtraScalarField> &extraScalarFields)
{
    return std::accumulate(extraScalarFields.begin(),
                           extraScalarFields.end(),
                           0,
                           [](unsigned int sum, const LasExtraScalarField &field)
                           { return sum + field.byteSize(); });
}

void LasExtraScalarField::resetScalarFieldsPointers()
{
    for (int i{0}; i < 3; ++i)
    {
        scalarFields[i] = nullptr;
    }
}

void LasExtraScalarField::UpdateByteOffsets(vector<LasExtraScalarField> &extraFields)
{
    unsigned int byteOffset{0};
    for (LasExtraScalarField &extraScalarField : extraFields)
    {
        extraScalarField.byteOffset = byteOffset;
        byteOffset += extraScalarField.byteSize();
    }
}
void LasExtraScalarField::MatchExtraBytesToScalarFields(vector<LasExtraScalarField> &extraScalarFields,
                                                        const ccPointCloud &pointCloud)
{
    for (LasExtraScalarField &extraScalarField : extraScalarFields)
    {
        if (extraScalarField.numElements() > 1)
        {
            // Array fields are split into multiple ccScalarField
            // and each of them has the index appended to the name
            char name[50];
            unsigned int found{0};
            for (unsigned int i = 0; i < extraScalarField.numElements(); ++i)
            {
                snprintf(name, 50, "%s [%d]", extraScalarField.name, i);
                int pos = pointCloud.getScalarFieldIndexByName(name);
                if (pos >= 0)
                {
                    extraScalarField.scalarFields[i] =
                        dynamic_cast<ccScalarField *>(pointCloud.getScalarField(pos));
                    found++;
                    ccLog::Warning("[LAS] field %s found", name);
                }
                else
                {
                    ccLog::Warning("[LAS] field %s not found", name);
                    extraScalarField.scalarFields[i] = nullptr;
                }
            }
        }
        else
        {
            const char *nameToSearch;
            if (!extraScalarField.ccName.empty())
            {
                // This field's name clashed with existing ccScalarField when created
                nameToSearch = extraScalarField.ccName.c_str();
            }
            else
            {
                nameToSearch = extraScalarField.name;
            }
            int pos = pointCloud.getScalarFieldIndexByName(nameToSearch);
            if (pos >= 0)
            {
                extraScalarField.scalarFields[0] =
                    dynamic_cast<ccScalarField *>(pointCloud.getScalarField(pos));
            }
            else
            {
                ccLog::Warning("[LAS] field %s not found", nameToSearch);
            }
        }
    }
    // Remove any Extra Scalar Field for which we could not get _all_ the corresponding
    // ccScalarField
    const auto notAllScalarFieldWereFound = [](const LasExtraScalarField &extraScalarField)
    {
        const auto ptrIsNull = [](const ccScalarField *ptr) { return ptr == nullptr; };
        return std::any_of(extraScalarField.scalarFields,
                           extraScalarField.scalarFields + extraScalarField.numElements(),
                           ptrIsNull);
    };

    auto firstToRemove =
        std::remove_if(extraScalarFields.begin(), extraScalarFields.end(), notAllScalarFieldWereFound);
    extraScalarFields.erase(firstToRemove, extraScalarFields.end());
}

bool EvlrHeader::isWaveFormDataPackets() const
{
    return recordID == 65'535 && strncmp(userID, "LASF_Spec", EvlrHeader::USER_ID_SIZE) == 0;
}

EvlrHeader EvlrHeader::Waveform()
{
    EvlrHeader self;
    self.recordID = 65'535;
    strncpy(self.userID, "LASF_Spec", EvlrHeader::USER_ID_SIZE);
    strncpy(self.description, "Waveform Data Packets", EvlrHeader::DESCRIPTION_SIZE);
    self.recordLength = 0;
    return self;
}

QDataStream &operator>>(QDataStream &stream, EvlrHeader &hdr)
{
    stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    uint16_t reserved;
    stream >> reserved;
    stream.readRawData(hdr.userID, EvlrHeader::USER_ID_SIZE);
    stream >> hdr.recordID;
    quint64 recordLength_;
    stream >> recordLength_;
    hdr.recordLength = recordLength_;
    stream.readRawData(hdr.description, EvlrHeader::DESCRIPTION_SIZE);

    return stream;
};

QDataStream &operator<<(QDataStream &stream, const EvlrHeader &hdr)
{
    uint16_t reserved{0};
    stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    stream << reserved;
    stream.writeRawData(hdr.userID, EvlrHeader::USER_ID_SIZE);
    stream << hdr.recordID;
    stream << (quint64)hdr.recordLength;
    stream.writeRawData(hdr.description, EvlrHeader::DESCRIPTION_SIZE);

    return stream;
}

LasVersion SelectBestVersion(const ccPointCloud &cloud)
{
    bool isExtendedRequired{false};
    // These are exclusive to 'extended' point formats (>= 6)
    bool hasOverlapFlag = cloud.getScalarFieldIndexByName(LasNames::OverlapFlag) != -1;
    bool hasNIR = cloud.getScalarFieldIndexByName(LasNames::NearInfrared) != -1;
    bool hasScannerChannel = cloud.getScalarFieldIndexByName(LasNames::ScannerChannel) != -1;
    bool hasScanAngle = cloud.getScalarFieldIndexByName(LasNames::ScanAngle) != -1;

    isExtendedRequired = hasOverlapFlag || hasNIR || hasScannerChannel || hasScanAngle;

    if (!isExtendedRequired)
    {
        // We may need extended point format because some fields need to store more value
        // than what is possible on non-extended point format.
        int classificationIdx = cloud.getScalarFieldIndexByName(LasNames::Classification);
        if (classificationIdx != -1)
        {
            const CCCoreLib::ScalarField *classification = cloud.getScalarField(classificationIdx);
            if (classification->getMax() > LasScalarField::ValueRange(LasScalarField::Classification).max)
            {
                isExtendedRequired = true;
            }
        }

        int returnNumberIdx = cloud.getScalarFieldIndexByName(LasNames::ReturnNumber);
        if (returnNumberIdx != -1)
        {
            const CCCoreLib::ScalarField *returnNumber = cloud.getScalarField(returnNumberIdx);
            if (returnNumber->getMax() >
                LasScalarField::ValueRange(LasScalarField::LasScalarField::ReturnNumber).max)
            {
                isExtendedRequired = true;
            }
        }

        int numReturnsIdx = cloud.getScalarFieldIndexByName(LasNames::NumberOfReturns);
        if (numReturnsIdx != -1)
        {
            const CCCoreLib::ScalarField *numReturns = cloud.getScalarField(numReturnsIdx);
            if (numReturns->getMax() > LasScalarField::ValueRange(LasScalarField::NumberOfReturns).max)
            {
                isExtendedRequired = true;
            }
        }
    }

    bool hasRGB = cloud.hasColors();
    bool hasWaveform = cloud.hasFWF();

    if (isExtendedRequired)
    {
        int pointFormat = 6;
        if (hasWaveform)
        {
            if (hasRGB)
            {
                pointFormat = 10;
            }
            else
            {
                pointFormat = 9;
            }
        }
        else
        {
            if (hasRGB && hasNIR)
            {
                pointFormat = 8;
            }
            else if (hasRGB && !hasNIR)
            {
                pointFormat = 7;
            }
        }
        return {pointFormat, 4};
    }
    else
    {
        bool hasGpsTime = cloud.getScalarFieldIndexByName(LasNames::GpsTime) != -1;
        int pointFormat = 0;
        if (hasWaveform)
        {
            if (hasGpsTime)
            {
                pointFormat = 4;
            }
            else if (hasRGB)
            {
                pointFormat = 5;
            }
        }
        else
        {
            if (hasGpsTime)
            {
                pointFormat += 1;
            }
            if (hasRGB)
            {
                pointFormat += 2;
            }
        }
        return {pointFormat, 2};
    }
}

void CloneVlrInto(const laszip_vlr_struct &src, laszip_vlr_struct &dst)
{
    dst = src;
    dst.data = new laszip_U8[src.record_length_after_header];
    std::copy(src.data, src.data + src.record_length_after_header, dst.data);
}
