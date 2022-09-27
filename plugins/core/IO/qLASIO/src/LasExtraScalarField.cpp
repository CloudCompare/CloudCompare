#include "LasExtraScalarField.h"
#include "LasDetails.h"

#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

#include <QDataStream>

#include <laszip/laszip_api.h>

#include <cstring>
#include <stdexcept>
#include <vector>


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
                                       LasDetails::IsExtraBytesVlr);
    if (extraBytesVlr < laszipHeader.vlrs + laszipHeader.number_of_variable_length_records)
    {
        return LasExtraScalarField::ParseExtraScalarFields(*extraBytesVlr);
    }
    return {};
}

std::vector<LasExtraScalarField>
LasExtraScalarField::ParseExtraScalarFields(const laszip_vlr_struct &extraBytesVlr)
{
    if (!LasDetails::IsExtraBytesVlr(extraBytesVlr))
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
