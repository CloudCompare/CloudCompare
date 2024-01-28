#include "LasExtraScalarField.h"

#include "LasDetails.h"

// qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
// LASzip
#include <laszip/laszip_api.h>
// Qt
#include <QDataStream>
// System
#include <cstring>
#include <stdexcept>

QDataStream& operator>>(QDataStream& dataStream, LasExtraScalarField& extraScalarField)
{
	dataStream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

	uint8_t dataType = 0;
	dataStream.skipRawData(2);
	dataStream >> dataType >> extraScalarField.options;
	dataStream.readRawData(extraScalarField.name, 32);
	dataStream.skipRawData(4);
	dataStream.readRawData(reinterpret_cast<char*>(extraScalarField.noData), 3 * 8);
	dataStream.readRawData(reinterpret_cast<char*>(extraScalarField.mins), 3 * 8);
	dataStream.readRawData(reinterpret_cast<char*>(extraScalarField.maxs), 3 * 8);
	dataStream >> extraScalarField.scales[0] >> extraScalarField.scales[1] >> extraScalarField.scales[2];
	dataStream >> extraScalarField.offsets[0] >> extraScalarField.offsets[1] >> extraScalarField.offsets[2];
	dataStream.readRawData(reinterpret_cast<char*>(extraScalarField.description), 32);

	auto type_and_dim_size      = LasExtraScalarField::DataTypeFromValue(dataType);
	extraScalarField.type       = std::get<0>(type_and_dim_size);
	extraScalarField.dimensions = std::get<1>(type_and_dim_size);

	return dataStream;
}

QDataStream& operator<<(QDataStream& dataStream, const LasExtraScalarField& extraScalarField)
{
	dataStream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

	uint8_t emptyByte{0};
	dataStream << emptyByte << emptyByte;
	dataStream << extraScalarField.typeCode() << extraScalarField.options;
	dataStream.writeRawData(extraScalarField.name, 32);
	dataStream << emptyByte << emptyByte << emptyByte << emptyByte;
	dataStream.writeRawData(reinterpret_cast<const char*>(extraScalarField.noData), 3 * 8);
	dataStream.writeRawData(reinterpret_cast<const char*>(extraScalarField.mins), 3 * 8);
	dataStream.writeRawData(reinterpret_cast<const char*>(extraScalarField.maxs), 3 * 8);
	dataStream << extraScalarField.scales[0] << extraScalarField.scales[1] << extraScalarField.scales[2];
	dataStream << extraScalarField.offsets[0] << extraScalarField.offsets[1] << extraScalarField.offsets[2];
	dataStream.writeRawData(reinterpret_cast<const char*>(extraScalarField.description), 32);

	return dataStream;
}

std::tuple<LasExtraScalarField::DataType, LasExtraScalarField::DimensionSize>
LasExtraScalarField::DataTypeFromValue(uint8_t value)
{
	if (value >= 31)
	{
		return {DataType::Invalid, DimensionSize::One};
	}

	if (value == 0)
	{
		return {DataType::Undocumented, DimensionSize::One};
	}

	DimensionSize dimSize = DimensionSize::One;

	// value is now in range [1..30]

	if (value >= 21)
	{
		value -= 20;
		dimSize = DimensionSize::Three;
	}

	if (value >= 11)
	{
		value -= 10;
		dimSize = DimensionSize::Two;
	}

	// value is now be in range [1..10]
	DataType dataType;
	switch (value)
	{
	case 1:
		dataType = DataType::u8;
		break;
	case 3:
		dataType = DataType::u16;
		break;
	case 5:
		dataType = DataType::u32;
		break;
	case 7:
		dataType = DataType::u64;
		break;
	case 2:
		dataType = DataType::i8;
		break;
	case 4:
		dataType = DataType::i16;
		break;
	case 6:
		dataType = DataType::i32;
		break;
	case 8:
		dataType = DataType::i64;
		break;
	case 9:
		dataType = DataType::f32;
		break;
	case 10:
		dataType = DataType::f64;
		break;
	}

	return {dataType, dimSize};
}

unsigned LasExtraScalarField::elementSize() const
{
	switch (type)
	{
	case Undocumented:
	case u8:
		return sizeof(uint8_t);
	case u16:
		return sizeof(uint16_t);
	case u32:
		return sizeof(uint32_t);
	case u64:
		return sizeof(uint64_t);
	case i8:
		return sizeof(int8_t);
	case i16:
		return sizeof(int16_t);
	case i32:
		return sizeof(int32_t);
	case i64:
		return sizeof(int64_t);
	case f32:
		return sizeof(float);
	case f64:
		return sizeof(double);
	case Invalid:
		return 0;
	}

	Q_ASSERT_X(false, "elementSize", "Unhandled data type");
	return 0;
}

unsigned LasExtraScalarField::byteSize() const
{
	return elementSize() * numElements();
}

unsigned LasExtraScalarField::numElements() const
{
	return static_cast<unsigned>(dimensions);
}

std::vector<LasExtraScalarField>
LasExtraScalarField::ParseExtraScalarFields(const laszip_header& laszipHeader)
{
	auto* extraBytesVlr = std::find_if(laszipHeader.vlrs,
	                                   laszipHeader.vlrs + laszipHeader.number_of_variable_length_records,
	                                   LasDetails::IsExtraBytesVlr);
	if (extraBytesVlr < laszipHeader.vlrs + laszipHeader.number_of_variable_length_records)
	{
		return LasExtraScalarField::ParseExtraScalarFields(*extraBytesVlr);
	}
	return {};
}

std::vector<LasExtraScalarField>
LasExtraScalarField::ParseExtraScalarFields(const laszip_vlr_struct& extraBytesVlr)
{
	if (!LasDetails::IsExtraBytesVlr(extraBytesVlr))
	{
		return {};
	}

	std::vector<LasExtraScalarField> info;
	QByteArray                       data(reinterpret_cast<char*>(extraBytesVlr.data), extraBytesVlr.record_length_after_header);
	QDataStream                      dataStream(data);

	uint16_t numExtraFields = extraBytesVlr.record_length_after_header / 192;

	unsigned byteOffset{0};
	for (uint16_t j = 0; j < numExtraFields; ++j)
	{
		LasExtraScalarField ebInfo;
		dataStream >> ebInfo;
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
		             ebInfo.typeName().c_str(),
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

void LasExtraScalarField::setScaleIsRelevant(bool isRelevant)
{
	if (isRelevant)
	{
		options |= 8;
	}
	else
	{
		options &= ~8;
	}
}

void LasExtraScalarField::setOffsetIsRelevant(bool isRelevant)
{
	if (isRelevant)
	{
		options |= 16;
	}
	else
	{
		options &= ~16;
	}
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
		return Unsigned;
	case LasExtraScalarField::i8:
	case LasExtraScalarField::i16:
	case LasExtraScalarField::i32:
	case LasExtraScalarField::i64:
		return Signed;
	case LasExtraScalarField::f32:
	case LasExtraScalarField::f64:
		return Floating;
	}
	return Unsigned;
}

void LasExtraScalarField::InitExtraBytesVlr(laszip_vlr_struct&                 vlr,
                                            const vector<LasExtraScalarField>& extraFields)
{
	strcpy(vlr.user_id, "LASF_Spec");
	vlr.record_id                  = 4;
	vlr.record_length_after_header = 192 * static_cast<laszip_U16>(extraFields.size());
	std::fill(vlr.description, vlr.description + 32, 0);
	vlr.data = new laszip_U8[vlr.record_length_after_header];

	QByteArray byteArray;
	byteArray.resize(vlr.record_length_after_header);
	QDataStream dataStream(&byteArray, QIODevice::WriteOnly);
	for (const LasExtraScalarField& extraScalarField : extraFields)
	{
		dataStream << extraScalarField;
	}
	Q_ASSERT(byteArray.size() == vlr.record_length_after_header);
	std::copy(byteArray.begin(), byteArray.end(), vlr.data);
}

uint8_t LasExtraScalarField::typeCode() const
{
	Q_ASSERT(type != DataType::Invalid);
	uint8_t code = static_cast<uint8_t>(type);
	code += (10 * (numElements() - 1));
	return code;
}

std::string LasExtraScalarField::typeName() const
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
	case Invalid:
		return "Invalid";
	}
	return "";
}

unsigned LasExtraScalarField::TotalExtraBytesSize(const std::vector<LasExtraScalarField>& extraScalarFields)
{
	return std::accumulate(extraScalarFields.begin(),
	                       extraScalarFields.end(),
	                       0,
	                       [](unsigned sum, const LasExtraScalarField& field)
	                       { return sum + field.byteSize(); });
}

void LasExtraScalarField::resetScalarFieldsPointers()
{
	for (unsigned i = 0; i < MAX_DIM_SIZE; ++i)
	{
		scalarFields[i] = nullptr;
	}
}

void LasExtraScalarField::UpdateByteOffsets(vector<LasExtraScalarField>& extraFields)
{
	unsigned byteOffset{0};
	for (LasExtraScalarField& extraScalarField : extraFields)
	{
		extraScalarField.byteOffset = byteOffset;
		byteOffset += extraScalarField.byteSize();
	}
}

void LasExtraScalarField::MatchExtraBytesToScalarFields(vector<LasExtraScalarField>& extraScalarFields,
                                                        const ccPointCloud&          pointCloud)
{
	for (LasExtraScalarField& extraScalarField : extraScalarFields)
	{
		if (extraScalarField.numElements() > 1)
		{
			if (extraScalarField.numElements() > MAX_DIM_SIZE)
			{
				assert(false);
				continue;
			}

			// Array fields are split into multiple ccScalarField
			// and each of them has the index appended to the name
			char     name[50];
			unsigned found{0};
			for (unsigned i = 0; i < extraScalarField.numElements(); ++i)
			{
				snprintf(name, 50, "%s [%d]", extraScalarField.name, i);
				int pos = pointCloud.getScalarFieldIndexByName(name);
				if (pos >= 0)
				{
					extraScalarField.scalarFields[i] = dynamic_cast<ccScalarField*>(pointCloud.getScalarField(pos));
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
			const char* nameToSearch = nullptr;
			if (extraScalarField.ccName[0] != 0)
			{
				// This field's name clashed with existing ccScalarField when created
				nameToSearch = extraScalarField.ccName;
			}
			else
			{
				nameToSearch = extraScalarField.name;
			}
			int pos = pointCloud.getScalarFieldIndexByName(nameToSearch);
			if (pos >= 0)
			{
				extraScalarField.scalarFields[0] = dynamic_cast<ccScalarField*>(pointCloud.getScalarField(pos));
			}
			else
			{
				ccLog::Warning("[LAS] field %s not found", nameToSearch);
			}
		}
	}
	// Remove any Extra Scalar Field for which we could not get _all_ the corresponding
	// ccScalarField
	const auto notAllScalarFieldWereFound = [](const LasExtraScalarField& extraScalarField)
	{
		const auto ptrIsNull = [](const ccScalarField* ptr)
		{
			return ptr == nullptr;
		};
		return std::any_of(extraScalarField.scalarFields,
		                   extraScalarField.scalarFields + extraScalarField.numElements(),
		                   ptrIsNull);
	};

	auto firstToRemove = std::remove_if(extraScalarFields.begin(), extraScalarFields.end(), notAllScalarFieldWereFound);

	extraScalarFields.erase(firstToRemove, extraScalarFields.end());
}
