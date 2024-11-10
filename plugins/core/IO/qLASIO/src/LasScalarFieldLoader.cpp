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

// qCC_db
#include <ccScalarField.h>
// System
#include <utility>

// TODO take by move
LasScalarFieldLoader::LasScalarFieldLoader(std::vector<LasScalarField>&      standardScalarFields,
                                           std::vector<LasExtraScalarField>& extraScalarFields,
                                           ccPointCloud&                     pointCloud)
    : m_standardFields(standardScalarFields)
    , m_extraScalarFields(extraScalarFields)
{
	createScalarFieldsForExtraBytes(pointCloud);
}

CC_FILE_ERROR LasScalarFieldLoader::handleScalarFields(ccPointCloud&       pointCloud,
                                                       const laszip_point& currentPoint)
{
	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	for (LasScalarField& lasScalarField : m_standardFields)
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
		{
			laszip_U8 classification = currentPoint.classification;
			if (!m_decomposeClassification)
			{
				classification |= (currentPoint.synthetic_flag << 5);
				classification |= (currentPoint.keypoint_flag << 6);
				classification |= (currentPoint.withheld_flag << 7);
			}
			error = handleScalarField(lasScalarField, pointCloud, classification);
			break;
		}
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
			error = handleScalarField(lasScalarField, pointCloud, currentPoint.gps_time);
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
			    handleScalarField(lasScalarField, pointCloud, currentPoint.extended_classification_flags & LasDetails::OVERLAP_FLAG_BIT_MASK);
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
CC_FILE_ERROR LasScalarFieldLoader::parseExtraScalarField(
    const LasExtraScalarField& extraField,
    const laszip_point&        currentPoint,
    ScalarType                 outputValues[3])
{

	if (currentPoint.num_extra_bytes <= 0 || currentPoint.extra_bytes == nullptr)
	{
		return CC_FERR_NO_ERROR;
	}

	if (extraField.byteOffset + extraField.byteSize() > static_cast<unsigned>(currentPoint.num_extra_bytes))
	{
		assert(false);
		return CC_FERR_READING;
	}

	laszip_U8* dataStart = currentPoint.extra_bytes + extraField.byteOffset;
	parseRawValues(extraField, dataStart);
	switch (extraField.kind())
	{
	case LasExtraScalarField::Unsigned:
		handleOptionsFor(extraField, m_rawValues.unsignedValues, outputValues);
		break;
	case LasExtraScalarField::Signed:
		handleOptionsFor(extraField, m_rawValues.signedValues, outputValues);
		break;
	case LasExtraScalarField::Floating:
		handleOptionsFor(extraField, m_rawValues.floatingValues, outputValues);
		break;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LasScalarFieldLoader::handleRGBValue(ccPointCloud& pointCloud, const laszip_point& currentPoint)
{
	if (!pointCloud.hasColors())
	{
		uint16_t currentOredRGB = currentPoint.rgb[0] | currentPoint.rgb[1] | currentPoint.rgb[2];
		if (m_ignoreFieldsWithDefaultValues && currentOredRGB == 0)
		{
			// nothing to do
			return CC_FERR_NO_ERROR;
		}

		if (!pointCloud.reserveTheRGBTable())
		{
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		if (!m_force8bitRgbMode && currentOredRGB > 255)
		{
			// LAS colors use 16bits (as they should)
			m_colorCompShift = 8;
		}

		if (pointCloud.size() != 0)
		{
			// set the previous points color (black by default)
			for (unsigned j = 0; j < pointCloud.size() - 1; ++j)
			{
				pointCloud.addColor(ccColor::blackRGB);
			}
		}
	}

	assert(pointCloud.hasColors());

	auto red   = static_cast<ColorCompType>(currentPoint.rgb[0] >> m_colorCompShift);
	auto green = static_cast<ColorCompType>(currentPoint.rgb[1] >> m_colorCompShift);
	auto blue  = static_cast<ColorCompType>(currentPoint.rgb[2] >> m_colorCompShift);
	pointCloud.addColor(ccColor::Rgb(red, green, blue));

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LasScalarFieldLoader::handleExtraScalarFields(const laszip_point& currentPoint)
{
	if (currentPoint.num_extra_bytes <= 0 || currentPoint.extra_bytes == nullptr)
	{
		return CC_FERR_NO_ERROR;
	}

	for (const LasExtraScalarField& extraField : m_extraScalarFields)
	{
		ScalarType finalValues[3]{0};

		const CC_FILE_ERROR err = parseExtraScalarField(extraField, currentPoint, finalValues);
		if (err != CC_FERR_NO_ERROR)
		{
			return err;
		}

		for (unsigned dimIndex = 0; dimIndex < extraField.numElements(); ++dimIndex)
		{
			if (extraField.scalarFields[dimIndex])
			{
				extraField.scalarFields[dimIndex]->addElement(finalValues[dimIndex]);
			}
		}
	}
	return CC_FERR_NO_ERROR;
}

template <typename T>
CC_FILE_ERROR
LasScalarFieldLoader::handleScalarField(LasScalarField& sfInfo, ccPointCloud& pointCloud, T currentValue)
{
	if (!sfInfo.sf)
	{
		if (m_ignoreFieldsWithDefaultValues && currentValue == T{})
		{
			return CC_FERR_NO_ERROR;
		}
		auto newSf = new ccScalarField(sfInfo.name());
		sfInfo.sf  = newSf;
		if (!newSf->reserveSafe(pointCloud.capacity()))
		{
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		for (unsigned j = 0; j < pointCloud.size() - 1; ++j)
		{
			newSf->addElement(static_cast<ScalarType>(T{}));
		}
	}

	if (sfInfo.sf)
	{
		sfInfo.sf->addElement(static_cast<ScalarType>(currentValue));
	}
	return CC_FERR_NO_ERROR;
}

bool LasScalarFieldLoader::createScalarFieldsForExtraBytes(ccPointCloud& pointCloud)
{
	for (LasExtraScalarField& extraField : m_extraScalarFields)
	{
		switch (extraField.numElements())
		{
		case 1:
			if (pointCloud.getScalarFieldIndexByName(extraField.name) != -1)
			{
				char name[LasExtraScalarField::MAX_NAME_SIZE + 8];
				snprintf(name, LasExtraScalarField::MAX_NAME_SIZE + 8, "%s (Extra)", extraField.name);
				extraField.scalarFields[0] = new ccScalarField(name);
				memcpy(extraField.ccName, name, LasExtraScalarField::MAX_NAME_SIZE + 8);
			}
			else
			{
				extraField.scalarFields[0] = new ccScalarField(extraField.name);
			}

			if (!extraField.scalarFields[0]->reserveSafe(pointCloud.capacity()))
			{
				return false;
			}
			break;
		case 2:
		case 3:
			for (unsigned dimIndex{0}; dimIndex < extraField.numElements(); ++dimIndex)
			{
				char name[LasExtraScalarField::MAX_NAME_SIZE + 8];
				snprintf(name, LasExtraScalarField::MAX_NAME_SIZE + 8, "%s [%d]", extraField.name, dimIndex);
				extraField.scalarFields[dimIndex] = new ccScalarField(name);
				if (!extraField.scalarFields[dimIndex]->reserveSafe(pointCloud.capacity()))
				{
					return false;
				}
			}
			break;
		}
		if (extraField.offsetIsRelevant())
		{
			for (unsigned i = 0; i < extraField.numElements(); ++i)
			{
				extraField.scalarFields[i]->setOffset(extraField.offsets[i]);
			}
		}
	}
	return true;
}

template <typename T>
ScalarType LasScalarFieldLoader::ParseValueOfType(uint8_t* source)
{
	return static_cast<ScalarType>(*reinterpret_cast<T*>(source));
}

template <typename T, typename V>
V LasScalarFieldLoader::ParseValueOfTypeAs(const uint8_t* source)
{
	return static_cast<V>(*reinterpret_cast<const T*>(source));
}

void LasScalarFieldLoader::parseRawValues(const LasExtraScalarField& extraField, const uint8_t* dataStart)
{
	for (unsigned i = 0; i < extraField.numElements(); ++i)
	{
		switch (extraField.type)
		{
		case LasExtraScalarField::Invalid:
		case LasExtraScalarField::Undocumented:
			break;
		case LasExtraScalarField::u8:
			m_rawValues.unsignedValues[i] = ParseValueOfTypeAs<uint8_t, uint64_t>(dataStart);
			break;
		case LasExtraScalarField::u16:
			m_rawValues.unsignedValues[i] = ParseValueOfTypeAs<uint16_t, uint64_t>(dataStart);
			break;
		case LasExtraScalarField::u32:
			m_rawValues.unsignedValues[i] = ParseValueOfTypeAs<uint32_t, uint64_t>(dataStart);
			break;
		case LasExtraScalarField::u64:
			m_rawValues.unsignedValues[i] = ParseValueOfTypeAs<uint64_t, uint64_t>(dataStart);
			break;
		case LasExtraScalarField::i8:
			m_rawValues.signedValues[i] = ParseValueOfTypeAs<int8_t, int64_t>(dataStart);
			break;
		case LasExtraScalarField::i16:
			m_rawValues.signedValues[i] = ParseValueOfTypeAs<int16_t, int64_t>(dataStart);
			break;
		case LasExtraScalarField::i32:
			m_rawValues.signedValues[i] = ParseValueOfTypeAs<int32_t, int64_t>(dataStart);
			break;
		case LasExtraScalarField::i64:
			m_rawValues.signedValues[i] = ParseValueOfTypeAs<int64_t, int64_t>(dataStart);
			break;
		case LasExtraScalarField::f32:
			m_rawValues.floatingValues[i] = ParseValueOfTypeAs<float, double>(dataStart);
			break;
		case LasExtraScalarField::f64:
			m_rawValues.floatingValues[i] = ParseValueOfTypeAs<double, double>(dataStart);
			break;
		}
		dataStart += extraField.elementSize();
	}
}

template <typename T>
void LasScalarFieldLoader::handleOptionsFor(const LasExtraScalarField& extraField, T inputValues[3], ScalarType outputValues[3])
{
	assert(extraField.numElements() <= 3);
	for (unsigned dimIndex = 0; dimIndex < extraField.numElements(); ++dimIndex)
	{
		if (extraField.noDataIsRelevant())
		{
			auto noDataValue = ParseValueOfTypeAs<T, T>(static_cast<const uint8_t*>(extraField.noData[dimIndex]));
			if (noDataValue == inputValues[dimIndex])
			{
				outputValues[dimIndex] = ccScalarField::NaN();
			}
			else
			{
				outputValues[dimIndex] = static_cast<ScalarType>(inputValues[dimIndex]);
			}
		}

		if (extraField.scaleIsRelevant())
		{
			double scaledValue     = (inputValues[dimIndex] * extraField.scales[dimIndex]) + (extraField.offsets[dimIndex]);
			outputValues[dimIndex] = static_cast<ScalarType>(scaledValue);
		}
		else
		{
			outputValues[dimIndex] = static_cast<ScalarType>(inputValues[dimIndex]);
		}
	}
}
