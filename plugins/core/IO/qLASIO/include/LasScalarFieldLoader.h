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
#include "LasScalarField.h"

// qCC_db
#include <FileIOFilter.h>
#include <ccPointCloud.h>

// Qt
#include <QFileInfo>

// LASzip
#include <laszip/laszip_api.h>

/// Class with the logic on how to load LAS dimensions values
/// from the LAS file into a ccPointCloud's scalar field.
///
/// This also handles LAS extra scalar fields, as well as RGB.
class LasScalarFieldLoader
{
  public:
	LasScalarFieldLoader(std::vector<LasScalarField>&      standardScalarFields,
	                     std::vector<LasExtraScalarField>& extraScalarFields,
	                     ccPointCloud&                     pointCloud);

	CC_FILE_ERROR handleScalarFields(ccPointCloud& pointCloud, const laszip_point& currentPoint);

	/// Parses the extra scalar field described by extraField, from currentPoint, into outputValues
	CC_FILE_ERROR parseExtraScalarField(const LasExtraScalarField& extraField, const laszip_point& currentPoint, ScalarType outputValues[3]);

	/// In LAS files, the red, green and blue channels are normal LAS fields,
	/// however in CloudCompare RGB is handled differently.
	CC_FILE_ERROR handleRGBValue(ccPointCloud& pointCloud, const laszip_point& currentPoint);

	CC_FILE_ERROR handleExtraScalarFields(const laszip_point& currentPoint);

	inline void setIgnoreFieldsWithDefaultValues(bool state)
	{
		m_ignoreFieldsWithDefaultValues = state;
	}

	inline void setForce8bitRgbMode(bool state)
	{
		m_force8bitRgbMode = state;
	}

	/// Sets whether the classification field should be decomposed into
	/// the classification, synthetic flag, key_point flag, withheld flag.
	///
	/// Only applies to point format <= 5 (ie field Classification, not ExtendedClassification)
	inline void setDecomposeClassification(bool state)
	{
		m_decomposeClassification = state;
	}

	inline const std::vector<LasScalarField>& standardFields() const
	{
		return m_standardFields;
	}

	inline const std::vector<LasExtraScalarField>& extraFields() const
	{
		return m_extraScalarFields;
	}

  private:
	/// Handles loading of LAS value into the scalar field that will be part
	/// of the pointCloud.
	///
	/// sfInfo: Info about the current scalar field we are loading the value into
	/// pointCloud: The point cloud where the scalar field will be loaded into
	/// currentValue: The current value of the LAS field we are loading.
	template <typename T>
	CC_FILE_ERROR handleScalarField(LasScalarField& sfInfo, ccPointCloud& pointCloud, T currentValue);

	/// creates the ccScalarFields that correspond to the LAS extra dimensions
	bool createScalarFieldsForExtraBytes(ccPointCloud& pointCloud);

	/// Re-interprets the bytes from the source as a value of type `T`
	/// and then cast the resulting value from the type T to ScalarType.
	template <typename T>
	static ScalarType ParseValueOfType(uint8_t* source);

	/// Re-interprets the bytes from the source as a value of type `T`
	/// and then cast the resulting value from the type T to a value of type `V`.
	template <typename T, typename V>
	static V ParseValueOfTypeAs(const uint8_t* source);

	/// Loads the values for the LAS extra field of the current point from the dataStart source.
	///
	/// The loaded values are stored into the member variable `rawValues`
	void parseRawValues(const LasExtraScalarField& extraField, const uint8_t* dataStart);

	template <typename T>
	void handleOptionsFor(const LasExtraScalarField& extraField, T inputValues[3], ScalarType outputValues[3]);

  private:
	bool                              m_force8bitRgbMode{false};
	bool                              m_decomposeClassification{true};
	bool                              m_ignoreFieldsWithDefaultValues{true};
	unsigned char                     m_colorCompShift{0};
	std::vector<LasScalarField>&      m_standardFields;
	std::vector<LasExtraScalarField>& m_extraScalarFields;

	union
	{
		uint64_t unsignedValues[LasExtraScalarField::MAX_DIM_SIZE];
		int64_t  signedValues[LasExtraScalarField::MAX_DIM_SIZE];
		double   floatingValues[LasExtraScalarField::MAX_DIM_SIZE];
	} m_rawValues{};
};
