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

#include <cstdint>
#include <string>
#include <vector>

class QDataStream;

class ccPointCloud;
class ccScalarField;

struct laszip_header;
struct laszip_vlr;
typedef laszip_vlr laszip_vlr_struct;

/// This serves the same purpose as LasScalarField but for extra bytes
class LasExtraScalarField
{
  public:
	/// When the extra dimension in an array type, it can have
	/// at most 3 elements.
	///
	/// Eg: A Color extra array dimension with 3 elements,
	/// so that each points have Color[0], Color[1], Color[2] as
	/// in a single extra dimension definition.
	static constexpr unsigned MAX_DIM_SIZE         = 3;
	static constexpr unsigned MAX_NAME_SIZE        = 32;
	static constexpr unsigned MAX_DESCRIPTION_SIZE = 32;

	enum class DimensionSize
	{
		One   = 1,
		Two   = 2,
		Three = 3,
	};

	/// Data types available LAS Extra Field
	enum DataType
	{
		Undocumented = 0,
		u8           = 1,
		i8,
		u16,
		i16,
		u32,
		i32,
		u64,
		i64,
		f32,
		f64,
		Invalid
	};

	enum Kind
	{
		Signed,
		Unsigned,
		Floating
	};

  public:
	LasExtraScalarField() = default;

	friend QDataStream& operator>>(QDataStream& dataStream, LasExtraScalarField& extraScalarField);
	friend QDataStream& operator<<(QDataStream& dataStream, const LasExtraScalarField& extraScalarField);

  public: // Static Helper functions that works on collection of LasExtraScalarFields
	static std::vector<LasExtraScalarField> ParseExtraScalarFields(const laszip_header& laszipHeader);
	static std::vector<LasExtraScalarField> ParseExtraScalarFields(const laszip_vlr_struct& extraBytesVlr);
	static void                             InitExtraBytesVlr(laszip_vlr_struct&                      vlr,
	                                                          const std::vector<LasExtraScalarField>& extraFields);
	static void                             UpdateByteOffsets(std::vector<LasExtraScalarField>& extraFields);
	static unsigned                         TotalExtraBytesSize(const std::vector<LasExtraScalarField>& extraScalarFields);
	static void                             MatchExtraBytesToScalarFields(std::vector<LasExtraScalarField>& extraScalarFields,
	                                                                      const ccPointCloud&               pointCloud);

  public: // methods
	// LAS Spec integer value for the type
	uint8_t typeCode() const;

	// Properties we can derive from the type attribute
	unsigned    elementSize() const;
	unsigned    numElements() const;
	unsigned    byteSize() const;
	Kind        kind() const;
	std::string typeName() const;

	// Properties we can derive from the type options attribute
	bool noDataIsRelevant() const;
	bool minIsRelevant() const;
	bool maxIsRelevant() const;
	bool scaleIsRelevant() const;
	bool offsetIsRelevant() const;

	void setOffsetIsRelevant(bool isRelevant);
	void setScaleIsRelevant(bool isRelevant);

	void resetScalarFieldsPointers();

	static std::tuple<DataType, DimensionSize> DataTypeFromValue(uint8_t value);

  public: // Data members
	DataType      type{Undocumented};
	DimensionSize dimensions{DimensionSize::One};
	// These fields are from the vlr itself
	uint8_t options{0};
	char    name[MAX_NAME_SIZE]               = "";
	char    description[MAX_DESCRIPTION_SIZE] = "";
	uint8_t noData[MAX_DIM_SIZE][8]           = {0};
	uint8_t mins[MAX_DIM_SIZE][8]             = {0};
	uint8_t maxs[MAX_DIM_SIZE][8]             = {0};
	double  scales[MAX_DIM_SIZE]              = {0.0};
	double  offsets[MAX_DIM_SIZE]             = {0.0};

	// These are added by us
	unsigned       byteOffset{0};
	ccScalarField* scalarFields[MAX_DIM_SIZE] = {nullptr};
	// TODO explain better
	// This strings store the name of the field in CC,
	// Extra fields name may clash with existing scalar fields name
	// (eg: the user calls one of his extra field "Intensity")
	// we have to alter the real name
	char ccName[MAX_NAME_SIZE + 8] = {0};
};
