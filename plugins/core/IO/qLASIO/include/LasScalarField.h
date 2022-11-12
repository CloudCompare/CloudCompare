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

// CCCoreLib
#include <CCTypes.h>
// qCC_db
#include <ccScalarField.h>

// System
#include <cstdint>

/// class used to link a LAS field defined by the LAS standard to the CloudCompare
/// scalar field that stores or will store the values.
struct LasScalarField
{
	/// Enum used to uniquely identify LAS fields
	/// with a clear distinction between 'normal' and 'extended' fields
	/// which may have the same name (e.g. Classification)
	enum Id
	{
		Intensity       = 0,
		ReturnNumber    = 1,
		NumberOfReturns = 2,
		ScanDirectionFlag,
		EdgeOfFlightLine,
		Classification,
		SyntheticFlag,
		KeypointFlag,
		WithheldFlag,
		ScanAngleRank,
		UserData,
		PointSourceId,
		GpsTime,
		// Extended (LAS 1.4)
		ExtendedScanAngle,
		ExtendedScannerChannel,
		OverlapFlag,
		ExtendedClassification,
		ExtendedReturnNumber,
		ExtendedNumberOfReturns,
		NearInfrared
	};

	/// Represents an inclusive range of value from a minimum to a maximum.
	///
	/// This is used to be able to detect and inform users from possible data loss.
	struct Range
	{
		/// Creates a new Range given the min and max value.
		///
		/// These values are converted to ScalarType
		template <class T>
		constexpr Range(T minValue, T maxValue)
		    : min(static_cast<ScalarType>(minValue))
		    , max(static_cast<ScalarType>(maxValue))
		{
		}

		/// Shortcut to create a Range from a native type like uint16_t
		template <class T>
		static constexpr Range ForType()
		{
			return Range(std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
		}

		static Range ForBitCount(uint8_t numBits)
		{
			return Range(0, ((1 << static_cast<uint32_t>(numBits)) - 1));
		}

		ScalarType min = 0;
		ScalarType max = 0;
	};

  public: // Methods and Constructors
	LasScalarField() = delete;

	explicit LasScalarField(LasScalarField::Id id, ccScalarField* sf = nullptr);

	const char* name() const;

  public: // Static functions
	/// Returns the name that correspond to the given LasScalarField::Id variant
	static constexpr const char* NameFromId(LasScalarField::Id id);
	/// Returns the  LasScalarField::Id variant that correspond to the given name
	///
	/// Point Format is needed, as we have different Id for the same name depending on the point format.
	/// For example: The "Classification" field has a different Id in fmt >= 6 as it allows to store more information.
	///
	/// Throws a logic_error if the name does not have a matching id
	static LasScalarField::Id IdFromName(const char* name, unsigned targetPointFormat);
	/// Returns the range of value the given field (ID) supports
	static LasScalarField::Range ValueRange(LasScalarField::Id id);
	/// Returns the scalar fields that correspond the the point format ID
	/// as per LAS specifications.
	static std::vector<LasScalarField> ForPointFormat(unsigned pointFormatId);

  public: // Members
	/// The Id of the LAS field this relates to.
	Id id;
	/// Pointer to the 'linked' CloudCompare scalar field.
	///
	/// When reading (loading points) values of the LAS field will be stored into the
	/// scalar field pointed by sf.
	///
	/// When writing (saving points) values of the scalar field pointed by sf will
	/// be stored to the corresponding LAS field (using the Id).
	ccScalarField* sf{nullptr};
	Range          range;
};
