#include "LasScalarField.h"

#include "LasDetails.h"

#include <stdexcept>

static constexpr bool IsPointFormatExtended(unsigned pointFormat)
{
	return pointFormat >= 6;
}

constexpr const char* LasScalarField::NameFromId(LasScalarField::Id id)
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

LasScalarField::Id LasScalarField::IdFromName(const char* name, unsigned targetPointFormat)
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
		return Range::ForBitCount(3);
	case NumberOfReturns:
		return Range::ForBitCount(3);
	case ScanDirectionFlag:
		return Range::ForBitCount(1);
	case EdgeOfFlightLine:
		return Range::ForBitCount(1);
	case Classification:
		return Range::ForBitCount(5);
	case SyntheticFlag:
		return Range::ForBitCount(1);
	case KeypointFlag:
		return Range::ForBitCount(1);
	case WithheldFlag:
		return Range::ForBitCount(1);
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
		return Range::ForBitCount(2);
	case OverlapFlag:
		return Range::ForBitCount(1);
	case ExtendedClassification:
		return Range::ForType<uint8_t>();
	case ExtendedReturnNumber:
		return Range::ForBitCount(4);
	case ExtendedNumberOfReturns:
		return Range::ForBitCount(4);
	case NearInfrared:
		return Range::ForType<uint16_t>();
	}

	Q_ASSERT_X(false, __FUNCTION__, "Unhandled las scalar field range");
	return Range::ForType<ScalarType>();
}

LasScalarField::LasScalarField(LasScalarField::Id id, ccScalarField* sf)
    : id(id)
    , sf(sf)
    , range(LasScalarField::ValueRange(id))
{
}

const char* LasScalarField::name() const
{
	return LasScalarField::NameFromId(id);
}

std::vector<LasScalarField> LasScalarField::ForPointFormat(unsigned pointFormatId)
{
	std::vector<LasScalarField> scalarFields;
	scalarFields.reserve(16);

	if (pointFormatId <= 5)
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

	if (LasDetails::HasGpsTime(pointFormatId))
	{
		scalarFields.emplace_back(LasScalarField::Id::GpsTime);
	}

	if (LasDetails::HasNearInfrared(pointFormatId))
	{
		scalarFields.emplace_back(LasScalarField::Id::NearInfrared);
	}

	return scalarFields;
}
