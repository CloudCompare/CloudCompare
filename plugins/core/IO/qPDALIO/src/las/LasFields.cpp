#include "LasFields.h"

#include <QtDebug>

pdal::Dimension::Id LasField::PdalIdLasFieldId(LasField::Id id,
                                               bool is14Format) {
  using Id = LasField::Id;
  using PdalId = pdal::Dimension::Id;
  PdalId pdalId = PdalId::Unknown;
  switch (id) {
  case LasField::Id::Intensity:
    pdalId = PdalId::Intensity;
    break;
  case LasField::Id::ReturnNumber:
    pdalId = PdalId::ReturnNumber;
    break;
  case LasField::Id::NumberOfReturns:
    pdalId = PdalId::NumberOfReturns;
    break;
  case LasField::Id::ScanDirectionFlag:
    pdalId = PdalId::ScanDirectionFlag;
    break;
  case LasField::Id::EdgeOfFlightLine:
    pdalId = PdalId::EdgeOfFlightLine;
    break;
  case LasField::Id::Classification:
    pdalId = PdalId::Classification;
    break;
  case LasField::Id::KeypointFlag:
  case LasField::Id::SyntheticFlag:
  case LasField::Id::WithheldFlag:
    if (is14Format) {
      pdalId = PdalId ::ClassFlags;
    } else {

      pdalId = PdalId::Classification;
    }
    break;
  case LasField::Id::ScanAngleRank:
    pdalId = PdalId::ScanAngleRank;
    break;
  case LasField::Id::UserData:
    pdalId = PdalId::UserData;
    break;
  case LasField::Id::PointSourceId:
    pdalId = PdalId::PointSourceId;
    break;
  case LasField::Id::GpsTime:
    pdalId = PdalId::GpsTime;
    break;
  case LasField::Id::ExtendedScanAngle:
    pdalId = PdalId::ScanAngleRank;
    break;
  case LasField::Id::ExtendedScannerChannel:
    pdalId = PdalId::ScanChannel;
    break;
  case LasField::Id::OverlapFlag:
    if (is14Format) {
      pdalId = PdalId::ClassFlags;
    }
    break;
  case LasField::Id::ExtendedClassification:
    pdalId = PdalId::Classification;
    break;
  case LasField::Id::ExtendedReturnNumber:
    pdalId = PdalId::ReturnNumber;
    break;
  case LasField::Id::ExtendedNumberOfReturns:
    pdalId = PdalId::NumberOfReturns;
    break;
  case LasField::Id::NearInfrared:
    pdalId = PdalId::Infrared;
    break;
  case LasField::Id::Extra:
    pdalId = PdalId::Unknown;
    break;
  default:
    Q_ASSERT(false);
    break;
  }
  return pdalId;
}

const char *LasField::NameFromId(LasField::Id id) {
  switch (id) {
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
  case Extra:
    Q_ASSERT(false);
    return "";
  }
  throw std::logic_error("unhandled id");
}
LasField::Id LasField::IdFromName(const char *name,
                                  unsigned int targetPointFormat) {

  bool isExtended = isPointFormatExtended(targetPointFormat);
  if (strcmp(name, LasNames::Intensity) == 0) {
    return LasField::Id::Intensity;
  }

  if (strcmp(name, LasNames::ReturnNumber) == 0) {
    if (!isExtended) {
      return LasField::Id::ReturnNumber;
    } else {
      return LasField::Id::ExtendedReturnNumber;
    }
  }

  if (strcmp(name, LasNames::NumberOfReturns) == 0) {
    if (!isExtended) {
      return LasField::Id::NumberOfReturns;
    } else {
      return LasField::Id::ExtendedNumberOfReturns;
    }
  }

  if (strcmp(name, LasNames::ScanDirectionFlag) == 0) {
    return LasField::Id::ScanDirectionFlag;
  }

  if (strcmp(name, LasNames::EdgeOfFlightLine) == 0) {
    return LasField::Id::EdgeOfFlightLine;
  }

  if (strcmp(name, LasNames::Classification) == 0) {
    if (!isExtended) {
      return LasField::Id::Classification;
    } else {
      return LasField::Id::ExtendedClassification;
    }
  }

  if (strcmp(name, LasNames::SyntheticFlag) == 0) {
    return LasField::Id::SyntheticFlag;
  }

  if (strcmp(name, LasNames::KeypointFlag) == 0) {
    return LasField::Id::KeypointFlag;
  }

  if (strcmp(name, LasNames::WithheldFlag) == 0) {
    return LasField::Id::KeypointFlag;
  }

  if (strcmp(name, LasNames::ScanAngleRank) == 0) {
    return LasField::Id::ScanAngleRank;
  }

  if (strcmp(name, LasNames::UserData) == 0) {
    return LasField::Id::UserData;
  }

  if (strcmp(name, LasNames::PointSourceId) == 0) {
    return LasField::Id::PointSourceId;
  }

  if (strcmp(name, LasNames::GpsTime) == 0) {
    return LasField::Id::GpsTime;
  }

  if (strcmp(name, LasNames::ScanAngle) == 0) {
    return LasField::Id::ExtendedScanAngle;
  }

  if (strcmp(name, LasNames::ScannerChannel) == 0) {
    return LasField::Id::ExtendedScannerChannel;
  }

  if (strcmp(name, LasNames::OverlapFlag) == 0) {
    return LasField::Id::OverlapFlag;
  }

  if (strcmp(name, LasNames::NearInfrared) == 0) {
    return LasField::Id::NearInfrared;
  }

  ccLog::Warning("Unhandled Name %s", name);
  throw std::logic_error("Unknown name");
}

LasField::Range LasField::ValueRange(LasField::Id id) {
  switch (id) {
  case Intensity:
    return Range::ForType<uint16_t>();
  case ReturnNumber:
  case NumberOfReturns:
    return Range::ForBitSize(3);
  case ScanDirectionFlag:
  case EdgeOfFlightLine:
    return Range::ForBitSize(1);
  case Classification:
    return Range::ForBitSize(5);
  case SyntheticFlag:
  case KeypointFlag:
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
    return {std::numeric_limits<ScalarType>::lowest(),
            std::numeric_limits<ScalarType>::max()};
  case ExtendedScanAngle:
    return {-30'000.0, 30'000.0};
  case ExtendedScannerChannel:
    return Range::ForBitSize(2);
  case OverlapFlag:
    return Range::ForBitSize(1);
  case ExtendedClassification:
    return Range::ForType<uint8_t>();
  case ExtendedReturnNumber:
  case ExtendedNumberOfReturns:
    return Range::ForBitSize(4);
  case NearInfrared:
    return Range::ForType<uint16_t>();
  }

  Q_ASSERT_X(false, __FUNCTION__, "Unhandled las scalar field range");
  return Range::ForType<ScalarType>();
}

LasField::Range LasField::ValueRange(pdal::Dimension::Type type) {
  switch (type) {
  case pdal::Dimension::Type::None:
    Q_ASSERT_X(false, __func__, "None type for an extra dimension");
    return Range::ForType<ScalarType>();
  case pdal::Dimension::Type::Unsigned8:
    return Range::ForType<uint8_t>();
  case pdal::Dimension::Type::Signed8:
    return Range::ForType<int8_t>();
  case pdal::Dimension::Type::Unsigned16:
    return Range::ForType<uint16_t>();
  case pdal::Dimension::Type::Signed16:
    return Range::ForType<int16_t>();
  case pdal::Dimension::Type::Unsigned32:
    return Range::ForType<uint32_t>();
  case pdal::Dimension::Type::Signed32:
    return Range::ForType<int32_t>();
  case pdal::Dimension::Type::Unsigned64:
    return Range::ForType<uint64_t>();
  case pdal::Dimension::Type::Signed64:
    return Range::ForType<int64_t>();
  case pdal::Dimension::Type::Float:
    return Range::ForType<float>();
  case pdal::Dimension::Type::Double:
    return Range::ForType<double>();
  }
  Q_ASSERT(false);
  return Range::ForType<ScalarType>();
}

LasField::Vector LasField::LasFieldsOfLayout(const pdal::PointLayout &layout,
                                             bool is14Format) {
  using Id = LasField::Id;
  Vector lasFields;
  lasFields.reserve(layout.dims().size());

  for (const pdal::Dimension::Id pdalId : layout.dims()) {

    switch (pdalId) {
    case pdal::Dimension::Id::Intensity:
      lasFields.emplace_back(Id::Intensity, pdalId);
      break;
    case pdal::Dimension::Id::ReturnNumber:
      lasFields.emplace_back(Id::ReturnNumber, pdalId);
      break;
    case pdal::Dimension::Id::NumberOfReturns:
      lasFields.emplace_back(Id::NumberOfReturns, pdalId);
      break;
    case pdal::Dimension::Id::ScanDirectionFlag:
      lasFields.emplace_back(Id::ScanDirectionFlag, pdalId);
      break;
    case pdal::Dimension::Id::EdgeOfFlightLine:
      lasFields.emplace_back(Id::EdgeOfFlightLine, pdalId);
      break;
    case pdal::Dimension::Id::Classification: {
      if (is14Format) {
        lasFields.emplace_back(Id::ExtendedClassification, pdalId);
      } else {
        lasFields.emplace_back(Id::Classification, pdalId);
        lasFields.emplace_back(Id::SyntheticFlag, pdalId);
        lasFields.emplace_back(Id::KeypointFlag, pdalId);
        lasFields.emplace_back(Id::KeypointFlag, pdalId);
      }
      break;
    }
    case pdal::Dimension::Id::ScanAngleRank:
      lasFields.emplace_back(Id::ScanAngleRank, pdalId);
      break;
    case pdal::Dimension::Id::UserData:
      lasFields.emplace_back(Id::UserData, pdalId);
      break;
    case pdal::Dimension::Id::PointSourceId:
      lasFields.emplace_back(Id::PointSourceId, pdalId);
      break;
    case pdal::Dimension::Id::GpsTime:
      lasFields.emplace_back(Id::GpsTime, pdalId);
      break;
    case pdal::Dimension::Id::ScanChannel:
      lasFields.emplace_back(Id::ExtendedScannerChannel, pdalId);
      break;
    case pdal::Dimension::Id::Infrared:
      lasFields.emplace_back(Id::NearInfrared, pdalId);
      break;
    case pdal::Dimension::Id::ClassFlags: {
      if (is14Format) {
        lasFields.emplace_back(Id::SyntheticFlag, pdalId);
        lasFields.emplace_back(Id::KeypointFlag, pdalId);
        lasFields.emplace_back(Id::WithheldFlag, pdalId);
        lasFields.emplace_back(Id::OverlapFlag, pdalId);
      }
      break;
    }
    case pdal::Dimension::Id::PointId:
      lasFields.emplace_back(Id::PointSourceId, pdalId);
      break;
    case pdal::Dimension::Id::X:
    case pdal::Dimension::Id::Y:
    case pdal::Dimension::Id::Z:
    case pdal::Dimension::Id::Red:
    case pdal::Dimension::Id::Green:
    case pdal::Dimension::Id::Blue:
      // Appear in Las files but don't handle them as
      // scalar fields
      break;
    default:
      std::string name = layout.dimName(pdalId);
      pdal::Dimension::Type dimType = layout.dimType(pdalId);
      lasFields.emplace_back(Id::Extra, pdalId, dimType, name);
      break;
    }
  }
  return lasFields;
}

LasField::Vector LasField::FieldsForPointFormat(int format) {
  using PdalId = pdal::Dimension::Id;
  std::vector<LasField> fields;
  switch (format) {
  case 3:
    // O as base + Time + RGB
  case 1:
    // => 0 as base + Time
    fields.emplace_back(Id::GpsTime, PdalId::GpsTime);
  case 2:
    // => 0 as base + RGB
  case 0:
    fields.emplace_back(Id::Intensity, PdalId::Intensity);
    fields.emplace_back(Id::ReturnNumber, PdalId::ReturnNumber);
    fields.emplace_back(Id::NumberOfReturns, PdalId::NumberOfReturns);
    fields.emplace_back(Id::ScanDirectionFlag, PdalId::ScanDirectionFlag);
    fields.emplace_back(Id::EdgeOfFlightLine, PdalId::EdgeOfFlightLine);
    fields.emplace_back(Id::Classification, PdalId::Classification);
    fields.emplace_back(Id::SyntheticFlag, PdalId::Classification);
    fields.emplace_back(Id::KeypointFlag, PdalId::Classification);
    fields.emplace_back(Id::WithheldFlag, PdalId::Classification);
    fields.emplace_back(Id::ScanAngleRank, PdalId::ScanAngleRank);
    fields.emplace_back(Id::UserData, PdalId::UserData);
    fields.emplace_back(Id::PointSourceId, PdalId::PointSourceId);
    return fields;
  case 7:
    // Adds RGB to point 6
  case 8:
    fields.emplace_back(Id::NearInfrared, PdalId::Infrared);
  case 6:
    fields.emplace_back(Id::Intensity, PdalId::Intensity);
    fields.emplace_back(Id::ExtendedReturnNumber, PdalId::ReturnNumber);
    fields.emplace_back(Id::ExtendedNumberOfReturns, PdalId::NumberOfReturns);
    fields.emplace_back(Id::SyntheticFlag, PdalId::ClassFlags);
    fields.emplace_back(Id::KeypointFlag, PdalId::ClassFlags);
    fields.emplace_back(Id::WithheldFlag, PdalId::ClassFlags);
    fields.emplace_back(Id::OverlapFlag, PdalId::ClassFlags);
    fields.emplace_back(Id::ExtendedScannerChannel, PdalId::ScanChannel);
    fields.emplace_back(Id::ScanDirectionFlag, PdalId::ScanDirectionFlag);
    fields.emplace_back(Id::EdgeOfFlightLine, PdalId::EdgeOfFlightLine);
    fields.emplace_back(Id::Classification, PdalId::Classification);
    fields.emplace_back(Id::UserData, PdalId::UserData);
    fields.emplace_back(Id::ExtendedScanAngle, PdalId::ScanAngleRank);
    fields.emplace_back(Id::PointSourceId, PdalId::PointSourceId);
    fields.emplace_back(Id::GpsTime, PdalId::GpsTime);
    return fields;

  default:
    qWarning() << "Unknown point format" << format << '\n';
    return fields;
  }
}

LasField::Range LasField::Range::ForTypeName(const QString &typeName) {
  if (typeName == "uint8_t") {
    return Range::ForType<uint8_t>();
  }
  if (typeName == "uint16_t") {
    return Range::ForType<uint16_t>();
  }
  if (typeName == "uint32_t") {
    return Range::ForType<uint32_t>();
  }
  if (typeName == "uint64_t") {
    return Range::ForType<uint64_t>();
  }

  if (typeName == "int8_t") {
    return Range::ForType<int8_t>();
  }
  if (typeName == "int16_t") {
    return Range::ForType<int16_t>();
  }
  if (typeName == "int32_t") {
    return Range::ForType<int32_t>();
  }
  if (typeName == "int64_t") {
    return Range::ForType<int64_t>();
  }

  if (typeName == "float") {
    return Range::ForType<float>();
  }

  if (typeName == "double") {
    return Range::ForType<double>();
  }

  qCritical() << typeName << " not handled\n";
  return Range::ForType<ScalarType>();
}
