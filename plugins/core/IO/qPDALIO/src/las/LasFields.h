//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_LAS_FIELDS_HEADER
#define CC_LAS_FIELDS_HEADER

// Local
#include "qCC_io.h"

// qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

// Qt
#include <QSharedPointer>
#include <QtDebug>

#include <pdal/PointLayout.hpp>

// System
#include <array>
#include <utility>
#include <vector>

#include "LasDetails.h"

class ccScalarField;
class ccPointCloud;

struct SavedExtraField {
  std::string name;
  pdal::Dimension::Type type;
};

inline std::array<const char *, 4> GetAvailableVersion() {
  std::array<const char *, 4> AvailableVersions = {"1.1", "1.2", "1.3", "1.4"};
  return AvailableVersions;
}

inline std::vector<int>
PointFormatsAvailableForVersion(const QString &version) {
  if (version == "1.0" | version == "1.1" || version == "1.2" ||
      version == "1.3") {
    return {0, 1, 2, 3};
  } else if (version == "1.4") {
    return {0, 1, 2, 3, 6, 7, 8};
  } else {
    qCritical() << "Unknown LAS version " << version << "\n";
    return {};
  }
}

//! LAS field descriptor
// TODO 2 static ctor, one for Stand and one for extra ?
struct LasField {

  /// Enum used to uniquely identify LAS fields
  /// with a clear distinction between 'normal' and 'extended' fields
  /// which may have the same name (e.g. Classification)
  enum Id {
    Intensity = 0,
    ReturnNumber,
    NumberOfReturns,
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
    NearInfrared,

    // Extra Bytes
    Extra
  };

  struct Range {

    template <class T>
    constexpr Range(T min_, T max_)
        : min(static_cast<ScalarType>(min_)),
          max(static_cast<ScalarType>(max_)) {}

    template <class T> constexpr static Range ForType() {
      return Range(std::numeric_limits<T>::lowest(),
                   std::numeric_limits<T>::max());
    }

    static Range ForBitSize(uint8_t numBits) {
      Range range(0.0, 0.0);
      range.max = static_cast<ScalarType>(std::pow(2, numBits) - 1.0);
      return range;
    }

    static Range ForTypeName(const QString &typeName);

    ScalarType min;
    ScalarType max;
  };

  using Vector = std::vector<LasField>;

  static Vector LasFieldsOfLayout(const pdal::PointLayout &layout,
                                  bool is14Format);
  static LasField::Vector FieldsForPointFormat(int format);

  // Returns the name we use for the ccScalarField given as LasField::Id
  static const char *NameFromId(LasField::Id id);
  static LasField::Id IdFromName(const char *name,
                                 unsigned int targetPointFormat);

  static pdal::Dimension::Id PdalIdLasFieldId(LasField::Id id, bool is14Format);

  static Range ValueRange(LasField::Id id);

  static Range ValueRange(pdal::Dimension::Type type);

  explicit LasField(LasField::Id fieldType, pdal::Dimension::Id pdalId)
      : ccId(fieldType), pdalId(pdalId), name(LasField::NameFromId(fieldType)),
        sf(nullptr), range(LasField::ValueRange(fieldType)) {}

  explicit LasField(LasField::Id fieldType, pdal::Dimension::Id pdalId,
                    pdal::Dimension::Type type, std::string name)
      : ccId(fieldType), pdalId(pdalId), name(std::move(name)), sf(nullptr),
        range(LasField::ValueRange(type)), pdalType(type) {
    Q_ASSERT_X(fieldType == LasField::Id::Extra, __func__,
               "This is meant for extra bytes scalar fields");
  }

  LasField(LasField::Id id, bool is14Format, ccScalarField *sf)
      : ccId(id), name(LasField::NameFromId(id)),
        pdalId(pdal::Dimension::Id::Unknown), sf(sf),
        range(LasField::ValueRange(id)) {
    pdalId = PdalIdLasFieldId(id, is14Format);
  }

  LasField() = default;

public:
  Id ccId{Id::Extra};
  ccScalarField *sf{nullptr};
  pdal::Dimension::Id pdalId{pdal::Dimension::Id::Unknown};
  std::string name{};
  Range range{Range::ForType<ScalarType>()};
  pdal::Dimension::Type pdalType{pdal::Dimension::Type::None};
};

#endif // CC_LAS_FIELDS_HEADER
