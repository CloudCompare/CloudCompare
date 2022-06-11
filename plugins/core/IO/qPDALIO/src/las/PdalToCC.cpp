#include "PdalToCC.h"

#include <memory>
#include <pdal/PointRef.hpp>

static CCVector3
ConvertPointFromPDAL(const pdal::PointRef &point,
                     const CCVector3d &shift = CCVector3d(0., 0., 0.)) {
  return CCVector3{
      static_cast<PointCoordinateType>(
          point.getFieldAs<double>(pdal::Dimension::Id::X) + shift.x),
      static_cast<PointCoordinateType>(
          point.getFieldAs<double>(pdal::Dimension::Id::Y) + shift.y),
      static_cast<PointCoordinateType>(
          point.getFieldAs<double>(pdal::Dimension::Id::Z) + shift.z)};
}

static void MaybeLoadValue(LasField &scalarField, uint64_t currentCloudSize,
                           double value) {
  ccScalarField *&ccField = scalarField.sf;
  if (!ccField && value != 0.0) {
    ccField = new ccScalarField(scalarField.name.c_str());
    for (pdal::PointId i{0}; i < currentCloudSize - 1; ++i) {
      ccField->addElement(0.0);
    }
  }
  if (ccField) {

    ccField->addElement(static_cast<ScalarType>(value));
  }
}

bool PdalToCc::processOne(pdal::PointRef &point) {
  Q_ASSERT(m_cloud != nullptr);

  if (m_cloud->size() == 0) {
    CCVector3d firstPoint{point.getFieldAs<double>(pdal::Dimension::Id::X),
                          point.getFieldAs<double>(pdal::Dimension::Id::Y),
                          point.getFieldAs<double>(pdal::Dimension::Id::Z)};
    m_determineShiftCallback(firstPoint, *m_cloud);
    m_shift = m_cloud->getGlobalShift();
  }

  m_cloud->addPoint(ConvertPointFromPDAL(point, m_shift));
  processScalarFields(point);

  if (m_loadColors) {
    processColors(point);
  }

  return true;
}

void PdalToCc::processScalarFields(pdal::PointRef &point) {
  auto currentCloudSize = m_cloud->size();
  for (LasField &field : m_scalarFieldsToLoad) {
    auto value = point.getFieldAs<double>(field.pdalId);

    using Id = LasField::Id;

    switch (field.ccId) {
    case Id::Intensity:
    case Id::ReturnNumber:
    case Id::NumberOfReturns:
    case Id::EdgeOfFlightLine:
    case Id::ScanDirectionFlag:
    case Id::ScanAngleRank:
    case Id::ExtendedScannerChannel:
    case Id::NearInfrared:
    case Id::UserData:
    case Id::ExtendedClassification:
    case Id::Extra:
    case Id::PointSourceId:
      MaybeLoadValue(field, currentCloudSize, value);
      break;
    case Id::GpsTime: {
      ccScalarField *&ccField = field.sf;
      if (!ccField && value != 0.0) {
        ccField = new ccScalarField(field.name.c_str());
        ccField->setGlobalShift(value);
        for (pdal::PointId i{0}; i < currentCloudSize - 1; ++i) {
          ccField->addElement(0.0);
        }
      }
      if (ccField) {
        ccField->addElement(
            static_cast<ScalarType>(value - ccField->getGlobalShift()));
      }
      break;
    }
    case Id::Classification: {
      if (m_is14Format) {
        MaybeLoadValue(field, currentCloudSize, value);
      } else {
        auto intValue = point.getFieldAs<uint8_t>(field.pdalId);
        intValue &= 0b0001'1111;
        MaybeLoadValue(field, currentCloudSize, intValue);
      }
      break;
    }
    case Id::SyntheticFlag: {
      auto intValue = point.getFieldAs<uint8_t>(field.pdalId);
      if (m_is14Format) {
        intValue &= 0b0000'0001;
      } else {
        intValue &= 0b0010'0000;
        intValue = intValue != 0;
      }
      MaybeLoadValue(field, currentCloudSize, intValue);
      break;
    }
    case Id::KeypointFlag: {
      auto intValue = point.getFieldAs<uint8_t>(field.pdalId);
      if (m_is14Format) {
        intValue &= 0b0000'0010;
        intValue = intValue != 0;
      } else {

        intValue &= 0b0100'0000;
        intValue = intValue != 0;
      }
      MaybeLoadValue(field, currentCloudSize, intValue);
      break;
    }
    case Id::WithheldFlag: {
      auto intValue = point.getFieldAs<uint8_t>(field.pdalId);
      if (m_is14Format) {
        intValue &= 0b0000'0100;
        intValue = intValue != 0;
      } else {
        intValue &= 0b1000'0000;
        intValue = intValue != 0;
      }
      MaybeLoadValue(field, currentCloudSize, intValue);
      break;
    }
    case Id::OverlapFlag: {
      auto intValue = point.getFieldAs<uint8_t>(field.pdalId);
      intValue &= 0b0000'0100;
      intValue = intValue != 0;
      MaybeLoadValue(field, currentCloudSize, intValue);
      break;
    }
    default:
      Q_ASSERT_X(false, __func__, "Unhandled Scalar Field Id in PdalToCC");
      break;
    }
  }
}

void PdalToCc::processColors(pdal::PointRef &point) {
  Q_ASSERT(m_loadColors);
  auto red = point.getFieldAs<uint16_t>(pdal::Dimension::Id::Red);
  auto green = point.getFieldAs<uint16_t>(pdal::Dimension::Id::Green);
  auto blue = point.getFieldAs<uint16_t>(pdal::Dimension::Id::Blue);

  bool isBlack = (red | green | blue) == 0;
  if (isBlack) {
    return;
  }

  if (m_colorShift == -1) {
    bool isStoredOn16bits = (red | green | blue) > 255;
    m_colorShift = isStoredOn16bits ? 8 : 0;
  }

  if (!m_cloud->hasColors()) {
    if (!m_cloud->reserveTheRGBTable()) {
      throw std::runtime_error("Not enough memory for RGB");
    }

    for (pdal::PointId i{0}; i < m_cloud->size() - 1; ++i) {
      m_cloud->addColor(ccColor::blackRGB);
    }
  }

  if (m_cloud->hasColors()) {
    m_cloud->addColor(red >> m_colorShift, green >> m_colorShift,
                      blue >> m_colorShift);
  }
}
