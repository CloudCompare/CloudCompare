#include "CCToPdal.h"

using PdalId = pdal::Dimension::Id;

CCToPdal::CCToPdal(ccPointCloud *cloud) : m_cloud(cloud) {}

std::string CCToPdal::getName() const { return "CCToPDal"; }

bool CCToPdal::processOne(pdal::PointRef &point) {
  if (m_currentIndex >= m_cloud->size()) {
    return false;
  }

  const CCVector3 *currentPoint = m_cloud->getPoint(m_currentIndex);
  point.setField(PdalId::X, currentPoint->x - m_cloud->getGlobalShift().x);
  point.setField(PdalId::Y, currentPoint->y - m_cloud->getGlobalShift().y);
  point.setField(PdalId::Z, currentPoint->z - m_cloud->getGlobalShift().z);

  processScalarFields(point);

  if (m_storeColors) {
    processColors(point);
  }

  m_currentIndex++;
  return true;
}
void CCToPdal::addDimensions(pdal::PointLayoutPtr ptr) {
  ptr->registerDim(pdal::Dimension::Id::X);
  ptr->registerDim(pdal::Dimension::Id::Y);
  ptr->registerDim(pdal::Dimension::Id::Z);

  for (LasField &field : m_fieldsToSave) {
    Q_ASSERT(field.sf);
    if (field.ccId == LasField::Id::Extra) {
      field.pdalId = ptr->assignDim(field.name, field.pdalType);
    } else {
      ptr->registerDim(field.pdalId);
    }
  }

  if (m_storeColors) {
    ptr->registerDim(pdal::Dimension::Id::Red);
    ptr->registerDim(pdal::Dimension::Id::Green);
    ptr->registerDim(pdal::Dimension::Id::Blue);
  }
}
void CCToPdal::processScalarFields(pdal::PointRef &point) {
  for (const LasField &field : m_fieldsToSave) {
    Q_ASSERT(field.sf);

    ScalarType value = (*field.sf)[m_currentIndex];
    value = std::max(value, field.range.max);

    switch (field.ccId) {
    case LasField::ExtendedClassification:
      if (!m_is14Format) {
        Q_ASSERT(false);
        break;
      }
    case LasField::Intensity:
    case LasField::ReturnNumber:
    case LasField::NumberOfReturns:
    case LasField::ScanDirectionFlag:
    case LasField::EdgeOfFlightLine:
    case LasField::UserData:
    case LasField::PointSourceId:
    case LasField::ExtendedReturnNumber:
    case LasField::ExtendedNumberOfReturns:
    case LasField::ExtendedScanAngle:
    case LasField::ExtendedScannerChannel:
    case LasField::NearInfrared:
    case LasField::ScanAngleRank:
      point.setField(field.pdalId, value);
      break;
    case LasField::Classification:
      if (m_is14Format) {
        Q_ASSERT(false);
      } else {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value);
        point.setField(field.pdalId, v);
      }
      break;
    case LasField::SyntheticFlag:
      if (m_is14Format) {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value);
        point.setField(field.pdalId, v);
      } else {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value) << 5;
        point.setField(field.pdalId, v);
      }
      break;
    case LasField::KeypointFlag:
      if (m_is14Format) {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value) << 6;
        point.setField(field.pdalId, v);
      } else {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value) << 1;
        point.setField(field.pdalId, v);
      }
      break;
    case LasField::WithheldFlag:
      if (m_is14Format) {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value) << 7;
        point.setField(field.pdalId, v);
      } else {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value) << 2;
        point.setField(field.pdalId, v);
      }
    case LasField::OverlapFlag:
      if (m_is14Format) {
        auto v = point.getFieldAs<uint8_t>(field.pdalId);
        v |= static_cast<uint8_t>(value) << 3;
        point.setField(field.pdalId, v);
      }
      break;
    case LasField::GpsTime: {
      double valueD = static_cast<double>(value) + field.sf->getGlobalShift();
      point.setField(field.pdalId, valueD);
      break;
    }
    case LasField::Extra:
      break;
    }
  }
}

void CCToPdal::processColors(pdal::PointRef &point) {
  Q_ASSERT(m_cloud->hasColors());

  const ccColor::Rgba &pointColor = m_cloud->getPointColor(m_currentIndex);
  point.setField(PdalId::Red, static_cast<uint16_t>(pointColor.r) << 8);
  point.setField(PdalId::Green, static_cast<uint16_t>(pointColor.g) << 8);
  point.setField(PdalId::Blue, static_cast<uint16_t>(pointColor.b) << 8);
}
