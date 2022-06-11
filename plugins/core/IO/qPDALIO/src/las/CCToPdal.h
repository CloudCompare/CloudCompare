#ifndef CLOUDCOMPAREPROJECTS_CCTOPDAL_H
#define CLOUDCOMPAREPROJECTS_CCTOPDAL_H

#include "LasFields.h"
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

class CCToPdal : public pdal::Reader, public pdal::Streamable {
public:
  explicit CCToPdal(ccPointCloud *cloud);
  std::string getName() const override;

  void setFieldsToSave(LasField::Vector &&fieldsToSave) {
    m_fieldsToSave = fieldsToSave;
  }
  void setStoreColors(bool state) { m_storeColors = state; }
  void setIs14Format(bool state) { m_is14Format = state; }

protected:
  bool processOne(pdal::PointRef &point) override;

  void processScalarFields(pdal::PointRef &point);

  void processColors(pdal::PointRef &point);

private:
  void addDimensions(pdal::PointLayoutPtr ptr) override;

private:
  ccPointCloud *m_cloud{nullptr};
  unsigned int m_currentIndex{0};
  bool m_storeColors{false};
  bool m_is14Format{false};
  LasField::Vector m_fieldsToSave{};
};

#endif // CLOUDCOMPAREPROJECTS_CCTOPDAL_H
