//
// Created by thomas on 29/11/2021.
//

#ifndef CLOUDCOMPAREPROJECTS_PDALTOCC_H
#define CLOUDCOMPAREPROJECTS_PDALTOCC_H

#include <pdal/Streamable.hpp>
#include <utility>

#include <ccPointCloud.h>
#include <pdal/Filter.hpp>

#include "LasFields.h"
using DetermineShiftCallBack =
    std::function<void(const CCVector3d &point, ccPointCloud &cloud)>;

class PdalToCc : public pdal::Filter, public pdal::Streamable {
public:
  PdalToCc() { m_cloud = std::make_unique<ccPointCloud>(); }
  void setIs14Format(bool state) { m_is14Format = state; }

  void setScalarFieldsToLoad(LasField::Vector &&fieldsToLoad) {
    m_scalarFieldsToLoad = fieldsToLoad;
  }

  void setDetermineShiftCallback(DetermineShiftCallBack callback) {
    m_determineShiftCallback = std::move(callback);
  };

  void setLoadColors(bool state) { m_loadColors = state; }

  const LasField::Vector &scalarFieldsToLoad() const {
    return m_scalarFieldsToLoad;
  }

  std::string getName() const override {
    return "converters.CloudCompareToPdal";
  }

  std::unique_ptr<ccPointCloud> loadedCloud() { return std::move(m_cloud); }

protected:
  bool processOne(pdal::PointRef &point) override;
  void processScalarFields(pdal::PointRef &point);
  void processColors(pdal::PointRef &point);

private:
private:
  std::unique_ptr<ccPointCloud> m_cloud{nullptr};
  LasField::Vector m_scalarFieldsToLoad;
  DetermineShiftCallBack m_determineShiftCallback = [](const CCVector3d &point,
                                                       ccPointCloud &cloud) {};
  bool m_is14Format{false};
  bool m_loadColors{false};
  int m_colorShift{-1};

  CCVector3d m_shift{};
};

#endif // CLOUDCOMPAREPROJECTS_PDALTOCC_H
