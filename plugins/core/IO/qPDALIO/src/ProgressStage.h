#ifndef CLOUDCOMPAREPROJECTS_PROGRESSSTAGE_H
#define CLOUDCOMPAREPROJECTS_PROGRESSSTAGE_H

#include <ccProgressDialog.h>

#include <pdal/Streamable.hpp>
class ProgressStage : public pdal::Filter, public pdal::Streamable {
public:
  explicit ProgressStage(CCCoreLib::NormalizedProgress &progressDialog)
      : m_progress(progressDialog) {}

  std::string getName() const override {
    return "stages.CloudCompareProgressDialog";
  }

private:
  bool processOne(pdal::PointRef &point) override {
    m_progress.oneStep();
    return true;
  }

private:
  CCCoreLib::NormalizedProgress &m_progress;
};

#endif // CLOUDCOMPAREPROJECTS_PROGRESSSTAGE_H
