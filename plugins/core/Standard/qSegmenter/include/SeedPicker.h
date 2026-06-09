#pragma once

#include <ccMainAppInterface.h>
#include <ccPickingListener.h>
#include <ccPointCloud.h>
#include <vector>

class ccMainAppInterface;
class ccPointCloud;
class SegmenterDlg;

class SeedPicker : public ccPickingListener
{
public:
    SeedPicker(ccMainAppInterface* app, SegmenterDlg* dialog);
    virtual ~SeedPicker();

    void startListening();
    void stopListening();

    void setPositiveMode(bool isPositive) { m_isPositive = isPositive; }
    int runRegionGrowing();

protected:
    // Inherited from ccPickingListener
    void onItemPicked(const PickedItem& pi) override;

private:
    ccMainAppInterface* m_app;
    SegmenterDlg* m_dialog;
    ccPointCloud* m_targetCloud;

    // Separate tracking layers for visualization markers
    ccPointCloud* m_posMarkerCloud;
    ccPointCloud* m_negMarkerCloud;
    ccPointCloud* m_previewCloud;

    bool m_isPositive = true;

    // Indices lists
    std::vector<unsigned int> m_positiveSeeds;
    std::vector<unsigned int> m_negativeSeeds;
    
    // Backup of original RGB values to ensure rerun ability
    std::vector<ccColor::Rgba> m_originalColors;
};