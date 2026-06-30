#pragma once

#include <ccMainAppInterface.h>
#include <ccPickingListener.h>
#include <ccPointCloud.h>
#include <vector>

constexpr double NEIGHBOURHOOD_RADIUS_M = 0.2;

// One directed edge in the precomputed adjacency graph.
// All three cost components are normalised to [0, 1] and stored as float
// to halve memory versus double while preserving sufficient precision.
struct AdjEdge {
    unsigned int nIdx;
    float normDist;
    float normColor;
    float normNormal; // 0 if cloud had no normals when the graph was built
};

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
    void onItemPicked(const PickedItem& pi) override;

private:
    void buildAdjacency();

    ccMainAppInterface* m_app;
    SegmenterDlg* m_dialog;
    ccPointCloud* m_targetCloud;

    ccPointCloud* m_posMarkerCloud;
    ccPointCloud* m_negMarkerCloud;
    ccPointCloud* m_previewCloud;

    bool m_isPositive = true;

    std::vector<unsigned int> m_positiveSeeds;
    std::vector<unsigned int> m_negativeSeeds;

    std::vector<ccColor::Rgba> m_originalColors;

    // Precomputed adjacency graph — built once per cloud, reused every run
    std::vector<std::vector<AdjEdge>> m_adjacency;
    bool m_normalsAtBuildTime = false;
};