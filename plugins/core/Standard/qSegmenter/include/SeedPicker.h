#pragma once

#include <ccMainAppInterface.h>
#include <ccPickingListener.h>
#include <ccPointCloud.h>
#include <vector>

constexpr double   NEIGHBOURHOOD_RADIUS_M = 0.05;
// Per-point neighbour cap: prevents pathological memory/time on dense clouds.
// Keeps only the MAX_NEIGHBOURS closest points within the search radius.
constexpr unsigned MAX_NEIGHBOURS = 50;

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

struct SeedState {
    std::vector<unsigned int> posSeeds;
    std::vector<unsigned int> negSeeds;
};

class SeedPicker : public ccPickingListener
{
public:
    SeedPicker(ccMainAppInterface* app, SegmenterDlg* dialog);
    virtual ~SeedPicker();

    void startListening();
    void stopListening();

    void setPositiveMode(bool isPositive) { m_isPositive = isPositive; }
    int  runRegionGrowing();
    void clearAll();
    void undo();
    void redo();
    void exportSegmentation();

protected:
    void onItemPicked(const PickedItem& pi) override;

private:
    void buildAdjacency();
    void rebuildMarkerClouds();
    void restoreOriginalColors();

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

    // Precomputed adjacency graph — built once per cloud+radius, reused every run
    std::vector<std::vector<AdjEdge>> m_adjacency;
    bool   m_normalsAtBuildTime = false;
    double m_builtRadius        = 0.0; // radius used when m_adjacency was built

    // Result of the last segmentation run — true = positive region
    std::vector<bool> m_positiveLabels;

    // Per-click undo/redo stacks (snapshots of seed index lists)
    std::vector<SeedState> m_undoStack;
    std::vector<SeedState> m_redoStack;
};