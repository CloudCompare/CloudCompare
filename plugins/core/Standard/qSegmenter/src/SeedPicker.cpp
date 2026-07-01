#include "SeedPicker.h"
#include "SegmenterDlg.h"

#include <ccPickingHub.h>
#include <ccLog.h>
#include <ccColorTypes.h>
#include <ccPointCloud.h>
#include <ccOctree.h>

#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>

SeedPicker::SeedPicker(ccMainAppInterface* app, SegmenterDlg* dialog)
    : m_app(app)
    , m_dialog(dialog)
    , m_targetCloud(nullptr)
    , m_posMarkerCloud(nullptr)
    , m_negMarkerCloud(nullptr)
    , m_previewCloud(nullptr)
{}

SeedPicker::~SeedPicker() //tilde?
{
    stopListening();
}

void SeedPicker::startListening()
{
    if (!m_app || !m_app->pickingHub()) return;
    m_app->pickingHub()->addListener(this, false, true, ccGLWindowInterface::POINT_PICKING);
}

void SeedPicker::stopListening()
{
    if (!m_app) return;
    if (m_app->pickingHub()) m_app->pickingHub()->removeListener(this);

    if (m_posMarkerCloud) { m_app->removeFromDB(m_posMarkerCloud); m_posMarkerCloud = nullptr; }
    if (m_negMarkerCloud) { m_app->removeFromDB(m_negMarkerCloud); m_negMarkerCloud = nullptr; }
    if (m_previewCloud)   { m_app->removeFromDB(m_previewCloud);   m_previewCloud = nullptr; }

    // Restore pristine original colors on exit
    if (m_targetCloud && !m_originalColors.empty())
    {
        for (unsigned i = 0; i < m_targetCloud->size(); ++i) {
            m_targetCloud->setPointColor(i, m_originalColors[i]);
        }
        m_targetCloud->showColors(true);
    }
    m_app->refreshAll();
}

void SeedPicker::onItemPicked(const PickedItem& pi)
{
    if (!pi.entity || !pi.entity->isA(CC_TYPES::POINT_CLOUD)) return;

    ccPointCloud* clickedCloud = static_cast<ccPointCloud*>(pi.entity);

    if (!m_targetCloud)
    {
        m_targetCloud = clickedCloud;
        if (m_targetCloud->hasColors())
        {
            m_originalColors.resize(m_targetCloud->size());
            for (unsigned i = 0; i < m_targetCloud->size(); ++i)
                m_originalColors[i] = m_targetCloud->getPointColor(i);
        }
    }
    else if (m_targetCloud != clickedCloud) return;

    // Snapshot state for undo before mutating seed lists
    m_undoStack.push_back({m_positiveSeeds, m_negativeSeeds});
    m_redoStack.clear();

    unsigned pointIndex = pi.itemIndex;
    const CCVector3& pointCoords = pi.P3D;

    if (m_isPositive)
    {
        m_positiveSeeds.push_back(pointIndex);
        if (!m_posMarkerCloud)
        {
            m_posMarkerCloud = new ccPointCloud("Positive Seeds");
            m_posMarkerCloud->setPointSize(15);
            m_posMarkerCloud->resizeTheRGBTable();
            m_posMarkerCloud->showColors(true);
            m_targetCloud->addChild(m_posMarkerCloud);
            m_app->addToDB(m_posMarkerCloud, false, true, false, false);
        }
        m_posMarkerCloud->addPoint(pointCoords);
        m_posMarkerCloud->addColor(ccColor::Rgb(190, 242, 58));
    }
    else
    {
        m_negativeSeeds.push_back(pointIndex);
        if (!m_negMarkerCloud)
        {
            m_negMarkerCloud = new ccPointCloud("Negative Seeds");
            m_negMarkerCloud->setPointSize(15);
            m_negMarkerCloud->resizeTheRGBTable();
            m_negMarkerCloud->showColors(true);
            m_targetCloud->addChild(m_negMarkerCloud);
            m_app->addToDB(m_negMarkerCloud, false, true, false, false);
        }
        m_negMarkerCloud->addPoint(pointCoords);
        m_negMarkerCloud->addColor(ccColor::Rgb(242, 19, 135));
    }

    m_app->refreshAll();
    runRegionGrowing();
}

struct QueueItem {
    double cost;
    unsigned int index;
    int label; // 1 = Positive, 2 = Negative
    bool operator>(const QueueItem& other) const { return cost > other.cost; } //forcing the queue to keep the point with the lowest cost at the very top, core of Dijkstra
};

void SeedPicker::buildAdjacency()
{
    m_dialog->setStatusMessage("Building adjacency graph...");

    ccOctree::Shared octree = m_targetCloud->getOctree();
    if (!octree)
    {
        m_targetCloud->computeOctree();
        octree = m_targetCloud->getOctree();
        if (!octree) return;
    }

    unsigned cloudSize = m_targetCloud->size();
    ccLog::Print(QString("[Segmenter] Building adjacency graph for %1 points...").arg(cloudSize));
    auto buildStart = std::chrono::high_resolution_clock::now();

    const double radius = m_dialog->getRadius();
    m_builtRadius = radius;

    unsigned char searchLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(
        static_cast<PointCoordinateType>(radius));

    m_normalsAtBuildTime = m_targetCloud->hasNormals();
    m_adjacency.resize(cloudSize);

    const double MAX_COLOR_DIST = 441.673; // sqrt(255^2 * 3)

    CCCoreLib::DgmOctree::NeighboursSet neighbors;
    for (unsigned i = 0; i < cloudSize; ++i)
    {
        neighbors.clear();
        octree->getPointsInSphericalNeighbourhood(
            *m_targetCloud->getPoint(i),
            static_cast<PointCoordinateType>(radius),
            neighbors,
            searchLevel);

        // Cap to MAX_NEIGHBOURS closest points: prevents memory/time explosion on
        // dense clouds where the fixed radius captures thousands of neighbours.
        if (neighbors.size() > MAX_NEIGHBOURS)
        {
            std::partial_sort(neighbors.begin(), neighbors.begin() + MAX_NEIGHBOURS, neighbors.end(),
                [](const CCCoreLib::DgmOctree::PointDescriptor& a,
                   const CCCoreLib::DgmOctree::PointDescriptor& b)
                { return a.squareDistd < b.squareDistd; });
            neighbors.resize(MAX_NEIGHBOURS);
        }

        const CCVector3* p_i   = m_targetCloud->getPoint(i);
        const ccColor::Rgba c_i = m_originalColors[i];
        const CCVector3 n_i    = m_normalsAtBuildTime ? m_targetCloud->getPointNormal(i) : CCVector3();

        m_adjacency[i].reserve(neighbors.size()); // already capped to MAX_NEIGHBOURS
        for (const auto& nb : neighbors)
        {
            unsigned int j = nb.pointIndex;

            const CCVector3* p_j = m_targetCloud->getPoint(j);
            double dx = p_i->x - p_j->x, dy = p_i->y - p_j->y, dz = p_i->z - p_j->z;
            float normDist = static_cast<float>(std::sqrt(dx*dx + dy*dy + dz*dz) / radius);

            const ccColor::Rgba c_j = m_originalColors[j];
            double dr = static_cast<double>(c_i.r) - c_j.r;
            double dg = static_cast<double>(c_i.g) - c_j.g;
            double db = static_cast<double>(c_i.b) - c_j.b;
            float normColor = static_cast<float>(std::sqrt(dr*dr + dg*dg + db*db) / MAX_COLOR_DIST);

            float normNormal = 0.0f;
            if (m_normalsAtBuildTime)
            {
                const CCVector3& n_j = m_targetCloud->getPointNormal(j);
                double dot = n_i.x*n_j.x + n_i.y*n_j.y + n_i.z*n_j.z;
                dot = std::max(-1.0, std::min(1.0, dot));
                normNormal = static_cast<float>(1.0 - std::abs(dot));
            }

            m_adjacency[i].push_back({j, normDist, normColor, normNormal});
        }
    }

    auto buildMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - buildStart).count();

    size_t totalEdges = 0;
    for (const auto& adj : m_adjacency) totalEdges += adj.size();
    // Outer vector array + per-edge structs (exact, matches O(NK) space complexity)
    size_t adjBytes = cloudSize * sizeof(std::vector<AdjEdge>) + totalEdges * sizeof(AdjEdge);
    double adjMB = adjBytes / (1024.0 * 1024.0);

    double avgK = cloudSize > 0 ? static_cast<double>(totalEdges) / cloudSize : 0.0;
    bool wasCapped = avgK > MAX_NEIGHBOURS - 1;
    ccLog::Print(QString("[Segmenter] Adjacency graph built in %1 ms | %2 edges | avg K=%3%4 | %5 MB")
        .arg(buildMs).arg(totalEdges)
        .arg(avgK, 0, 'f', 1)
        .arg(wasCapped ? " (neighbour cap active)" : "")
        .arg(adjMB, 0, 'f', 2));
}

int SeedPicker::runRegionGrowing()
{
    if (!m_targetCloud || m_positiveSeeds.empty() || m_originalColors.empty() || !m_dialog) return 0;

    // Radius change invalidates the graph — clear so buildAdjacency rebuilds it
    if (!m_adjacency.empty() && std::abs(m_dialog->getRadius() - m_builtRadius) > 1e-9)
    {
        m_adjacency.clear();
        ccLog::Print("[Segmenter] Neighbourhood radius changed — rebuilding adjacency graph.");
    }

    if (m_adjacency.empty())
        buildAdjacency();

    auto segStart = std::chrono::high_resolution_clock::now();

    // Pull normalized parameters straight from the UI
    double ws = m_dialog->getSpatialWeight();
    double wc = m_dialog->getChromaticWeight();
    double wn = m_dialog->getNormalWeight();
    double tauThreshold = m_dialog->getThreshold();

    if (wn > 0.0 && !m_normalsAtBuildTime)
    {
        if (m_targetCloud->hasNormals())
            ccLog::Warning("[SeedPicker] Normals were added after the adjacency graph was built — normal cost is unavailable. Clear all seeds and re-add one to rebuild the graph.");
        else
            ccLog::Warning("[SeedPicker] Normal weight > 0, but cloud has no normals! Normal cost will be ignored. (You can calculate normals via CC's standard menu.)");
        wn = 0.0;
    }

    // Normalise weights so their sum equals 1, keeping tau scale-invariant.
    // Without this, large weights push all edge costs above tau and small weights
    // let the algorithm flood the entire cloud, making tau unintuitive to tune.
    const double weightSum = ws + wc + wn;
    if (weightSum > 0.0) { ws /= weightSum; wc /= weightSum; wn /= weightSum; }

    unsigned cloudSize = m_targetCloud->size();
    std::vector<double> minCost(cloudSize, std::numeric_limits<double>::infinity());
    std::vector<int> labels(cloudSize, 0); // 0=Unassigned, 1=Pos, 2=Neg

    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

    // Load competing armies into priority queue
    for (unsigned int idx : m_positiveSeeds) {
        if (idx < cloudSize) { pq.push({0.0, idx, 1}); minCost[idx] = 0.0; }
    }
    for (unsigned int idx : m_negativeSeeds) {
        if (idx < cloudSize) { pq.push({0.0, idx, 2}); minCost[idx] = 0.0; }
    }

    // Competitive Dijkstra Territory Expansion
    // Edge costs are pre-normalised in m_adjacency — inner loop is now just
    // a weighted sum and a heap push; no sqrts or pointer chasing per neighbour.
    while (!pq.empty())
    {
        QueueItem current = pq.top();
        pq.pop();

        if (current.cost > minCost[current.index]) continue;
        labels[current.index] = current.label;

        for (const AdjEdge& e : m_adjacency[current.index])
        {
            double step_cost = 100.0 * (ws * e.normDist + wc * e.normColor + wn * e.normNormal);
            if (step_cost > tauThreshold) continue;

            double total_path_cost = current.cost + step_cost;
            if (total_path_cost < minCost[e.nIdx])
            {
                minCost[e.nIdx] = total_path_cost;
                pq.push({total_path_cost, e.nIdx, current.label});
            }
        }
    }

    auto segMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - segStart).count();
    ccLog::Print(QString("[Segmenter] Segmentation completed in %1 ms").arg(segMs));

    // Persist label result for export, then paint positive region magenta
    m_positiveLabels.assign(cloudSize, false);
    int positiveCount = 0;
    for (unsigned i = 0; i < cloudSize; ++i)
    {
        if (labels[i] == 1) {
            m_positiveLabels[i] = true;
            m_targetCloud->setPointColor(i, ccColor::magenta);
            positiveCount++;
        } else {
            m_targetCloud->setPointColor(i, m_originalColors[i]);
        }
    }

    m_targetCloud->showColors(true);
    m_targetCloud->prepareDisplayForRefresh();
    m_app->refreshAll();

    m_dialog->setPointCount(positiveCount);
    m_dialog->setStatusMessage("Segmentation completed successfully.");

    return positiveCount;
}

void SeedPicker::restoreOriginalColors()
{
    if (!m_targetCloud || m_originalColors.empty()) return;
    for (unsigned i = 0; i < m_targetCloud->size(); ++i)
        m_targetCloud->setPointColor(i, m_originalColors[i]);
    m_targetCloud->showColors(true);
    m_targetCloud->prepareDisplayForRefresh();
    m_app->refreshAll();
}

void SeedPicker::rebuildMarkerClouds()
{
    if (m_posMarkerCloud) { m_app->removeFromDB(m_posMarkerCloud); m_posMarkerCloud = nullptr; }
    if (m_negMarkerCloud) { m_app->removeFromDB(m_negMarkerCloud); m_negMarkerCloud = nullptr; }
    if (!m_targetCloud) return;

    if (!m_positiveSeeds.empty())
    {
        m_posMarkerCloud = new ccPointCloud("Positive Seeds");
        m_posMarkerCloud->setPointSize(15);
        m_posMarkerCloud->resizeTheRGBTable();
        m_posMarkerCloud->showColors(true);
        m_targetCloud->addChild(m_posMarkerCloud);
        m_app->addToDB(m_posMarkerCloud, false, true, false, false);
        for (unsigned idx : m_positiveSeeds)
        {
            m_posMarkerCloud->addPoint(*m_targetCloud->getPoint(idx));
            m_posMarkerCloud->addColor(ccColor::Rgb(190, 242, 58));
        }
    }

    if (!m_negativeSeeds.empty())
    {
        m_negMarkerCloud = new ccPointCloud("Negative Seeds");
        m_negMarkerCloud->setPointSize(15);
        m_negMarkerCloud->resizeTheRGBTable();
        m_negMarkerCloud->showColors(true);
        m_targetCloud->addChild(m_negMarkerCloud);
        m_app->addToDB(m_negMarkerCloud, false, true, false, false);
        for (unsigned idx : m_negativeSeeds)
        {
            m_negMarkerCloud->addPoint(*m_targetCloud->getPoint(idx));
            m_negMarkerCloud->addColor(ccColor::Rgb(242, 19, 135));
        }
    }
}

void SeedPicker::clearAll()
{
    restoreOriginalColors();

    m_positiveSeeds.clear();
    m_negativeSeeds.clear();
    m_undoStack.clear();
    m_redoStack.clear();

    // Full reset so the user can start fresh on any cloud
    m_originalColors.clear();
    m_positiveLabels.clear();
    m_adjacency.clear();
    m_builtRadius = 0.0;
    m_targetCloud = nullptr;

    if (m_posMarkerCloud) { m_app->removeFromDB(m_posMarkerCloud); m_posMarkerCloud = nullptr; }
    if (m_negMarkerCloud) { m_app->removeFromDB(m_negMarkerCloud); m_negMarkerCloud = nullptr; }

    m_dialog->setPointCount(0);
    m_dialog->setStatusMessage("Cleared. Click a point to start.");
}

void SeedPicker::undo()
{
    if (m_undoStack.empty() || !m_targetCloud) return;

    m_redoStack.push_back({m_positiveSeeds, m_negativeSeeds});
    SeedState prev = m_undoStack.back();
    m_undoStack.pop_back();
    m_positiveSeeds = prev.posSeeds;
    m_negativeSeeds = prev.negSeeds;

    rebuildMarkerClouds();

    if (!m_positiveSeeds.empty())
    {
        runRegionGrowing();
    }
    else
    {
        restoreOriginalColors();
        m_dialog->setPointCount(0);
        m_dialog->setStatusMessage("Undone to initial state.");
    }
}

void SeedPicker::redo()
{
    if (m_redoStack.empty() || !m_targetCloud) return;

    m_undoStack.push_back({m_positiveSeeds, m_negativeSeeds});
    SeedState next = m_redoStack.back();
    m_redoStack.pop_back();
    m_positiveSeeds = next.posSeeds;
    m_negativeSeeds = next.negSeeds;

    rebuildMarkerClouds();

    if (!m_positiveSeeds.empty())
    {
        runRegionGrowing();
    }
    else
    {
        restoreOriginalColors();
        m_dialog->setPointCount(0);
        m_dialog->setStatusMessage("Redone.");
    }
}

void SeedPicker::exportSegmentation()
{
    if (!m_targetCloud || m_positiveLabels.empty())
    {
        m_dialog->setStatusMessage("Nothing to export — run segmentation first.");
        return;
    }

    unsigned count = 0;
    for (bool b : m_positiveLabels) if (b) count++;
    if (count == 0)
    {
        m_dialog->setStatusMessage("Positive region is empty — nothing to export.");
        return;
    }

    QString exportName = m_targetCloud->getName() + "_segment";
    ccPointCloud* exportCloud = new ccPointCloud(exportName);

    if (!exportCloud->reserve(count))
    {
        delete exportCloud;
        ccLog::Error("[Segmenter] Not enough memory to allocate export cloud.");
        return;
    }

    bool hasColors  = !m_originalColors.empty();
    bool hasNormals = m_normalsAtBuildTime;

    if (hasColors)  exportCloud->reserveTheRGBTable();
    if (hasNormals) exportCloud->reserveTheNormsTable();

    for (unsigned i = 0; i < m_targetCloud->size(); ++i)
    {
        if (!m_positiveLabels[i]) continue;
        exportCloud->addPoint(*m_targetCloud->getPoint(i));
        if (hasColors)  exportCloud->addColor(m_originalColors[i]); // original, not magenta
        if (hasNormals) exportCloud->addNorm(m_targetCloud->getPointNormal(i));
    }

    if (hasColors)  exportCloud->showColors(true);
    if (hasNormals) exportCloud->showNormals(true);

    m_app->addToDB(exportCloud);
    m_app->refreshAll();

    ccLog::Print(QString("[Segmenter] Exported %1 points as '%2'.").arg(count).arg(exportName));
    m_dialog->setStatusMessage(QString("Exported %1 pts as \"%2\".").arg(count).arg(exportName));
}