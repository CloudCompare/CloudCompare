#include "SeedPicker.h"
#include "SegmenterDlg.h"

#include <ccPickingHub.h>
#include <ccLog.h>
#include <ccColorTypes.h>
#include <ccPointCloud.h>
#include <ccOctree.h>

#include <queue>
#include <cmath>
#include <limits>

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
        // Safeguard original colors before modifying anything
        if (m_targetCloud->hasColors())
        {
            m_originalColors.resize(m_targetCloud->size());
            for (unsigned i = 0; i < m_targetCloud->size(); ++i) {
                m_originalColors[i] = m_targetCloud->getPointColor(i);
            }
        }
    }
    else if (m_targetCloud != clickedCloud) return;

    unsigned pointIndex = pi.itemIndex;
    const CCVector3& pointCoords = pi.P3D;

    if (m_isPositive)
    {
        m_positiveSeeds.push_back(pointIndex);
        if (!m_posMarkerCloud)
        {
            m_posMarkerCloud = new ccPointCloud("Positive Seeds");
            m_posMarkerCloud->setPointSize(15);
            m_posMarkerCloud->resizeTheRGBTable(); //could do a check if it's the first time, then an error would not show at the beginning.
            m_posMarkerCloud->showColors(true);
            m_targetCloud->addChild(m_posMarkerCloud);
            m_app->addToDB(m_posMarkerCloud, false, true, false, false);
        }
        m_posMarkerCloud->addPoint(pointCoords);
        m_posMarkerCloud->addColor(ccColor::Rgb(190, 242, 58)); // neon lime
    }
    else
    {
        m_negativeSeeds.push_back(pointIndex);
        if (!m_negMarkerCloud)
        {
            m_negMarkerCloud = new ccPointCloud("Negative Seeds");
            m_negMarkerCloud->setPointSize(15);
            m_negMarkerCloud->showColors(true);
            m_targetCloud->addChild(m_negMarkerCloud);
            m_app->addToDB(m_negMarkerCloud, false, true, false, false);
        }
        m_negMarkerCloud->addPoint(pointCoords);
        m_negMarkerCloud->addColor(ccColor::Rgb(242, 19, 135)); // hottt pink
    }

    m_app->refreshAll();

    // Rerun the competitive segmentation instantly upon a new point placement
    runRegionGrowing();
}

struct QueueItem {
    double cost;
    unsigned int index;
    int label; // 1 = Positive, 2 = Negative
    bool operator>(const QueueItem& other) const { return cost > other.cost; } //what
};

int SeedPicker::runRegionGrowing()
{
    if (!m_targetCloud || m_positiveSeeds.empty() || m_originalColors.empty() || !m_dialog) return 0;

    ccOctree::Shared octree = m_targetCloud->getOctree(); //shared??
    if (!octree)
    {
        m_targetCloud->computeOctree();
        octree = m_targetCloud->getOctree();
        if (!octree) return 0;
    }

    // Pull normalized parameters straight from the UI
    double ws = m_dialog->getSpatialWeight();
    double wc = m_dialog->getChromaticWeight();
    double tauThreshold = m_dialog->getThreshold();
    double searchRadius = 0.2; // Fixed 20cm neighborhood search step size 

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

    unsigned char searchLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(searchRadius));
    const double MAX_COLOR_DIST = 441.673; // sqrt(255^2 * 3)

    // Competitive Dijkstra Territory Expansion
    while (!pq.empty())
    {
        QueueItem current = pq.top();
        pq.pop();

        if (current.cost > minCost[current.index]) continue;
        labels[current.index] = current.label;

        const CCVector3* p_curr = m_targetCloud->getPoint(current.index);
        const ccColor::Rgba c_curr = m_originalColors[current.index];

        CCCoreLib::DgmOctree::NeighboursSet neighbors;
        octree->getPointsInSphericalNeighbourhood(*p_curr, static_cast<PointCoordinateType>(searchRadius), neighbors, searchLevel);

        for (const auto& neighbor : neighbors)
        {
            unsigned int nIdx = neighbor.pointIndex;

            // Geometry cost
            const CCVector3* p_n = m_targetCloud->getPoint(nIdx);
            double dist = std::sqrt((p_curr->x - p_n->x)*(p_curr->x - p_n->x) + (p_curr->y - p_n->y)*(p_curr->y - p_n->y) + (p_curr->z - p_n->z)*(p_curr->z - p_n->z));
            double normDist = dist / searchRadius;

            // Color cost
            const ccColor::Rgba c_n = m_originalColors[nIdx];
            double dr = static_cast<double>(c_curr.r) - c_n.r;
            double dg = static_cast<double>(c_curr.g) - c_n.g;
            double db = static_cast<double>(c_curr.b) - c_n.b;
            double normColor = std::sqrt(dr*dr + dg*dg + db*db) / MAX_COLOR_DIST;

            // Total localized step cost scaled to 0-100 range matching the threshold slider
            double step_cost = 100.0 * ((ws * normDist) + (wc * normColor));
            if (step_cost > tauThreshold) continue;

            double total_path_cost = current.cost + step_cost;
            if (total_path_cost < minCost[nIdx])
            {
                minCost[nIdx] = total_path_cost;
                pq.push({total_path_cost, nIdx, current.label});
            }
        }
    }

    // Paint positive region magenta and rest back to original colors
    int positiveCount = 0;
    for (unsigned i = 0; i < cloudSize; ++i)
    {
        if (labels[i] == 1) {
            m_targetCloud->setPointColor(i, ccColor::magenta);
            positiveCount++;
        } else {
            m_targetCloud->setPointColor(i, m_originalColors[i]);
        }
    }

    m_targetCloud->showColors(true);
    m_app->refreshAll();

    m_dialog->setPointCount(positiveCount);
    m_dialog->setStatusMessage("Segmentation completed successfully.");

    return positiveCount;
}