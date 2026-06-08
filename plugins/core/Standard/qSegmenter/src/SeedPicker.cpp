#include "SeedPicker.h"

#include <ccPickingHub.h>
#include <cc2DViewportObject.h>
#include <ccLog.h>
#include <cc2DLabel.h>
#include <ccColorTypes.h>
#include <ccPointCloud.h>
#include <ccOctree.h>

#include <queue>
#include <vector>
#include <cmath>
#include <limits>

SeedPicker::SeedPicker(ccMainAppInterface* app)
    : m_app(app)
    , m_targetCloud(nullptr)
    , m_posMarkerCloud(nullptr)
    , m_negMarkerCloud(nullptr)
    , m_previewCloud(nullptr)
{
}

SeedPicker::~SeedPicker()
{
	stopListening();
}

void SeedPicker::startListening()
{
	if (!m_app || !m_app->pickingHub()) return;

	bool success = m_app->pickingHub()->addListener(
	    this, false, true, ccGLWindowInterface::POINT_PICKING);

	if (success)
		m_app->dispToConsole("[Segmenter] Picking mode activated. Click points!", ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

void SeedPicker::stopListening()
{
    if (!m_app) return;
    if (m_app->pickingHub()) m_app->pickingHub()->removeListener(this);

    if (m_posMarkerCloud) { m_app->removeFromDB(m_posMarkerCloud); m_posMarkerCloud = nullptr; }
    if (m_negMarkerCloud) { m_app->removeFromDB(m_negMarkerCloud); m_negMarkerCloud = nullptr; }
    if (m_previewCloud)   { m_app->removeFromDB(m_previewCloud);   m_previewCloud = nullptr; }

    // Restore original colors if we are completely stopping
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
	
	if (!m_targetCloud) {
		m_targetCloud = clickedCloud;
        // Backup original colors immediately on first click
        if (m_targetCloud->hasColors()) {
            m_originalColors.reserve(m_targetCloud->size());
            for (unsigned i = 0; i < m_targetCloud->size(); ++i) {
                m_originalColors.push_back(m_targetCloud->getPointColor(i));
            }
        }
	}
	else if (m_targetCloud != clickedCloud) return;
	
	unsigned pointIndex = pi.itemIndex;
    const CCVector3& pointCoords = pi.P3D;

    if (m_isPositive)
    {
        m_positiveSeeds.push_back(pointIndex);
        if (!m_posMarkerCloud) {
            m_posMarkerCloud = new ccPointCloud("Positive Seeds");
            m_posMarkerCloud->setPointSize(10);
            m_posMarkerCloud->resizeTheRGBTable();
            m_posMarkerCloud->showColors(true);
            m_targetCloud->addChild(m_posMarkerCloud);
            m_app->addToDB(m_posMarkerCloud, false, true, false, false);
        }
        m_posMarkerCloud->addPoint(pointCoords);
        m_posMarkerCloud->addColor(ccColor::Rgb(190, 242, 58)); // Lime
    }
    else
    {
        m_negativeSeeds.push_back(pointIndex);
        if (!m_negMarkerCloud) {
            m_negMarkerCloud = new ccPointCloud("Negative Seeds");
            m_negMarkerCloud->setPointSize(10);
            m_negMarkerCloud->resizeTheRGBTable();
            m_negMarkerCloud->showColors(true);
            m_targetCloud->addChild(m_negMarkerCloud);
            m_app->addToDB(m_negMarkerCloud, false, true, false, false);
        }
        m_negMarkerCloud->addPoint(pointCoords);
        m_negMarkerCloud->addColor(ccColor::Rgb(242, 19, 135)); // Pink
    }

    m_app->refreshAll(); 
    
    // UI will trigger the runRegionGrowing via the StateChanged signal, 
    // but we emit a dummy change to force it if needed, or rely on ActionA.
    // For now, let's let ActionA handle the rerunning to get slider values.
    
}

// Custom Struct for the Priority Queue
struct QueueItem {
    double cost;
    unsigned int index;
    int label; // 1 = Positive, 2 = Negative
    
    // Min-heap comparison
    bool operator>(const QueueItem& other) const {
        return cost > other.cost; 
    }
};

int SeedPicker::runRegionGrowing(double searchRadius, double tauThreshold, double ws, double wc)
{
    if (!m_targetCloud || m_positiveSeeds.empty() || m_originalColors.empty()) return 0;

    ccOctree::Shared octree = m_targetCloud->getOctree();
    if (!octree) {
        m_targetCloud->computeOctree();
        octree = m_targetCloud->getOctree();
        if (!octree) return 0;
    }

    // 1. Reset Cloud to Original Colors (Erase previous pink segment)
    for (unsigned i = 0; i < m_targetCloud->size(); ++i) {
        m_targetCloud->setPointColor(i, m_originalColors[i]);
    }

    // 2. Initialize tracking arrays
    unsigned cloudSize = m_targetCloud->size();
    std::vector<double> minCost(cloudSize, std::numeric_limits<double>::infinity());
    std::vector<int> labels(cloudSize, 0); // 0=Unassigned, 1=Pos, 2=Neg

    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

    // Load initial seeds
    for (unsigned int idx : m_positiveSeeds) {
        if (idx < cloudSize) {
            pq.push({0.0, idx, 1});
            minCost[idx] = 0.0;
        }
    }
    for (unsigned int idx : m_negativeSeeds) {
        if (idx < cloudSize) {
            pq.push({0.0, idx, 2});
            minCost[idx] = 0.0;
        }
    }

    unsigned char searchLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(searchRadius));
    const double MAX_COLOR_DIST = 441.673; // sqrt(255^2 * 3)

    // 3. Dijkstra Growing Loop
    while (!pq.empty())
    {
        QueueItem current = pq.top();
        pq.pop();

        // If we already found a cheaper path to this point, skip
        if (current.cost > minCost[current.index]) continue;
        
        labels[current.index] = current.label;

        const CCVector3* p_curr = m_targetCloud->getPoint(current.index);
        const ccColor::Rgba c_curr = m_originalColors[current.index];

        CCCoreLib::DgmOctree::NeighboursSet neighbors;
        octree->getPointsInSphericalNeighbourhood(*p_curr, static_cast<PointCoordinateType>(searchRadius), neighbors, searchLevel);

        for (const auto& neighbor : neighbors)
        {
            unsigned int nIdx = neighbor.pointIndex;

            // Calculate Spatial Cost (0 to 1)
            const CCVector3* p_n = m_targetCloud->getPoint(nIdx);
            double dist = CCVector3::vdistance(p_curr->u, p_n->u);
            double normDist = dist / searchRadius;

            // Calculate Chromatic Cost (0 to 1)
            const ccColor::Rgba c_n = m_originalColors[nIdx];
            double dr = static_cast<double>(c_curr.r) - c_n.r;
            double dg = static_cast<double>(c_curr.g) - c_n.g;
            double db = static_cast<double>(c_curr.b) - c_n.b;
            double colorDist = std::sqrt(dr*dr + dg*dg + db*db);
            double normColor = colorDist / MAX_COLOR_DIST;

            // Combined Step Cost (Scaled to 0 - 100 to match tau slider)
            double step_cost = 100.0 * ((ws * normDist) + (wc * normColor));

            // Stop growing along this path if step exceeds the user threshold
            if (step_cost > tauThreshold) continue;

            double new_cumulative_cost = current.cost + step_cost;

            // If this is a strictly cheaper path to this neighbor, update it
            if (new_cumulative_cost < minCost[nIdx])
            {
                minCost[nIdx] = new_cumulative_cost;
                pq.push({new_cumulative_cost, nIdx, current.label});
            }
        }
    }

    // 4. Colorize the result and count points
    int positiveCount = 0;
    for (unsigned i = 0; i < cloudSize; ++i)
    {
        if (labels[i] == 1) {
            m_targetCloud->setPointColor(i, ccColor::magenta); // Bright pink for visualization
            positiveCount++;
        }
    }

    m_targetCloud->showColors(true);
    m_app->refreshAll(); 

    return positiveCount;
}