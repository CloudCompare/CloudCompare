#include "ccFFDDeformationApplier.h"
#include "FFDLattice.h"

#include <ccPointCloud.h>
#include <ccBBox.h>
#include <cstdlib>
#include <ctime>

ccFFDDeformationApplier::ccFFDDeformationApplier(ccPointCloud* cloud, FFDLattice* lattice)
    : m_cloud(cloud)
    , m_lattice(lattice)
{
    // Seed random number generator for subsampling
    srand(static_cast<unsigned>(time(nullptr)));
}

bool ccFFDDeformationApplier::initializeOriginalPositions()
{
    if (!m_cloud || !m_lattice)
        return false;

    m_originalPositions.clear();
    m_subsampledIndices.clear();
    m_pointsInsideLattice = 0;

    size_t pointCount = m_cloud->size();
    m_originalPositions.reserve(pointCount);

    // Store original positions of ALL points
    // Build subsampled indices for faster preview
    for (size_t i = 0; i < pointCount; ++i)
    {
        const CCVector3* pt = m_cloud->getPoint(i);
        CCVector3d pt3d(pt->x, pt->y, pt->z);
        m_originalPositions.push_back(pt3d);

        // Count points initially inside lattice (informational only)
        if (isPointInsideLattice(pt3d))
        {
            m_pointsInsideLattice++;
        }

        // Add to subsampled list based on ratio
        if ((rand() / static_cast<float>(RAND_MAX)) < m_subsampleRatio)
        {
            m_subsampledIndices.push_back(i);
        }
    }

    return true;
}

CCVector3d ccFFDDeformationApplier::transformPointByLatticeDeformation(const CCVector3d& originalPos) const
{
    if (!m_lattice)
        return originalPos;

    // Use FFD lattice's deform function for cubic B-spline interpolation
    // This smoothly deforms ALL points based on lattice control point positions
    return m_lattice->deformPoint(originalPos);
}

bool ccFFDDeformationApplier::updateDeformedCloud()
{
    if (!m_cloud || !m_lattice || m_originalPositions.empty())
        return false;

    // Apply FFD deformation to SUBSAMPLED points for preview (much faster)
    for (size_t idx : m_subsampledIndices)
    {
        if (idx >= m_originalPositions.size())
            continue;

        const CCVector3d& origPos = m_originalPositions[idx];

        // Apply FFD transformation - smooth B-spline interpolation
        CCVector3d deformedPos = transformPointByLatticeDeformation(origPos);

        // Update the cloud point
        CCVector3 newPos(static_cast<float>(deformedPos.x),
                        static_cast<float>(deformedPos.y),
                        static_cast<float>(deformedPos.z));
        *const_cast<CCVector3*>(m_cloud->getPoint(idx)) = newPos;
    }

    // Notify cloud of changes
    m_cloud->invalidateBoundingBox();
    m_cloud->pointsHaveChanged();

    return true;
}

bool ccFFDDeformationApplier::applyDeformation()
{
    if (!m_cloud || !m_lattice || m_originalPositions.empty())
        return false;

    // Apply deformation to ALL points (should already be done by preview)
    for (size_t i = 0; i < m_originalPositions.size(); ++i)
    {
        const CCVector3d& origPos = m_originalPositions[i];

        // Apply FFD transformation
        CCVector3d deformedPos = transformPointByLatticeDeformation(origPos);
        CCVector3 newPos(static_cast<float>(deformedPos.x),
                        static_cast<float>(deformedPos.y),
                        static_cast<float>(deformedPos.z));
        *const_cast<CCVector3*>(m_cloud->getPoint(i)) = newPos;
    }

    // Update original positions to current (committed)
    for (size_t i = 0; i < m_originalPositions.size(); ++i)
    {
        const CCVector3* pt = m_cloud->getPoint(i);
        m_originalPositions[i] = CCVector3d(pt->x, pt->y, pt->z);
    }

    m_cloud->invalidateBoundingBox();
    m_cloud->pointsHaveChanged();

    return true;
}

bool ccFFDDeformationApplier::resetDeformation()
{
    if (!m_cloud || m_originalPositions.empty())
        return false;

    // Restore all original positions
    for (size_t i = 0; i < m_originalPositions.size(); ++i)
    {
        const CCVector3d& origPos = m_originalPositions[i];
        CCVector3 newPos(static_cast<float>(origPos.x),
                        static_cast<float>(origPos.y),
                        static_cast<float>(origPos.z));
        *const_cast<CCVector3*>(m_cloud->getPoint(i)) = newPos;
    }

    m_cloud->invalidateBoundingBox();
    m_cloud->pointsHaveChanged();

    return true;
}

bool ccFFDDeformationApplier::isPointInsideLattice(const CCVector3d& point) const
{
    if (!m_lattice)
        return false;

    const auto& bbox = m_lattice->getBoundingBox();
    return bbox.contains(CCVector3(static_cast<float>(point.x),
                                   static_cast<float>(point.y),
                                   static_cast<float>(point.z)));
}
