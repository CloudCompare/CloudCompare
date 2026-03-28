#pragma once

#include <CCGeom.h>
#include <vector>

class ccPointCloud;
class FFDLattice;

//! Helper class to apply FFD deformation to point clouds with smooth continuous transformation
class ccFFDDeformationApplier
{
public:
    ccFFDDeformationApplier(ccPointCloud* cloud, FFDLattice* lattice);
    ~ccFFDDeformationApplier() = default;

    //! Store original cloud state (call once at the beginning)
    bool initializeOriginalPositions();

    //! Apply current deformation to ALL points for smooth preview
    bool updateDeformedCloud();

    //! Apply deformation permanently to the original cloud (full resolution)
    bool applyDeformation();

    //! Reset cloud to original positions
    bool resetDeformation();

    //! Returns the number of points that are inside the lattice
    size_t getPointsInsideLattice() const { return m_pointsInsideLattice; }

    //! Set subsample rate for preview (0.0-1.0, where 1.0 = all points)
    void setSubsampleRatio(float ratio) { m_subsampleRatio = std::max(0.01f, std::min(1.0f, ratio)); }

private:
    ccPointCloud* m_cloud = nullptr;
    FFDLattice* m_lattice = nullptr;

    std::vector<CCVector3d> m_originalPositions;
    std::vector<size_t> m_subsampledIndices;  // Indices of subsampled points for faster preview
    
    size_t m_pointsInsideLattice = 0;
    float m_subsampleRatio = 1.0f;  // Update all points by default
    
    bool isPointInsideLattice(const CCVector3d& point) const;
    CCVector3d transformPointByLatticeDeformation(const CCVector3d& originalPos) const;
};
