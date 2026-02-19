#pragma once

#include <ccHObject.h>
#include <ccBBox.h>
#include <CCGeom.h>

#include <array>
#include <vector>

class ccFFDLatticeDisplay : public ccHObject
{
public:
	ccFFDLatticeDisplay(const ccBBox& bbox,
	                   const std::array<unsigned int, 3>& dims,
	                   const std::vector<CCVector3d>& controlPoints);

	ccBBox getOwnBB(bool withGLFeatures = false) override;

	void setControlPoints(const std::vector<CCVector3d>& controlPoints);
	void setSelectedIndices(const std::vector<int>& indices);

protected:
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

private:
	void updateBoundingBox();
	const CCVector3d& getControlPoint(unsigned int x, unsigned int y, unsigned int z) const;

	ccBBox m_bbox;
	std::array<unsigned int, 3> m_dims;
	std::vector<CCVector3d> m_controlPoints;
	std::vector<int> m_selectedIndices;
};
