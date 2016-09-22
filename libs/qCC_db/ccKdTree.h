//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_KD_TREE_HEADER
#define CC_KD_TREE_HEADER

//CCLib
#include <TrueKdTree.h>

//Local
#include "ccHObject.h"

//System
#include <unordered_set>

class ccGenericPointCloud;

//! KD-tree structure
/** Extends the CCLib::TrueKdTree class.
**/
class QCC_DB_LIB_API ccKdTree : public CCLib::TrueKdTree, public ccHObject
{
public:

	//! Default constructor
	/** \param aCloud a point cloud
	**/
	explicit ccKdTree(ccGenericPointCloud* aCloud);

	//! Multiplies the bounding-box of the tree
	/** If the cloud coordinates are simply multiplied by the same factor,
		there is no use to recompute the tree structure. It's sufficient
		to update its bounding-box.
		\param  multFactor multiplication factor
	**/
	void multiplyBoundingBox(const PointCoordinateType multFactor);

	//! Translates the bounding-box of the tree
	/** If the cloud has simply been translated, there is no use to recompute
		the tree structure. It's sufficient to update its bounding-box.
		\param T translation vector
	**/
	void translateBoundingBox(const CCVector3& T);

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POINT_KDTREE; }

	//Inherited from ccHObject
	virtual ccBBox getOwnBB(bool withGLFeatures = false) override;

	//! Flag points with cell index (as a scalar field)
	bool convertCellIndexToSF();
	//! Flag points with a random color per leaf
	bool convertCellIndexToRandomColor();

	//! Returns the bounding-box of a given cell
	ccBBox getCellBBox(BaseNode* node) const;

	//! A set of leaves
	typedef std::unordered_set<Leaf*> LeafSet;

	//! Returns the neighbor leaves around a given cell
	bool getNeighborLeaves(BaseNode* cell, ccKdTree::LeafSet& neighbors, const int* userDataFilter = 0);

	//! Returns associated (generic) point cloud
	inline ccGenericPointCloud* associatedGenericCloud() const { return m_associatedGenericCloud; }

protected:

	//Inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Associated cloud
	ccGenericPointCloud* m_associatedGenericCloud;

};

#endif //CC_KD_TREE_HEADER
