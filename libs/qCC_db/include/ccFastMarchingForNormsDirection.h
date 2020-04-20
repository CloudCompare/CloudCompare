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

#ifndef CC_FAST_MARCHING_DIRECTION_HEADER
#define CC_FAST_MARCHING_DIRECTION_HEADER

//CCLib
#include <DgmOctree.h>
#include <FastMarching.h>

//qCC_db
#include "ccAdvancedTypes.h"

//system
#include <vector>

class ccGenericPointCloud;
class ccPointCloud;
class ccOctree;
class ccProgressDialog;

//! Fast Marching algorithm for normals direction resolution
/** Extends the FastMarching class.
**/
class ccFastMarchingForNormsDirection : public CCLib::FastMarching
{

public:

	//! Static entry point (helper)
	static int OrientNormals(	ccPointCloud* theCloud,
								unsigned char octreeLevel,
								ccProgressDialog* progressCb = nullptr);
	//! Default constructor
	ccFastMarchingForNormsDirection();

	//! Initializes the grid with a point cloud (and ist corresponding octree)
	/** The points should be associated to an (active) scalar field.
		The Fast Marching grid will have the same dimensions as
		the input octree considered at a given level of subdivision.
		\param cloud the point cloud
		\param theNorms the normals array
		\param theOctree the associated octree
		\param gridLevel the level of subdivision
		\return a negative value if something went wrong
	**/
	int init(	ccGenericPointCloud* cloud,
				NormsIndexesTableType* theNorms,
				ccOctree* theOctree,
				unsigned char gridLevel);

	//! Updates a list of point flags, indicating the points already processed
	/** \return the number of resolved points
	**/
	unsigned updateResolvedTable(	ccGenericPointCloud* theCloud,
									std::vector<unsigned char>& resolved,
									NormsIndexesTableType* theNorms);

	//inherited methods (see FastMarchingAlgorithm)
	int propagate() override;

protected:

	//! A Fast Marching grid cell for normals direction resolution
	class DirectionCell : public CCLib::FastMarching::Cell
	{
	public:
		//! Default constructor
		DirectionCell()
			: Cell()
			, N(0,0,0)
			, C(0,0,0)
			, cellCode(0)
			, signConfidence(1)
#ifdef QT_DEBUG
			, scalar(0)
#endif
		{}

		///! Destructor
		~DirectionCell() override = default;

		//! The local cell normal
		CCVector3 N;
		//! The local cell center
		CCVector3 C;
		//! the code of the equivalent cell in the octree
		CCLib::DgmOctree::CellCode cellCode;
		//! Confidence value
		float signConfidence;
#ifdef QT_DEBUG
		//! Undefined scalar for debug purposes
		float scalar;
#endif
	};

	//inherited methods (see FastMarchingAlgorithm)
	float computeTCoefApprox(CCLib::FastMarching::Cell* currentCell, CCLib::FastMarching::Cell* neighbourCell) const override;
	int step() override;
	void initTrialCells() override;
	bool instantiateGrid(unsigned size) override { return instantiateGridTpl<DirectionCell*>(size); }

	//! Computes relative 'confidence' between two cells (orientations)
	/** \return confidence between 0 and 1
	**/
	float computePropagationConfidence(DirectionCell* originCell, DirectionCell* destCell) const;

	//! Resolves the direction of a given cell (once and for all)
	void resolveCellOrientation(unsigned index);
};

#endif
