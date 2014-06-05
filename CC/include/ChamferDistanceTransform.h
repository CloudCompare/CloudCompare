//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CHAMFER_DISTANCE_TRANSFORM_HEADER
#define CHAMFER_DISTANCE_TRANSFORM_HEADER

//Local
#include "CCCoreLib.h"
#include "CCConst.h"
#include "MathTools.h"

namespace CCLib
{

class GenericProgressCallback;
class NormalizedProgress;

//! Class to compute and handle Chamfer distances on a 3D grid
class CC_CORE_LIB_API ChamferDistanceTransform : MathTools
{

public:

	//! Chamfer distance type
	typedef unsigned short GridElement;//'short' to limit memory consumption

	//! Default constructor
	/** \param Di the grid size along the X dimension
		\param Dj the grid size along the Y dimension
		\param Dk the grid size along the Z dimension
	**/
	ChamferDistanceTransform(unsigned Di, unsigned Dj, unsigned Dk);

	//! Default destructor
	virtual ~ChamferDistanceTransform();

	//! Initializes the 3D grid
	/** The Chamfer distance is computed on a 3D grid.
		This grid must be initialized in memory prior to any action.
		\return true if the initialization succeeded
	**/
	bool init();

	//! Computes the Chamfer distance on the whole grid
	/** Propagates the distances on the whole grid. The 'zeros' should
		have already been initialized before calling this method (see
		ChamferDistanceTransform::setZero).
		\param type the Chamfer distance type
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return max distance (or -1 if an error occurred)
	**/
	int propagateDistance(CC_CHAMFER_DISTANCE_TYPE type, GenericProgressCallback* progressCb = 0);

	//! Sets a cell as a "zero"
	/** Chamfer distance is computed on the whole grid relatively to the
		"zero" cells.
		\param i the cell coordinate along the X dimension
		\param j the cell coordinate along the Y dimension
		\param k the cell coordinate along the Z dimension
	**/
	void setZero(int i, int j, int k);

	//! Sets a cell as a "zero"
	/** Chamfer distance is computed on the whole grid relatively to the
		"zero" cells.
		\param cellPos the cell position (as a 3-size array)
	**/
	void setZero(int cellPos[]);

	//! Returns a cell "distance"
	/** This method should be called after the Chamfer distance had
		been properly propagated (see ChamferDistanceTransform::propagateDistance).
		\param i the cell coordinate along the X dimension
		\param j the cell coordinate along the Y dimension
		\param k the cell coordinate along the Z dimension
		\return the Chamfer distance value
	**/
	GridElement getValue(int i, int j, int k) const;

	//! Returns a cell "distance"
	/** This method should be called after the Chamfer distance had
		been properly propagated (see ChamferDistanceTransform::propagateDistance).
		\param cellPos the cell position (as a 3-size array)
		\return the Chamfer distance value
	**/
	GridElement getValue(int cellPos[]);

protected:

	//! Internal method for distance propagation
	/** \return max distance
	**/
	GridElement propagateDistance(	GridElement iStart,
									GridElement jStart,
									GridElement kStart,
									int sign,
									const int neighbours[14][4],
									NormalizedProgress* normProgress = 0);

    //! Grid structure
	GridElement *m_grid;

	//! Grid dimension along the X dimension
	unsigned m_gridX;
	//! Grid dimension along the Y dimension
	unsigned m_gridY;
	//! Grid dimension along the Z dimension
	unsigned m_gridZ;
    //! Shift along the Y dimension for faster element access
	int m_decY;
    //! Shift along the Z dimension for faster element access
	int m_decZ;
	//! First index of innerbound grid
	int m_decIndex;
};

}

#endif //CHAMFER_DISTANCE_TRANSFORM_HEADER
