//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                      COPYRIGHT: Thomas Dewez, BRGM                     #
//#                                                                        #
//##########################################################################

#ifndef QFACET_KD_TREE_BASED_FACET_EXTRACTION_HEADER
#define QFACET_KD_TREE_BASED_FACET_EXTRACTION_HEADER

//qCC_db
#include <ccKdTree.h>

class ccKdTreeForFacetExtraction
{
public:

	//! Default constructor
	ccKdTreeForFacetExtraction();

	//! Fuses cells
	/** Creates a new scalar fields with the groups indexes.
		\param kdTree Kd-tree
		\param maxError max error after fusion (see errorMeasure)
		\param errorMeasure error measure type
		\param maxAngle_deg maximum angle between two sets to allow fusion (in degrees)
		\param overlapCoef maximum relative distance between two sets to accept fusion (1 = no distance, < 1 = overlap, > 1 = gap)
		\param closestFirst
		\param progressCb for progress notifications (optional)
	**/
	static bool FuseCells(	ccKdTree* kdTree,
							double maxError,
							CCLib::DistanceComputationTools::ERROR_MEASURES errorMeasure,
							double maxAngle_deg,
							PointCoordinateType overlapCoef = 1,
							bool closestFirst = true,
							CCLib::GenericProgressCallback* progressCb = 0);

};

#endif //QFACET_KD_TREE_BASED_FACET_EXTRACTION_HEADER
