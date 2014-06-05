//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_MST_FOR_NORMS_DIRECTION_HEADER
#define CC_MST_FOR_NORMS_DIRECTION_HEADER

//CCLib
#include <GenericProgressCallback.h>
#include <DgmOctree.h>

class ccPointCloud;

//! Minimum Spanning Tree for normals direction resolution
/** See http://people.maths.ox.ac.uk/wendland/research/old/reconhtml/node3.html
**/
class ccMinimumSpanningTreeForNormsDirection
{

public:

	//! Main entry point
	static bool Process(	ccPointCloud* cloud,
							CCLib::GenericProgressCallback* progressCb = 0,
							CCLib::DgmOctree* octree = 0);
};

#endif //CC_MST_FOR_NORMS_DIRECTION_HEADER
