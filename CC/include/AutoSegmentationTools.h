//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef AUTO_SEGMENTATION_TOOLS_HEADER
#define AUTO_SEGMENTATION_TOOLS_HEADER

#include <vector>

//Local
#include "CCToolbox.h"
#include "CCTypes.h"

namespace CCLib
{

class GenericIndexedCloudPersist;
class GenericProgressCallback;
class DgmOctree;
class ReferenceCloud;

//! A standard container to store several subsets of points
/** Several algorithms of the AutoSegmentationTools toolbox return a collection of subsets of points
	corresponding to each segmented part. Such a collection is generally stored in this type of container.
**/
using ReferenceCloudContainer = std::vector<ReferenceCloud *>;

//! Several point cloud auto-segmentation algorithms (Connected Components, Front propagation, etc.)
class CC_CORE_LIB_API AutoSegmentationTools : public CCToolbox
{
public:

	//! Labels connected components from a point cloud
	/** The algorithm is described in Daniel Girardeau-Montaut's PhD manuscript
		(Chapter 3, section 3.2.4). The main parameter is the maximal distance
		between two points in order to consider them as belonging to the same
		connected component. Actually, this parameter is not expressed as is, but
		as the length of a cell of the octree for a given level of subdivision (this
		leeds to a great enhancement of the process speed). If the level is "n",
		then the cell size will be equal to the maximum length of the bounding box
		of the point cloud divided by 2^n. The greater the level "n" is, the smaller
		the cell size will be, and therefore the process will produce more
		connected components.To label the CCs, this implementation of the algorithm
		use the distance field (via Generic3dPoint::setDist). So be sure to
		store the original distance field (or to deviate the setDist process) if
		you don't want it to be replaced.
		\param theCloud the point cloud to label
		\param level the level of subdivision of the octree (between 1 and MAX_OCTREE_LEVEL)
		\param sixConnexity indicates if the CC's 3D connexity should be 6 (26 otherwise)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree the cloud octree if it has already been computed
		\return the number of components (>= 0) or an error code (< 0 - see DgmOctree::extractCCs)
	**/
	static int labelConnectedComponents(GenericIndexedCloudPersist* theCloud,
										unsigned char level,
										bool sixConnexity = false,
										CCLib::GenericProgressCallback* progressCb = nullptr,
										CCLib::DgmOctree* inputOctree = nullptr);

	//! Extracts connected components from a point cloud
	/** This method shloud only be called after the connected components have been
		labeled (see AutoSegmentationTools::labelConnectedComponents). This
		implementation of the algorithm assumes that the CCs labels are stored for 
		each point in the associated scalar field.
		Warning: be sure to set the labels S.F. as OUTPUT (reading)
		\param theCloud the point cloud to segment
		\param ccc the extracted connected compenents (as a list of subsets of points)
		\return success
	**/
	static bool extractConnectedComponents(GenericIndexedCloudPersist* theCloud, ReferenceCloudContainer& ccc);

	//! Segment a point cloud by propagating fronts constrained by values of the point cloud associated scalar field
	/** The algorithm is described in Daniel Girardeau-Montaut's PhD manuscript
		(Chapter 3, section 3.3). It consists mainly in propagating a front on
		the surface implicitly represented by the point cloud and making this
		propagation dependent on the scalar values associated to each point (such
		as the distance information computed between the point cloud and another
		entity). The propgation is realized with the Fast Marching Algorithm applied
		on a gridded structure (the octree in this case).
		Warning: be sure to activate an OUTPUT scalar field on the input cloud
		\param theCloud the point cloud to segment
		\param minSeedDist the minimum value associated to the point where to start the propagation from ("security" value)
		\param radius spherical neighborhood size (or 0 for automatic size)
		\param octreeLevel level of subdivision where to apply the gridding (the greater it is, the smaller and numerous the segmented parts will be)
		\param theSegmentedLists the segmented parts (as a list of subsets of points)
		\param applyGaussianFilter to specify if a gaussian filter should be applied after computing the scalar field gradient (to smooth the results)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree the cloud octree if it has already be computed
		\param alpha the gaussian filter kernel size (needed only if a gaussian filtering pass is required)
		\return success
	**/
	static bool frontPropagationBasedSegmentation(	GenericIndexedCloudPersist* theCloud,
													PointCoordinateType radius,
													ScalarType minSeedDist,
													unsigned char octreeLevel,
													ReferenceCloudContainer& theSegmentedLists,
													CCLib::GenericProgressCallback* progressCb = nullptr,
													CCLib::DgmOctree* inputOctree = nullptr,
													bool applyGaussianFilter = false,
													float alpha = 2.0f);

};

}

#endif //AUTO_SEGMENTATION_TOOLS_HEADER
