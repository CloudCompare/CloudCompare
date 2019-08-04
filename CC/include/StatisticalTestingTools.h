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

#ifndef STATISTICAL_TESTING_TOOLS_HEADER
#define STATISTICAL_TESTING_TOOLS_HEADER

//Local
#include "CCToolbox.h"
#include "DgmOctree.h"

namespace CCLib
{

class GenericCloud;
class GenericDistribution;
class GenericIndexedCloud;
class GenericIndexedCloudPersist;
class GenericProgressCallback;

//! Statistical testing algorithms (Chi2 distance computation, statistic filtering, etc.)
class CC_CORE_LIB_API StatisticalTestingTools : public CCToolbox
{
public:

	//! Computes the Chi2 distance on a sample of scalar values
	/** The Chi2 distance is computed between an empiric distribution generated from a
		set of scalar values (with a specific number of classes), and a theoretical distribution.
		It assures that each class of the empirical distribution is such that it respects n.pi>=5
		(where n is the total number of points, and pi is the cumulative probability of the class).
		Therefore the number of classes can be changed by the method.
		If the 'noClassCompression' parameter is set to true, the above condition is not
		checked and distance can diverge (which should not be possible according to the Chi2 Test theory,
		but it can be useful for classification purposes).
		\param distrib a theoretical distribution
		\param cloud a subset of points (associated to scalar values)
		\param numberOfClasses initial number of classes for the empirical distribution (0 for automatic determination, >1 otherwise)
		\param finalNumberOfClasses final number of classes of the empirical distribution
		\param noClassCompression prevent the algorithm from performing classes compression (faster but less accurate)
		\param histoMin [optional] minimum histogram value
		\param histoMax [optional] maximum histogram value
		\param[out] histoValues [optional] histogram array (its size should be equal to the initial number of classes)
		\param[out] npis [optional] array containing the theoretical probabilities for each class (its size should be equal to the initial number of classes)
		\return the Chi2 distance (or -1.0 if an error occurred)
	**/
	static double computeAdaptativeChi2Dist(const GenericDistribution* distrib,
											const GenericCloud* cloud,
											unsigned numberOfClasses,
											unsigned &finalNumberOfClasses,
											bool noClassCompression = false,
											const ScalarType* histoMin = nullptr,
											const ScalarType* histoMax = nullptr,
											unsigned* histoValues = nullptr,
											double* npis = nullptr);

	//! Computes the Chi2 fractile
	/** Returns the max Chi2 Distance for a given "confidence" probability and a given number of
		"degrees of liberty" (equivalent to the number of classes-1).
		\param p the result "confidence"
		\param d the number of d.o.l.
		\return the Chi2 fractile
	**/
	static double computeChi2Fractile(double p, int d);

	//! Computes the Chi2 confidence probability
	/** Returns the Chi2 confidence probability for a given Chi2 distance value and a given
		number of "degress of liberty" (equivalent to the number of classes-1).
		\param chi2result the Chi2 distance
		\param d the number of d.o.l.
		\return the Chi2 confidence probability
	**/
	static double computeChi2Probability(double chi2result, int d);

	//! Classfies the point cloud in two category by locally applying a statistical (Chi2) test
	/** This algorithm is described in Daniel Girardeau-Montaut's PhD manuscript (Chapter 3.
		section 3.2.3). It mainly consists in determining if a point associated scalar value
		is part of the measurements noise (in which case the point will be considered as
		"unchanged") or if not (in which case the point will be considered as a trully "changed").
		This determination is based on a statistical analysis (Chi2 Test) of the scalar values
		distribution around each point (by considering a small neighbourhood of points around
		each point). The classification result will depend on the Chi2 Test parameters (e.g.
		the number of classes - which in this case is equal to the square root of the neighbourhood
		size - and the confidence probability - the greater it is, the more severe the result is).
		WARNING : the scalar field behind the GenericIndexedCloud interface should be splet in 2,
		one for reading the scalar values to test (OUT - getScalarValue) and the other to save the
		resulting Chi2 distance (IN - setScalarValue).
		\param distrib a theoretical noise distribution
		\param theCloud the point cloud to classify
		\param numberOfNeighbours the neighbourhood size for the local analysis
		\param pTrust the Chi2 Test confidence probability
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree the cloud octree if it has already be computed
		\return the distance threshold for filtering (or -1 if someting went wrong during the process)
	**/
	static double testCloudWithStatisticalModel(const GenericDistribution* distrib,
												GenericIndexedCloudPersist* theCloud,
												unsigned numberOfNeighbours,
												double pTrust,
												GenericProgressCallback* progressCb = nullptr,
												DgmOctree* inputOctree = nullptr);

protected:

	//! Computes (locally) the Chi2 distance inside an octree cell
	/** Additional parameters are:
		- (GenericDistribution*) the theoretical noise distribution
		- (int) the size of a neighbourhood for local analysis
		- (int) the number of classes for the Chi2 distance computation
		- (unsigned*) a pre-allocated array (of a size equal to the number of classes)for computation acceleration
		- (bool) specifies whether negative values should be included in computation
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computeLocalChi2DistAtLevel(const DgmOctree::octreeCell& cell,
											void** additionalParameters,
											NormalizedProgress* nProgress = nullptr);

};

}

#endif //STATISTICAL_TESTING_TOOLS_HEADER
