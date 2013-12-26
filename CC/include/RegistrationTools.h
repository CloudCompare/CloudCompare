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

#ifndef REGISTRATION_TOOLS_HEADER
#define REGISTRATION_TOOLS_HEADER

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

#include "CCToolbox.h"
#include "PointProjectionTools.h"
#include "KdTree.h"


//system
#include <vector>

namespace CCLib
{

class GenericProgressCallback;
class GenericCloud;
class GenericIndexedCloud;
class ScalarField;

//! Common point cloud registration algorithms
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API RegistrationTools : public CCToolbox
#else
class RegistrationTools : public CCToolbox
#endif
{
protected:

	//! Shortcut to PointProjectionTools::ScaledTransformation
    typedef PointProjectionTools::Transformation ScaledTransformation;

    //! ICP Registration procedure with optional scale estimation
    /** Determines the best quaternion (a couple qR|qT) and optionally
		a scale 's' (different from a priori scale Sa) to bring the cloud
		P closer to the reference cloud X (one step). Refer to the ICP
		algorithm theory for more details about this procedure, and to
		"Point Set Registration with Integrated Scale Estimation",
		Znisser et al, PRIP 2005 for the scale estimation.

			X = Sa.s.R.P + T (with Sa = s = 1 by default)

		Warning: P and X must have the same size, and must be in the same
		order (i.e. P[i] is the point equivalent to X[i] for all 'i').

        \param P the cloud to register (data)
        \param X the reference cloud (model)
        \param trans the resulting transformation
		\param adjustScale whether to estimate scale (s) as well (see jschmidt 2005)
        \param weightsP weights for the registered points (optional)
        \param weightsX weights for the reference points (optional)
        \param aPrioriScale 'a priori' scale (Sa) between P and X
        \return success
    **/
    static bool RegistrationProcedure(GenericCloud* P,
										GenericCloud* X,
										ScaledTransformation& trans,
										bool adjustScale = false,
										ScalarField* weightsP = 0,
										ScalarField* weightsX = 0,
										PointCoordinateType aPrioriScale = 1.0f);

};

//! Horn point cloud registration algorithm (Horn).
#ifdef CC_USE_AS_DLL
class CC_DLL_API HornRegistrationTools : public RegistrationTools
#else
class HornRegistrationTools : public RegistrationTools
#endif
{
public:

	//! Returns "absolute orientation" (scale + transformation) between two set of (unordered) points
	/** Warning: both clouds must have the same size (and at least 3 points)
		Output transformation is from the left (L) to the right (R) coordinate system
		\param lCloud left cloud {Pl}
		\param rCloud right cloud {Pr}
		\param trans resulting transformation: Pr = s.R.Pl + T
		\param fixedScale force scale parameter to 1.0
		\return success
	**/
	static bool FindAbsoluteOrientation(GenericCloud* lCloud,
										GenericCloud* rCloud,
										ScaledTransformation& trans,
										bool fixedScale = false);

	//! Computes RMS between two clouds given a transformation and a scale
	/** Warning: both clouds must have the same size (and at least 3 points)
		RMS = Sqrt ( Sum( square_norm( Pr - s*R*Pl+T ) ) / count )
		\param lCloud left cloud {Pl}
		\param rCloud right cloud {Pr}
		\param trans transformation: Pr = s.R.Pl + T
		\return RMS (or -1.0 if an error occurred)
	**/
	static double ComputeRMS(GenericCloud* lCloud,
								GenericCloud* rCloud,
								const ScaledTransformation& trans);

};

//! ICP point cloud registration algorithm (Besl et al.).
#ifdef CC_USE_AS_DLL
class CC_DLL_API ICPRegistrationTools : public RegistrationTools
#else
class ICPRegistrationTools : public RegistrationTools
#endif
{
public:

    //! Convergence control method
    enum CC_ICP_CONVERGENCE_TYPE
    {
        MAX_ERROR_CONVERGENCE   = 0,
        MAX_ITER_CONVERGENCE    = 1,
    };

    //! Errors
    enum CC_ICP_RESULT
    {
        ICP_NOTHING_TO_DO               = 0,
        ICP_APPLY_TRANSFO               = 1,
        ICP_ERROR                       = 100,
        //all errors should be greater than ICP_ERROR
        ICP_ERROR_REGISTRATION_STEP     = 101,
        ICP_ERROR_DIST_COMPUTATION      = 102,
        ICP_ERROR_NOT_ENOUGH_MEMORY     = 103,
    };

	//! Registers two point clouds
	/** This method implements the ICP algorithm (Besl et al.).
		Warning: be sure to activate an INPUT/OUTPUT scalar field on the data cloud
		\param modelList the reference cloud (won't move)
		\param dataList the cloud to register (will move)
		\param totalTrans the resulting transformation (once the algorithm has converged)
		\param convType convergence type
		\param minErrorDecrease the minimum (mean square) error decrease between two consecutive steps to continue process (ignored if convType is not MAX_ERROR_CONVERGENCE)
		\param nbMaxIterations the maximum number of iteration (ignored if convType is not MAX_ITER_CONVERGENCE)
		\param finalError [output] final error (rms)
		\param adjustScale release the scale during the registration procedure
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param filterOutFarthestPoints if true, the algorithm will automatically ignore farthest points from the reference, for better convergence
		\param samplingLimit maximum number of points per cloud (they are randomly resampled below this limit otherwise)
		\param modelWeights weights for model points (optional)
		\param dataWeights weights for data points (optional)
		\return algorithm result
	**/
	static CC_ICP_RESULT RegisterClouds(GenericIndexedCloudPersist* modelList,
                                        GenericIndexedCloudPersist* dataList,
                                        ScaledTransformation& totalTrans,
                                        CC_ICP_CONVERGENCE_TYPE convType,
                                        double minErrorDecrease,
                                        unsigned nbMaxIterations,
                                        double& finalError,
                                        bool adjustScale = false,
                                        GenericProgressCallback* progressCb = 0,
                                        bool filterOutFarthestPoints = false,
                                        unsigned samplingLimit = 20000,
										ScalarField* modelWeights = 0,
										ScalarField* dataWeights = 0);
};


//! Four Points Congruent Sets (4PCS) registration algorithm (Dror Aiger, Niloy J. Mitra, Daniel Cohen-Or)
#ifdef CC_USE_AS_DLL
class CC_DLL_API FPCSRegistrationTools : public RegistrationTools
#else
class FPCSRegistrationTools : public RegistrationTools
#endif
{
public:
    //! Registers two point clouds
    /** Implements the 4 Points Congruent Sets Algorithm (Dror Aiger, Niloy J. Mitra, Daniel Cohen-Or
        \param modelCloud the reference cloud (won't move)
		\param dataCloud the cloud to register (will move)
		\param transform the resulting transformation (output)
		\param delta maximal distance to the reference cloud for the data points to be considered as registered
		\param beta is used for bases selection (error tolerance)
		\param overlap estimation of the two clouds overlap rate
        \param nbBases number of iteration for the algorithm
        \param nbTries number of tries to find a base in the reference cloud
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
        \param nbMaxCandidates if>0, maximal number of candidate bases allowed for each step. Otherwise the number of candidates is not bounded
		\return false: failure ; true: success.
    **/
    static bool RegisterClouds(GenericIndexedCloud* modelCloud,
                                GenericIndexedCloud* dataCloud,
                                ScaledTransformation& transform,
                                ScalarType delta,
                                ScalarType beta,
                                PointCoordinateType overlap,
                                unsigned nbBases,
                                unsigned nbTries,
                                GenericProgressCallback* progressCb=0,
                                unsigned nbMaxCandidates = 0);

protected:

    //! FCPS base
    struct Base
    {
        unsigned a,b,c,d;
        void init(unsigned _a, unsigned _b, unsigned _c, unsigned _d) {a=_a; b=_b; c=_c; d=_d;}
        void copy(const struct Base& b) {init(b.a, b.b, b.c, b.d);}
        unsigned getIndex(unsigned i) {if(i==0) return a; if(i==1) return b; if(i==2) return c; if(i==3) return d; return 0;}
    };

    //! Randomly finds a 4 points base in a cloud
    /** \param cloud the point cloud in which we want to find a base
		\param overlap estimation of the overlap rate
        \param nbTries the maximum number of tries to find a base
        \param base the resulting base
        \return false: failure ; true: success
    **/
    static bool FindBase(GenericIndexedCloud* cloud,
                            PointCoordinateType overlap,
                            unsigned nbTries,
                            Base &base);

    /*! Find bases which are congruent to a specified 4 points base
        \param tree the KD-tree build from data cloud
        \param delta used for the tolerance when searching for congruent bases
        \param base the reference base made of 4 points
        \param results the resulting bases
        \return the number of bases found (number of element in the results array) or -1 is a problem occurred
    **/
    static int FindCongruentBases(KDTree* tree,
                                            ScalarType delta,
                                            const CCVector3* base[4],
                                            std::vector<Base>& results);

    //! Registration score computation function
    /**!
        \param modelTree KD-tree containing the model point cloud
        \param dataCloud data point cloud
        \param dataToModel transformation that, applied to data points, register model and data clouds
        \param delta tolerance above which data points are not counted (if a point is less than delta-appart from de model cloud, then it is counted)
        \return the number of data points which are distance-appart from the model cloud
    **/
    static unsigned ComputeRegistrationScore(KDTree *modelTree,
                                                    GenericIndexedCloud *dataCloud,
                                                    ScalarType delta,
                                                    ScaledTransformation& dataToModel);

    //! Find the 3D pseudo intersection between two lines
    /** This function finds the 3D point which is the nearest from the both lines (when this point is unique, i.e. when
        the lines are not parallel)
        \param p0 first of the two distinct points defining the first line (lying on the line)
        \param p1 second of the two distinct points defining the first line (lying on the line)
        \param p2 first of the two distinct points defining the second line
        \param p3 first of the two distinct points defining the second line
        \param inter [out] is the computed intersection (function output)
        \param lambda [out] coeff such that p0+lambda(p1-p0) is the point of [p0, p1] which is the nearest from [p2, p3]
        \param mu [out] coeff such that p2+mu(p3-p2) is the point of [p2, p3] which is the nearest from [p0, p1]
        \return false: no intersection was found (lines may be parallel); true: inter is the pseudo intersection
    **/
    static bool LinesIntersections(const CCVector3 &p0,
                                    const CCVector3 &p1,
                                    const CCVector3 &p2,
                                    const CCVector3 &p3,
                                    CCVector3 &inter,
                                    PointCoordinateType& lambda,
                                    PointCoordinateType& mu);

    /**!function to keep only the N best candidates bases (by comparison with the reference base invariants)
        Let B1 and B2 be 2 candidates, R be the reference, B1 and B2 aligned with R.
        B1 is better than B2 if the distance between B1 and R points is smaller than distance between B2 and R points.
        This function also computes and store the rigid transforms that align candidates with reference base
        \param modelCloud the model point cloud to work on
        \param dataCloud the data point cloud to work on
        \param reference reference base
        \param candidates array of candidates bases. At the end of the function, this array contains the nbMaxBases best candidates only
        \param nbMaxCandidates maximal number of candidates allowed (if 0, number of candidates is not bounded)
        \param transforms array of rigid transforms that align candidates bases with the reference base
        \return false if something went wrong
    **/
    static bool FilterCandidates(GenericIndexedCloud *modelCloud,
                                    GenericIndexedCloud *dataCloud,
                                    Base& reference,
                                    std::vector<Base>& candidates,
                                    unsigned nbMaxCandidates,
                                    std::vector<ScaledTransformation>& transforms);
};

}

#endif //REGISTRATION_TOOLS_HEADER
