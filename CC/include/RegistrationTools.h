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

#ifndef REGISTRATION_TOOLS_HEADER
#define REGISTRATION_TOOLS_HEADER

//Local
#include "PointProjectionTools.h"


namespace CCLib
{

class GenericProgressCallback;
class GenericCloud;
class GenericIndexedMesh;
class GenericIndexedCloud;
class KDTree;
class ScalarField;

//! Common point cloud registration algorithms
class CC_CORE_LIB_API RegistrationTools : public CCToolbox
{
public:

	//! Shortcut to PointProjectionTools::ScaledTransformation
	using ScaledTransformation = PointProjectionTools::Transformation;

	//! Transformation constraints
	enum TRANSFORMATION_FILTERS
	{
		SKIP_NONE			= 0,
		SKIP_RXY			= 1,
		SKIP_RYZ			= 2,
		SKIP_RXZ			= 4,
		SKIP_ROTATION		= 7,
		SKIP_TX				= 8,
		SKIP_TY				= 16,
		SKIP_TZ				= 32,
		SKIP_TRANSLATION	= 56,
	};

	//! 'Filters' a transformation by constraining it along certain rotation axes and translation directions
	/**	\param inTrans input transformation
		\param transformationFilters filters to be applied on the resulting transformation at each step (experimental) - see RegistrationTools::TRANSFORMATION_FILTERS flags
		\param outTrans output transformation
	**/
	static void FilterTransformation(	const ScaledTransformation& inTrans,
										int transformationFilters,
										ScaledTransformation& outTrans );

protected:

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
		\param coupleWeights weights for each (Pi,Xi) couple (optional)
		\param aPrioriScale 'a priori' scale (Sa) between P and X
		\return success
	**/
	static bool RegistrationProcedure(	GenericCloud* P,
										GenericCloud* X,
										ScaledTransformation& trans,
										bool adjustScale = false,
										ScalarField* coupleWeights = nullptr,
										PointCoordinateType aPrioriScale = 1.0f);

};

//! Horn point cloud registration algorithm
/** See 'Closed-form solution of absolute orientation using unit quaternions', B.K.P. Horn, 1987.
**/
class CC_CORE_LIB_API HornRegistrationTools : public RegistrationTools
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
	static double ComputeRMS(	GenericCloud* lCloud,
								GenericCloud* rCloud,
								const ScaledTransformation& trans);

};

//! ICP point cloud registration algorithm (Besl et al.).
class CC_CORE_LIB_API ICPRegistrationTools : public RegistrationTools
{
public:

	//! Convergence control method
	enum CONVERGENCE_TYPE
	{
		MAX_ERROR_CONVERGENCE	= 0,
		MAX_ITER_CONVERGENCE	= 1,
	};

	//! Errors
	enum RESULT_TYPE
	{
		ICP_NOTHING_TO_DO				= 0,
		ICP_APPLY_TRANSFO				= 1,
		ICP_ERROR						= 100,
		//all errors should be greater than ICP_ERROR
		ICP_ERROR_REGISTRATION_STEP		= 101,
		ICP_ERROR_DIST_COMPUTATION		= 102,
		ICP_ERROR_NOT_ENOUGH_MEMORY		= 103,
		ICP_ERROR_CANCELED_BY_USER		= 104,
		ICP_ERROR_INVALID_INPUT			= 105,
	};

	//! ICP Parameters
	struct Parameters
	{
		Parameters()
			: convType(MAX_ERROR_CONVERGENCE)
			, minRMSDecrease(1.0e-5)
			, nbMaxIterations(20)
			, adjustScale(false)
			, filterOutFarthestPoints(false)
			, samplingLimit(50000)
			, finalOverlapRatio(1.0)
			, modelWeights(nullptr)
			, dataWeights(nullptr)
			, transformationFilters(SKIP_NONE)
			, maxThreadCount(0)
		{}

		//! Convergence type
		CONVERGENCE_TYPE convType;

		//! The minimum error (RMS) reduction between two consecutive steps to continue process (ignored if convType is not MAX_ERROR_CONVERGENCE)
		double minRMSDecrease;

		//! The maximum number of iteration (ignored if convType is not MAX_ITER_CONVERGENCE)
		unsigned nbMaxIterations;

		//! Whether to release the scale parameter during the registration procedure or not
		bool adjustScale;

		//! If true, the algorithm will automatically ignore farthest points from the reference, for better convergence
		bool filterOutFarthestPoints;

		//! Maximum number of points per cloud (they are randomly resampled below this limit otherwise)
		unsigned samplingLimit;

		//! Theoretical overlap ratio (at each iteration, only this percentage (between 0 and 1) will be used for registration
		double finalOverlapRatio;

		//! Weights for model points (i.e. only if the model entity is a cloud) (optional)
		ScalarField* modelWeights;

		//! Weights for data points (optional)
		ScalarField* dataWeights;

		//! Filters to be applied on the resulting transformation at each step (experimental) - see RegistrationTools::TRANSFORMATION_FILTERS flags
		int transformationFilters;

		//! Maximum number of threads to use (0 = max)
		int maxThreadCount;
	};

	//! Registers two clouds or a cloud and a mesh
	/** This method implements the ICP algorithm (Besl et al.).
		\warning Be sure to activate an INPUT/OUTPUT scalar field on the point cloud.
		\warning The mesh is always the reference/model entity.
		\param modelCloud the reference cloud or the vertices of the reference mesh --> won't move
		\param modelMesh the reference mesh (optional) --> won't move
		\param dataCloud the cloud to register --> will move
		\param params ICP parameters
		\param[out] totalTrans the resulting transformation (once the algorithm has converged)
		\param[out] finalRMS final error (RMS)
		\param[out] finalPointCount number of points used to compute the final RMS
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return algorithm result
	**/
	static RESULT_TYPE Register(	GenericIndexedCloudPersist* modelCloud,
									GenericIndexedMesh* modelMesh,
									GenericIndexedCloudPersist* dataCloud,
									const Parameters& params,
									ScaledTransformation& totalTrans,
									double& finalRMS,
									unsigned& finalPointCount,
									GenericProgressCallback* progressCb = nullptr);


};


//! Four Points Congruent Sets (4PCS) registration algorithm (Dror Aiger, Niloy J. Mitra, Daniel Cohen-Or)
class CC_CORE_LIB_API FPCSRegistrationTools : public RegistrationTools
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
    static bool RegisterClouds(	GenericIndexedCloud* modelCloud,
                                GenericIndexedCloud* dataCloud,
                                ScaledTransformation& transform,
                                ScalarType delta,
                                ScalarType beta,
                                PointCoordinateType overlap,
                                unsigned nbBases,
                                unsigned nbTries,
                                GenericProgressCallback* progressCb = nullptr,
                                unsigned nbMaxCandidates = 0);

protected:

    //! FCPS base
    struct Base
    {
		unsigned a, b, c, d;
		void init(unsigned _a, unsigned _b, unsigned _c, unsigned _d) { a = _a; b = _b; c = _c; d = _d; }
        void copy(const struct Base& _b) {init(_b.a, _b.b, _b.c, _b.d);}
		unsigned getIndex(unsigned i) { if (i == 0) return a; if (i == 1) return b; if (i == 2) return c; if (i == 3) return d; return 0; }
    };

    //! Randomly finds a 4 points base in a cloud
    /** \param cloud the point cloud in which we want to find a base
		\param overlap estimation of the overlap rate
        \param nbTries the maximum number of tries to find a base
        \param base the resulting base
        \return false: failure ; true: success
    **/
    static bool FindBase(	GenericIndexedCloud* cloud,
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
    static int FindCongruentBases(	KDTree* tree,
									ScalarType delta,
									const CCVector3* base[4],
									std::vector<Base>& results);

    //! Registration score computation function
    /**!
        \param modelTree KD-tree containing the model point cloud
        \param dataCloud data point cloud
        \param dataToModel transformation that, applied to data points, register model and data clouds
        \param delta tolerance above which data points are not counted (if a point is less than delta-apart from the model cloud, then it is counted)
        \return the number of data points which are distance-apart from the model cloud
    **/
    static unsigned ComputeRegistrationScore(	KDTree *modelTree,
												GenericIndexedCloud *dataCloud,
												ScalarType delta,
												const ScaledTransformation& dataToModel);

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
    static bool LinesIntersections(	const CCVector3 &p0,
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
    static bool FilterCandidates(	GenericIndexedCloud *modelCloud,
									GenericIndexedCloud *dataCloud,
									Base& reference,
									std::vector<Base>& candidates,
									unsigned nbMaxCandidates,
									std::vector<ScaledTransformation>& transforms);
};

}

#endif //REGISTRATION_TOOLS_HEADER
