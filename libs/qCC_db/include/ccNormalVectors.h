// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#ifndef CC_NORMAL_VECTORS_HEADER
#define CC_NORMAL_VECTORS_HEADER

// Local
#include "ccGenericPointCloud.h"

// System
#include <vector>

//! Compressed normal vectors handler
class QCC_DB_LIB_API ccNormalVectors
{
  public:
	//! Returns unique instance
	static ccNormalVectors* GetUniqueInstance();

	//! Releases unique instance
	/** Call to this method is now optional.
	 **/
	static void ReleaseUniqueInstance();

	//! Returns the number of compressed normal vectors
	static inline unsigned GetNumberOfVectors()
	{
		return static_cast<unsigned>(GetUniqueInstance()->m_theNormalVectors.size());
	}

	//! Static access to ccNormalVectors::getNormal
	static inline const CCVector3& GetNormal(unsigned normIndex)
	{
		return GetUniqueInstance()->getNormal(normIndex);
	}

	//! Returns the precomputed normal corresponding to a given compressed index
	inline const CCVector3& getNormal(unsigned normIndex) const
	{
		return m_theNormalVectors[normIndex];
	}

	//! Returns the compressed index corresponding to a normal vector
	static CompressedNormType GetNormIndex(const PointCoordinateType N[]);
	//! Returns the compressed index corresponding to a normal vector (shortcut)
	static inline CompressedNormType GetNormIndex(const CCVector3& N)
	{
		return GetNormIndex(N.u);
	}

	//! 'Default' orientations
	enum Orientation
	{

		PLUS_X              = 0,  //!< N.x always positive
		MINUS_X             = 1,  //!< N.x always negative
		PLUS_Y              = 2,  //!< N.y always positive
		MINUS_Y             = 3,  //!< N.y always negative
		PLUS_Z              = 4,  //!< N.z always positive
		MINUS_Z             = 5,  //!< N.z always negative
		PLUS_BARYCENTER     = 6,  //!< Normals always opposite to the cloud barycenter
		MINUS_BARYCENTER    = 7,  //!< Normals always towards the cloud barycenter
		PLUS_ORIGIN         = 8,  //!< Normals always opposite to the origin
		MINUS_ORIGIN        = 9,  //!< Normals always towards the origin
		PREVIOUS            = 10, //!< Re-use previous normal (if any)
		PLUS_SENSOR_ORIGIN  = 11, //!< Normals opposite to the associated sensor origin (if any, and if multiple, the first one will be used)
		MINUS_SENSOR_ORIGIN = 12, //!< Normals towards the associated sensor origin (if any, and if multiple, the first one will be used)
		UNDEFINED           = 255 //!< Undefined (no orientation is required)
	};

	//! Computes normal at each point of a given cloud
	/** \param cloud point cloud on which to process the normals.
	    \param theNormsCodes array in which the normals indexes are stored
	    \param localModel which kind of model to use for the computation (LS = plane, QUADRIC = quadratic Height Function, TRI = triangulation)
	    \param localRadius local neighborhood radius (not necessary for TRI)
	    \param preferredOrientation specifies a preferred orientation for normals (optional)
	    \param progressCb progress notification (optional)
	    \param inputOctree inputOctree input cloud octree (optional).
	    \return success
	**/
	static bool ComputeCloudNormals(ccGenericPointCloud*                cloud,
	                                NormsIndexesTableType&              theNormsCodes,
	                                CCCoreLib::LOCAL_MODEL_TYPES        localModel,
	                                PointCoordinateType                 localRadius,
	                                Orientation                         preferredOrientation = UNDEFINED,
	                                CCCoreLib::GenericProgressCallback* progressCb           = nullptr,
	                                CCCoreLib::DgmOctree*               inputOctree          = nullptr);

	//! Updates normals orientation based on a preferred orientation
	/** \param theCloud point cloud on which to process the normals.
	    \param theNormsCodes array in which the normals indexes are stored
	    \param preferredOrientation specifies a preferred orientation for normals
	    \return success
	**/
	static bool UpdateNormalOrientations(ccGenericPointCloud*   theCloud,
	                                     NormsIndexesTableType& theNormsCodes,
	                                     Orientation            preferredOrientation);

	//! Converts a normal vector to geological 'strike & dip' parameters (N[dip]°E - [strike]°)
	/** \param[in] N normal (should be normalized!)
	    \param[out] strike_deg strike value (in degrees)
	    \param[out] dip_deg dip value (in degrees)
	**/
	static void ConvertNormalToStrikeAndDip(const CCVector3& N, PointCoordinateType& strike_deg, PointCoordinateType& dip_deg);

	//! Converts a normal vector to geological 'dip direction & dip' parameters (float version)
	/** See http://en.wikipedia.org/wiki/Strike_and_dip
	    The dip direction is the azimuth of the direction (in [0,360[).
	    The dip is always in [0,90].
	    \param[in] N normal (should be normalized!)
	    \param[out] dip_deg value (in degrees)
	    \param[out] dipDir_deg dip direction value (in degrees)
	**/
	static void ConvertNormalToDipAndDipDir(const CCVector3f& N, float& dip_deg, float& dipDir_deg);

	//! Converts a normal vector to geological 'dip direction & dip' parameters (double version)
	/** See http://en.wikipedia.org/wiki/Strike_and_dip
	    The dip direction is the azimuth of the direction (in [0,360[).
	    The dip is always in [0,90].
	    \param[in] N normal (should be normalized!)
	    \param[out] dip_deg value (in degrees)
	    \param[out] dipDir_deg dip direction value (in degrees)
	**/
	static void ConvertNormalToDipAndDipDir(const CCVector3d& N, double& dip_deg, double& dipDir_deg);

	//! Converts a couple of geological 'dip direction & dip' parameters to a unit normal vector (float version)
	/** \param[in] dip_deg value (in degrees)
	    \param[in] dipDir_deg dip direction value(in degrees)
	    \param[in] upward whether the output normal vector should point towards Z+ (true) or Z- (false)
	    \return unit normal vector
	**/
	static CCVector3f ConvertDipAndDipDirToNormal(float dip_deg, float dipDir_deg, bool upward = true);

	//! Converts a couple of geological 'dip direction & dip' parameters to a unit normal vector (double version)
	/** \param[in] dip_deg value (in degrees)
	    \param[in] dipDir_deg dip direction value(in degrees)
	    \param[in] upward whether the output normal vector should point towards Z+ (true) or Z- (false)
	    \return unit normal vector
	**/
	static CCVector3d ConvertDipAndDipDirToNormal(double dip_deg, double dipDir_deg, bool upward = true);

	//! Converts geological 'strike & dip' parameters (N[dip]°E - [strike]°) to a string
	/** \param[in] strike_deg strike value (in degrees)
	    \param[in] dip_deg dip  value (in degrees)
	    \return formatted string "N[strike]°E - [dip]°"
	**/
	static QString ConvertStrikeAndDipToString(double& strike_deg, double& dip_deg);

	//! Converts geological 'dip direction & dip' parameters to a string
	/** \param[in] dip_deg dip angle value (in degrees)
	    \param[in] dipDir_deg dip direction value (in degrees)
	    \return formatted string "Dip direction: [dipDir]° - Dip angle: [dip]°"
	**/
	static QString ConvertDipAndDipDirToString(PointCoordinateType dip_deg, PointCoordinateType dipDir_deg);

	//! Converts a normal vector to HSV color space
	/** Uses 'strike & dip' parameters (H=strike, S=dip, V=constant)
	    \param[in]  N normal (should be normalized!)
	    \param[out] H hue [0;360[
	    \param[out] S saturation [0;1]
	    \param[out] V value [0;1]
	**/
	static void ConvertNormalToHSV(const CCVector3& N, float& H, float& S, float& V);

	//! Converts a normal vector to RGB color space
	/** Uses 'ConvertNormalToHSV' then converts HSV to RGB.
	    \param[in] N normal (should be normalized!)
	    \return RGB value (components between 0 and MAX_COLOR_COMP)
	**/
	static ccColor::Rgb ConvertNormalToRGB(const CCVector3& N);

  public:
	//! Default destructor
	virtual ~ccNormalVectors() = default;

	//! Allocates normal HSV colors array
	/** Mandatory for HSV color related methods (getNormalHSVColor, etc.)
	 **/
	bool enableNormalHSVColorsArray();

	//! Returns the HSV color equivalent to a given compressed normal index
	const ccColor::Rgb& getNormalHSVColor(unsigned index) const;

	//! Returns the HSV color array
	inline const std::vector<ccColor::Rgb>& getNormalHSVColorArray() const
	{
		return m_theNormalHSVColors;
	}

	//! Helper: computes the normal (with best LS fit)
	static bool ComputeNormalWithLS(CCCoreLib::GenericIndexedCloudPersist* pointAndNeighbors, CCVector3& N);

	//! Helper: computes the normal (with Delaunay 2.5D)
	/** The normal is computed at the first point (assuming the others are its neighbors).
	 **/
	static bool ComputeNormalWithTri(CCCoreLib::GenericIndexedCloudPersist* pointAndNeighbors, CCVector3& N);

	//! Helper: computes the normal (with Delaunay 2.5D)
	/** The normal is computed at the first point (assuming the others are its neighbors).
	 **/
	static bool ComputeNormalWithQuadric(CCCoreLib::GenericIndexedCloudPersist* points, const CCVector3& P, CCVector3& N);

  protected:
	//! Default constructor
	/** Shouldn't be called directly. Use 'GetUniqueInstance' instead.
	 **/
	ccNormalVectors();

	//! Inits internal structures
	bool init();

	//! Compressed normal vectors
	std::vector<CCVector3> m_theNormalVectors;

	//! 'HSV' colors corresponding to each compressed normal index
	/** In fact, HSV color has already been converted to RGB here for faster display.
	 **/
	std::vector<ccColor::Rgb> m_theNormalHSVColors;

	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithQuadric(const CCCoreLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCCoreLib::NormalizedProgress* nProgress = nullptr);
	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithLS(const CCCoreLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCCoreLib::NormalizedProgress* nProgress = nullptr);
	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithTri(const CCCoreLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCCoreLib::NormalizedProgress* nProgress = nullptr);
};

#endif // CC_NORMAL_VECTORS_HEADER
