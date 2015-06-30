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

#ifndef CC_NORMAL_VECTORS_HEADER
#define CC_NORMAL_VECTORS_HEADER

//CCLib
#include <CCGeom.h>
#include <GenericIndexedMesh.h>
#include <GenericProgressCallback.h>
#include <DgmOctree.h>
#include <GeometricalAnalysisTools.h>

//Local
#include "qCC_db.h"
#include "ccGenericPointCloud.h"

//system
#include <math.h>
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
	static inline unsigned GetNumberOfVectors() { return static_cast<unsigned>(GetUniqueInstance()->m_theNormalVectors.size()); }

	//! Static access to ccNormalVectors::getNormal
	static inline const CCVector3& GetNormal(unsigned normIndex) { return GetUniqueInstance()->getNormal(normIndex); }

	//! Returns the precomputed normal corresponding to a given compressed index
	inline const CCVector3& getNormal(unsigned normIndex) const { return m_theNormalVectors[normIndex]; }

	//! Compressed normals quantization level (number of directions/bits: 2^(2*N+3))
	static const unsigned QUANTIZE_LEVEL = 6;

	//! Computes the normal corresponding to a given compressed index
	/** Warning: slower than 'GetNormal' (but avoids computation of the whole table)
	**/
	static inline void ComputeNormal(normsType normIndex, PointCoordinateType N[]) { Quant_dequantize_normal(normIndex,QUANTIZE_LEVEL,N); }

	//! Returns the compressed index corresponding to a normal vector
	static inline normsType GetNormIndex(const PointCoordinateType N[]) { return static_cast<normsType>( Quant_quantize_normal(N,QUANTIZE_LEVEL) ); }
	//! Returns the compressed index corresponding to a normal vector (shortcut)
	static inline normsType GetNormIndex(const CCVector3& N) { return GetNormIndex(N.u); }

	//! Inverts normal corresponding to a given compressed index
	/** Warning: compressed index is directly updated!
	**/
	static void InvertNormal(normsType &code);

	//! 'Default' orientations
	enum Orientation {

		PLUS_X  = 0,
		MINUS_X = 1,
		PLUS_Y  = 2,
		MINUS_Y = 3,
		PLUS_Z  = 4,
		MINUS_Z = 5,
		PLUS_BARYCENTER  = 6,
		MINUS_BARYCENTER = 7,
		PLUS_ZERO  = 8,
		MINUS_ZERO = 9,
		PREVIOUS   = 10,
		
		UNDEFINED  = 255
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
	static bool ComputeCloudNormals(ccGenericPointCloud* cloud,
									NormsIndexesTableType& theNormsCodes,
									CC_LOCAL_MODEL_TYPES localModel,
									PointCoordinateType localRadius,
									Orientation preferredOrientation = UNDEFINED,
									CCLib::GenericProgressCallback* progressCb = 0,
									CCLib::DgmOctree* inputOctree = 0);

	//! Tries to guess a very naive 'local radius' for normals computation (see ComputeCloudNormals)
	/** \param cloud point cloud on which to process the normals.
		\return naive radius (percentage of the cloud bounding-box)
	**/
	static PointCoordinateType GuessNaiveRadius(ccGenericPointCloud* cloud);

	//! Tries to guess the best 'local radius' for normals computation (see ComputeCloudNormals)
	/** \param cloud point cloud on which to process the normals.
		\param cloudOctree input cloud octree (optional)
		\param progressCb progress notification (optional)
		\return the best radius (strictly positive value) or 0 if an error occurred
	**/
	static PointCoordinateType GuessBestRadius(	ccGenericPointCloud* cloud,
												CCLib::DgmOctree* cloudOctree = 0,
												CCLib::GenericProgressCallback* progressCb = 0);

	//! Updates normals orientation based on a preferred orientation
	/** \param theCloud point cloud on which to process the normals.
		\param theNormsCodes array in which the normals indexes are stored
		\param preferredOrientation specifies a preferred orientation for normals
		\return success
	**/
	static bool UpdateNormalOrientations(	ccGenericPointCloud* theCloud,
											NormsIndexesTableType& theNormsCodes,
											Orientation preferredOrientation);

	//! Converts a normal vector to geological 'strike & dip' parameters (N[dip]°E - [strike]°)
	/** \param[in] N normal (should be normalized!)
		\param[out] strike_deg strike value (in degrees)
		\param[out] dip_deg dip value (in degrees)
	**/
	static void ConvertNormalToStrikeAndDip(const CCVector3& N, double& strike_deg, double& dip_deg);

	//! Converts a normal vector to geological 'dip direction & dip' parameters
	/** See http://en.wikipedia.org/wiki/Strike_and_dip
		The dip direction is the azimuth of the direction (in [0,360[).
		The dip is always in [0,90].
		\param[in] N normal (should be normalized!)
		\param[out] dip_deg value (in degrees)
		\param[out] dipDir_deg dip direction value (in degrees)
	**/
	static void ConvertNormalToDipAndDipDir(const CCVector3& N, PointCoordinateType& dip_deg, PointCoordinateType& dipDir_deg);

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
		\param N [in] normal (should be normalized!)
		\param H [out] hue [0;360[
		\param S [out] saturation [0;1]
		\param V [out] value [0;1]
	**/
	static void ConvertNormalToHSV(const CCVector3& N, double& H, double& S, double& V);

	//! Converts a normal vector to RGB color space
	/** Uses 'ConvertNormalToHSV' then converts HSV to RGB.
		\param N [in] normal (should be normalized!)
		\param R [out] red [0;MAX_COLOR_COMP]
		\param G [out] green [0;MAX_COLOR_COMP]
		\param B [out] blue [0;MAX_COLOR_COMP]
	**/
	static void ConvertNormalToRGB(const CCVector3& N, colorType& R, colorType& G, colorType& B);

	//! Converts a HSV color to RGB color space
	/** \param H [out] hue [0;360[
		\param S [out] saturation [0;1]
		\param V [out] value [0;1]
		\param R [out] red [0;MAX_COLOR_COMP]
		\param G [out] green [0;MAX_COLOR_COMP]
		\param B [out] blue [0;MAX_COLOR_COMP]
	**/
	static void ConvertHSVToRGB(double H, double S, double V, colorType& R, colorType& G, colorType& B);

public:

	//! Default destructor
	virtual ~ccNormalVectors();

	//! Allocates normal HSV colors array
	/** Mandatory for HSV color related methods (getNormalHSVColor, etc.)
	**/
	bool enableNormalHSVColorsArray();

	//! Returns the HSV color equivalent to a given compressed normal index
	const colorType* getNormalHSVColor(unsigned index) const;

	//! Returns the HSV color array
	const colorType* getNormalHSVColorArray() const;

protected:

	//! Default constructor
	/** Shouldn't be called directly. Use 'GetUniqueInstance' instead.
	**/
	ccNormalVectors();

	//! Inits internal structures
	bool init(unsigned quantizeLevel);

	//! Compressed normal vectors
	std::vector<CCVector3> m_theNormalVectors;

	//! 'HSV' colors corresponding to each compressed normal index
	/** In fact, HSV color has already been converted to RGB here for faster display.
	**/
	colorType* m_theNormalHSVColors;

	//! Decompression algorithm
	static void Quant_dequantize_normal(unsigned q, unsigned level, PointCoordinateType* res);
	//! Compression algorithm
	static unsigned Quant_quantize_normal(const PointCoordinateType* n, unsigned level);

	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithQuadric(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCLib::NormalizedProgress* nProgress = 0);
	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithLS(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCLib::NormalizedProgress* nProgress = 0);
	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithTri(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCLib::NormalizedProgress* nProgress = 0);
};

 #endif //CC_NORMAL_VECTORS_HEADER
