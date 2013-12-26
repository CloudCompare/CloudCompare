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
#include <GenericIndexedMesh.h>
#include <GenericProgressCallback.h>
#include <DgmOctree.h>
#include <GeometricalAnalysisTools.h>

//Local
#include "ccGenericPointCloud.h"

//system
#include <math.h>

//! Compressed normals quantization level (number of directions/bits: 2^(2*N+3))
const unsigned NORMALS_QUANTIZE_LEVEL	=	6;

//! Compressed normal vectors handler
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccNormalVectors
#else
class ccNormalVectors
#endif
{
public:

	//! Returns unique instance
	static ccNormalVectors* GetUniqueInstance();

	//! Releases unique instance
	static void ReleaseUniqueInstance();

	//! Returns the number of compressed normal vectors
	static inline unsigned GetNumberOfVectors() { return GetUniqueInstance()->m_numberOfVectors; }

	//! Static access to ccNormalVectors::getNormal
	static inline const PointCoordinateType* GetNormal(unsigned normIndex) { return GetUniqueInstance()->getNormal(normIndex); }

	//! Returns the precomputed normal corresponding to a given compressed index
	inline const PointCoordinateType* getNormal(unsigned normIndex) const { return m_theNormalVectors+normIndex*3; }

	//! Computes the normal corresponding to a given compressed index
	/** Warning: slower than 'GetNormal' (but avoids computation of the whole table)
	**/
	static inline void ComputeNormal(normsType normIndex, PointCoordinateType N[]) { Quant_dequantize_normal(normIndex,NORMALS_QUANTIZE_LEVEL,N); }

	//! Returns the compressed index corresponding to a normal vector
	static inline normsType GetNormIndex(const PointCoordinateType N[]) { return (normsType)Quant_quantize_normal(N,NORMALS_QUANTIZE_LEVEL); }

	//! Inverts normal corresponding to a given compressed index
	/** Warning: compressed index is directly updated!
	**/
	static void InvertNormal(normsType &code);

    //! Computes normal at each point of a given cloud
    /**
        \param theCloud point cloud on which to process the normals.
        \param theNormsCodes array in which the normals indexes are stored
        \param method which kind of model to use for the computation (LS = plane, HF = quadratic Height Function, TRI = triangulation)
		\param radius local neighborhood radius (not necessary for TRI)
        \param preferedOrientation specifies a preferred orientation for normals (-1: no preferred orientation, 0:X, 1:-X, 2:Y, 3:-Y, 4:Z, 5: -Z, 6:+Barycenter, 7:-Barycenter)
        \param progressCb progress bar
        \param _theOctree octree associated with theCloud.
    **/
	static bool ComputeCloudNormals(ccGenericPointCloud* theCloud,
                                    NormsIndexesTableType& theNormsCodes,
                                    CC_LOCAL_MODEL_TYPES method,
									PointCoordinateType radius,
                                    int preferedOrientation=-1,
                                    CCLib::GenericProgressCallback* progressCb=0,
                                    CCLib::DgmOctree* _theOctree=0);

	//! Converts a normal vector to geological 'strike & dip' parameters (N[dip]�E - [strike]�)
	/** \param[in] N normal (should be normalized!)
		\param[out] strike strike value (in degrees)
		\param[out] dip dip value (in degrees)
	**/
	static void ConvertNormalToStrikeAndDip(const CCVector3& N, double& strike, double& dip);

	//! Converts a normal vector to geological 'dip direction & dip' parameters
	/** See http://en.wikipedia.org/wiki/Strike_and_dip
		The dip direction is the azimuth of the direction (in [0,360[).
		The dip is always in [0,90].
		\param[in] N normal (should be normalized!)
		\param[out] dip value (in degrees)
		\param[out] dipDir dip direction value (in degrees)
	**/
	static void ConvertNormalToDipAndDipDir(const CCVector3& N, PointCoordinateType& dip, PointCoordinateType& dipDir);

	//! Converts geological 'strike & dip' parameters (N[dip]�E - [strike]�) to a string
	/** \param[in] strike strike value (in degrees)
		\param[in] dip dip  value (in degrees)
		\return formatted string "N[strike]�E - [dip]�"
	**/
	static QString ConvertStrikeAndDipToString(double& strike, double& dip);

	//! Converts geological 'dip direction & dip' parameters to a string
	/** \param[in] dip dip angle value (in degrees)
		\param[in] dipDir dip direction value (in degrees)
		\return formatted string "Dip direction: [dipDir]� - Dip angle: [dip]�"
	**/
	static QString ConvertDipAndDipDirToString(PointCoordinateType dip, PointCoordinateType dipDir);

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

	//! Default destructor
	virtual ~ccNormalVectors();

	//! Inits internal structures
	void init(unsigned quantizeLevel);

	//! Compressed normal vectors array
	PointCoordinateType* m_theNormalVectors;

	//! 'HSV' colors corresponding to each compressed normal index
	/** In fact, HSV color has already been converted to RGB here for faster display.
	**/
	colorType* m_theNormalHSVColors;

	//! Number of compressed normal vectors
	unsigned m_numberOfVectors;

	//! Decompression algorithm
    static void Quant_dequantize_normal(unsigned q, unsigned level, PointCoordinateType* res);
	//! Compression algorithm
    static unsigned Quant_quantize_normal(const PointCoordinateType* n, unsigned level);

	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithHF(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCLib::NormalizedProgress* nProgress = 0);
	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithLS(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCLib::NormalizedProgress* nProgress = 0);
	//! Cellular method for octree-based normal computation
	static bool ComputeNormsAtLevelWithTri(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters, CCLib::NormalizedProgress* nProgress = 0);
};

 #endif //CC_NORMAL_VECTORS_HEADER
