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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2219                                                              $
//$LastChangedDate:: 2012-07-20 18:03:24 +0200 (ven., 20 juil. 2012)       $
//**************************************************************************
//

#ifndef CC_CALIBRATED_IMAGE_HEADER
#define CC_CALIBRATED_IMAGE_HEADER

//CCLib
#include <CCGeom.h>
#include <GenericIndexedCloud.h>

#include "ccImage.h"
#include "ccGLMatrix.h"

#include <vector>

class ccPointCloud;
class QDir;

//! Image associated to 3D calibration information
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccCalibratedImage : public ccImage
#else
class ccCalibratedImage : public ccImage
#endif
{
public:

	//! Default constructor
	ccCalibratedImage();

    //! Returns unique class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_CALIBRATED_IMAGE;};

	//! Sets camera focal (in pixels)
	/** \param deduceFOV automatically deduce f.o.v. from focal value and image height
	**/
	void setFocal(float focal_pix, bool deduceFOV = true);

	//! Returns focal (in pixels)
	inline float getFocal() const {return m_focal_pix;}

	//! Sets camera f.o.v. (field of view) value in degrees
	inline void setFov(float fov_deg) {m_fov_deg = fov_deg;}

	//! Returns camera f.o.v. (field of view) value in degrees
	inline float getFov() const {return m_fov_deg;}

	//! Sets camera matrix (orientation as an axis + angle)
	/** \param axis view axis
		\param angle_rad rotation angle around view axis (in radians)
		\param t translation
	**/
	void setCameraMatrix(const CCVector3& axis, float angle_rad, const CCVector3& t);

	//! Sets camera matrix (rotation + translation)
	/** \param mat 4x4 transformation matrix
	**/
	void setCameraMatrix(const ccGLMatrix& mat);

	//! Returns camera matrix
	const ccGLMatrix& getCameraMatrix() const;

	//! Sets distortion coefficients
	void setDistortionCoefficients(float k1, float k2);

	//! Returns distortion coefficients
	void getDistortionCoefficients(float& k1, float& k2) const;

	//! Applies undistortion coefficients
	/** Correction: r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
		Be sure to call setFocal & setDistortionCoefficients first!
	**/
	bool undistort();

	//! Key point
	struct KeyPoint
	{
		//! 'x' coordinate in pixels
		float x;
		
		//! 'y' coordinate in pixels
		float y;

		//! Index in associated point cloud
		unsigned index;

		//! Default constructor
		KeyPoint()
			: x(0)
			, y(0)
			, index(0)
		{
		}

		//! Constructor from a pixel and its index in associated cloud
		KeyPoint(float Px, float Py, unsigned indexInCloud)
			: x(Px)
			, y(Py)
			, index(indexInCloud)
		{
		};
	};

	//! Projective ortho-rectification (as cloud)
	/** Requires at least 4 key points!
		\param keypoints3D keypoints in 3D
		\param keypointsImage corresponding keypoints in image
		\return ortho-rectified image as a point cloud
	**/
	ccPointCloud* orthoRectifyAsCloud(CCLib::GenericIndexedCloud* keypoints3D, std::vector<KeyPoint>& keypointsImage) const;

	//! Projective ortho-rectification (as image)
	/** Requires at least 4 key points!
		\param keypoints3D keypoints in 3D
		\param keypointsImage corresponding keypoints in image
		\param pixelSize pixel size (auto if -1)
		\param minCorner (optional) outputs 3D min corner (2 values)
		\param maxCorner (optional) outputs 3D max corner (2 values)
		\param realCorners (optional) image real 3D corners (4*2 values)
		\return ortho-rectified image
	**/
	ccImage* orthoRectifyAsImage(CCLib::GenericIndexedCloud* keypoints3D, 
									std::vector<KeyPoint>& keypointsImage, 
									double& pixelSize,
									double* minCorner=0,
									double* maxCorner=0,
									double* realCorners = 0) const;

	//! Projective ortho-rectification for mutliple images
	/** \param images set of N calibrated images
		\param a {a0, a1, a2} triplets for all images (size: 3*N)
		\param b {b0, b1, b2} triplets for all images (size: 3*N)
		\param c {c0(=1), c1, c2} triplets for all images (size: 3*N)
		\param maxSize output image(s) max dimension
		\param result [out] resulting images (is successful)
		\param relativePos [out] relative position (relatively to first image)
		\return true if successful
	**/
	static bool OrthoRectifyAsImages(std::vector<ccCalibratedImage*> images, 
									double a[], double b[], double c[],
									unsigned maxSize,
									QDir* outputDir=0,
									std::vector<ccImage*>* orthoRectifiedImages=0,
									std::vector<std::pair<double,double> >* relativePos=0);

	//! Computes ortho-rectification parameters
	/** Requires at least 4 key points!
		Collinearity equation:
		* x'i = (a0+a1.xi+a2.yi)/(1+c1.xi+c2.yi)
		* y'i = (b0+b1.xi+b2.yi)/(1+c1.xi+c2.yi)
		\param keypoints3D keypoints in 3D
		\param keypointsImage corresponding keypoints in image
		\param a a0, a1 & a2 parameters
		\param b b0, b1 & b2 parameters
		\param c c0(=1), c1 & c2 parameters
		\return success
	**/
	bool computeOrthoRectificationParams(CCLib::GenericIndexedCloud* keypoints3D, std::vector<KeyPoint>& keypointsImage, double a[], double b[], double c[]) const;

protected:

	//! Computes f.o.v. from focal value (in pixels)
	void setFovFromFocal(float focal_pix);

	//! Camera field of view (in degrees)
	/** Equivalent to OpenGL 'fovy' (see gluPerspective)
	**/
	float m_fov_deg;

	//! Focal distance (in pixels)
	float m_focal_pix;

	//! Camera position (optic center)
	CCVector3 m_pos;

	//! Camera matrix (orientation + translation)
	ccGLMatrix m_trans;

	//! 1st radial distortion coefficient (k1)
	float m_k1;
	//! 2nd radial distortion coefficient (k1)
	float m_k2;

};

#endif
