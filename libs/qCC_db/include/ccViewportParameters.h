#pragma once

//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Local
#include "ccSerializableObject.h"
#include "ccGLMatrix.h"

//CCCoreLib
#include <CCGeom.h>

class QRect;

//! Standard parameters for GL displays/viewports
class QCC_DB_LIB_API ccViewportParameters : public ccSerializableObject
{
public: //functions

	//! Default constructor
	ccViewportParameters();

	//! Copy constructor
	ccViewportParameters(const ccViewportParameters& params);

	//inherited from ccSerializableObject
	bool isSerializable() const override { return true; }
	bool toFile(QFile& out, short dataVersion) const override;
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion() const override;

	//! Sets the pivot point (for object-centered view mode)
	void setPivotPoint(const CCVector3d& P, bool autoUpdateFocal);

	//! Returns the pivot point (for object-centered view mode)
	const CCVector3d& getPivotPoint() const { return pivotPoint; }

	//! Sets the camera center
	//* _
	void setCameraCenter(const CCVector3d& C, bool autoUpdateFocal);

	//! Returns the camera center
	const CCVector3d& getCameraCenter() const { return cameraCenter; }

	//! Sets the 'focal' distance
	/** \warning changes the camera center position in object-centered view mode
	**/
	void setFocalDistance(double distance);

	//! Computes the 'focal' distance
	double getFocalDistance() const { return focalDistance; }

	//! Computes the view matrix
	ccGLMatrixd computeViewMatrix() const;

	//! Computes the scale matrix
	ccGLMatrixd computeScaleMatrix(const QRect& glViewport) const;

	//! Returns the viewing direction
	/** This is the direction normal to the screen
		(pointing 'forward') in the world coordinate system.
	**/
	CCVector3d getViewDir() const;

	//! Returns the up direction
	/** This is the vertical direction of the screen
		(pointing 'upward') in the world coordinate system.
	**/
	CCVector3d getUpDir() const;

	//! Returns the view rotation 'center'
	/** The rotation center is defined as:
		- the pivot point in object-centered view mode
		- the camera center in viewer-centered view mode
	**/
	const CCVector3d& getRotationCenter() const;

	//! Computes the ratio 'distance to half width' (based on the current FOV)
	/** Half width = ratio * distance = tan(fov / 2) * distance
	**/
	double computeDistanceToHalfWidthRatio() const;

	//! Computes the ratio 'distance to width' (based on the current FOV)
	/** Width = ratio * distance = (2 * tan(fov / 2)) * distance
	**/
	double computeDistanceToWidthRatio() const;

	//! Computes the object 'width' at the 'focal' distance
	double computeWidthAtFocalDist() const;

	//! Computes the pixel size at the 'focal' distance
	double computePixelSize(int glWidth) const;

	//! Logs the viewport parameters
	void log() const;

public: //variables

	//! Visualization matrix (rotation only)
	ccGLMatrixd viewMat;

	//! Point size
	float defaultPointSize;
	//! Line width
	float defaultLineWidth;

	//! Perspective view state
	bool perspectiveView;
	//! Whether view is centered on displayed scene (true) or on the user eye (false)
	/** Always true for ortho. mode.
	**/
	bool objectCenteredView;

	//! Theoretical perspective 'zNear' relative position
	double zNearCoef;
	
	//! Depth of the near clipping plane (if any)
	double nearClippingDepth;
	//! Depth of the far clipping plane (if any)
	double farClippingDepth;
	
	//! Current zNear value
	double zNear;
	//! Current zFar value
	double zFar;
	
	//! Camera F.O.V. (field of view) in degrees
	float fov_deg;
	//! Camera aspect ratio
	float cameraAspectRatio;

protected:

	//! Focal distance
	double focalDistance;

	//! Rotation pivot point (for object-centered view modes)
	CCVector3d pivotPoint;

	//! Camera center
	CCVector3d cameraCenter;
};
