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

#ifndef CC_GENERIC_GL_DISPLAY
#define CC_GENERIC_GL_DISPLAY

//Always first
#include "ccIncludeGL.h"

//Local
#include "ccViewportParameters.h"
#include "ccColorTypes.h"

//Qt
#include <QFont>

class QWidget;
class ccDrawableObject;

//! OpenGL camera parameters
struct ccGLCameraParameters
{
	ccGLCameraParameters()
		: viewport{0, 0, 0, 0}
		, perspective(false)
		, fov_deg(0.0f)
		, pixelSize(0.0)
		, nearClippingDepth(std::numeric_limits<double>::quiet_NaN())
		, farClippingDepth(std::numeric_limits<double>::quiet_NaN())
	{
	}

	//! Projects a 3D point in 2D (+ normalized 'z' coordinate)
	inline bool project(const CCVector3d& input3D, CCVector3d& output2D, bool* inFrustum = nullptr) const
	{
		return ccGL::Project<double, double>(	input3D,
												modelViewMat.data(),
												projectionMat.data(),
												viewport,
												output2D,
												inFrustum,
												(inFrustum && !std::isnan(nearClippingDepth)) ? &nearClippingDepth : nullptr,
												(inFrustum && !std::isnan(farClippingDepth)) ? &farClippingDepth : nullptr);
	}
	
	//! Projects a 3D point in 2D (+ normalized 'z' coordinate)
	inline bool project(const CCVector3& input3D, CCVector3d& output2D, bool* inFrustum = nullptr) const
	{
		return ccGL::Project<PointCoordinateType, double>(	input3D,
															modelViewMat.data(),
															projectionMat.data(),
															viewport,
															output2D,
															inFrustum,
															(inFrustum && !std::isnan(nearClippingDepth)) ? &nearClippingDepth : nullptr,
															(inFrustum && !std::isnan(farClippingDepth)) ? &farClippingDepth : nullptr);
	}

	//! Unprojects a 2D point (+ normalized 'z' coordinate) in 3D
	inline bool unproject(const CCVector3d& input2D, CCVector3d& output3D) const
	{
		return ccGL::Unproject<double, double>(input2D, modelViewMat.data(), projectionMat.data(), viewport, output3D);
	}
	
	//! Unprojects a 2D point (+ normalized 'z' coordinate) in 3D
	inline bool unproject(const CCVector3& input2D, CCVector3d& output3D) const
	{
		return ccGL::Unproject<PointCoordinateType, double>(input2D, modelViewMat.data(), projectionMat.data(), viewport, output3D);
	}

	//! Equality operator
	bool operator==(const ccGLCameraParameters& otherParams)
	{
		return (perspective == otherParams.perspective)
			&& (fov_deg == otherParams.fov_deg)
			&& (pixelSize == otherParams.pixelSize)
			&& (viewport[0] == otherParams.viewport[0])
			&& (viewport[1] == otherParams.viewport[1])
			&& (viewport[2] == otherParams.viewport[2])
			&& (viewport[3] == otherParams.viewport[3])
			&& (nearClippingDepth == otherParams.nearClippingDepth)
			&& (farClippingDepth == otherParams.farClippingDepth)
			&& (modelViewMat == otherParams.modelViewMat)
			&& (projectionMat == otherParams.projectionMat);
	}

	//! Model view matrix (GL_MODELVIEW)
	ccGLMatrixd modelViewMat;
	//! Projection matrix (GL_PROJECTION)
	ccGLMatrixd projectionMat;
	//! Viewport (GL_VIEWPORT)
	int viewport[4];
	//! Perspective mode
	bool perspective;
	//! F.O.V. (in degrees)
	float fov_deg;
	//! Pixel size (approximate if in perspective mode)
	double pixelSize;
	//! Near clipping depth
	double nearClippingDepth;
	//! Far clipping depth
	double farClippingDepth;
};

//! Generic interface for GL displays
class ccGenericGLDisplay
{
public:
	virtual ~ccGenericGLDisplay() = default;
		
	//! Returns the screen size
	virtual QSize getScreenSize() const = 0;

	//! Redraws display immediately
	virtual void redraw(bool only2D = false, bool resetLOD = true) = 0;

	//! Flags display as 'to be refreshed'
	/** See ccGenericGLDisplay::refresh.
	**/
	virtual void toBeRefreshed() = 0;

	//! Redraws display only if flagged as 'to be refreshed'
	/** See ccGenericGLDisplay::toBeRefreshed. Flag is turned
		to false after a call to this method.
		\param only2D whether to redraw everything (false) or only the 2D layer (true)
	**/
	virtual void refresh(bool only2D = false) = 0;

	//! Invalidates current viewport setup
	/** On next redraw, viewport information will be recomputed.
	**/
	virtual void invalidateViewport() = 0;

	//! Invalidates the 3D layer (FBO)
	/** On next redraw, the 3D layer will be updated
	**/
	virtual void deprecate3DLayer() = 0;

	//! Returns default text display font
	/** Warning: already takes rendering zoom into account!
	**/
	virtual QFont getTextDisplayFont() const = 0;

	//! Returns default label display font
	/** Warning: already takes rendering zoom into account!
	**/
	virtual QFont getLabelDisplayFont() const = 0;

	//! Text alignment
	enum TextAlign { ALIGN_HLEFT	= 1,
					 ALIGN_HMIDDLE	= 2,
					 ALIGN_HRIGHT	= 4,
					 ALIGN_VTOP		= 8,
					 ALIGN_VMIDDLE	= 16,
					 ALIGN_VBOTTOM	= 32,
					 ALIGN_DEFAULT	= 1 | 8};

	//! Displays a string at a given 2D position
	/** This method should be called solely during 2D pass rendering.
		The coordinates are expressed relatively to the current viewport (y = 0 at the top!).
		\param text string
		\param x horizontal position of string origin
		\param y vertical position of string origin
		\param align alignment position flags
		\param bkgAlpha background transparency (0 by default)
		\param color text color (optional)
		\param font optional font (otherwise default one will be used)
	**/
	virtual void displayText(	QString text,
								int x,
								int y,
								unsigned char align = ALIGN_DEFAULT,
								float bkgAlpha = 0.0f,
								const ccColor::Rgba* color = nullptr,
								const QFont* font = nullptr) = 0;

	//! Displays a string at a given 3D position
	/** This method should be called solely during 3D pass rendering (see paintGL).
		\param str string
		\param pos3D 3D position of string origin
		\param color RGBA color (optional: if let to 0, default text rendering color is used)
		\param font font (optional)
	**/
	virtual void display3DLabel(const QString& str,
								const CCVector3& pos3D,
								const ccColor::Rgba* color = nullptr,
								const QFont& font=QFont()) = 0;

	//! Returns the current OpenGL camera parameters
	virtual void getGLCameraParameters(ccGLCameraParameters& params) = 0;

	//! Converts 2D screen coordinates to 'centered' 2D OpenGL context coordinates
	virtual QPointF toCenteredGLCoordinates(int x, int y) const = 0;
	//! Converts 2D screen coordinates to 'corner-based' 2D OpenGL context coordinates
	virtual QPointF toCornerGLCoordinates(int x, int y) const = 0;

	//! Returns viewport parameters (zoom, etc.)
	virtual const ccViewportParameters& getViewportParameters() const = 0;

	//! Setups a (projective) camera
	/** \param cameraMatrix orientation/position matrix of the camera
		\param fov_deg vertical field of view (in degrees). Optional (ignored if 0).
		\param viewerBasedPerspective whether the perspective view should be object-centered (false) or camera-centered (true)
		\param bubbleViewMode set whether bubble-view mode should be enabled or not (in which case viewerBasedPerspective is forced by default)
	**/
	virtual void setupProjectiveViewport(	const ccGLMatrixd& cameraMatrix,
											float fov_deg = 0.0f,
											bool viewerBasedPerspective = true,
											bool bubbleViewMode = false) = 0;

	//! Warns the display that the enity is about to be removed
	virtual void aboutToBeRemoved(ccDrawableObject* entity) = 0;

	//! Returns this window as a proper Qt widget
	virtual QWidget* asWidget() { return nullptr; }
};

#endif //CC_GENERIC_GL_DISPLAY
