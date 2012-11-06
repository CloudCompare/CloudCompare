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
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef CC_GENERIC_GL_DISPLAY
#define CC_GENERIC_GL_DISPLAY

#include "ccSerializableObject.h"
#include "ccGLMatrix.h"

//Qt
#include <QImage>
#include <QString>
#include <QFont>

//CCLib
#include <CCGeom.h>

//! Standard parameters for GL displays/viewports
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccViewportParameters : public ccSerializableObject
#else
class ccViewportParameters : public ccSerializableObject
#endif
{
public:
	//! Default constructor
	ccViewportParameters();

	//! Copy constructor
	ccViewportParameters(const ccViewportParameters& params);

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }
	virtual bool toFile(QFile& out) const;
	virtual bool fromFile(QFile& in, short dataVersion);

    //! Global zoom '1:1'(ortho. mode only)
	/** Equals the scaling value that make the diagonal of the
		total bounding-box (= all displayed entities) fit
		inside the biggest screen dimension, i.e. max(w,h).
	**/
	float globalZoom;

	//! Current zoom
    float zoom;

	//! Base visualization matrix (rotation only - GL style)
	ccGLMatrix baseViewMat;

	//! Point size
	float defaultPointSize;
	//! Line width
	float defaultLineWidth;

	//! Perspective view state
	bool perspectiveView;
	//! Whether perspective view is centered on displayed scene (true) or on the user (false)
	bool objectCenteredPerspective;
	//! Rotation pivot point (ortho. mode only) or camera center (perspective mode)
	CCVector3 pivotPoint;
	//! Camera F.O.V. (field of view - for perspective mode only)
	float fov;
	//! Camera aspect ratio (for perspective mode only)
	float aspectRatio;
    //! Screen panning (post-zoom shifting)
	float screenPan[2];
};

//! Generic interface for GL displays
class ccGenericGLDisplay
{
public:

    //! Redraws display immediately
	virtual void redraw() = 0;

	//! Flags display as 'to be refreshed'
	/** See ccGenericGLDisplay::refresh.
	**/
    virtual void toBeRefreshed() = 0;

    //! Redraws display only if flagged as 'to be refreshed'
    /** See ccGenericGLDisplay::toBeRefreshed. Flag is turned
        to false after a call to this method.
    **/
    virtual void refresh() = 0;

    //! Invalidates current viewport setup
    /** On next redraw, viewport information will be recomputed.
    **/
    virtual void invalidateViewport() = 0;

    //! Get texture ID from image
    virtual unsigned getTexture(const QImage& image) = 0;

    //! Release texture from context
    virtual void releaseTexture(unsigned texID) = 0;

	//! Returns font
	virtual const QFont& getTextDisplayFont() = 0;

    //! Displays a string at a given 2D position
    /** This method should be called solely during 2D pass rendering.
		The coordinates are expressed relatively to the current viewport (y=0 at the top!).
		\param text string
        \param x horizontal position of string origin
        \param y vertical position of string origin
		\param alignRight whether to align text to the right or not
		\param rgbColor text color (optional)
        \param font font (optional)
	**/
    virtual void displayText(QString text, int x, int y, bool alignRight=false, const unsigned char* rgbColor=0, const QFont& font=QFont()) = 0;

	//! Displays a string at a given 3D position
    /** This method should be called solely during 3D pass rendering
        (see paintGL).
        \param str string
        \param pos3D 3D position of string origin
        \param rgbColor color (optional: if let to 0, default text rendering color is used)
        \param font font (optional)
    **/
	virtual void display3DLabel(const QString& str, const CCVector3& pos3D, const unsigned char* rgbColor=0, const QFont& font=QFont()) = 0;

	//! Returns whether a given version of OpenGL is supported
	/** \param openGLVersionFlag see QGLFormat::OpenGLVersionFlag
	**/
	virtual bool supportOpenGLVersion(unsigned openGLVersionFlag) = 0;

	//! Returns current model view matrix (GL_MODELVIEW)
	virtual const double* getModelViewMatd() = 0;

	//! Returns current projection matrix (GL_PROJECTION)
    virtual const double* getProjectionMatd() = 0;

	//! Returns current viewport (GL_VIEWPORT)
    virtual void getViewportArray(int vp[/*4*/]) = 0;

	//! Returns viewport parameters (zoom, etc.)
	virtual const ccViewportParameters& getViewportParameters() const = 0;
};

#endif
