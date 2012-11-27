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
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef CC_IMAGE_HEADER
#define CC_IMAGE_HEADER

#include "ccHObject.h"

//Qt
#include <QImage>
#include <QString>

class ccGenericGLDisplay;

//! Generic image
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccImage : public ccHObject
#else
class ccImage : public ccHObject
#endif
{
public:

	//! Default constructor
	ccImage();

	//! Constructor from QImage
	ccImage(const QImage& image, const QString& name = QString("unknown"));

    //! Returns unique class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_IMAGE;};

    //! Loads image from file
	/** \param filename image filename
		\param error a human readable description of what went wrong (if method fails)
		\return success
	**/
	bool load(const QString& filename, QString& error);

	//! Sets image
	/** \param image image
		\return success
	**/
	virtual void setImage(const QImage& image);

	//! Returns image width
	inline unsigned getW() const {return m_width;}

	//! Returns image height
	inline unsigned getH() const {return m_height;}

	//! Sets image texture transparency
	virtual void setAlpha(float value);

	//! Returns image texture transparency
	virtual inline float getAlpha() const {return m_texAlpha;}

#ifdef INCLUDE_IMAGE_FILENAME
	//! Returns complete filename
	QString getCompleteFileName() const {return m_completeFileName;}

	//! Sets complete filename
	void setCompleteFileName(const QString& name) {m_completeFileName = name;}
#endif

	//! Returns coresponding QImage
	virtual const QImage& data() const {return m_image;}

	//! Manually sets aspect ratio
	void setAspectRatio(float ar) { m_aspectRatio = ar; }

	//! Returns aspect ratio
	float getAspectRatio() const { return m_aspectRatio; }

protected:

    //inherited from ccHObject
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! Unbinds texture from currently associated GL context
    virtual bool unbindTexture();

	//! Binds texture to a GL context (and creates texture if necessary)
	/** \param win 3D display to which to bind the texture
		\param pow2Texture whether a texture with power of 2 dimensions is requested (OpenGL version < 2.0)
		\return success
	**/
	virtual bool bindToGlTexture(ccGenericGLDisplay* win, bool pow2Texture = false);

	//! Image width (in pixels)
	unsigned m_width;
	//! Image height (in pixels)
	unsigned m_height;

	//! Aspect ratio w/h
	/** Default is m_width/m_height.
		Should be changed if pixels are not square.
	**/
	float m_aspectRatio;

	//! Texture coordinate (width) of bottom-right pixel
	float m_texU;
	//! Texture coordinate (height) of bottom-right pixel
	float m_texV;

	//! Texture transparency
	float m_texAlpha;

	//! Texture GL ID
	unsigned m_textureID;

	//! Currently bound GL window
	ccGenericGLDisplay* m_boundWin;

	//! Image data
	QImage m_image;

#ifdef INCLUDE_IMAGE_FILENAME
	//! Complete filename
	QString m_completeFileName;
#endif
};

#endif
