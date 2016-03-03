//##########################################################################
//#                                                                        #
//#                               CCFBO                                    #
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

#ifndef CC_FRAME_BUFFER_OBJECT
#define CC_FRAME_BUFFER_OBJECT

#include <QOpenGLFunctions_3_0>

//! F.B.O. encapsulation
/** Compared to the QOpenGLFramebufferObject class, this one offers the possibility to:
	- get the attached depth texture ID
	- attach a custom COLOR texture
**/
class ccFrameBufferObject
{
public:
	ccFrameBufferObject();
	~ccFrameBufferObject();

	bool init(unsigned w, unsigned h, QOpenGLFunctions_3_0* glFunc);
	void reset();
	void start();
	void stop();

	inline bool isValid() const { return m_fboId; }

	bool initColor(	GLint internalformat,
					GLenum format,
					GLenum type,
					GLint minMagFilter = GL_LINEAR,
					GLenum target = GL_TEXTURE_2D);

	bool attachColor(	GLuint texID,
						bool ownTexture = false,
						GLenum target = GL_TEXTURE_2D);

	bool initDepth(	GLint wrapParam = GL_CLAMP_TO_BORDER,
					GLenum internalFormat = GL_DEPTH_COMPONENT24,
					GLint minMagFilter = GL_NEAREST,
					GLenum textureTarget = GL_TEXTURE_2D);

	GLuint getID();
	inline GLuint getColorTexture() const { return m_colorTexture; }
	inline GLuint getDepthTexture() const { return m_depthTexture; }

	//! Returns width
	inline unsigned width() const { return m_width; }
	//! Returns height
	inline unsigned height() const { return m_height; }

protected:

	//! Deletes/releases the color texture
	void deleteColorTexture();

	//! Width
	unsigned m_width;
	//! Height
	unsigned m_height;

	//! Depth texture GL ID
	GLuint m_depthTexture;

	//! Color texture GL ID
	GLuint m_colorTexture;

	//! Whether the color texture is owned by this FBO or not
	bool m_ownColorTexture;

	//! ID
	GLuint m_fboId;

	//! Associated OpenGL functions
	QOpenGLFunctions_3_0* m_glFunc;
};

#endif
