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

#include "ccGlew.h"

const GLenum FBO_COLORS[] = {	GL_COLOR_ATTACHMENT0_EXT,
								GL_COLOR_ATTACHMENT1_EXT,
								GL_COLOR_ATTACHMENT2_EXT,
								GL_COLOR_ATTACHMENT3_EXT};

//! F.B.O. encapsulation
class ccFrameBufferObject
{
public:
	ccFrameBufferObject();
	~ccFrameBufferObject();

	bool init(unsigned w, unsigned h);
	void reset();
	void start();
	void stop();

	bool initTexture(	unsigned index,
						GLint internalformat,
						GLenum format,
						GLenum type,
						GLint minMagFilter = GL_LINEAR,
						GLenum target = GL_TEXTURE_2D);

	bool initTextures(	unsigned count,
						GLint internalformat,
						GLenum format,
						GLenum type,
						GLint minMagFilter = GL_LINEAR,
						GLenum target = GL_TEXTURE_2D);

	bool initDepth(	GLint wrapParam = GL_CLAMP_TO_BORDER,
					GLenum internalFormat = GL_DEPTH_COMPONENT24,
					GLint minMagFilter = GL_NEAREST,
					GLenum textureTarget = GL_TEXTURE_2D);

	void setDrawBuffers(GLsizei n, const GLenum* buffers);	//GLenum buffers[n]	= {GL_COLOR_ATTACHMENT0_EXT,GL_COLOR_ATTACHMENT1_EXT};
	void setDrawBuffers1();
	void setDrawBuffersN(GLsizei n); //n=1..4

	GLuint getID();
	GLuint getColorTexture(unsigned i);
	GLuint getDepthTexture();

	//! Returns width
	inline unsigned width() const { return m_width; }
	//! Returns height
	inline unsigned height() const { return m_height; }

protected:

	//! Width
	unsigned m_width;
	//! Height
	unsigned m_height;

	//! Depth texture GL ID
	GLuint m_depthTexture;

	//! Color textures GL IDs
	GLuint m_colorTextures[4];

	//! ID
	GLuint m_fboId;
};

#endif
