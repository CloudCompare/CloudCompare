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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1595                                                              $
//$LastChangedDate:: 2010-07-02 18:04:17 +0200 (ven., 02 juil. 2010)       $
//**************************************************************************
//

#ifndef CC_FRAME_BUFFER_OBJECT
#define CC_FRAME_BUFFER_OBJECT

#include "ccGlew.h"

const GLenum FBO_COLORS[] = {GL_COLOR_ATTACHMENT0_EXT,
                             GL_COLOR_ATTACHMENT1_EXT,
                             GL_COLOR_ATTACHMENT2_EXT,
                             GL_COLOR_ATTACHMENT3_EXT};

class ccFrameBufferObject
{
    public:
        ccFrameBufferObject();
        ~ccFrameBufferObject();

        bool init(unsigned w, unsigned h);

        void reset();

        void start();
        void stop();

        bool initTexture(unsigned index,
                            GLint internalformat,
                            GLenum format,
                            GLenum type,
                            GLint minMagFilter = GL_LINEAR,
                            GLenum target = GL_TEXTURE_2D);

        bool initTextures(unsigned count,
                            GLint internalformat,
                            GLenum format,
                            GLenum type,
                            GLint minMagFilter = GL_LINEAR,
                            GLenum target = GL_TEXTURE_2D);

        bool initDepth(GLint wrapParam = GL_CLAMP_TO_BORDER,
                        GLenum internalFormat = GL_DEPTH_COMPONENT24,
                        GLint minMagFilter = GL_NEAREST,
                        GLenum textureTarget = GL_TEXTURE_2D);

        //void bindAll();

        void setDrawBuffers(GLsizei n, const GLenum* buffers);	//GLenum buffers[n]	= {GL_COLOR_ATTACHMENT0_EXT,GL_COLOR_ATTACHMENT1_EXT};

        void setDrawBuffers1();
        void setDrawBuffersN(GLsizei n); //n=1..4

        GLuint getID();
        GLuint getColorTexture(unsigned i);
        GLuint getDepthTexture();

    protected:

        unsigned width,height;

        //! Depth texture GL ID
        GLuint depthTexture;

        //! Color textures GL IDs
        GLuint colorTextures[4];

        GLuint fboId;
};

#endif
