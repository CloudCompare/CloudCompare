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

#include "ccFrameBufferObject.h"
#include "ccShader.h"

#include "ccFBOUtils.h"

#include <assert.h>

ccFrameBufferObject::ccFrameBufferObject()
	: m_width(0)
	, m_height(0)
	, m_depthTexture(0)
	, m_fboId(0)
{
	//color textures
	m_colorTextures[0] = 0;
	m_colorTextures[1] = 0;
	m_colorTextures[2] = 0;
	m_colorTextures[3] = 0;
}

ccFrameBufferObject::~ccFrameBufferObject()
{
	reset();
}

void ccFrameBufferObject::reset()
{
	if (m_depthTexture != 0)
	{
		glDeleteTextures(1,&m_depthTexture);
		m_depthTexture = 0;
	}

	for (int i=0; i<4; ++i)
	{
		if (m_colorTextures[i])
		{
			glDeleteTextures(1,m_colorTextures+i);
			m_colorTextures[i] = 0;
		}
	}

	if (m_fboId != 0)
	{
		glDeleteFramebuffersEXT(1, &m_fboId);
		m_fboId = 0;
	}

	m_width = m_height = 0;
}

bool ccFrameBufferObject::init(unsigned w, unsigned h)
{
	//we check if FBO extension is supported by video card
	if (!ccFBOUtils::CheckExtension("GL_EXT_framebuffer_object"))
		return false;

	//to support reinit
	reset();

	m_width = w;
	m_height = h;

	// create a framebuffer object
	glGenFramebuffersEXT(1, &m_fboId);

	return true;
}

void ccFrameBufferObject::start()
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,m_fboId);
}

void ccFrameBufferObject::stop()
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
}

GLuint ccFrameBufferObject::getID()
{
	return m_fboId;
}

GLuint ccFrameBufferObject::getColorTexture(unsigned i)
{
	assert(i<4);
	return m_colorTextures[i];
}

GLuint ccFrameBufferObject::getDepthTexture()
{
	return m_depthTexture;
}

bool ccFrameBufferObject::initTexture(	unsigned index,
										GLint internalformat,
										GLenum format,
										GLenum type,
										GLint minMagFilter /*= GL_LINEAR*/,
										GLenum target /*= GL_TEXTURE_2D*/)
{
	if (index >= 4)
	{
		//wrong index
		return false;
	}

	if (m_fboId == 0)
	{
		//ccLog::Warning("[FBO::initTexture] Internal error: FBO not yet initialized!");
		return false;
	}

	start();

	if (glIsTexture(m_colorTextures[index]))
		glDeleteTextures(1,m_colorTextures+index);

	glGenTextures(1,m_colorTextures+index);
	glBindTexture(target,m_colorTextures[index]);

	/*INITIAL VERSION
	glTexImage2D(target,0,GL_RGBA,width,height,0,GL_BGRA,GL_UNSIGNED_INT_8_8_8_8_REV,NULL);
	glTexParameteri(target,GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glBindTexture(target, 0);
	//*/
	glTexParameteri(target, GL_TEXTURE_MAG_FILTER, minMagFilter );
	glTexParameteri(target, GL_TEXTURE_MIN_FILTER, minMagFilter );
	glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(target,0,internalformat,m_width,m_height,0,format,type,0);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,GL_COLOR_ATTACHMENT0_EXT,target,m_colorTextures[index],0);
	//glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,m_colorTextures[index],0);

	glBindTexture(target, 0);

	bool success = false;
	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	switch (status)
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		success = true;
		break;
	default:
		//ccLog::Warning("[FBO] Color texture %i init. error: %i",index+1,status);
		break;
	}

	stop();

	return success;
}

bool ccFrameBufferObject::initTextures(	unsigned count,
										GLint internalformat,
										GLenum format,
										GLenum type,
										GLint minMagFilter /*= GL_LINEAR*/,
										GLenum target /*= GL_TEXTURE_2D*/)
{
	assert(count < 5);

	if (m_fboId == 0)
	{
		//ccLog::Warning("[FBO::initTextures] Internal error: FBO not yet initialized!");
		return false;
	}

	for(unsigned i=0; i<count; ++i)
		initTexture(i,internalformat,format,type,minMagFilter,target);

	return true;
}

bool ccFrameBufferObject::initDepth(GLint wrapParam /*=GL_CLAMP_TO_BORDER*/,
									GLenum internalFormat /*=GL_DEPTH_COMPONENT24*/,
									GLint minMagFilter /*= GL_NEAREST*/,
									GLenum target/*=GL_TEXTURE_2D*/)
{
	if (m_fboId==0)
	{
		//ccLog::Warning("[FBO::initDepth] Internal error: FBO not yet initialized!");
		return false;
	}

	start();

	if (glIsTexture(m_depthTexture))
		glDeleteTextures(1,&m_depthTexture);

	glGenTextures(1,&m_depthTexture);
	glBindTexture(target,m_depthTexture);

	/* INITIAL VERSION
	glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT24,width,height,0,GL_DEPTH_COMPONENT,GL_UNSIGNED_INT,NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);
	//*/
	float border[] = { 1.0f,1.0f,1.0f,1.0f };
	glTexParameteri(target, GL_TEXTURE_WRAP_S, wrapParam);
	glTexParameteri(target, GL_TEXTURE_WRAP_T, wrapParam);
	glTexParameterfv(target, GL_TEXTURE_BORDER_COLOR, border);
	glTexParameteri(target, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
	glTexParameteri(target, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	glTexParameteri(target, GL_TEXTURE_MIN_FILTER, minMagFilter);
	glTexParameteri(target, GL_TEXTURE_MAG_FILTER, minMagFilter);
	glTexImage2D(target,0,internalFormat,m_width,m_height,0,GL_DEPTH_COMPONENT,GL_UNSIGNED_BYTE,0);

	//glFramebufferTextureEXT(GL_FRAMEBUFFER_EXT,GL_DEPTH_ATTACHMENT_EXT,m_depthTexture,0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,GL_DEPTH_ATTACHMENT_EXT,target,m_depthTexture,0);

	glBindTexture(target, 0);

	bool success = false;
	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	switch (status)
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		//ccLog::Print("[FBO] Depth init. ok");
		success = true;
		break;
	default:
		//ccLog::Warning("[FBO] Depth texture init. error: %i",status);
		break;
	}

	stop();

	return success;
}

void ccFrameBufferObject::setDrawBuffers(GLsizei n, const GLenum* buffers)
{
	assert(n>0 && n<5);
	glDrawBuffers(n,buffers);
	glReadBuffer(GL_NONE);
}

void ccFrameBufferObject::setDrawBuffers1()
{
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
}

void ccFrameBufferObject::setDrawBuffersN(GLsizei n)
{
	setDrawBuffers(n,FBO_COLORS);
}
