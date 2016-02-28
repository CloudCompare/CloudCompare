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

//system
#include <assert.h>

ccFrameBufferObject::ccFrameBufferObject()
	: m_width(0)
	, m_height(0)
	, m_depthTexture(0)
	, m_colorTexture(0)
	, m_ownColorTexture(false)
	, m_fboId(0)
{}

ccFrameBufferObject::~ccFrameBufferObject()
{
	reset();
}

void ccFrameBufferObject::reset()
{
	if (m_depthTexture != 0)
	{
		glDeleteTextures(1, &m_depthTexture);
		m_depthTexture = 0;
	}

	deleteColorTexture();

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
	{
		return false;
	}

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
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fboId);
}

void ccFrameBufferObject::stop()
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

GLuint ccFrameBufferObject::getID()
{
	return m_fboId;
}

void ccFrameBufferObject::deleteColorTexture()
{
	if (m_colorTexture && glIsTexture(m_colorTexture))
	{
		if (m_ownColorTexture)
		{
			glDeleteTextures(1, &m_colorTexture);
		}
	}
	m_colorTexture = 0;
	m_ownColorTexture = false;
}

bool ccFrameBufferObject::attachColor(	GLuint texID,
										bool ownTexture/*=false*/,
										GLenum target/*=GL_TEXTURE_2D*/)
{
	if (m_fboId == 0)
	{
		assert(false);
		return false;
	}

	if (!glIsTexture(texID))
	{
		//error or simple warning?
		assert(false);
		//return false;
	}

	//remove the previous texture (if any)
	deleteColorTexture();

	m_colorTexture = texID;
	m_ownColorTexture = ownTexture;

	start();

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, target, texID, 0);

	bool success = false;
	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	switch (status)
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		success = true;
		break;
	default:
		//ccLog::Warning("[FBO] Color texture %i init. error: %i", index+1, status);
		break;
	}

	stop();

	return success;
}

bool ccFrameBufferObject::initColor(	GLint internalformat,
										GLenum format,
										GLenum type,
										GLint minMagFilter /*= GL_LINEAR*/,
										GLenum target /*= GL_TEXTURE_2D*/)
{
	if (m_fboId == 0)
	{
		//ccLog::Warning("[FBO::initTexture] Internal error: FBO not yet initialized!");
		return false;
	}

	//even if 'attachTexture' can do this, we prefer to do it now
	//so as to release memory before creating a new texture!
	deleteColorTexture();

	//create the new texture
	GLuint texID = 0;
	glGenTextures(1, &texID);

	glBindTexture  (target, texID);
	glTexParameteri(target, GL_TEXTURE_MAG_FILTER, minMagFilter );
	glTexParameteri(target, GL_TEXTURE_MIN_FILTER, minMagFilter );
	glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D   (target, 0, internalformat, m_width, m_height, 0, format, type, 0);
	glBindTexture  (target, 0);

	return attachColor(texID, true, target);
}

bool ccFrameBufferObject::initDepth(GLint wrapParam /*=GL_CLAMP_TO_BORDER*/,
									GLenum internalFormat /*=GL_DEPTH_COMPONENT24*/,
									GLint minMagFilter /*= GL_NEAREST*/,
									GLenum target/*=GL_TEXTURE_2D*/)
{
	if (m_fboId == 0)
	{
		//ccLog::Warning("[FBO::initDepth] Internal error: FBO not yet initialized!");
		return false;
	}

	start();

	if (glIsTexture(m_depthTexture))
	{
		glDeleteTextures(1, &m_depthTexture);
	}
	glGenTextures(1, &m_depthTexture);
	glBindTexture(target, m_depthTexture);

	//float border[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	//glTexParameterfv(target, GL_TEXTURE_BORDER_COLOR, border);
	glTexParameteri(target, GL_TEXTURE_WRAP_S, wrapParam);
	glTexParameteri(target, GL_TEXTURE_WRAP_T, wrapParam);
	glTexParameteri(target, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
	glTexParameteri(target, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	glTexParameteri(target, GL_TEXTURE_MIN_FILTER, minMagFilter);
	glTexParameteri(target, GL_TEXTURE_MAG_FILTER, minMagFilter);
	glTexImage2D(target, 0, internalFormat, m_width, m_height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, target, m_depthTexture, 0);

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
