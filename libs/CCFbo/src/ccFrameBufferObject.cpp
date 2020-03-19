//##########################################################################
//#                                                                        #
//#                               CCFBO                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccFrameBufferObject.h"

//system
#include <assert.h>

ccFrameBufferObject::ccFrameBufferObject()
	: m_isValid(false)
	, m_width(0)
	, m_height(0)
	, m_depthTexture(0)
	, m_ownDepthTexture(false)
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
	if (!m_isValid)
	{
		return;
	}
	
	deleteDepthTexture();
	deleteColorTexture();

	if (m_fboId != 0)
	{
		m_glExtFunc.glDeleteFramebuffers(1, &m_fboId);
		m_fboId = 0;
	}

	m_width = m_height = 0;
}

bool ccFrameBufferObject::init(unsigned w, unsigned h)
{
	if (!m_isValid)
	{
		if (!m_glFunc.initializeOpenGLFunctions())
		{
			return false;
		}

		if (!m_glExtFunc.initializeOpenGLFunctions())
		{
			return false;
		}
	}
	else
	{
		//to support reinit
		reset();
	}

	m_width = w;
	m_height = h;

	// create a framebuffer object
	m_glExtFunc.glGenFramebuffers(1, &m_fboId);

	m_isValid = true;

	return m_fboId != 0;
}

bool ccFrameBufferObject::start()
{
	if (m_isValid && m_fboId != 0)
	{
		m_glExtFunc.glBindFramebuffer(GL_FRAMEBUFFER_EXT, m_fboId);
		return true;
	}
	else
	{
		return false;
	}
}

void ccFrameBufferObject::stop()
{
	if (m_isValid && m_fboId != 0)
	{
		m_glExtFunc.glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
	}
}

void ccFrameBufferObject::deleteColorTexture()
{
	if (m_isValid && m_ownColorTexture && m_glFunc.glIsTexture(m_colorTexture))
	{
		m_glFunc.glDeleteTextures(1, &m_colorTexture);
	}
	m_colorTexture = 0;
	m_ownColorTexture = false;
}

void ccFrameBufferObject::deleteDepthTexture()
{
	if (m_isValid && m_ownDepthTexture && m_glFunc.glIsTexture(m_depthTexture))
	{
		m_glFunc.glDeleteTextures(1, &m_depthTexture);
	}
	m_depthTexture = 0;
	m_ownDepthTexture = false;
}

bool ccFrameBufferObject::initColor(GLint internalformat/*=GL_RGBA*/,
									GLenum format/*=GL_RGBA*/,
									GLenum type/*=GL_UNSIGNED_BYTE*/,
									GLint minMagFilter/*=GL_NEAREST*/,
									GLenum target/*=GL_TEXTURE_2D*/)
{
	if (!m_isValid || m_fboId == 0)
	{
		assert(false);
		return false;
	}

	//create the new texture
	m_glFunc.glPushAttrib(GL_ENABLE_BIT);
	m_glFunc.glEnable(GL_TEXTURE_2D);

	GLuint texID = 0;
	m_glFunc.glGenTextures(1, &texID);
	m_glFunc.glBindTexture(target, texID);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_MAG_FILTER, minMagFilter);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_MIN_FILTER, minMagFilter);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	m_glFunc.glTexImage2D(target, 0, internalformat, m_width, m_height, 0, format, type, nullptr);
	m_glFunc.glBindTexture(target, 0);

	m_glFunc.glPopAttrib();

	if (attachColor(texID, true, target))
	{
		return true;
	}
	else
	{
		m_glFunc.glDeleteTextures(1, &texID);
		return false;
	}
}

bool ccFrameBufferObject::attachColor(	GLuint texID,
										bool ownTexture/*=false*/,
										GLenum target/*=GL_TEXTURE_2D*/)
{
	if (!m_isValid || m_fboId == 0)
	{
		assert(false);
		return false;
	}

	if (!m_glFunc.glIsTexture(texID))
	{
		//error or simple warning?
		assert(false);
		//return false;
	}

	if (!start())
	{
		return false;
	}

	m_glExtFunc.glFramebufferTexture2D(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, target, texID, 0);
	GLenum status = m_glExtFunc.glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);

	stop();

	bool success = true;
	switch (status)
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		//remove the previous texture (if any)
		deleteColorTexture();
		//save the new one
		m_colorTexture = texID;
		m_ownColorTexture = ownTexture;
		break;
	
	default:
		qDebug("[%s line %d] OpenGL Error: %d", __FILE__, __LINE__, status);
		success = false;
		break;
	}

	return success;
}

bool ccFrameBufferObject::initDepth(GLint wrapParam/*=GL_CLAMP_TO_BORDER*/,
									GLenum internalFormat/*=GL_DEPTH_COMPONENT32*/,
									GLint minMagFilter/*=GL_NEAREST*/,
									GLenum target/*=GL_TEXTURE_2D*/)
{
	if (!m_isValid || m_fboId == 0)
	{
		assert(false);
		return false;
	}

	if (!start())
	{
		return false;
	}

	//create the depth texture
	m_glFunc.glPushAttrib(GL_ENABLE_BIT);
	m_glFunc.glEnable(GL_TEXTURE_2D);

	GLuint texID = 0;
	m_glFunc.glGenTextures(1, &texID);
	m_glFunc.glBindTexture(target, texID);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_WRAP_S, wrapParam);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_WRAP_T, wrapParam);
	m_glFunc.glTexParameteri(target, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_MIN_FILTER, minMagFilter);
	m_glFunc.glTexParameteri(target, GL_TEXTURE_MAG_FILTER, minMagFilter);
	m_glFunc.glTexImage2D   (target, 0, internalFormat, m_width, m_height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, nullptr);
	m_glFunc.glBindTexture  (target, 0);

	m_glFunc.glPopAttrib();

	if (attachDepth(texID, true, target))
	{
		return true;
	}
	else
	{
		m_glFunc.glDeleteTextures(1, &texID);
		return false;
	}
}

bool ccFrameBufferObject::attachDepth(	GLuint texID,
										bool ownTexture/*=false*/,
										GLenum target/*=GL_TEXTURE_2D*/)
{
	if (!m_isValid || m_fboId == 0)
	{
		assert(false);
		return false;
	}

	if (!m_glFunc.glIsTexture(texID))
	{
		//error or simple warning?
		assert(false);
		//return false;
	}

	if (!start())
	{
		return false;
	}

	m_glExtFunc.glFramebufferTexture2D(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, target, texID, 0);
	GLenum status = m_glExtFunc.glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);

	stop();

	bool success = true;
	switch (status)
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		//remove the previous texture (if any)
		deleteDepthTexture();
		//save the new one
		m_depthTexture = texID;
		m_ownDepthTexture = ownTexture;
		break;
	
	default:
		qDebug("[%s line %d] OpenGL Error: %d", __FILE__, __LINE__, status);
		success = false;
		break;
	}

	return success;
}