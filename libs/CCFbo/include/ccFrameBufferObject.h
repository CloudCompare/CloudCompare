#pragma once
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

#include "CCFbo.h"

//Qt
#include <QOpenGLExtensions>
#include <QOpenGLFunctions_2_1>

/**
 * \brief Enhanced Frame Buffer Object (FBO) wrapper for OpenGL rendering
 * 
 * \details Provides an advanced encapsulation of OpenGL Frame Buffer Objects 
 * with extended functionality beyond Qt's standard implementation.
 * 
 * Key features:
 * - Flexible color and depth texture management
 * - Custom texture attachment support
 * - Simplified FBO initialization and manipulation
 * - Compatibility with OpenGL 2.1 and extensions
 * 
 * Advantages over QOpenGLFramebufferObject:
 * - Direct access to depth texture ID
 * - Ability to attach custom color textures
 * - More granular control over texture parameters
 * 
 * \note Designed for portability with OpenGL 2.1 and ARB framebuffer extensions
 * 
 * \example
 * \code
 * // Create a Frame Buffer Object
 * ccFrameBufferObject fbo;
 * 
 * // Initialize FBO with specific dimensions
 * if (!fbo.init(800, 600))
 * {
 *     qDebug() << "FBO initialization failed";
 *     return;
 * }
 * 
 * // Initialize color texture with custom parameters
 * fbo.initColor(GL_RGBA, 
 *               GL_RGBA, 
 *               GL_UNSIGNED_BYTE, 
 *               GL_LINEAR);
 * 
 * // Initialize depth texture
 * fbo.initDepth(GL_CLAMP_TO_BORDER, 
 *               GL_DEPTH_COMPONENT32F, 
 *               GL_NEAREST);
 * 
 * // Start rendering to the FBO
 * if (fbo.start())
 * {
 *     // Perform OpenGL rendering operations
 *     // ...
 * 
 *     // Stop rendering
 *     fbo.stop();
 * }
 * 
 * // Access FBO textures
 * GLuint colorTexture = fbo.getColorTexture();
 * GLuint depthTexture = fbo.getDepthTexture();
 * \endcode
 */
class CCFBO_LIB_API ccFrameBufferObject
{
public:
	/**
	 * \brief Default constructor
	 * 
	 * Creates an uninitialized Frame Buffer Object
	 */
	ccFrameBufferObject();

	/**
	 * \brief Destructor
	 * 
	 * Cleans up OpenGL resources associated with the FBO
	 */
	~ccFrameBufferObject();

	/**
	 * \brief Initialize the Frame Buffer Object with specified dimensions
	 * 
	 * \param[in] w Width of the Frame Buffer Object
	 * \param[in] h Height of the Frame Buffer Object
	 * 
	 * \return True if initialization was successful, false otherwise
	 */
	bool init(unsigned w, unsigned h);

	/**
	 * \brief Reset the Frame Buffer Object to its initial state
	 * 
	 * Releases all associated OpenGL resources
	 */
	void reset();

	/**
	 * \brief Begin rendering to the Frame Buffer Object
	 * 
	 * Prepares the FBO for rendering operations
	 * 
	 * \return True if successfully started, false otherwise
	 */
	bool start();

	/**
	 * \brief Stop rendering to the Frame Buffer Object
	 * 
	 * Finalizes rendering and restores the previous rendering state
	 */
	void stop();

	/**
	 * \brief Check if the Frame Buffer Object is valid
	 * 
	 * \return True if the FBO has been successfully initialized, false otherwise
	 */
	inline bool isValid() const { return m_fboId; }

	/**
	 * \brief Initialize a color texture for the FBO
	 * 
	 * \param[in] internalformat Internal storage format of the texture
	 * \param[in] format Format of the pixel data
	 * \param[in] type Data type of the pixel data
	 * \param[in] minMagFilter Texture minification/magnification filter
	 * \param[in] target Texture target (default: GL_TEXTURE_2D)
	 * 
	 * \return True if color texture initialization was successful
	 */
	bool initColor(	GLint internalformat = GL_RGBA,
					GLenum format = GL_RGBA,
					GLenum type = GL_UNSIGNED_BYTE,
					GLint minMagFilter = GL_NEAREST,
					GLenum target = GL_TEXTURE_2D);

	/**
	 * \brief Attach an existing color texture to the FBO
	 * 
	 * \param[in] texID OpenGL texture ID to attach
	 * \param[in] ownTexture If true, FBO will manage the texture's lifecycle
	 * \param[in] target Texture target (default: GL_TEXTURE_2D)
	 * 
	 * \return True if color texture attachment was successful
	 */
	bool attachColor(	GLuint texID,
						bool ownTexture = false,
						GLenum target = GL_TEXTURE_2D);

	/**
	 * \brief Initialize a depth texture for the FBO
	 * 
	 * \param[in] wrapParam Texture wrapping parameter
	 * \param[in] internalFormat Internal storage format of the depth texture
	 * \param[in] minMagFilter Texture minification/magnification filter
	 * \param[in] textureTarget Texture target (default: GL_TEXTURE_2D)
	 * 
	 * \return True if depth texture initialization was successful
	 */
	bool initDepth(	GLint wrapParam = GL_CLAMP_TO_BORDER,
					GLenum internalFormat = GL_DEPTH_COMPONENT32F,
					GLint minMagFilter = GL_NEAREST,
					GLenum textureTarget = GL_TEXTURE_2D);

	/**
	 * \brief Attach an existing depth texture to the FBO
	 * 
	 * \param[in] texID OpenGL texture ID to attach
	 * \param[in] ownTexture If true, FBO will manage the texture's lifecycle
	 * \param[in] target Texture target (default: GL_TEXTURE_2D)
	 * 
	 * \return True if depth texture attachment was successful
	 */
	bool attachDepth(	GLuint texID,
						bool ownTexture = false,
						GLenum target = GL_TEXTURE_2D);

	/**
	 * \brief Get the OpenGL Frame Buffer Object ID
	 * 
	 * \return OpenGL FBO identifier
	 */
	inline GLuint getID() const { return m_fboId;  }

	/**
	 * \brief Get the color texture ID
	 * 
	 * \return OpenGL color texture identifier
	 */
	inline GLuint getColorTexture() const { return m_colorTexture; }

	/**
	 * \brief Get the depth texture ID
	 * 
	 * \return OpenGL depth texture identifier
	 */
	inline GLuint getDepthTexture() const { return m_depthTexture; }

	/**
	 * \brief Get the width of the Frame Buffer Object
	 * 
	 * \return Width of the FBO in pixels
	 */
	inline unsigned width() const { return m_width; }

	/**
	 * \brief Get the height of the Frame Buffer Object
	 * 
	 * \return Height of the FBO in pixels
	 */
	inline unsigned height() const { return m_height; }

protected: //methods
	/**
	 * \brief Delete and release the color texture
	 * 
	 * Frees OpenGL resources associated with the color texture
	 */
	void deleteColorTexture();

	/**
	 * \brief Delete and release the depth texture
	 * 
	 * Frees OpenGL resources associated with the depth texture
	 */
	void deleteDepthTexture();

protected: //members
	/**
	 * \brief Flag indicating FBO validity
	 */
	bool m_isValid;

	/**
	 * \brief Width of the Frame Buffer Object
	 */
	unsigned m_width;

	/**
	 * \brief Height of the Frame Buffer Object
	 */
	unsigned m_height;

	/**
	 * \brief OpenGL depth texture identifier
	 */
	GLuint m_depthTexture;

	/**
	 * \brief Flag indicating ownership of depth texture
	 */
	bool m_ownDepthTexture;

	/**
	 * \brief OpenGL color texture identifier
	 */
	GLuint m_colorTexture;

	/**
	 * \brief Flag indicating ownership of color texture
	 */
	bool m_ownColorTexture;

	/**
	 * \brief OpenGL Frame Buffer Object identifier
	 */
	GLuint m_fboId;

	/**
	 * \brief OpenGL 2.1 function set for portability
	 */
	QOpenGLFunctions_2_1 m_glFunc;

	/**
	 * \brief OpenGL ARB framebuffer extension functions
	 */
	QOpenGLExtension_ARB_framebuffer_object	m_glExtFunc;
};
