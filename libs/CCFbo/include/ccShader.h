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
#include <QString>
#include <QOpenGLShaderProgram>

class QObject;

/**
 * \brief Wrapper for QOpenGLShaderProgram with enhanced shader loading capabilities
 * 
 * \details Provides a simplified interface for loading and managing OpenGL shader programs
 * 
 * Key features:
 * - Extends QOpenGLShaderProgram with additional helper methods
 * - Supports loading shaders from files with automatic path and extension handling
 * - Provides error reporting for shader compilation and linking
 * 
 * Typical use cases:
 * - Loading vertex and fragment shaders
 * - Creating custom rendering effects
 * - Implementing advanced rendering techniques
 * 
 * \note Inherits from QOpenGLShaderProgram and adds convenience methods
 * 
 * \example
 * \code
 * // Creating and loading a shader program
 * ccShader* shader = new ccShader();
 * QString errorMsg;
 * 
 * // Load shader from files with automatic extension handling
 * if (shader->fromFile("/path/to/shaders/", "myShader", errorMsg))
 * {
 *     // Shader loaded successfully
 *     shader->bind(); // Activate the shader
 *     
 *     // Set shader uniforms
 *     shader->setUniformValue("u_resolution", QVector2D(width, height));
 *     shader->setUniformValue("u_time", elapsedTime);
 *     
 *     // Render using the shader
 *     // ...
 *     
 *     shader->release(); // Deactivate the shader
 * }
 * else
 * {
 *     // Handle shader loading error
 *     qDebug() << "Shader loading failed:" << errorMsg;
 * }
 * 
 * // Alternatively, load shaders directly with full paths
 * shader->loadProgram("/path/to/vertex.vert", "/path/to/fragment.frag", errorMsg);
 * \endcode
 */
class CCFBO_LIB_API ccShader : public QOpenGLShaderProgram
{
	Q_OBJECT
	
public:
	/**
	 * \brief Default constructor
	 * 
	 * \param[in] parent Optional parent QObject for memory management
	 */
	ccShader(QObject* parent = 0);

	/**
	 * \brief Virtual destructor
	 * 
	 * Ensures proper cleanup of shader resources
	 */
	virtual ~ccShader() = default;

	/**
	 * \brief Create shader program from two shader files with same base filename
	 * 
	 * \details Automatically adds .vert and .frag extensions to shader base filename
	 * 
	 * \param[in] shaderBasePath Path to shader files
	 * \param[in] shaderBaseFilename Base filename for vertex and fragment shaders
	 * \param[out] error String to store any loading errors
	 * 
	 * \return True if shader program loaded successfully, false otherwise
	 * 
	 * \note Shortcut to ccShader::loadProgram
	 * 
	 * \sa loadProgram
	 */
	virtual bool fromFile(QString shaderBasePath, QString shaderBaseFilename, QString& error);

	/**
	 * \brief Create shader program from one or two shader files
	 * 
	 * \details Loads vertex and fragment shaders from specified absolute file paths
	 * 
	 * \param[in] vertShaderFile Full path to vertex shader file
	 * \param[in] fragShaderFile Full path to fragment shader file
	 * \param[out] error String to store any loading errors
	 * 
	 * \return True if shader program loaded and linked successfully, false otherwise
	 * 
	 * \note Filenames must be absolute (full path)
	 */
	virtual bool loadProgram(QString vertShaderFile, QString fragShaderFile, QString& error);
};
