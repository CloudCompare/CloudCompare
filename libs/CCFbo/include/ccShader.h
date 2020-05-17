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

//! Shader program
/** Now a simple encapsulation of QOpenGLShaderProgram providing two helper functions.
**/
class CCFBO_LIB_API ccShader : public QOpenGLShaderProgram
{
	Q_OBJECT
	
public:

	//! Default constructor
	ccShader(QObject* parent = 0);

	//! Destructor
	virtual ~ccShader() = default;

	//! Creates program from two shader files with same base filename
	/** Path and extensions (.vert and .frag) are automatically
		added to shader base filename (shortcut to ccShader::loadProgram).
		\param shaderBasePath shader files path
		\param shaderBaseFilename shader base filename
		\param error error string (if any error occurred)
		\return success
	**/
	virtual bool fromFile(QString shaderBasePath, QString shaderBaseFilename, QString& error);

	//! Creates program from one or two shader files
	/** Filenames must be absolute (full path).
		\param vertShaderFile vertex shader filename
		\param fragShaderFile fragment shader filename
		\param error error string (if any error occurred)
	**/
	virtual bool loadProgram(QString vertShaderFile, QString fragShaderFile, QString& error);
};
