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

#ifndef CC_SHADER_HEADER
#define CC_SHADER_HEADER

#include "ccGlew.h"

//Qt
#include <QString>
#include <QByteArray>

//! Generic shader class
class ccShader
{
public:

	//! Default constructor
	ccShader();

	//! Destructor
	virtual ~ccShader();

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

	virtual void reset();

	virtual void start();
	virtual void stop();

	//! Returns program GL ID
	inline GLuint getProgram() const { return m_prog; }

	/** UNIFORMS **/

	virtual void setUniform1i(int loc, int value);
	virtual void setUniform1f(int loc, float value);
	virtual void setUniform4fv(int loc, float* value);

	virtual void setUniform1i(const char* variable, int val);
	virtual void setUniform2iv(const char* variable, int* p_val);
	virtual void setUniform3iv(const char* variable, int* p_val);
	virtual void setUniform4iv(const char* variable, int* p_val);
	virtual void setTabUniform4iv(const char* uniform, int size, int* p_val);

	virtual void setUniform1f(const char* variable, float val);
	virtual void setUniform2fv(const char* variable, float* p_val);
	virtual void setUniform3fv(const char* variable, float* p_val);
	virtual void setUniform4fv(const char* variable, float* p_val);
	virtual void setTabUniform1fv(const char* uniform, int size, float* p_val);
	virtual void setTabUniform2fv(const char* uniform, int size, float* p_val);
	virtual void setTabUniform3fv(const char* uniform, int size, float* p_val);
	virtual void setTabUniform4fv(const char* uniform, int size, float* p_val);
	virtual void setUniformMatrix4fv(const char* variable, float* p_val, bool transpose = false);

	/** ATTRIBUTES **/

	virtual void setAttrib4iv(int loc, int* p_val);
	virtual void setAttrib1f(int loc, float val);
	virtual void setAttrib2fv(int loc, float* p_val);
	virtual void setAttrib3fv(int loc, float* p_val);
	virtual void setAttrib4fv(int loc, float* p_val);

protected:

	//! Loads a shader from a file
	static GLuint LoadShader(GLenum type, QString filename, QString& error);

	//! Bufferizes a shader file in memory
	static QByteArray ReadShaderFile(QString filename);

	//! Program ID
	GLuint m_prog;
};

#endif
