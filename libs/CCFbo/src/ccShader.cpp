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

#include "ccShader.h"

//Qt
#include <QFile>

#include <assert.h>
#include <string.h>
#include <stdio.h>

ccShader::ccShader(QOpenGLFunctions_3_2_Compatibility* glFunc)
	: m_prog(0)
	, m_glFunc(glFunc)
{}

ccShader::~ccShader()
{
	reset();
}

void ccShader::reset()
{
	if (m_glFunc && m_glFunc->glIsProgram(m_prog))
	{
		m_glFunc->glDeleteProgram(m_prog);
	}
	m_prog = 0;
}

void ccShader::start()
{
	if (m_glFunc)
		m_glFunc->glUseProgram(m_prog);
}

void ccShader::stop()
{
	if (m_glFunc)
		m_glFunc->glUseProgram(0);
}

bool ccShader::fromFile(QString shaderBasePath, QString shaderBaseFilename, QString& error)
{
	if (shaderBasePath.isEmpty() || shaderBaseFilename.isEmpty())
	{
		error = "Missing input argument for ccShader::fromFile";
		return false;
	}

	QString vertFilename = QString("%1/%2.vert").arg(shaderBasePath).arg(shaderBaseFilename);
	QString fragFilename = QString("%1/%2.frag").arg(shaderBasePath).arg(shaderBaseFilename);

	return loadProgram(vertFilename, fragFilename, error);
}
//*/

bool ccShader::loadProgram(QString vertexShaderFile, QString pixelShaderFile, QString& error)
{
	if (!m_glFunc)
	{
		return false;
	}
	reset();
	assert(m_prog == 0);

	//GL ids
	GLuint vs = 0, ps = 0;

	//we load shader files
	if (!vertexShaderFile.isEmpty())
	{
		vs = LoadShader(m_glFunc, GL_VERTEX_SHADER, qPrintable(vertexShaderFile), error);
		if (!vs)
		{
			return false;
		}
	}
	if (!pixelShaderFile.isEmpty())
	{
		ps = LoadShader(m_glFunc, GL_FRAGMENT_SHADER, qPrintable(pixelShaderFile), error);
		if (!ps)
		{
			//release already loaded vertex program
			if (vs)
				m_glFunc->glDeleteShader(vs);
			return false;
		}
	}

	//we create an empty GL program
	m_prog = m_glFunc->glCreateProgram();

	//we attach loaded shaders to it
	if (vs)
		m_glFunc->glAttachShader(m_prog, vs);
	if (ps)
		m_glFunc->glAttachShader(m_prog, ps);

	//we link them alltogether
	m_glFunc->glLinkProgram(m_prog);

	//we check for success
	GLint linkStatus = GL_TRUE;
	m_glFunc->glGetProgramiv(m_prog, GL_LINK_STATUS, &linkStatus);
	if (linkStatus != GL_TRUE)
	{
		GLint infoLogLength = 0;
		m_glFunc->glGetProgramiv(m_prog, GL_INFO_LOG_LENGTH, &infoLogLength);
		GLchar* strInfoLog = new GLchar[infoLogLength + 1];
		if (strInfoLog)
		{
			m_glFunc->glGetProgramInfoLog(m_prog, infoLogLength, 0, strInfoLog);
			error = QString("Failed to compile shader program: %1").arg(strInfoLog);
			delete[] strInfoLog;
			strInfoLog = 0;
		}
		else
		{
			error = "Failed to compile shader program (unable to retrieve OpenGL log)";
		}

		reset();
	}

	// even if program creation was successful, we don't need the shaders anymore
	if (vs)
		m_glFunc->glDeleteShader(vs);
	if (ps)
		m_glFunc->glDeleteShader(ps);

	return true;
}

QByteArray ccShader::ReadShaderFile(QString filename)
{
	//we try to open the ASCII file (containing the "program" source code)
	QFile file(filename);
	if (!file.open(QFile::ReadOnly))
	{
		//failed to open the file
		return QByteArray();
	}

	return file.readAll();
}

GLuint ccShader::LoadShader(QOpenGLFunctions_3_2_Compatibility* glFunc, GLenum type, QString filename, QString& error)
{
	if (!glFunc)
	{
		error = QString("No Qt/OpenGL compatibility functions defined");
		return 0;
	}
	//load shader file
	QByteArray shaderFileContent = ReadShaderFile(filename);
	if (shaderFileContent.isEmpty())
	{
		error = QString("Failed to open shader file '%1'").arg(filename);
		return 0;
	}

	//create shader
	GLuint shader = glFunc->glCreateShader(type);
	if (!glFunc->glIsShader(shader))
	{
		error = QString("Failed to create shader (type = %1").arg(type);
		return 0;
	}

	const GLchar* shaderStr = static_cast<const GLchar*>(shaderFileContent.data());
	glFunc->glShaderSource(shader, 1, &shaderStr, NULL);

	glFunc->glCompileShader(shader);

	//we must check compilation result
	GLint compileStatus = GL_TRUE;
	glFunc->glGetShaderiv(shader, GL_COMPILE_STATUS, &compileStatus);
	if (compileStatus != GL_TRUE)
	{
		GLint infoLogLength = 0;
		glFunc->glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLength);
		GLchar* strInfoLog = new GLchar[infoLogLength + 1];
		if (strInfoLog)
		{
			glFunc->glGetShaderInfoLog(shader, infoLogLength, 0, strInfoLog);
			error = QString("Failed to compile shader (%1): %2").arg(filename).arg(strInfoLog);
			delete[] strInfoLog;
			strInfoLog = 0;
		}
		else
		{
			error = QString("Failed to compile shader (%1): unable to retrieve OpenGL log)").arg(filename);
		}

		glFunc->glDeleteShader(shader);
		shader = 0;
	}

	return shader;
}

void ccShader::setUniform1i(int loc, int value)
{
	if (m_glFunc)
		m_glFunc->glUniform1i(loc, value);
}

void ccShader::setUniform1f(int loc, float value)
{
	if (m_glFunc)
		m_glFunc->glUniform1f(loc, value);
}

void ccShader::setUniform4fv(int loc, float* value)
{
	if (m_glFunc)
		m_glFunc->glUniform4fv(loc, 1, value);
}

void ccShader::setUniform1i(const char* uniform, int value)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform1i(loc, value);
	}
}

void ccShader::setUniform2iv(const char* uniform, int* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform2iv(loc, 1, val);
	}
}


void ccShader::setUniform3iv(const char* uniform, int* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform3iv(loc, 1, val);
	}
}

void ccShader::setUniform4iv(const char* uniform, int* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform4iv(loc, 1, val);
	}
}

void ccShader::setTabUniform4iv(const char* uniform, int size, int* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform4iv(loc, size, val);
	}
}

void ccShader::setUniform1f(const char* uniform, float value)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform1f(loc, value);
	}
}


void ccShader::setUniform2fv(const char* uniform, float* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform2fv(loc, 1, val);
	}
}


void ccShader::setUniform3fv(const char* uniform, float* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform3fv(loc, 1, val);
	}
}


void ccShader::setUniform4fv(const char* uniform, float* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform4fv(loc, 1, val);
	}
}


void ccShader::setTabUniform1fv(const char* uniform, int size, float* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform1fv(loc, size, val);
	}
}


void ccShader::setTabUniform2fv(const char* uniform, int size, float* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform2fv(loc, size, val);
	}
}

void ccShader::setTabUniform3fv(const char* uniform, int size, float* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform3fv(loc, size, val);
	}
}


void ccShader::setTabUniform4fv(const char* uniform, int size, float* val)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniform4fv(loc, size, val);
	}
}


void ccShader::setUniformMatrix4fv(const char* uniform, float* val, bool transpose)
{
	if (m_glFunc)
	{
		int loc = m_glFunc->glGetUniformLocation(m_prog, uniform);
		m_glFunc->glUniformMatrix4fv(loc, 1, transpose, val);
	}
}


void ccShader::setAttrib4iv(int loc, int* val)
{
	if (m_glFunc)
		m_glFunc->glVertexAttrib4iv(loc, val);
}

void ccShader::setAttrib1f(int loc, float value)
{
	if (m_glFunc)
		m_glFunc->glVertexAttrib1f(loc, value);
}

void ccShader::setAttrib2fv(int loc, float* val)
{
	if (m_glFunc)
		m_glFunc->glVertexAttrib2fv(loc, val);
}

void ccShader::setAttrib3fv(int loc, float* val)
{
	if (m_glFunc)
		m_glFunc->glVertexAttrib3fv(loc, val);
}

void ccShader::setAttrib4fv(int loc, float* val)
{
	if (m_glFunc)
		m_glFunc->glVertexAttrib4fv(loc, val);
}
