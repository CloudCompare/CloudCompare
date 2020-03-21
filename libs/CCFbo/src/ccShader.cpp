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

#include "ccShader.h"

//Qt
#include <QFile>

//system
#include <assert.h>
#include <string.h>
#include <stdio.h>

ccShader::ccShader(QObject* parent/*=0*/)
	: QOpenGLShaderProgram(parent)
{}

bool ccShader::fromFile(QString shaderBasePath, QString shaderBaseFilename, QString& error)
{
	if (shaderBasePath.isEmpty() || shaderBaseFilename.isEmpty())
	{
		error = "Missing input argument for ccShader::fromFile";
		return false;
	}

	QString vertFilename = QString("%1/%2.vert").arg(shaderBasePath, shaderBaseFilename);
	QString fragFilename = QString("%1/%2.frag").arg(shaderBasePath, shaderBaseFilename);

	return loadProgram(vertFilename, fragFilename, error);
}

bool ccShader::loadProgram(QString vertexShaderFile, QString fragShaderFile, QString& error)
{
	if (!vertexShaderFile.isEmpty() && !addShaderFromSourceFile(QOpenGLShader::Vertex, vertexShaderFile))
	{
		error = log();
		return false;
	}

	if (!fragShaderFile.isEmpty() && !addShaderFromSourceFile(QOpenGLShader::Fragment, fragShaderFile))
	{
		error = log();
		return false;
	}

	if (!link())
	{
		error = log();
		return false;
	}

	return true;
}
