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
//$Rev:: 1992                                                              $
//$LastChangedDate:: 2012-01-18 12:17:49 +0100 (mer., 18 janv. 2012)       $
//**************************************************************************
//

#include "ccShader.h"

#include <assert.h>
#include <string.h>
#include <stdio.h>

ccShader::ccShader()
{
    prog = 0;
}

ccShader::~ccShader()
{
    reset();
}

void ccShader::reset()
{
    if (prog>0)
        glDeleteProgram(prog);
    prog = 0;
}

GLuint ccShader::getProgram()
{
    return prog;
}

void ccShader::start()
{
    glUseProgram(prog);
}

void ccShader::stop()
{
    glUseProgram(0);
}

bool ccShader::fromFile(const char *shaderBasePath, const char *shaderBaseFilename)
{
    if (!shaderBasePath || !shaderBaseFilename)
        return false;

    char vertFilename[1024];
    sprintf(vertFilename,"%s/%s.vert",shaderBasePath,shaderBaseFilename);
    char fragFilename[1024];
    sprintf(fragFilename,"%s/%s.frag",shaderBasePath,shaderBaseFilename);

    return loadProgram(vertFilename,fragFilename);
}
//*/

bool ccShader::loadProgram(const char *vertexShaderFile, const char *pixelShaderFile)
{
    assert(vertexShaderFile || pixelShaderFile);

    reset();
    assert(prog == 0);

    //GL ids
    GLuint vs = 0, ps = 0;

    //we load shader files
    if(vertexShaderFile)
    {
        vs = LoadShader(GL_VERTEX_SHADER, vertexShaderFile);
        if(!vs)
            return false;
    }
    if(pixelShaderFile)
    {
        ps = LoadShader(GL_FRAGMENT_SHADER, pixelShaderFile);
        if(!ps)
        {
            if(glIsShader(vs))
                glDeleteShader(vs);
            return false;
        }
    }

    //we create an empty GL program
    prog = glCreateProgram();

    //we attach loaded shaders to it
    if(vs)
        glAttachShader(prog, vs);
    if(ps)
        glAttachShader(prog, ps);

    //we link them alltogether
    glLinkProgram(prog);

    //we check for success
    GLint linkStatus = GL_TRUE;
    glGetProgramiv(prog, GL_LINK_STATUS, &linkStatus);
    if(linkStatus != GL_TRUE)
    {
        GLint logSize = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logSize);
        char* log = new char[logSize+1];
        if(log)
        {
            memset(log, '\0', logSize + 1);
            glGetProgramInfoLog(prog, logSize, &logSize, log);

            //ccConsole::Error("Can't create shader program! (%s)", log);

            delete[] log;
        }
        //else ccConsole::Error("Can't create shader program! (unable to get GL log)");

        glDeleteProgram(prog);
        prog = 0;
    }

    // even if program creation was successful, we don't need the shaders anymore
    if(vs)
        glDeleteShader(vs);
    if(ps)
        glDeleteShader(ps);

    return true;
}

char* ccShader::ReadShaderFile(const char *filename)
{
    //we try to open the ASCII file (containing the "program" source code)
    FILE *fp = fopen(filename, "rb");
    if(!fp)
    {
        //ccConsole::Error("Can't open file '%s'", filename);
        return NULL;
    }

    //we try to determine the file size (no need for 64bits handling here!)
    fseek(fp, 0, SEEK_END);
    long int count = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char *src = new char[count+1];
    if(!src)
    {
        fclose(fp);
        //ccConsole::Error("Not enough memory!");
        return NULL;
    }

    //we upload the program in memory
    fread(src,sizeof(char),count,fp);

    //last character is 0 to form a proper string
    src[count] = 0;

    fclose(fp);

    return src;
}

GLuint ccShader::LoadShader(GLenum type, const char *filename)
{
    //Shader creation
    GLuint shader = glCreateShader(type);
    if(shader == 0)
    {
        //ccConsole::Error("Can't create shader!");
        return 0;
    }

    //Program loading
    char *src = ReadShaderFile(filename);
    if(!src)
    {
        glDeleteShader(shader);
        return 0;
    }

    glShaderSource(shader, 1, (const GLchar**)&src, NULL);
    glCompileShader(shader);

    //we don't need the program code anymore
    delete[] src;
    src=0;

    //we must check compilation result
    GLint status = GL_TRUE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if(status != GL_TRUE)
    {
        //log size
        GLsizei logSize;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logSize);

        //buffer to get log from OpenGL
        char* log = new char[logSize+1];
        if(!log)
        {
            //ccConsole::Warning("Not enough memory to log shader creation...");
            return 0;
        }
        memset(log, 0, logSize+1);

        glGetShaderInfoLog(shader, logSize, &logSize, log);
        //ccConsole::Error("Can't compile shader (file:'%s').\nLog: %s",filename,log);

        //free memory
        delete[] log;
        glDeleteShader(shader);

        return 0;
    }

    return shader;
}

void ccShader::setUniform1i(int loc, int value)
{
	glUniform1i(loc,value);
}

void ccShader::setUniform1f(int loc, float value)
{
	glUniform1f(loc,value);
}

void ccShader::setUniform4fv(int loc, float* value)
{
	glUniform4fv(loc, 1, value);
}

void ccShader::setUniform1i(const char* uniform, int value)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform1i(loc,value);
}

void ccShader::setUniform2iv(const char* uniform, int* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform2iv(loc,1,val);
}

void ccShader::setUniform3iv(const char* uniform, int* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform3iv(loc,1,val);
}

void ccShader::setUniform4iv(const char* uniform, int* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform4iv(loc,1,val);
}

void ccShader::setTabUniform4iv(const char* uniform, int size, int* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform4iv(loc,size,val);
}

void ccShader::setUniform1f(const char* uniform, float value)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform1f(loc,value);
}

void ccShader::setUniform2fv(const char* uniform, float* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform2fv(loc,1,val);
}

void ccShader::setUniform3fv(const char* uniform, float* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform3fv(loc,1,val);
}

void ccShader::setUniform4fv(const char* uniform, float* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform4fv(loc,1,val);
}

void ccShader::setTabUniform1fv(const char* uniform, int size, float* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform1fv(loc,size,val);
}

void ccShader::setTabUniform3fv(const char* uniform, int size, float* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform3fv(loc,size,val);
}

void ccShader::setTabUniform4fv(const char* uniform, int size, float* val)
{
	int loc = glGetUniformLocation(prog,uniform);
	glUniform4fv(loc,size,val);
}

void ccShader::setUniformMatrix4fv(const char* uniform, float* val, bool transpose)
{
	int loc	= glGetUniformLocation(prog,uniform);
	glUniformMatrix4fv(loc,1,transpose,val);
}

void ccShader::setAttrib4iv(int loc, int* val)
{
	glVertexAttrib4iv(loc, val);
}

void ccShader::setAttrib1f(int loc, float value)
{
	glVertexAttrib1f(loc,value);
}

void ccShader::setAttrib2fv(int loc, float* val)
{
	glVertexAttrib2fv(loc, val);
}

void ccShader::setAttrib3fv(int loc, float* val)
{
	glVertexAttrib3fv(loc, val);
}

void ccShader::setAttrib4fv(int loc, float* val)
{
	glVertexAttrib4fv(loc, val);
}

/************************************
            ccShaderARB
************************************/


ccShaderARB::ccShaderARB() : ccShader()
{
    vert=0;
    frag=0;
}

ccShaderARB::~ccShaderARB()
{
    reset();
}

void ccShaderARB::reset()
{
    if (prog>0)
    {
        if (vert)
            glDetachObjectARB(prog, vert);
        if (frag)
            glDetachObjectARB(prog, frag);
        glDeleteObjectARB(prog);
    }
    prog=0;

    if (vert)
        glDeleteObjectARB(vert);
    vert=0;

    if (frag)
        glDeleteObjectARB(frag);
    frag=0;
}

void ccShaderARB::start()
{
    glUseProgramObjectARB(prog);
}

void ccShaderARB::stop()
{
    glUseProgramObjectARB(0);
}

void ccShaderARB::setUniform1i(int loc, int value)
{
	glUniform1iARB(loc,value);
}

void ccShaderARB::setUniform1f(int loc, float value)
{
	glUniform1fARB(loc,value);
}

void ccShaderARB::setUniform4fv(int loc, float* value)
{
	glUniform4fvARB(loc, 1, value);
}

void ccShaderARB::setUniform1i(const char* uniform, int value)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform1iARB(loc,value);
}

void ccShaderARB::setUniform2iv(const char* uniform, int* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform2ivARB(loc,1,val);
}

void ccShaderARB::setUniform3iv(const char* uniform, int* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform3ivARB(loc,1,val);
}

void ccShaderARB::setUniform4iv(const char* uniform, int* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform4ivARB(loc,1,val);
}

void ccShaderARB::setTabUniform4iv(const char* uniform, int size, int* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform4ivARB(loc,size,val);
}

void ccShaderARB::setUniform1f(const char* uniform, float value)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform1fARB(loc,value);
}

void ccShaderARB::setUniform2fv(const char* uniform, float* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform2fvARB(loc,1,val);
}

void ccShaderARB::setUniform3fv(const char* uniform, float* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform3fvARB(loc,1,val);
}

void ccShaderARB::setUniform4fv(const char* uniform, float* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform4fvARB(loc,1,val);
}

void ccShaderARB::setTabUniform1fv(const char* uniform, int size, float* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform1fvARB(loc,size,val);
}

void ccShaderARB::setTabUniform4fv(const char* uniform, int size, float* val)
{
	int loc = glGetUniformLocationARB(prog,uniform);
	glUniform4fvARB(loc,size,val);
}

void ccShaderARB::setUniformMatrix4fv(const char* uniform, float* val, bool transpose)
{
	int loc	= glGetUniformLocationARB(prog,uniform);
	glUniformMatrix4fvARB(loc,1,transpose,val);
}

void ccShaderARB::setAttrib4iv(int loc, int* val)
{
	glVertexAttrib4ivARB(loc, val);
}

void ccShaderARB::setAttrib1f(int loc, float value)
{
	glVertexAttrib1fARB(loc,value);
}

void ccShaderARB::setAttrib2fv(int loc, float* val)
{
	glVertexAttrib2fvARB(loc, val);
}

void ccShaderARB::setAttrib3fv(int loc, float* val)
{
	glVertexAttrib3fvARB(loc, val);
}

void ccShaderARB::setAttrib4fv(int loc, float* val)
{
	glVertexAttrib4fvARB(loc, val);
}

bool ccShaderARB::loadProgram(const char *vertexShaderFile, const char *pixelShaderFile)
{
    assert(vertexShaderFile || pixelShaderFile);

    reset();
    assert(prog == 0);

    if (vertexShaderFile)
        vert=LoadShaderARB(GL_VERTEX_SHADER_ARB,vertexShaderFile);

    if (pixelShaderFile)
        frag=LoadShaderARB(GL_FRAGMENT_SHADER_ARB,pixelShaderFile);

    if (!frag && !vert)
    {
        //ccConsole::Warning("No shader loaded! Wrong filename?");
        return false;
    }

	//creating program
	prog = glCreateProgramObjectARB();

	//attaching shaders
	if (vert)
        glAttachObjectARB(prog,vert);
	if (frag)
        glAttachObjectARB(prog,frag);

    //linking
	glLinkProgramARB(prog);

    //we check for success
    /*GLint linkStatus = GL_TRUE;
    glGetProgramiv(prog, GL_LINK_STATUS, &linkStatus);
    if(linkStatus != GL_TRUE)
    {
        GLint logSize = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logSize);
        char* log = new char[logSize+1];
        if(log)
        {
            memset(log, '\0', logSize + 1);
            glGetProgramInfoLog(prog, logSize, &logSize, log);

            ccConsole::Error("Can't create shader program! (%s)", log);

            delete[] log;
        }
        else ccConsole::Error("Can't create shader program! (unable to get GL log)");

        glDeleteProgram(prog);
        prog = 0;
    }
    //*/

	stop();

    return true;
}

GLhandleARB ccShaderARB::LoadShaderARB(GLenum type, const char *filename)
{
    //Shader creation
    GLhandleARB shader = glCreateShaderObjectARB(type);
    if(shader == 0)
    {
        //ccConsole::Error("Can't create shader!");
        return 0;
    }

    //code loading
    char *src = ReadShaderFile(filename);
    if(!src)
    {
        glDeleteObjectARB(shader);
        return 0;
    }

    glShaderSourceARB(shader, 1, (const GLcharARB**)&src, NULL);
    glCompileShaderARB(shader);

    //we don't need the program code anymore
    delete[] src;
    src=0;

    //we must check compilation result
    /*GLint status = GL_TRUE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if(status != GL_TRUE)
    {
        //log size
        GLsizei logSize;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logSize);

        //buffer to get log from OpenGL
        char* log = new char[logSize+1];
        if(!log)
        {
            ccConsole::Warning("Not enough memory to log shader creation...");
            return 0;
        }
        memset(log, 0, logSize+1);

        glGetShaderInfoLog(shader, logSize, &logSize, log);
        ccConsole::Error("Can't compile shader (file:'%s').\nLog: %s",filename,log);

        //free memory
        delete[] log;
        glDeleteShader(shader);

        return 0;
    }
    //*/

    return shader;
}
