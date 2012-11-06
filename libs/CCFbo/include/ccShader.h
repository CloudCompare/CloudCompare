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
//$Rev:: 1696                                                              $
//$LastChangedDate:: 2010-10-26 17:34:10 +0200 (mar., 26 oct. 2010)        $
//**************************************************************************
//

#ifndef CC_SHADER_HEADER
#define CC_SHADER_HEADER

#include "ccGlew.h"

class ccShader
{
public:

    ccShader();
    virtual ~ccShader();

    //! Creates program from two shader files with same base filename
    /** Path and extensions (.vert and .frag) are automatically
        added to shader base filename (shortcut to ccShader::loadProgram).
        \param shaderBasePath shader files path
        \param shaderBaseFilename shader base filename
        \return success
    **/
    virtual bool fromFile(const char *shaderBasePath, const char *shaderBaseFilename);

    //! Creates program from one or two shader files
    /** Filenames must be absolute (full path).
    **/
    virtual bool loadProgram(const char *vertShaderFile, const char *fragShaderFile);

    virtual void reset();

    virtual void start();
    virtual void stop();

    //! Returns program GL ID
    GLuint getProgram();

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
    static GLuint LoadShader(GLenum type, const char *filename);

    //! Bufferizes a shader file in memory
    static char* ReadShaderFile(const char *filename);

    GLuint prog;
};

class ccShaderARB : public ccShader
{
public:

    ccShaderARB();
    virtual ~ccShaderARB();

    virtual void reset();

    virtual void start();
    virtual void stop();

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
    virtual void setTabUniform4fv(const char* uniform, int size, float* p_val);
    virtual void setUniformMatrix4fv(const char* variable, float* p_val, bool transpose = false);

    /** ATTRIBUTES **/

    virtual void setAttrib4iv(int loc, int* p_val);
    virtual void setAttrib1f(int loc, float val);
    virtual void setAttrib2fv(int loc, float* p_val);
    virtual void setAttrib3fv(int loc, float* p_val);
    virtual void setAttrib4fv(int loc, float* p_val);

protected:

    //! Creates program with one (or two) shaders
    virtual bool loadProgram(const char *vertexShaderFile, const char *pixelShaderFile);

    //! Loads an ARB shader from a file
    static GLhandleARB LoadShaderARB(GLenum type, const char *filename);

    GLhandleARB vert;
    GLhandleARB frag;
};

#endif
