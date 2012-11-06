//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
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
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#include "ccGLUtils.h"
#include "ccConsole.h"

#include <CCConst.h>

//*********** OPENGL TEXTURES ***********//

void ccGLUtils::DisplayTexture2DPosition(GLuint tex, int x, int y, int w, int h, unsigned char alpha/*=255*/)
{
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex);

    glColor4ub(255, 255, 255, alpha);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0,0.0);
    glVertex2i(x, y+h);
    glTexCoord2f(0.0,1.0);
    glVertex2i(x, y);
    glTexCoord2f(1.0,1.0);
    glVertex2i(x+w, y);
    glTexCoord2f(1.0,0.0);
    glVertex2i(x+w, y+h);
    glEnd();

    glBindTexture(GL_TEXTURE_2D,0);
    glDisable(GL_TEXTURE_2D);
}

void ccGLUtils::DisplayTexture2D(GLuint tex, int w, int h, unsigned char alpha/*=255*/)
{
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex);

    float halfW = float(w)*0.5;
    float halfH = float(h)*0.5;

    glColor4ub(255, 255, 255, alpha);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex2f(-halfW, -halfH);
    glTexCoord2f(1.0, 0.0);
    glVertex2f( halfW, -halfH);
    glTexCoord2f(1.0, 1.0);
    glVertex2f( halfW,  halfH);
    glTexCoord2f(0.0, 1.0);
    glVertex2f(-halfW,  halfH);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
}

void ccGLUtils::DisplayTexture2DCorner(GLuint tex, int w, int h, unsigned char alpha/*=255*/)
{
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex);

    glColor4ub(255, 255, 255, alpha);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex2f(0, 0);
    glTexCoord2f(1.0, 0.0);
    glVertex2f( w, 0);
    glTexCoord2f(1.0, 1.0);
    glVertex2f( w,  h);
    glTexCoord2f(0.0, 1.0);
    glVertex2f(0,  h);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
}

#ifndef GL_BGRA
#define GL_BGRA 0x80E1
#endif

#define READ_PIXELS_PER_LINE
void ccGLUtils::SaveTextureToArray(unsigned char* data, GLuint texID, unsigned w, unsigned h)
{
	assert(data);

#ifdef READ_PIXELS_PER_LINE
	//to avoid memory issues, we read line by line
	for (int i=0;i<(int)h;++i)
	{
		glReadPixels(0,i,w,1,GL_BGRA,GL_UNSIGNED_BYTE,data+((int)h-1-i)*(int)w*4);
		ccGLUtils::CatchGLError("ccGLUtils::SaveTextureToArray");
	}
#else
	//GLint width,height,format,align;
	//glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
	//glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
	//glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &format);
	//glGetIntegerv(GL_PACK_ALIGNMENT,&align);

    //we grab texture data
    glBindTexture(GL_TEXTURE_2D, texID);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
    glBindTexture(GL_TEXTURE_2D, 0);
#endif
}

bool ccGLUtils::SaveTextureToFile(const char* filename, GLuint texID, unsigned w, unsigned h)
{
	QImage temp(w,h,QImage::Format_ARGB32);
    GLubyte* data = temp.bits();

    if (!data)
    {
        ccConsole::Error("[ccGLUtils::saveTextureToFile] Not enough memory!");
        return false;
    }

	SaveTextureToArray(data,texID,w,h);

    //we save image
#ifdef READ_PIXELS_PER_LINE
	temp.save(filename);
#else
    temp.mirrored(false,true).save(filename);
#endif

    return true;
}

//*********** OPENGL MATRICES ***********//

void ccGLUtils::MultGLMatrices(const float* A, const float* B, float* dest)
{
    //we backup actual matrix...
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glLoadMatrixf(A);
    glMultMatrixf(B);
    glGetFloatv(GL_MODELVIEW_MATRIX, dest);

    //... and restore it
    glPopMatrix();
}

void ccGLUtils::TransposeGLMatrix(const float* A, float* dest)
{
    unsigned char i,j;

    for (i=0;i<4;i++)
        for (j=0;j<4;j++)
            dest[(i<<2)+j] = A[(j<<2)+i];
}

ccGLMatrix ccGLUtils::GenerateGLRotationMatrixFromVectors(const float* sourceVec, const float* destVec)
{
    //we compute scalar prod between the two vectors
    float ps = CCVector3::vdot(sourceVec,destVec);

    //we bound result (in case vecors are not exactly unit)
    if (ps>1.0)
        ps=1.0;
    else if (ps<-1.0)
        ps=-1.0;

    //we deduce angle from scalar prod
    float angle_deg = acos(ps)*CC_RAD_TO_DEG;

    //we compute rotation axis with scalar prod
    float axis[3];
    CCVector3::vcross(sourceVec,destVec,axis);

    //we eventually compute the rotation matrix with axis and angle
    return GenerateGLRotationMatrixFromAxisAndAngle(axis, angle_deg);
}

ccGLMatrix ccGLUtils::GenerateGLRotationMatrixFromAxisAndAngle(const float* axis, float angle_deg)
{
    //we backup actual matrix...
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glLoadIdentity();
    glRotatef(angle_deg, axis[0], axis[1], axis[2]);
	ccGLMatrix mat;
    glGetFloatv(GL_MODELVIEW_MATRIX, mat.data());

    //... and restore it
    glPopMatrix();

	return mat;
}

ccGLMatrix ccGLUtils::GenerateViewMat(CC_VIEW_ORIENTATION orientation)
{
    float U[3],V[3];
    U[0] = U[1] = U[2] = 0.0;
    V[0] = V[1] = V[2] = 0.0;

    switch (orientation)
    {
    case CC_TOP_VIEW: //pXY
        U[2] = 1.0;
        V[1] = 1.0;
        break;
    case CC_BOTTOM_VIEW: //mXY
        U[2] = -1.0;
        V[1] = 1.0;
        break;
    case CC_FRONT_VIEW: //pXZ
        U[1] = -1.0;
        V[2] = 1.0;
        break;
    case CC_BACK_VIEW: //mXZ
        U[1] = 1.0;
        V[2] = 1.0;
        break;
    case CC_LEFT_VIEW: //pYZ
        U[0] = 1.0;
        V[2] = 1.0;
        break;
    case CC_RIGHT_VIEW: //mYZ
        U[0] = -1.0;
        V[2] = 1.0;
        break;
    }

    ccGLMatrix result;

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    gluLookAt(U[0],U[1],U[2],0.0,0.0,0.0,V[0],V[1],V[2]);
    glGetFloatv(GL_MODELVIEW_MATRIX, result.data());
    result.data()[14] = 0.0; //annoying value (?!)
    glPopMatrix();

    return result;
}

//*********** OPENGL EXTENSIONS ***********//

//! Loads all available OpenGL extensions
bool ccGLUtils::InitGLEW()
{
    #ifdef USE_GLEW
    // GLEW initialization
    GLenum code = glewInit();
    if(code != GLEW_OK)
    {
        ccConsole::Error("Error while initializing OpenGL extensions! (see console)");
        ccConsole::Warning("GLEW error: %s",glewGetErrorString(code));
        return false;
    }

    ccConsole::Print("GLEW: initialized!");
    return true;
    #else
    return false;
    #endif
}

//! Check for availability of a given OpenGL extension
bool ccGLUtils::CheckExtension(const char *extName)
{
    #ifdef USE_GLEW
    return glewIsSupported(extName);
    #else
    return false;
    #endif
}

bool ccGLUtils::CheckShadersAvailability()
{
    bool bARBShadingLanguage       = CheckExtension("GL_ARB_shading_language_100");
    bool bARBShaderObjects         = CheckExtension("GL_ARB_shader_objects");
    bool bARBVertexShader          = CheckExtension("GL_ARB_vertex_shader");
    bool bARBFragmentShader        = CheckExtension("GL_ARB_fragment_shader");

    bool bShadersSupported = bARBShadingLanguage &&
                             bARBShaderObjects &&
                             bARBVertexShader &&
                             bARBFragmentShader;

    return bShadersSupported;
}

bool ccGLUtils::CheckFBOAvailability()
{
    return CheckExtension("GL_EXT_framebuffer_object");
}

bool ccGLUtils::CheckVBOAvailability()
{
    return CheckExtension("GL_ARB_vertex_buffer_object");
}

bool ccGLUtils::CatchGLError(const char* context)
{
	GLenum err = glGetError();
	switch(err)
	{
	case GL_INVALID_VALUE:
		ccConsole::Warning("[%s] OpenGL error: invalid value",context);
		break;
	case GL_INVALID_OPERATION:
		ccConsole::Warning("[%s] OpenGL error: invalid operation",context);
		break;
	case GL_INVALID_ENUM:
		ccConsole::Warning("[%s] OpenGL error: invalid enumerator",context);
		break;
	case GL_STACK_OVERFLOW:
		ccConsole::Error("[%s] OpenGL error: stack overflow",context);
		break;
	case GL_STACK_UNDERFLOW:
		ccConsole::Error("[%s] OpenGL error: stack underflow",context);
		break;
	case GL_OUT_OF_MEMORY:
		ccConsole::Error("[%s] OpenGL error: out of memory",context);
		break;
	//case NO_ERROR: //DGM: OpenGL@Linux doesn't know this enum!
	default:
		//ccConsole::Error(stdout,"[%s] No error\n",context);
		return false;
	}

	return true;
}
