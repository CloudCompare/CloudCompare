//##########################################################################
//#                                                                        #
//#                                PCV                                     #
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
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "PCVContext.h"

//CCLib
#include <CCMiscTools.h>

//OpenGL
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

//system
#include <assert.h>

using namespace CCLib;

#define ZTWIST 1e-3f

PCVContext::PCVContext()
	: m_vertices(0)
	, m_mesh(0)
	, m_zoom(1.0f)
	, m_pixBuffer(0)
	, m_width(0)
	, m_height(0)
	, m_snapZ(0)
	, m_snapC(0)
{
}

PCVContext::~PCVContext()
{
	if (m_pixBuffer)
		delete m_pixBuffer;

	if (m_snapZ)
		delete[] m_snapZ;
	if (m_snapC)
		delete[] m_snapC;
}

bool PCVContext::init(unsigned W,
					  unsigned H,
					  CCLib::GenericCloud* cloud,
					  CCLib::GenericMesh* mesh/*=0*/,
					  bool closedMesh/*=true*/)
{
	if (!QGLPixelBuffer::hasOpenGLPbuffers())
		return false;

	assert(!m_pixBuffer);

	m_pixBuffer = new QGLPixelBuffer(W,H);
	if (!m_pixBuffer || !m_pixBuffer->isValid())
		return false;

	unsigned size = W*H;
	m_snapZ=new float[size];
	if (!m_snapZ)
	{
		delete m_pixBuffer;
		m_pixBuffer=0;
		return false;
	}

	m_meshIsClosed = (closedMesh || !mesh);
	if (!m_meshIsClosed)
	{
		//buffer for color
		m_snapC=new uchar[4*size];
		if (!m_snapC)
		{
			delete m_pixBuffer;
			m_pixBuffer=0;
			delete m_snapZ;
			m_snapZ=0;
			return false;
		}
	}

	m_width=W;
	m_height=H;

	associateToEntity(cloud,mesh);

	glInit();

	return true;
}

void PCVContext::associateToEntity(GenericCloud* cloud, GenericMesh* mesh)
{
	assert(cloud);
	if (!cloud)
        return;

	m_vertices = cloud;
	m_mesh = mesh;

	//we get cloud bounding box
	CCVector3 bbMin,bbMax;
	m_vertices->getBoundingBox(bbMin.u,bbMax.u);

	//we compute bbox diagonal
	PointCoordinateType maxD = (bbMax-bbMin).norm();

	//we deduce default zoom
	m_zoom = (maxD > (float)ZERO_TOLERANCE ? (float)std::min(m_width,m_height) / maxD : 1.0f);

	//as well as display center
	m_viewCenter = (bbMax+bbMin)*0.5f;
}

void PCVContext::glInit()
{
	if (!m_pixBuffer || !m_pixBuffer->isValid())
		return;

	m_pixBuffer->makeCurrent();

	glClearColor(0.0,0.0,0.0,0.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glDepthMask(GL_TRUE);
	glDisable(GL_LIGHTING);

	glPixelStorei(GL_PACK_ROW_LENGTH, 0);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);

	//model view matrix initialization
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glGetFloatv(GL_MODELVIEW_MATRIX, m_viewMat);
	glPushMatrix();

	//projection matrix initialization
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float w2 = 0.5f*(float)m_width;
	float h2 = 0.5f*(float)m_height;
	float maxD = (float)std::max(m_width,m_height);
	glOrtho(-w2,w2,-h2,h2,-maxD,maxD);
	glPushMatrix();
}

void PCVContext::setViewDirection(const float* V)
{
	if (!m_pixBuffer || !m_pixBuffer->isValid())
		return;

	m_pixBuffer->makeCurrent();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
    glLoadIdentity();

	float U[3]={0.0,0.0,1.0};
	if (1.0f-fabs(CCVector3::vdot(V,U))<1e-4f)
	{
		U[1]=1.0;
		U[2]=0.0;
	}

	gluLookAt(-V[0],-V[1],-V[2],0.0,0.0,0.0,U[0],U[1],U[2]);
	glGetFloatv(GL_MODELVIEW_MATRIX, m_viewMat);
	glPopMatrix();
}

void PCVContext::drawEntity()
{
	assert(m_vertices);

	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(m_viewMat);
	glScalef(m_zoom,m_zoom,m_zoom);
	glTranslatef(-m_viewCenter.x, -m_viewCenter.y, -m_viewCenter.z);

	glColor3ub(255,255,0); //yellow by default

	if (m_mesh)
	{
		unsigned nTri=m_mesh->size();
		m_mesh->placeIteratorAtBegining();

		glBegin(GL_TRIANGLES);
		for (unsigned i=0;i<nTri;++i)
		{
			const GenericTriangle* t = m_mesh->_getNextTriangle();
			glVertex3fv(t->_getA()->u);
			glVertex3fv(t->_getB()->u);
			glVertex3fv(t->_getC()->u);
		}
		glEnd();
	}
	else
	{
		unsigned nPts = m_vertices->size();
		m_vertices->placeIteratorAtBegining();

		glBegin(GL_POINTS);
		for (unsigned i=0;i<nPts;++i)
            glVertex3fv(m_vertices->getNextPoint()->u);
		glEnd();
	}
}

void openGLSnapshot(GLenum format, GLenum type, void* buffer)
{
	assert(buffer);

	int vp[4];
	glGetIntegerv(GL_VIEWPORT ,vp);

	glReadPixels(vp[0],vp[1],vp[2],vp[3],format,type,buffer);
}

//The method below is inspired from ShadeVis' "GLAccumPixel" (Cignoni et al.)
/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*****************************************************************************/
int PCVContext::GLAccumPixel(int* pixelsSeen)
{
	if (!m_pixBuffer || !m_pixBuffer->isValid())
		return -1;
	if (!m_vertices)
		return -1;
	assert(m_snapZ);

	m_pixBuffer->makeCurrent();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthRange(2.0f*ZTWIST,1.0f);

	if (m_meshIsClosed)
		glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
	else 
		glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

	//display 3D entity (front)
	glCullFace(GL_BACK);
	drawEntity();

	if (!m_meshIsClosed)
	{
		//display it again (back)
		glCullFace(GL_FRONT);
		drawEntity();

		assert(m_snapC);
		openGLSnapshot(GL_RGBA,GL_UNSIGNED_BYTE,m_snapC);
	}
	openGLSnapshot(GL_DEPTH_COMPONENT,GL_FLOAT,m_snapZ);

	if (m_meshIsClosed)
		glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

	glDepthRange(0,1.0f-2.0f*ZTWIST);
	double MM[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,MM);
	double MP[16];
	glGetDoublev(GL_PROJECTION_MATRIX,MP);
	int VP[4];
	glGetIntegerv(GL_VIEWPORT,VP);

	int cnt=0;
	int sx4 = (m_width<<2);

	unsigned nVert = m_vertices->size();
	m_vertices->placeIteratorAtBegining();
	for(unsigned i=0;i<nVert;++i)
	{
		const CCVector3* P = m_vertices->getNextPoint();

		double tx,ty,tz;
		gluProject(P->x,P->y,P->z,MM,MP,VP,&tx,&ty,&tz);

		int txi=(int)floor(tx);
		int tyi=(int)floor(ty);
		if(txi>=0 && txi<(int)m_width && tyi>=0 && tyi<(int)m_height)
		{
			int dec = txi+tyi*(int)m_width;
			int col=1;

			if (!m_meshIsClosed)
			{
				//int c1 = std::max(m_snapC[dec],m_snapC[dec<<2+4]);
				//int c2 = std::max(m_snapC[dec<<2+sx4],m_snapC[dec<<2+sx4+4]);
				const uchar* pix = m_snapC+(dec<<2);
				int c1 = std::max(pix[0],pix[4]);
				pix += sx4;
				int c2 = std::max(pix[0],pix[4]);
				col = std::max(c1,c2);
			}

			if (col!=0)
			{
				if ((float)tz < m_snapZ[dec])
				{
					++pixelsSeen[i];
					++cnt;
				}
			}
	   }
	}

	return cnt;
}
