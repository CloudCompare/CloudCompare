//##########################################################################
//#                                                                        #
//#                                PCV                                     #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  license.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "PCVContext.h"

//CCCoreLib
#include <CCMath.h>
#include <CCMiscTools.h>
#include <GenericTriangle.h>

//Qt
#include <QWindow>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLVersionFunctionsFactory>
#include <QOpenGLFunctions_2_1>
#include <QCoreApplication>

//OpenGL
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#if defined(_WIN32)
#include <Windows.h>
#endif
#include <GL/glu.h>
#endif

//system
#include <cassert>
#include <cstring>

//type-less glVertex3Xv call (X=f,d)
static inline void GLVertex3v(const float* v, QOpenGLFunctions_2_1* functions) { functions->glVertex3fv(v); }
static inline void GLVertex3v(const double* v, QOpenGLFunctions_2_1* functions) { functions->glVertex3dv(v); }

using namespace CCCoreLib;

#ifndef ZTWIST
#define ZTWIST 1e-3
#endif

//#define SHOW_LIVE_RENDERING // for tests
//#define GENERATE_TEST_CAPTURE // for tests

PCVContext::PCVContext()
	: m_vertices(nullptr)
	, m_mesh(nullptr)
	, m_diagonal(0)
	, m_glSurface(nullptr)
	, m_glContext(nullptr)
	, m_pixBuffer(nullptr)
	, m_width(0)
	, m_height(0)
	, m_meshIsClosed(false)
{
}

PCVContext::~PCVContext()
{
	delete m_pixBuffer;
	if (m_glContext)
	{
		m_glContext->doneCurrent();
		delete m_glContext;
		m_glContext = nullptr;
	}
	delete m_glSurface;
}

bool PCVContext::init(unsigned W,
					  unsigned H,
					  CCCoreLib::GenericCloud* cloud,
					  CCCoreLib::GenericMesh* mesh/*=nullptr*/,
					  bool closedMesh/*=true*/)
{
	assert(!m_pixBuffer);

	m_meshIsClosed = (closedMesh || !mesh);

	unsigned size = W * H;
	try
	{
		// depth buffer
		m_snapZ.resize(size, 0);
#if not defined(SHOW_LIVE_RENDERING) && not defined(GENERATE_TEST_CAPTURE)
		if (!m_meshIsClosed)
#endif
		{
			// color buffer
			m_snapC.resize(size * 4, 0);
		}
	}
	catch (const std::bad_alloc)
	{
		// not enough memory
		return false;
	}

	m_width = W;
	m_height = H;

	associateToEntity(cloud, mesh);

	if (!glInit())
	{
		// failed to initialize the GL context
		return false;
	}

	if (!makeCurrent())
	{
		return false;
	}

	auto* functions = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(m_glContext);
	if (!functions)
	{
		// OpenGL version not supported
		return false;
	}
	functions->initializeOpenGLFunctions();
	functions->glPixelStorei(GL_PACK_ROW_LENGTH, 0);
	functions->glPixelStorei(GL_PACK_ALIGNMENT, 1);
	functions->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	m_pixBuffer = new QOpenGLBuffer(QOpenGLBuffer::PixelPackBuffer);
	if (!m_pixBuffer->create())
	{
		delete m_pixBuffer;
		m_pixBuffer = nullptr;
		return false;
	}

	m_pixBuffer->setUsagePattern(QOpenGLBuffer::StreamRead);

	// As per Qt's doc: The buffer must be bound to the same QOpenGLContext current when create() was called,
	// or to another QOpenGLContext that is sharing with it. Otherwise, false will be returned from this function.
	if (!m_pixBuffer->bind())
	{
		return false;
	}

	// As per Qt's doc: It is assumed that create() has been called on this buffer
	// and that it has been bound to the current context.
	m_pixBuffer->allocate(W * H * sizeof(float));

	m_pixBuffer->release();

	return true;
}

void PCVContext::associateToEntity(GenericCloud* cloud, GenericMesh* mesh)
{
	m_vertices = nullptr;
	m_diagonal = static_cast<PointCoordinateType>(std::max(m_width, m_height));
	m_viewCenter = CCVector3(0, 0, 0);

	if (!cloud)
	{
		assert(false);
		return;
	}

	m_vertices = cloud;
	m_mesh = mesh;

	//we retrieve the cloud bounding box
	CCVector3 bbMin;
	CCVector3 bbMax;
	m_vertices->getBoundingBox(bbMin, bbMax);

	//we compute the bbox diagonal
	m_diagonal = (bbMax - bbMin).norm();

	//as well as the display center
	m_viewCenter = (bbMax + bbMin) / 2;
}

bool PCVContext::makeCurrent()
{
	if (!m_glContext || !m_glSurface)
	{
		return false;
	}
	if (!m_glContext->makeCurrent(m_glSurface))
	{
        return false;
	}
	return true;
}

bool PCVContext::glInit()
{
	if (!m_glContext)
	{
		m_glContext = new QOpenGLContext;
		if (!m_glContext->create())
		{
			delete m_glContext;
			m_glContext = nullptr;
			return false;
		}
	}

	if (!m_glSurface)
	{
		QWindow* surface = new QWindow;
		surface->setSurfaceType(QSurface::OpenGLSurface);
		surface->setFormat(m_glContext->format());
		surface->create();
		surface->resize(m_width, m_height);
#ifdef SHOW_LIVE_RENDERING
		surface->show();
		QCoreApplication::processEvents();
#endif
		m_glSurface = surface;
	}

	return true;
}

void PCVContext::drawEntity()
{
	auto* functions = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(m_glContext);
	if (!functions)
	{
		// OpenGL version not supported
		return;
	}
	
	if (m_mesh)
	{
		unsigned nTri = m_mesh->size();
		m_mesh->placeIteratorAtBeginning();

		functions->glBegin(GL_TRIANGLES);
		for (unsigned i = 0; i < nTri; ++i)
		{
			const GenericTriangle* t = m_mesh->_getNextTriangle();
			GLVertex3v(t->_getA()->u, functions);
			GLVertex3v(t->_getB()->u, functions);
			GLVertex3v(t->_getC()->u, functions);
		}
		functions->glEnd();
	}
	else if (m_vertices)
	{
		unsigned nPts = m_vertices->size();
		m_vertices->placeIteratorAtBeginning();

		functions->glBegin(GL_POINTS);
		for (unsigned i = 0; i < nPts; ++i)
		{
			GLVertex3v(m_vertices->getNextPoint()->u, functions);
		}
		functions->glEnd();
	}
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
int PCVContext::glAccumPixel(std::vector<int>& visibilityCount, const CCVector3d& viewDir)
{
	if (!m_pixBuffer || !m_pixBuffer->isCreated())
		return -1;
	if (!m_vertices)
		return -1;
	if (m_vertices->size() != visibilityCount.size())
		return -1;

	assert(m_snapZ.data());

	if (!makeCurrent())
	{
		return -2;
	}

	auto* functions = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(m_glContext);
	if (!functions)
	{
		// OpenGL version not supported
		return -3;
	}

	functions->glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//functions->glDepthRange(2.0*ZTWIST, 1.0);

	functions->glPointSize(1.0f);
	functions->glDisable(GL_BLEND);
	functions->glEnable(GL_DEPTH_TEST);
	functions->glEnable(GL_CULL_FACE);
	functions->glDisable(GL_LIGHTING);

	//projection matrix initialization
	{
		functions->glMatrixMode(GL_PROJECTION);
		functions->glLoadIdentity();
		double ar = static_cast<double>(m_height) / m_width;
		double xMax = m_diagonal/2;
		double yMax = xMax * ar;
		functions->glOrtho(-xMax, xMax, -yMax, yMax, -xMax, xMax);
	}
	double projectionMat[OPENGL_MATRIX_SIZE];
	functions->glGetDoublev(GL_PROJECTION_MATRIX, projectionMat);

	//model view matrix initialization
	{
		functions->glMatrixMode(GL_MODELVIEW);
		functions->glLoadIdentity();

		CCVector3d U(0.0, 0.0, 1.0);
		if (1.0 - std::abs(viewDir.dot(U)) < 1.0e-4)
		{
			// swap Y and Z
			U.y = 1.0;
			U.z = 0.0;
		}

		gluLookAt(	m_viewCenter.x+-viewDir.x, m_viewCenter.y-viewDir.y, m_viewCenter.z-viewDir.z,
					m_viewCenter.x, m_viewCenter.y, m_viewCenter.z,
					U.x, U.y, U.z );
	}
	double viewMat[OPENGL_MATRIX_SIZE];
	functions->glGetDoublev(GL_MODELVIEW_MATRIX, viewMat);

	functions->glColor3ub(255, 255, 0); //yellow by default

	GLboolean drawColor = GL_TRUE;
#if not defined(GENERATE_TEST_CAPTURE) && not defined(SHOW_LIVE_RENDERING)
	drawColor = m_meshIsClosed ? GL_FALSE : GL_TRUE;
#endif
	functions->glColorMask(drawColor, drawColor, drawColor, drawColor);

	int viewPort[4]{0, 0, static_cast<int>(m_width), static_cast<int>(m_height)};
	functions->glViewport(viewPort[0], viewPort[1], viewPort[2], viewPort[3]);

	//display 3D entity (front)
	functions->glCullFace(GL_BACK);
	drawEntity();

	if (m_mesh && !m_meshIsClosed)
	{
		//display it again (back)
		functions->glCullFace(GL_FRONT);
		drawEntity();

		if (!m_pixBuffer->bind())
		{
			return -4;
		}
		functions->glReadPixels(viewPort[0], viewPort[1], viewPort[2], viewPort[3], GL_RGBA, GL_UNSIGNED_BYTE, nullptr/*m_snapC.data()*/);
		m_pixBuffer->read(0, m_snapC.data(), static_cast<int>(m_width * m_height * 4));
		m_pixBuffer->release();
	}
#ifdef GENERATE_TEST_CAPTURE
	else
	{
		if (!m_pixBuffer->bind())
		{
			return -4;
		}
		functions->glReadPixels(viewPort[0], viewPort[1], viewPort[2], viewPort[3], GL_BGRA, GL_UNSIGNED_BYTE, nullptr/*m_snapC.data()*/);
		m_pixBuffer->read(0, m_snapC.data(), static_cast<int>(m_width * m_height * 4));
		m_pixBuffer->release();

		QImage image(m_snapC.data(), m_width, m_height, QImage::Format::Format_ARGB32);
		image.save("E:/Temp/capture.png");
	}
#endif

	// Retrieve Z-buffer
	{
		if (!m_pixBuffer->bind())
		{
			return -4;
		}
		functions->glReadPixels(viewPort[0], viewPort[1], viewPort[2], viewPort[3], GL_DEPTH_COMPONENT, GL_FLOAT, nullptr /*m_snapZ.data()*/);
		m_pixBuffer->read(0, m_snapZ.data(), static_cast<int>(m_width * m_height * sizeof(float)));
		m_pixBuffer->release();
	}

#ifdef SHOW_LIVE_RENDERING
	m_glContext->swapBuffers(m_glSurface);
#endif

	int count = 0;
	int sx4 = (m_width << 2);

	unsigned nVert = m_vertices->size();
	m_vertices->placeIteratorAtBeginning();
	for (unsigned i = 0; i < nVert; ++i)
	{
		const CCVector3* P = m_vertices->getNextPoint();

		double tx = 0.0;
		double ty = 0.0;
		double tz = 0.0;
		gluProject(P->x, P->y, P->z, viewMat, projectionMat, viewPort, &tx, &ty, &tz);

		int txi = static_cast<int>(floor(tx));
		int tyi = static_cast<int>(floor(ty));
		if (txi >= 0 && txi < static_cast<int>(m_width)
			&& tyi >= 0 && tyi < static_cast<int>(m_height))
		{
			int dec = txi + tyi*static_cast<int>(m_width);
			uint8_t col = 1;

			if (!m_meshIsClosed)
			{
				// uint8_t c1 = std::max(m_snapC[dec], m_snapC[dec<<2+4]);
				// uint8_t c2 = std::max(m_snapC[dec<<2+sx4], m_snapC[dec<<2+sx4+4]);
				const uint8_t* pix = m_snapC.data() + (static_cast<size_t>(dec) << 2);
				uint8_t c1 = std::max(pix[0], pix[4]);
				pix += sx4;
				uint8_t c2 = std::max(pix[0], pix[4]);
				col = std::max(c1, c2);
			}

			if (col != 0)
			{
				if (tz < static_cast<double>(m_snapZ[dec]))
				{
					assert(i < visibilityCount.size());
					++visibilityCount[i];
					++count;
				}
			}
		}
	}

	assert(m_glContext);
	m_glContext->doneCurrent();

	return count;
}
