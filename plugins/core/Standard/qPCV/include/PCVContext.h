#pragma once

//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN                               #
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

//CCCoreLib
#include <GenericCloud.h>
#include <GenericMesh.h>

//system
#include <array>
#include <vector>

class QSurface;
class QOpenGLBuffer;
class QOpenGLContext;

//! PCV (Portion de Ciel Visible / Ambiant Illumination) OpenGL context
/** Similar to Cignoni's ShadeVis
**/
class PCVContext
{
	public:
		//! Default constructor
		PCVContext();

		//! Destructor
		virtual ~PCVContext();

		//! Initialization
		/** \param W OpenGL render context width (pixels)
			\param H OpenGL render context height (pixels)
			\param cloud associated cloud (or mesh vertices)
			\param mesh associated mesh (if any)
			\param closedMesh whether mesh is closed (faster) or not (need more memory)
			\return initialization success
		**/
		bool init(	unsigned W,
					unsigned H,
					CCCoreLib::GenericCloud* cloud,
					CCCoreLib::GenericMesh* mesh = nullptr,
					bool closedMesh = true);

		//! Increments the visibility counter for points viewed in the current pass (see setViewDirection)
		/** \param visibilityCount per-vertex visibility count (same size as the number of vertices)
			\return number of vertices seen during this pass
		**/
		int glAccumPixel(std::vector<int>& visibilityCount, const CCVector3d& viewDir);

		// Makes the OpenGL context current
		bool makeCurrent();

	protected:
		bool glInit();
		void drawEntity();
		void associateToEntity(CCCoreLib::GenericCloud* cloud, CCCoreLib::GenericMesh* mesh = nullptr);

		//! Displayed entity (cloud or mesh vertices)
		CCCoreLib::GenericCloud* m_vertices;

		//! Displayed entity (mesh - optional)
		CCCoreLib::GenericMesh* m_mesh;

		//! Entity bounding-box diagonal
		PointCoordinateType m_diagonal;
		//! Translation to the entity center
		CCVector3 m_viewCenter;

		//! OpenGL (offline) surface
		QSurface* m_glSurface;

		//! OpenGL context
		QOpenGLContext* m_glContext;

		//! Associated pixel buffer
		QOpenGLBuffer* m_pixBuffer;

		//! Pixel buffer width (pixels)
		unsigned m_width;
		//! Pixel buffer height (pixels)
		unsigned m_height;

		//! Model view matrix size (OpenGL)
		/** \warning Never pass a 'constant initializer' by reference
		**/
		static const unsigned OPENGL_MATRIX_SIZE = 16;

		//! Depth buffer
		std::vector<float> m_snapZ;
		//! Color buffer
		std::vector<uint8_t> m_snapC;

		//! Whether displayed mesh is closed or not
		bool m_meshIsClosed;
};
