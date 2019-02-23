//##########################################################################
//#                                                                        #
//#                                PCV                                     #
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

#ifndef PCV_CONTEXT_HEADER
#define PCV_CONTEXT_HEADER

//CCLib
#include <GenericCloud.h>
#include <GenericMesh.h>

//system
#include <vector>

class QGLPixelBuffer;

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
					CCLib::GenericCloud* cloud,
					CCLib::GenericMesh* mesh = nullptr,
					bool closedMesh = true);

		//! Set the viewing directions
		void setViewDirection(const CCVector3& V);

		//! Increments the visibility counter for points viewed in the current pass (see setViewDirection)
		/** \param visibilityCount per-vertex visibility count (same size as the number of vertices)
			\return number of vertices seen during this pass
		**/
		int GLAccumPixel(std::vector<int>& visibilityCount);

	protected:
		void glInit();
		void drawEntity();
		void associateToEntity(CCLib::GenericCloud* cloud, CCLib::GenericMesh* mesh = nullptr);

		//! Displayed entity (cloud or mesh vertices)
		CCLib::GenericCloud* m_vertices;

		//! Displayed entity (mesh - optional)
		CCLib::GenericMesh* m_mesh;

		//zoom courant
		PointCoordinateType m_zoom;
		//translation vers le centre de l'entitee a afficher
		CCVector3 m_viewCenter;

		//associated pixel buffer
		QGLPixelBuffer* m_pixBuffer;

		//! Pixel buffer width (pixels)
		unsigned m_width;
		//! Pixel buffer height (pixels)
		unsigned m_height;

		//! Model view matrix size (OpenGL)
		/** \warning Never pass a 'constant initializer' by reference
		**/
		static const unsigned OPENGL_MATRIX_SIZE = 16;

		//! Current model view matrix
		float m_viewMat[OPENGL_MATRIX_SIZE];

		//! Depth buffer
		float* m_snapZ;
		//! Color buffer
		unsigned char* m_snapC;

		//! Whether displayed mesh is closed or not
		bool m_meshIsClosed;
};

#endif
