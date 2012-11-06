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

#ifndef PCV_CONTEXT_HEADER
#define PCV_CONTEXT_HEADER

//CCLib
#include <GenericCloud.h>
#include <GenericMesh.h>

//Qt
#include <QGLPixelBuffer>

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
		/** \param OpenGL render context width (pixels)
			\param OpenGL render context height (pixels)
			\param associated cloud (or mesh vertices)
			\param associated mesh (if any)
			\param whether mesh is closed (faster) or not (need more memory)
			\return initialization success
		**/
		bool init(unsigned W,
					unsigned H,
					CCLib::GenericCloud* cloud,
					CCLib::GenericMesh* mesh=0,
					bool closedMesh=true);

		//défini la direction de visée
		void setViewDirection(const float* V);

		//! Increments counter for points viewed in the current display orientation (see setViewDirection)
		/** \param pixelsSeen array of the same size as the number of vertices
			\return number of vertices seen
		**/
		int GLAccumPixel(int* pixelsSeen);

	private:

		void glInit();
		void drawEntity();
		void associateToEntity(CCLib::GenericCloud* aCloud, CCLib::GenericMesh* aMesh=NULL);

		//! Displayed entity (cloud or mesh vertices)
		CCLib::GenericCloud* m_vertices;

		//! Displayed entity (mesh - optional)
		CCLib::GenericMesh* m_mesh;

		//zoom courant
		float m_zoom;
		//translation vers le centre de l'entitée à afficher
		CCVector3 m_viewCenter;

		//associated pixel buffer
        QGLPixelBuffer* m_pixBuffer;

		//! Pixel buffer width (pixels)
		unsigned m_width;
		//! Pixel buffer height (pixels)
		unsigned m_height;

		//! Current model view matrix
		float m_viewMat[16];

		//! Depth buffer
		float* m_snapZ;
		//! Color buffer
		uchar* m_snapC;

		//! Whether displayed mesh is closed or not
		bool m_meshIsClosed;

};

#endif
