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

#ifndef MESH_ILLUMINATION_HEADER
#define MESH_ILLUMINATION_HEADER

//CCLib
#include <GenericCloud.h>
#include <GenericIndexedMesh.h>
#include <GenericProgressCallback.h>

//Qt
#include <QString>

//System
#include <vector>

//! PCV (Portion de Ciel Visible) algorithm
/** "Ambient Occlusion" in english!
**/
class PCV
{
public:
	//! Simulates global illumination on a cloud (or a mesh) with OpenGL - shortcut version
	/** Computes per-vertex illumination intensity as a scalar field.
		\param numberOfRays (approxiamate) number of rays to generate
		\param mode360 whether light rays should be generated on the half superior sphere (false) or the whole sphere (true)
		\param vertices vertices (eventually corresponding to a mesh - see below) to englight
		\param mesh optional mesh structure associated to the vertices
		\param meshIsClosed if a mesh is passed as argument (see above), specifies if the mesh surface is closed (enables optimization)
		\param width width  of the OpenGL context used to simulate illumination
		\param height height of the OpenGL context used to simulate illumination
		\param progressCb optional progress bar (optional)
		\param entityName entity name (optional)
		\return number of 'light' directions actually used (or a value <0 if an error occurred)
	**/
	static int Launch(	unsigned numberOfRays,
						CCLib::GenericCloud* vertices,
						CCLib::GenericMesh* mesh = nullptr,
						bool meshIsClosed = false,
						bool mode360 = true,
						unsigned width = 1024,
						unsigned height = 1024,
						CCLib::GenericProgressCallback* progressCb = nullptr,
						const QString& entityName = QString());

	//! Simulates global illumination on a cloud (or a mesh) with OpenGL
	/** Computes per-vertex illumination intensity as a scalar field.
		\param rays light directions that will be used to compute global illumination
		\param vertices vertices (eventually corresponding to a mesh - see below) to englight
		\param mesh optional mesh structure associated to the vertices
		\param meshIsClosed if a mesh is passed as argument (see above), specifies if the mesh surface is closed (enables optimization)
		\param width width  of the OpenGL context used to simulate illumination
		\param height height of the OpenGL context used to simulate illumination
		\param progressCb optional progress bar (optional)
		\param entityName entity name (optional)
		\return success
	**/
	static bool Launch(	const std::vector<CCVector3>& rays,
						CCLib::GenericCloud* vertices,
						CCLib::GenericMesh* mesh = nullptr,
						bool meshIsClosed = false,
						unsigned width = 1024,
						unsigned height = 1024,
						CCLib::GenericProgressCallback* progressCb = nullptr,
						const QString& entityName = QString());

	//! Generates a given number of rays
	static bool GenerateRays(	unsigned numberOfRays,
								std::vector<CCVector3>& rays,
								bool mode360 = true);
};

#endif
