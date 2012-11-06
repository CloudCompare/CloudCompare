//##########################################################################
//#                                                                        #
//#               CLOUDCOMPARE WRAPPER: PoissonReconLib                    #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef CC_POISSON_RECON_LIB_WRAPPER
#define CC_POISSON_RECON_LIB_WRAPPER

#include "../Src/Geometry.h"

//! Wrapper to use PoissonRecon (Kazdan et. al) as a library
class PoissonReconLib
{
public:

	//! Some information on a (successful) reconstruction
	struct PoissonReconResultInfo
	{
		float center[3];
		float scale;
	};

	//! Launches reconstruction process
	/** \param count point cloud size
		\param P array of points (3 floats per point)
		\param N array of normals (3 floats per point)
		\param outMesh output mesh (if successful)
		\param depth reconstruction octree depth
		\param info optional info
		\return reconstruction success
	**/
	static bool reconstruct(unsigned count, const float* P, const float* N, CoredVectorMeshData& outMesh, int depth=8, PoissonReconResultInfo* pInfo=0);
};

#endif // CC_POISSON_RECON_LIB_WRAPPER
