//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Always first
//#include "ccIncludeGL.h"

#include "ccOctreeProxy.h"

//Local
//#include "ccCameraSensor.h"
//#include "ccNormalVectors.h"
//#include "ccBox.h"

//CCLib
//#include <ScalarFieldTools.h>
//#include <RayAndBox.h>

ccOctreeProxy::ccOctreeProxy(	ccOctree::Shared octree/*=ccOctree::Shared(0)*/,
								QString name/*="Octree"*/)
	: ccHObject(name)
	, m_octree(octree)
{
	setVisible(false);
	lockVisibility(false);
}

ccBBox ccOctreeProxy::getOwnBB(bool withGLFeatures/*=false*/)
{
	if (!m_octree)
	{
		assert(false);
		return ccBBox();
	}
	
	return withGLFeatures ? m_octree->getSquareBB() : m_octree->getPointsBB();
}

void ccOctreeProxy::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!m_octree)
	{
		assert(false);
		return;
	}

	if (!MACRO_Draw3D(context))
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	bool pushName = MACRO_DrawEntityNames(context);

	if (pushName)
	{
		//not fast at all!
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	m_octree->draw(context);

	if (pushName)
	{
		glFunc->glPopName();
	}
}
