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

#include "ccHObjectCaster.h"

//types
#include "ccHObject.h"
#include "ccShiftedObject.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccMesh.h"
#include "ccSubMesh.h"
#include "ccFacet.h"
#include "ccPolyline.h"
#include "ccOctree.h"
#include "ccOctreeProxy.h"
#include "ccKdTree.h"
#include "ccImage.h"
#include "ccGBLSensor.h"
#include "ccCameraSensor.h"
#include "cc2DLabel.h"
#include "cc2DViewportLabel.h"
#include "cc2DViewportObject.h"
#include "ccGenericPrimitive.h"
#include "ccSphere.h"
#include "ccCylinder.h"
#include "ccCone.h"
#include "ccPlane.h"
#include "ccDish.h"
#include "ccExtru.h"
#include "ccTorus.h"

/*** helpers ***/

ccPointCloud* ccHObjectCaster::ToPointCloud(ccHObject* obj, bool* lockedVertices /*= 0*/)
{
	if (lockedVertices)
	{
		*lockedVertices = false;
	}

	if (obj)
	{
		if (obj->isA(CC_TYPES::POINT_CLOUD))
		{
			return static_cast<ccPointCloud*>(obj);
		}
		else if (obj->isKindOf(CC_TYPES::MESH))
		{
			ccGenericPointCloud* vertices = static_cast<ccGenericMesh*>(obj)->getAssociatedCloud();
			if (vertices)
			{
				if (!obj->isA(CC_TYPES::MESH) && lockedVertices) //no need to 'lock' the vertices if the user works on the parent mesh
				{
					*lockedVertices = vertices->isLocked();
				}
				return ccHObjectCaster::ToPointCloud(vertices);
			}
		}
	}

	return 0;
}

ccGenericPointCloud* ccHObjectCaster::ToGenericPointCloud(ccHObject* obj, bool* lockedVertices /*= 0*/)
{
	if (lockedVertices)
	{
		*lockedVertices = false;
	}

	if (obj)
	{
		if (obj->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			return static_cast<ccGenericPointCloud*>(obj);
		}
		else if (obj->isKindOf(CC_TYPES::MESH))
		{
			ccGenericPointCloud* vertices = static_cast<ccGenericMesh*>(obj)->getAssociatedCloud();
			if (vertices)
			{
				if (!obj->isA(CC_TYPES::MESH) && lockedVertices) //no need to 'lock' the vertices if the user works on the parent mesh
				{
					*lockedVertices = vertices->isLocked();
				}
				return vertices;
			}
		}
	}

	return 0;
}

ccShiftedObject* ccHObjectCaster::ToShifted(ccHObject* obj, bool* lockedVertices /*= 0*/)
{
	ccGenericPointCloud* cloud = ToGenericPointCloud(obj, lockedVertices /*= 0*/);
	if (cloud)
		return cloud;

	if (obj && obj->isKindOf(CC_TYPES::POLY_LINE))
	{
		if (lockedVertices)
		{
			*lockedVertices = false;
		}
		return static_cast<ccPolyline*>(obj);
	}

	return 0;
}

ccGenericMesh* ccHObjectCaster::ToGenericMesh(ccHObject* obj)
{
	return (obj && obj->isKindOf(CC_TYPES::MESH) ? static_cast<ccGenericMesh*>(obj) : 0);
}

ccMesh* ccHObjectCaster::ToMesh(ccHObject* obj)
{
	return (obj && (obj->isA(CC_TYPES::MESH) || obj->isKindOf(CC_TYPES::PRIMITIVE)) ? static_cast<ccMesh*>(obj) : 0);
}

ccSubMesh* ccHObjectCaster::ToSubMesh(ccHObject* obj)
{
	return (obj && obj->isA(CC_TYPES::SUB_MESH) ? static_cast<ccSubMesh*>(obj) : 0);
}

ccPolyline* ccHObjectCaster::ToPolyline(ccHObject* obj)
{
	return (obj && obj->isA(CC_TYPES::POLY_LINE) ? static_cast<ccPolyline*>(obj) : 0);
}

ccFacet* ccHObjectCaster::ToFacet(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::FACET) ? static_cast<ccFacet*>(obj) : 0;
}

ccPlanarEntityInterface* ccHObjectCaster::ToPlanarEntity(ccHObject* obj)
{
	if (obj)
	{
		if (obj->isA(CC_TYPES::FACET))
		{
			return static_cast<ccFacet*>(obj);
		}
		else if (obj->isA(CC_TYPES::PLANE))
		{
			return static_cast<ccPlane*>(obj);
		}
	}
	return 0;
}

ccGenericPrimitive* ccHObjectCaster::ToPrimitive(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::PRIMITIVE) ? static_cast<ccGenericPrimitive*>(obj) : 0;
}

ccSphere*	ccHObjectCaster::ToSphere(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::SPHERE) ? static_cast<ccSphere*>(obj) : 0;
}

ccCylinder*	ccHObjectCaster::ToCylinder(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::CYLINDER) ? static_cast<ccCylinder*>(obj) : 0;
}

ccCone*		ccHObjectCaster::ToCone(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::CONE) ? static_cast<ccCone*>(obj) : 0;
}

ccPlane*	ccHObjectCaster::ToPlane(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::PLANE) ? static_cast<ccPlane*>(obj) : 0;
}

ccDish*		ccHObjectCaster::ToDish(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::DISH) ? static_cast<ccDish*>(obj) : 0;
}

ccExtru*	ccHObjectCaster::ToExtru(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::EXTRU) ? static_cast<ccExtru*>(obj) : 0;
}

ccTorus*	ccHObjectCaster::ToTorus(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::TORUS) ? static_cast<ccTorus*>(obj) : 0;
}

ccOctreeProxy* ccHObjectCaster::ToOctreeProxy(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::POINT_OCTREE) ? static_cast<ccOctreeProxy*>(obj) : 0;
}

ccOctree* ccHObjectCaster::ToOctree(ccHObject* obj)
{
	ccOctreeProxy* proxy = ToOctreeProxy(obj);
	return proxy ? proxy->getOctree().data() : 0;
}

ccKdTree* ccHObjectCaster::ToKdTree(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::POINT_KDTREE) ? static_cast<ccKdTree*>(obj) : 0;
}

ccSensor* ccHObjectCaster::ToSensor(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::SENSOR) ? static_cast<ccSensor*>(obj) : 0;
}

ccGBLSensor* ccHObjectCaster::ToGBLSensor(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::GBL_SENSOR) ? static_cast<ccGBLSensor*>(obj) : 0;
}

ccCameraSensor* ccHObjectCaster::ToCameraSensor(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::CAMERA_SENSOR) ? static_cast<ccCameraSensor*>(obj) : 0;
}

ccImage* ccHObjectCaster::ToImage(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::IMAGE) ? static_cast<ccImage*>(obj) : 0;
}

cc2DLabel* ccHObjectCaster::To2DLabel(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::LABEL_2D) ? static_cast<cc2DLabel*>(obj) : 0;
}

cc2DViewportLabel* ccHObjectCaster::To2DViewportLabel(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::VIEWPORT_2D_LABEL) ? static_cast<cc2DViewportLabel*>(obj) : 0;
}

cc2DViewportObject* ccHObjectCaster::To2DViewportObject(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT) ? static_cast<cc2DViewportObject*>(obj) : 0;
}

ccIndexedTransformationBuffer* ccHObjectCaster::ToTransBuffer(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::TRANS_BUFFER) ? static_cast<ccIndexedTransformationBuffer*>(obj) : 0;
}
