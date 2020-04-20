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
#include "cc2DLabel.h"
#include "cc2DViewportLabel.h"
#include "cc2DViewportObject.h"
#include "ccCameraSensor.h"
#include "ccCone.h"
#include "ccCylinder.h"
#include "ccDish.h"
#include "ccExtru.h"
#include "ccFacet.h"
#include "ccGBLSensor.h"
#include "ccGenericMesh.h"
#include "ccGenericPointCloud.h"
#include "ccGenericPrimitive.h"
#include "ccHObject.h"
#include "ccImage.h"
#include "ccKdTree.h"
#include "ccMesh.h"
#include "ccOctree.h"
#include "ccOctreeProxy.h"
#include "ccPlane.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccShiftedObject.h"
#include "ccSphere.h"
#include "ccSubMesh.h"
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

	return nullptr;
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

	return nullptr;
}

ccShiftedObject* ccHObjectCaster::ToShifted(ccHObject* obj, bool* lockedVertices /*= 0*/)
{
	ccGenericPointCloud* cloud = ToGenericPointCloud(obj, lockedVertices);
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

	return nullptr;
}

ccGenericMesh* ccHObjectCaster::ToGenericMesh(ccHObject* obj)
{
	return (obj && obj->isKindOf(CC_TYPES::MESH) ? static_cast<ccGenericMesh*>(obj) : nullptr);
}

ccMesh* ccHObjectCaster::ToMesh(ccHObject* obj)
{
	return (obj && (obj->isA(CC_TYPES::MESH) || obj->isKindOf(CC_TYPES::PRIMITIVE)) ? static_cast<ccMesh*>(obj) : nullptr);
}

ccSubMesh* ccHObjectCaster::ToSubMesh(ccHObject* obj)
{
	return (obj && obj->isA(CC_TYPES::SUB_MESH) ? static_cast<ccSubMesh*>(obj) : nullptr);
}

ccPolyline* ccHObjectCaster::ToPolyline(ccHObject* obj)
{
	return (obj && obj->isA(CC_TYPES::POLY_LINE) ? static_cast<ccPolyline*>(obj) : nullptr);
}

ccFacet* ccHObjectCaster::ToFacet(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::FACET) ? static_cast<ccFacet*>(obj) : nullptr;
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
	return nullptr;
}

ccGenericPrimitive* ccHObjectCaster::ToPrimitive(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::PRIMITIVE) ? static_cast<ccGenericPrimitive*>(obj) : nullptr;
}

ccSphere*	ccHObjectCaster::ToSphere(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::SPHERE) ? static_cast<ccSphere*>(obj) : nullptr;
}

ccCylinder*	ccHObjectCaster::ToCylinder(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::CYLINDER) ? static_cast<ccCylinder*>(obj) : nullptr;
}

ccCone*		ccHObjectCaster::ToCone(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::CONE) ? static_cast<ccCone*>(obj) : nullptr;
}

ccPlane*	ccHObjectCaster::ToPlane(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::PLANE) ? static_cast<ccPlane*>(obj) : nullptr;
}

ccDish*		ccHObjectCaster::ToDish(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::DISH) ? static_cast<ccDish*>(obj) : nullptr;
}

ccExtru*	ccHObjectCaster::ToExtru(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::EXTRU) ? static_cast<ccExtru*>(obj) : nullptr;
}

ccTorus*	ccHObjectCaster::ToTorus(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::TORUS) ? static_cast<ccTorus*>(obj) : nullptr;
}

ccOctreeProxy* ccHObjectCaster::ToOctreeProxy(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::POINT_OCTREE) ? static_cast<ccOctreeProxy*>(obj) : nullptr;
}

ccOctree* ccHObjectCaster::ToOctree(ccHObject* obj)
{
	ccOctreeProxy* proxy = ToOctreeProxy(obj);
	return proxy ? proxy->getOctree().data() : nullptr;
}

ccKdTree* ccHObjectCaster::ToKdTree(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::POINT_KDTREE) ? static_cast<ccKdTree*>(obj) : nullptr;
}

ccSensor* ccHObjectCaster::ToSensor(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::SENSOR) ? static_cast<ccSensor*>(obj) : nullptr;
}

ccGBLSensor* ccHObjectCaster::ToGBLSensor(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::GBL_SENSOR) ? static_cast<ccGBLSensor*>(obj) : nullptr;
}

ccCameraSensor* ccHObjectCaster::ToCameraSensor(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::CAMERA_SENSOR) ? static_cast<ccCameraSensor*>(obj) : nullptr;
}

ccImage* ccHObjectCaster::ToImage(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::IMAGE) ? static_cast<ccImage*>(obj) : nullptr;
}

cc2DLabel* ccHObjectCaster::To2DLabel(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::LABEL_2D) ? static_cast<cc2DLabel*>(obj) : nullptr;
}

cc2DViewportLabel* ccHObjectCaster::To2DViewportLabel(ccHObject* obj)
{
	return obj && obj->isA(CC_TYPES::VIEWPORT_2D_LABEL) ? static_cast<cc2DViewportLabel*>(obj) : nullptr;
}

cc2DViewportObject* ccHObjectCaster::To2DViewportObject(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT) ? static_cast<cc2DViewportObject*>(obj) : nullptr;
}

ccIndexedTransformationBuffer* ccHObjectCaster::ToTransBuffer(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_TYPES::TRANS_BUFFER) ? static_cast<ccIndexedTransformationBuffer*>(obj) : nullptr;
}
