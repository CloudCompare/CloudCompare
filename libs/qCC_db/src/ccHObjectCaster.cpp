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
#include "ccCircle.h"
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
#include "ccCoordinateSystem.h"

/*** helpers ***/

ccPointCloud* ccHObjectCaster::ToPointCloud(ccHObject* obj, bool* lockedVertices /*= nullptr*/)
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

ccGenericPointCloud* ccHObjectCaster::ToGenericPointCloud(ccHObject* obj, bool* lockedVertices/*=nullptr*/)
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
		else if (obj->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = static_cast<ccPolyline*>(obj);
			ccGenericPointCloud* vertices = dynamic_cast<ccGenericPointCloud*>(poly->getAssociatedCloud());
			if (lockedVertices)
			{
				*lockedVertices = true;
			}
			return vertices;
		}
	}

	return nullptr;
}

ccShiftedObject* ccHObjectCaster::ToShifted(ccHObject* obj, bool* lockedVertices/*=nullptr*/)
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
	return (obj && obj->isKindOf(CC_TYPES::POLY_LINE) ? static_cast<ccPolyline*>(obj) : nullptr);
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

ccCoordinateSystem* ccHObjectCaster::ToCoordinateSystem(ccHObject* obj)
{
	return (obj && obj->isKindOf(CC_TYPES::COORDINATESYSTEM) ? static_cast<ccCoordinateSystem*>(obj) : nullptr);
}

ccCircle* ccHObjectCaster::ToCircle(ccHObject* obj)
{
	return (obj && obj->isKindOf(CC_TYPES::CIRCLE) ? static_cast<ccCircle*>(obj) : nullptr);
}

bool ccHObjectCaster::CloneChildren(const ccHObject* sourceEntity,
									ccHObject* destEntity,
									std::vector<int>* newPointOrTriangleIndex/*=nullptr*/,
									const ccHObject* sourceEntityProxy/*=nullptr*/,
									ccHObject* destEntityProxy/*=nullptr*/)
{
	if (!sourceEntity || !destEntity)
	{
		assert(false);
		return false;
	}

	bool sourceIsCloud = sourceEntity->isKindOf(CC_TYPES::POINT_CLOUD);
	bool destIsCloud = destEntity->isKindOf(CC_TYPES::POINT_CLOUD);
	bool sourceAndDestAreCloud = sourceIsCloud && destIsCloud;

	bool sourceIsMesh = sourceEntity->isKindOf(CC_TYPES::MESH);
	bool destIsMesh = destEntity->isKindOf(CC_TYPES::MESH);
	bool sourceAndDestAreMeshes = sourceIsMesh && destIsMesh;

	unsigned numberOfPointOrTriangle = 0;
	if (sourceIsCloud)
		numberOfPointOrTriangle = static_cast<const ccGenericPointCloud*>(sourceEntity)->size();
	else if (sourceIsMesh)
		numberOfPointOrTriangle = static_cast<const ccGenericMesh*>(sourceEntity)->size();

	if (newPointOrTriangleIndex)
	{
		if (sourceEntity == destEntity)
		{
			ccLog::Warning("[ccHObjectCaster::CloneChildren] Providing a point/triangle correspondance map while the source and destination entities are the same");
			// we can live with that...
		}

		if (!sourceAndDestAreCloud && !sourceAndDestAreMeshes)
		{
			ccLog::Warning("[ccHObjectCaster::CloneChildren] A point/triangle correspondance map can only work between 2 entities of the same type");
			return false;
		}

		if (newPointOrTriangleIndex->size() != numberOfPointOrTriangle)
		{
			ccLog::Warning("[ccHObjectCaster::CloneChildren] Mismatch between the point/triangle correspondance map and the source entity size");
			return false;
		}
	}

	QMap<ccCameraSensor*, ccCameraSensor*> clonedCameraSensors;

	const ccHObject* currentSourceEntity = (sourceEntityProxy ? sourceEntityProxy : sourceEntity);
	ccHObject* currentDestEntity = (destEntityProxy ? destEntityProxy : destEntity);

	// for each child
	for (unsigned i = 0; i < currentSourceEntity->getChildrenNumber(); ++i)
	{
		ccHObject* child = currentSourceEntity->getChild(i);
		
		switch (child->getClassID())
		{
		// 2D Label
		case CC_TYPES::LABEL_2D:
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(child);
			
			// check if we can keep this label
			bool keepThisLabel = true;
			if (newPointOrTriangleIndex)
			{
				for (unsigned i = 0; i < label->size(); ++i)
				{
					const cc2DLabel::PickedPoint& pp = label->getPickedPoint(i);
					if (pp.entity() == sourceEntity && newPointOrTriangleIndex->at(pp.index) < 0)
					{
						// this label relies on a point or triangle that has no correspondance in the destination entity
						keepThisLabel = false;
						break;
					}
				}
			}

			if (keepThisLabel)
			{
				cc2DLabel* clonedLabel = new cc2DLabel(*label, false);

				for (unsigned i = 0; i < label->size(); ++i)
				{
					cc2DLabel::PickedPoint pp = label->getPickedPoint(i);
					if (pp.entity() == sourceEntity)
					{
						if (sourceIsCloud)
							pp._cloud = static_cast<ccGenericPointCloud*>(destEntity);
						else
							pp._mesh = static_cast<ccGenericMesh*>(destEntity);
						
						if (newPointOrTriangleIndex)
						{
							pp.index = static_cast<unsigned>(newPointOrTriangleIndex->at(pp.index)); // we've checked above that it's >= 0
						}
					}
					clonedLabel->addPickedPoint(pp);
				}
				clonedLabel->setName(label->getName()); //the label name is overridden by calls to addPickedPoint
				
				currentDestEntity->addChild(clonedLabel);
			}
		}
		break;

		// Image
		case CC_TYPES::IMAGE:
		{
			ccImage* image = static_cast<ccImage*>(child);
			ccImage* clonedImage = new ccImage(*image, false);

			ccCameraSensor* camSensor = image->getAssociatedSensor();
			if (camSensor)
			{
				if (clonedCameraSensors.contains(camSensor))
				{
					// if we have already cloned the sensor on which this image depends,
					// we can simply update the link
					clonedImage->setAssociatedSensor(clonedCameraSensors[camSensor]);
				}
				else
				{
					// else we have to clone the sensor
					ccCameraSensor* clonedCamSensor = new ccCameraSensor(*camSensor);
					clonedCameraSensors.insert(camSensor, clonedCamSensor);

					clonedImage->setAssociatedSensor(clonedCamSensor);
					clonedImage->addChild(clonedCamSensor);
				}
			}

			currentDestEntity->addChild(clonedImage);
		}
		break;

		// Camera sensor
		case CC_TYPES::CAMERA_SENSOR:
		{
			ccCameraSensor* camSensor = static_cast<ccCameraSensor*>(child);
			ccCameraSensor* clonedCamSensor = new ccCameraSensor(*camSensor);
			clonedCameraSensors.insert(camSensor, clonedCamSensor);

			currentDestEntity->addChild(clonedCamSensor);
		}
		break;

		// GBL sensor
		case CC_TYPES::GBL_SENSOR:
		{
			ccGBLSensor* gblSensor = static_cast<ccGBLSensor*>(child);
			ccGBLSensor* clonedGBLSensor = new ccGBLSensor(*gblSensor, false);

			currentDestEntity->addChild(clonedGBLSensor);
		}
		break;

		// 2D Viewport object
		case CC_TYPES::VIEWPORT_2D_OBJECT:
		{
			cc2DViewportObject* viewportObject = static_cast<cc2DViewportObject*>(child);;
			cc2DViewportObject* clonedViewportObject = new cc2DViewportObject(*viewportObject);
			
			currentDestEntity->addChild(clonedViewportObject);
		}
		break;

		// 2D Viewport label
		case CC_TYPES::VIEWPORT_2D_LABEL:
		{
			cc2DViewportLabel* viewportLabel = static_cast<cc2DViewportLabel*>(child);;
			cc2DViewportLabel* clonedViewportLabel = new cc2DViewportLabel(*viewportLabel);

			currentDestEntity->addChild(clonedViewportLabel);
		}
		break;

		// Groups
		case CC_TYPES::HIERARCHY_OBJECT:
		{
			ccHObject* newGroup = new ccHObject(*child);
			// start (or proceed with) the recursion
			if (CloneChildren(sourceEntity, destEntity, newPointOrTriangleIndex, child, newGroup))
			{
				if (newGroup->getChildrenNumber() != 0)
				{
					currentDestEntity->addChild(newGroup);
				}
				else
				{
					// empty group, no need to keep it
					delete newGroup;
					newGroup = nullptr;
				}
			}
			else
			{
				// something bad happened
				return false;
			}
		}
		break;

		// Meshes
		case CC_TYPES::MESH:
		{
			// TODO
		}
		break;

		default:
		{
			// nothing to do
		}
		break;
		}
	}

	return true;
}
