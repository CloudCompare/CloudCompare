//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2251                                                              $
//$LastChangedDate:: 2012-10-08 23:56:41 +0200 (lun., 08 oct. 2012)        $
//**************************************************************************
//

#include "ccHObjectCaster.h"

//types
#include "ccHObject.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccMesh.h"
#include "ccMeshGroup.h"
#include "ccOctree.h"
#include "ccImage.h"
#include "ccCalibratedImage.h"
#include "ccGBLSensor.h"
#include "cc2DLabel.h"
#include "cc2DViewportLabel.h"
#include "cc2DViewportObject.h"
#include "ccGenericPrimitive.h"

/*** helpers ***/

ccPointCloud* ccHObjectCaster::ToPointCloud(ccHObject* obj, bool* lockedVertices /*= 0*/)
{
	if (lockedVertices)
		*lockedVertices = false;

	if (!obj)
        return 0;

    if (obj->isA(CC_POINT_CLOUD))
        return static_cast<ccPointCloud*>(obj);

    if (obj->isKindOf(CC_MESH))
    {
        ccGenericPointCloud* vertices = static_cast<ccGenericMesh*>(obj)->getAssociatedCloud();
        if (vertices && vertices->isA(CC_POINT_CLOUD))
        {
			if (lockedVertices)
				*lockedVertices = vertices->isLocked();
            return static_cast<ccPointCloud*>(vertices);
        }
    }

    return 0;
}

ccGenericPointCloud* ccHObjectCaster::ToGenericPointCloud(ccHObject* obj, bool* lockedVertices /*= 0*/)
{
	if (lockedVertices)
		*lockedVertices = false;

    if (!obj)
        return 0;

    if (obj->isKindOf(CC_POINT_CLOUD))
        return static_cast<ccGenericPointCloud*>(obj);

    if (obj->isKindOf(CC_MESH))
    {
        ccGenericPointCloud* vertices = static_cast<ccGenericMesh*>(obj)->getAssociatedCloud();
        if (vertices)
        {
			if (lockedVertices)
				*lockedVertices = vertices->isLocked();
            return static_cast<ccGenericPointCloud*>(vertices);
        }
    }

    return 0;
}

ccGenericMesh* ccHObjectCaster::ToGenericMesh(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_MESH) ? static_cast<ccGenericMesh*>(obj) : 0;
}

ccMesh* ccHObjectCaster::ToMesh(ccHObject* obj)
{
	return obj && obj->isA(CC_MESH) ? static_cast<ccMesh*>(obj) : 0;
}

ccMeshGroup* ccHObjectCaster::ToMeshGroup(ccHObject* obj)
{
	return obj && obj->isA(CC_MESH_GROUP) ? static_cast<ccMeshGroup*>(obj) : 0;
}

ccGenericPrimitive* ccHObjectCaster::ToPrimitive(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_PRIMITIVE) ? static_cast<ccGenericPrimitive*>(obj) : 0;
}

ccOctree* ccHObjectCaster::ToOctree(ccHObject* obj)
{
	return obj && obj->isA(CC_POINT_OCTREE) ? static_cast<ccOctree*>(obj) : 0;
}

ccSensor* ccHObjectCaster::ToSensor(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_SENSOR) ? static_cast<ccSensor*>(obj) : 0;
}

ccGBLSensor* ccHObjectCaster::ToGBLSensor(ccHObject* obj)
{
	return obj && obj->isA(CC_GBL_SENSOR) ? static_cast<ccGBLSensor*>(obj) : 0;
}

ccImage* ccHObjectCaster::ToImage(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_IMAGE) ? static_cast<ccImage*>(obj) : 0;
}

ccCalibratedImage* ccHObjectCaster::ToCalibratedImage(ccHObject* obj)
{
	return obj && obj->isA(CC_CALIBRATED_IMAGE) ? static_cast<ccCalibratedImage*>(obj) : 0;
}

cc2DLabel* ccHObjectCaster::To2DLabel(ccHObject* obj)
{
	return obj && obj->isA(CC_2D_LABEL) ? static_cast<cc2DLabel*>(obj) : 0;
}

cc2DViewportLabel* ccHObjectCaster::To2DViewportLabel(ccHObject* obj)
{
	return obj && obj->isA(CC_2D_VIEWPORT_LABEL) ? static_cast<cc2DViewportLabel*>(obj) : 0;
}

cc2DViewportObject* ccHObjectCaster::To2DViewportObject(ccHObject* obj)
{
	return obj && obj->isKindOf(CC_2D_VIEWPORT_OBJECT) ? static_cast<cc2DViewportObject*>(obj) : 0;
}
