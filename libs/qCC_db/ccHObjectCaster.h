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

#ifndef CC_HIERARCHY_OBJECT_CASTER_HEADER
#define CC_HIERARCHY_OBJECT_CASTER_HEADER

class ccHObject;
class ccGenericPointCloud;
class ccPointCloud;
class ccGenericMesh;
class ccMesh;
class ccMeshGroup;
class ccGenericPrimitive;
class ccOctree;
class ccSensor;
class ccGBLSensor;
class ccImage;
class ccCalibratedImage;
class cc2DLabel;
class cc2DViewportObject;
class cc2DViewportLabel;

//! Useful class to (try to) statically cast a basic ccHObject to a given type
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccHObjectCaster
#else
class ccHObjectCaster
#endif
{
    public:

        //! converts current object to 'equivalent' ccPointCloud
        /** Waring: if a mesh is passed, this method returns its vertices.
			\param obj ccHObject to dynamically cast to a ccPointCloud object
			\param the caller can be warned if the returned cloud corresponds to locked vertices
        **/
        static ccPointCloud* ToPointCloud(ccHObject* obj, bool* isLockedVertices = 0);

        //! converts current object to 'equivalent' ccGenericPointCloud
        /** Waring: if a mesh is passed, this method returns its vertices.
        **/
        static ccGenericPointCloud* ToGenericPointCloud(ccHObject* obj, bool* isLockedVertices = 0);

        //! converts current object to ccGenericMesh (if possible)
        static ccGenericMesh* ToGenericMesh(ccHObject* obj);

        //! converts current object to ccMesh (if possible)
        static ccMesh* ToMesh(ccHObject* obj);

        //! converts current object to ccMeshGroup (if possible)
        static ccMeshGroup* ToMeshGroup(ccHObject* obj);

        //! converts current object to ccGenericPrimitive (if possible)
		static ccGenericPrimitive* ToPrimitive(ccHObject* obj);

        //! converts current object to ccOctree (if possible)
        static ccOctree* ToOctree(ccHObject* obj);

        //! converts current object to ccSensor (if possible)
        static ccSensor* ToSensor(ccHObject* obj);

        //! converts current object to ccGBLSensor (if possible)
        static ccGBLSensor* ToGBLSensor(ccHObject* obj);

        //! converts current object to ccImage (if possible)
        static ccImage* ToImage(ccHObject* obj);

        //! converts current object to ccCalibratedImage (if possible)
        static ccCalibratedImage* ToCalibratedImage(ccHObject* obj);

		//! converts current object to cc2DLabel (if possible)
        static cc2DLabel* To2DLabel(ccHObject* obj);

		//! converts current object to cc2DViewportLabel (if possible)
        static cc2DViewportLabel* To2DViewportLabel(ccHObject* obj);

		//! converts current object to cc2DViewportObject (if possible)
        static cc2DViewportObject* To2DViewportObject(ccHObject* obj);
		
};

#endif
