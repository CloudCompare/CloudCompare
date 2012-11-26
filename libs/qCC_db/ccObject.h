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
//$Rev:: 2265                                                              $
//$LastChangedDate:: 2012-10-13 22:22:51 +0200 (sam., 13 oct. 2012)        $
//**************************************************************************
//

#ifndef CC_OBJECT_HEADER
#define CC_OBJECT_HEADER

//Local
#include "ccSerializableObject.h"

//Qt
#include <QString>

//! Object state flag
enum CC_OBJECT_FLAG {
    CC_FATHER_DEPENDANT     =   1,
    CC_ENABLED              =   2,
    CC_LOCKED               =   4,
};

//Bits for object type flags
#define CC_HIERARCH_BIT					0x00000001      //Hierarchical object
#define CC_LEAF_BIT						0x00000002      //Tree leaf (no children)
#define CC_GROUP_BIT					0x00000004      //Group (no data, aggregation only)
#define CC_PRIMITIVE_BIT				0x00000008		//Primitive (sphere, plane, torus, cylinder, etc.)
#define CC_ARRAY_BIT					0x00000010		//Array
#define CC_LABEL_BIT					0x00000020		//2D label
#define CC_VIEWPORT_BIT					0x00000040		//2D viewport
//#define CC_FREE_BIT					0x00000080
#define CC_CLOUD_BIT					0x00000100      //Point Cloud
#define CC_MESH_BIT						0x00000200      //Mesh
#define CC_OCTREE_BIT					0x00000400      //Octree
#define CC_POLYLINE_BIT					0x00000800      //Polyline
#define CC_IMAGE_BIT					0x00001000      //Picture
#define CC_SENSOR_BIT					0x00002000      //Sensor def.
#define CC_PLANE_BIT					0x00004000		//Plane (primitive)
#define CC_SPHERE_BIT					0x00008000		//Sphere (primitive)
#define CC_TORUS_BIT					0x00010000		//Torus (primitive)
#define CC_CYLINDER_BIT					0x00020000		//Cylinder (primitive)
#define CC_CONE_BIT						0x00040000		//Cone (primitive)
#define CC_BOX_BIT						0x00080000		//Box (primitive)
#define CC_DISH_BIT						0x00100000		//Dish (primitive)
#define CC_EXTRU_BIT					0x00200000		//Extrusion (primitive)
//#define CC_FREE_BIT					0x00400000
//#define CC_FREE_BIT					0x00800000
#define CC_MATERIAL_BIT					0x01000000		//Material
//#define CC_FREE_BIT					0x02000000
//#define CC_FREE_BIT					0x04000000
//#define CC_FREE_BIT					0x08000000
#define CC_RGB_COLOR_BIT				0x10000000		//Color (R,G,B)
#define CC_NORMAL_BIT					0x20000000		//Normal (Nx,Ny,Nz)
#define CC_COMPRESSED_NORMAL_BIT		0x40000000		//Compressed normal (index)
#define CC_TEX_COORDS_BIT				0x80000000		//Texture coordinates (u,v)

//! CloudCompare object type flags
enum CC_CLASS_ENUM {
    CC_OBJECT               =   0,
    CC_HIERARCHY_OBJECT     =   CC_HIERARCH_BIT,
    CC_POINT_CLOUD          =   CC_HIERARCHY_OBJECT | CC_CLOUD_BIT,
    CC_MESH                 =   CC_HIERARCHY_OBJECT | CC_MESH_BIT,
    CC_MESH_GROUP           =   CC_MESH | CC_GROUP_BIT,
    CC_POINT_OCTREE         =   CC_HIERARCHY_OBJECT | CC_OCTREE_BIT | CC_LEAF_BIT,
    CC_POLY_LINE            =   CC_HIERARCHY_OBJECT | CC_POLYLINE_BIT | CC_LEAF_BIT,
    CC_IMAGE				=   CC_HIERARCH_BIT | CC_IMAGE_BIT,
    CC_CALIBRATED_IMAGE		=   CC_IMAGE  | CC_LEAF_BIT,
    CC_SENSOR				=   CC_HIERARCH_BIT | CC_SENSOR_BIT,
    CC_GBL_SENSOR			=	CC_SENSOR | CC_LEAF_BIT,
	CC_PRIMITIVE			=   CC_MESH | CC_PRIMITIVE_BIT, //primitives are meshes!
	CC_PLANE				=	CC_PRIMITIVE | CC_PLANE_BIT,
	CC_SPHERE				=	CC_PRIMITIVE | CC_SPHERE_BIT,
	CC_TORUS				=	CC_PRIMITIVE | CC_TORUS_BIT,
	CC_CYLINDER				=	CC_PRIMITIVE | CC_CYLINDER_BIT,
	CC_CONE					=	CC_PRIMITIVE | CC_CONE_BIT,
	CC_BOX					=	CC_PRIMITIVE | CC_BOX_BIT,
	CC_DISH					=	CC_PRIMITIVE | CC_DISH_BIT,
	CC_EXTRU				=	CC_PRIMITIVE | CC_EXTRU_BIT,
	CC_MATERIAL_SET			=	CC_MATERIAL_BIT | CC_GROUP_BIT | CC_LEAF_BIT,
	CC_CHUNKED_ARRAY		=	CC_ARRAY_BIT,
	CC_NORMALS_ARRAY		=	CC_ARRAY_BIT | CC_NORMAL_BIT | CC_LEAF_BIT,
	CC_NORMAL_INDEXES_ARRAY	=	CC_ARRAY_BIT | CC_COMPRESSED_NORMAL_BIT | CC_LEAF_BIT,
	CC_RGB_COLOR_ARRAY		=	CC_ARRAY_BIT | CC_RGB_COLOR_BIT | CC_LEAF_BIT,
	CC_TEX_COORDS_ARRAY		=	CC_ARRAY_BIT | CC_TEX_COORDS_BIT | CC_LEAF_BIT,
	CC_2D_LABEL				=	CC_HIERARCHY_OBJECT | CC_LABEL_BIT | CC_LEAF_BIT,
	CC_2D_VIEWPORT_OBJECT	=	CC_HIERARCH_BIT | CC_VIEWPORT_BIT | CC_LEAF_BIT,
	CC_2D_VIEWPORT_LABEL	=	CC_2D_VIEWPORT_OBJECT | CC_LABEL_BIT,
};


//! Generic "CloudCompare Object" template
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccObject : public ccSerializableObject
#else
class ccObject : public ccSerializableObject
#endif
{
public:

    //! Default constructor
    /** \param name object name (optional)
    **/
    ccObject(QString name = QString());

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const = 0;

    //! Returns object name
    virtual QString getName() const;

    //! Sets object name
    virtual void setName(const QString& name);

    //! Returns object unqiue ID
    virtual unsigned getUniqueID() const;

	//! Changes unique ID
	/** WARNING: HANDLE WITH CARE!
		Updates persistent settings (last unique ID) if necessary.
	**/
    virtual void setUniqueID(unsigned ID);

    //! Returns flag state
    virtual bool getFlagState(CC_OBJECT_FLAG flag) const;

    //! Sets flag state
    /** \param flag object flag to set
        \param state flag state
    **/
    virtual void setFlagState(CC_OBJECT_FLAG flag, bool state);

    //! Returns whether the object is enabled or not
    /** Shortcut to access flag CC_ENABLED
    **/
    virtual bool isEnabled() const;

    //! Sets the "enabled" property
    /** Shortcut to modify flag CC_ENABLED
    **/
    virtual void setEnabled(bool state);

    //! Returns whether the object is locked  or not
    /** Shortcut to access flag CC_LOCKED
    **/
    virtual bool isLocked() const;

    //! Sets the "enabled" property
    /** Shortcut to modify flag CC_LOCKED
    **/
    virtual void setLocked(bool state);

    //shortcuts
    inline bool isGroup() const {return (getClassID() & CC_GROUP_BIT)>0;};
    inline bool isLeaf() const {return (getClassID() & CC_LEAF_BIT)>0;};
    inline bool isHierarchy() const {return (getClassID() & CC_HIERARCH_BIT)>0;};

    inline bool isKindOf(CC_CLASS_ENUM type) const {return (getClassID() & type) == type;};
    inline bool isA(CC_CLASS_ENUM type) const {return (getClassID() == type);};

	//! Resets the object's unique ID counter
	/** Warning: should be called only once, on program startup.
	**/
	static void ResetUniqueIDCounter();

	//! Returns a new unassigned unique ID
	/** Unique IDs are handled with persistent settings
		in order to assure consistency between main app 
		and plugins!
	**/
	static unsigned GetNextUniqueID();

	//! Returns last assigned unique ID
	/** Unique IDs are handled with persistent settings
		in order to assure consistency between main app 
		and plugins!
	**/
	static unsigned GetLastUniqueID();

	//! Helper: reads out class ID from a binary stream
	/** Must be called before 'fromFile'!
	**/
	static bool ReadClassIDFromFile(unsigned& classID, QFile& in, short dataVersion);

protected:

	//inherited from ccSerializableObject
	virtual bool toFile(QFile& out) const;

	//! Reimplemented from ccSerializableObject::fromFile
	/** Be sure to call ccObject::ReadClassIDFromFile (once)
		before calling this method, as the classID is voluntarily
		skipped (in order to let the user instantiate the object
		first)
	**/
	virtual bool fromFile(QFile& in, short dataVersion);

	//! Sets last assigned unique ID
	/** Unique IDs are handled with persistent settings
		in order to assure consistency between main app 
		and plugins!
	**/
	static void UpdateLastUniqueID(unsigned lastID);

    //! Object name
    QString m_name;

    //! Object flags
    unsigned m_flags;

private:

    //! Object unique ID
    unsigned m_uniqueID;
};

#endif
