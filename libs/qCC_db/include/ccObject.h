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

#ifndef CC_OBJECT_HEADER
#define CC_OBJECT_HEADER

//Local
#include "ccSerializableObject.h"

//Qt
#include <QSharedPointer>
#include <QVariant>


//! Object state flag
enum CC_OBJECT_FLAG {	//CC_UNUSED			= 1, //DGM: not used anymore (former CC_FATHER_DEPENDENT)
						CC_ENABLED			= 2,
						CC_LOCKED			= 4,
};

//Bits for object type flags (64 bits)
#define CC_HIERARCH_BIT					0x00000000000001	//Hierarchical object
#define CC_LEAF_BIT						0x00000000000002	//Tree leaf (no children)
#define CC_GROUP_BIT					0x00000000000004	//Group (no data, aggregation only)
#define CC_PRIMITIVE_BIT				0x00000000000008	//Primitive (sphere, plane, torus, cylinder, etc.)
#define CC_ARRAY_BIT					0x00000000000010	//Array
#define CC_LABEL_BIT					0x00000000000020	//2D label
#define CC_VIEWPORT_BIT					0x00000000000040	//2D viewport
#define CC_CUSTOM_BIT					0x00000000000080	//For custom (plugin defined) objects
#define CC_CLOUD_BIT					0x00000000000100	//Point Cloud
#define CC_MESH_BIT						0x00000000000200	//Mesh
#define CC_OCTREE_BIT					0x00000000000400	//Octree
#define CC_POLYLINE_BIT					0x00000000000800	//Polyline
#define CC_IMAGE_BIT					0x00000000001000	//Picture
#define CC_SENSOR_BIT					0x00000000002000	//Sensor def.
#define CC_PLANE_BIT					0x00000000004000	//Plane (primitive)
#define CC_SPHERE_BIT					0x00000000008000	//Sphere (primitive)
#define CC_TORUS_BIT					0x00000000010000	//Torus (primitive)
#define CC_CYLINDER_BIT					0x00000000020000	//Cylinder (primitive)
#define CC_CONE_BIT						0x00000000040000	//Cone (primitive)
#define CC_BOX_BIT						0x00000000080000	//Box (primitive)
#define CC_DISH_BIT						0x00000000100000	//Dish (primitive)
#define CC_EXTRU_BIT					0x00000000200000	//Extrusion (primitive)
#define CC_KDTREE_BIT					0x00000000400000	//Kd-tree
#define CC_FACET_BIT					0x00000000800000	//Facet (composite object: cloud + 2D1/2 mesh + 2D1/2 polyline)
#define CC_MATERIAL_BIT					0x00000001000000	//Material
#define CC_CLIP_BOX_BIT					0x00000002000000	//Clipping box
#define CC_TRANS_BUFFER_BIT				0x00000004000000	//Indexed transformation buffer
#define CC_GROUND_BASED_BIT				0x00000008000000	//For Ground Based Lidar Sensors
#define CC_RGB_COLOR_BIT				0x00000010000000	//Color (R,G,B)
#define CC_NORMAL_BIT					0x00000020000000	//Normal (Nx,Ny,Nz)
#define CC_COMPRESSED_NORMAL_BIT		0x00000040000000	//Compressed normal (index)
#define CC_TEX_COORDS_BIT				0x00000080000000	//Texture coordinates (u,v)
#define CC_CAMERA_BIT					0x00000100000000	//For camera sensors (projective sensors)
#define CC_QUADRIC_BIT					0x00000200000000	//Quadric (primitive)
#define CC_RGBA_COLOR_BIT				0x00000400000000	//Color (R,G,B,A)
#define CC_COORDINATESYSTEM_BIT			0x00000800000000	//CoordinateSystem (primitive)
#define CC_CLIP_BOX_PART_BIT			0x00001000000000	//Cliping-box component
#define CC_CIRCLE_BIT					0x00002000000000	//'3D' circle (polyline)
//#define CC_FREE_BIT					0x00004000000000
//#define CC_FREE_BIT					0x00008000000000
//#define CC_FREE_BIT					0x00010000000000
//#define CC_FREE_BIT					...

//! Type of object type flags (64 bits)
using CC_CLASS_ENUM = int64_t;

//! CloudCompare object type flags
namespace CC_TYPES
{
	enum : CC_CLASS_ENUM {
		OBJECT = 0,
		HIERARCHY_OBJECT	=	CC_HIERARCH_BIT,
		POINT_CLOUD			=	HIERARCHY_OBJECT	| CC_CLOUD_BIT,
		MESH				=	HIERARCHY_OBJECT	| CC_MESH_BIT,
		SUB_MESH			=	HIERARCHY_OBJECT	| CC_MESH_BIT				| CC_LEAF_BIT,
		MESH_GROUP			=	MESH				| CC_GROUP_BIT,								//DEPRECATED; DEFINITION REMAINS FOR BACKWARD COMPATIBILITY ONLY
		FACET				=	HIERARCHY_OBJECT	| CC_FACET_BIT,
		POINT_OCTREE		=	HIERARCHY_OBJECT	| CC_OCTREE_BIT				| CC_LEAF_BIT,
		POINT_KDTREE		=	HIERARCHY_OBJECT	| CC_KDTREE_BIT				| CC_LEAF_BIT,
		POLY_LINE			=	HIERARCHY_OBJECT	| CC_POLYLINE_BIT,
		IMAGE				=	CC_HIERARCH_BIT		| CC_IMAGE_BIT,
		CALIBRATED_IMAGE	=	IMAGE				| CC_LEAF_BIT,
		SENSOR				=	CC_HIERARCH_BIT		| CC_SENSOR_BIT,
		GBL_SENSOR			=	SENSOR				| CC_GROUND_BASED_BIT,
		CAMERA_SENSOR		=	SENSOR				| CC_CAMERA_BIT,
		PRIMITIVE			=	MESH				| CC_PRIMITIVE_BIT,							//primitives are meshes
		PLANE				=	PRIMITIVE			| CC_PLANE_BIT,
		SPHERE				=	PRIMITIVE			| CC_SPHERE_BIT,
		TORUS				=	PRIMITIVE			| CC_TORUS_BIT,
		CONE				=	PRIMITIVE			| CC_CONE_BIT,
		OLD_CYLINDER_ID		=	PRIMITIVE			| CC_CYLINDER_BIT,							//for backward compatibility
		CYLINDER			=	PRIMITIVE			| CC_CYLINDER_BIT			| CC_CONE_BIT,	//cylinders are cones
		BOX					=	PRIMITIVE			| CC_BOX_BIT,
		DISH				=	PRIMITIVE			| CC_DISH_BIT,
		EXTRU				=	PRIMITIVE			| CC_EXTRU_BIT,
		QUADRIC				=	PRIMITIVE			| CC_QUADRIC_BIT,
		CIRCLE				=	POLY_LINE			| CC_CIRCLE_BIT,
		MATERIAL_SET		=	CC_MATERIAL_BIT		| CC_GROUP_BIT				| CC_LEAF_BIT,
		ARRAY				=	CC_ARRAY_BIT,
		NORMALS_ARRAY		=	CC_ARRAY_BIT		| CC_NORMAL_BIT				| CC_LEAF_BIT,
		NORMAL_INDEXES_ARRAY=	CC_ARRAY_BIT		| CC_COMPRESSED_NORMAL_BIT	| CC_LEAF_BIT,
		RGB_COLOR_ARRAY		=	CC_ARRAY_BIT		| CC_RGB_COLOR_BIT			| CC_LEAF_BIT,
		RGBA_COLOR_ARRAY	=	CC_ARRAY_BIT		| CC_RGBA_COLOR_BIT			| CC_LEAF_BIT,
		TEX_COORDS_ARRAY	=	CC_ARRAY_BIT		| CC_TEX_COORDS_BIT			| CC_LEAF_BIT,
		LABEL_2D			=	HIERARCHY_OBJECT	| CC_LABEL_BIT				| CC_LEAF_BIT,
		VIEWPORT_2D_OBJECT	=	HIERARCHY_OBJECT	| CC_VIEWPORT_BIT			| CC_LEAF_BIT,
		VIEWPORT_2D_LABEL	=	VIEWPORT_2D_OBJECT	| CC_LABEL_BIT,
		CLIPPING_BOX		=	CC_CLIP_BOX_BIT		| CC_LEAF_BIT,
		CLIPPING_BOX_PART	=	CC_CLIP_BOX_PART_BIT| CC_LEAF_BIT,
		TRANS_BUFFER		=	HIERARCHY_OBJECT	| CC_TRANS_BUFFER_BIT		| CC_LEAF_BIT,
		COORDINATESYSTEM	=	PRIMITIVE			| CC_COORDINATESYSTEM_BIT,
		//  Custom types
		/** Custom objects are typically defined by plugins. They can be inserted in an object
			hierarchy or displayed in an OpenGL context like any other ccHObject.
			To differentiate custom objects, use the meta-data mechanism (see ccObject::getMetaData
			and ccObject::setMetaData). You can also define a custom icon (see ccHObject::getIcon).
	
			It is highly advised to use the ccCustomHObject and ccCustomLeafObject interfaces to
			define a custom types. Carefully read the ccCustomHObject::isDeserialized method's
			description and the warning below!
	
			Warning: custom objects can't be 'fully' serialized. Don't overload the
			'ccSerializableObject::toFile' method for them as this would break the deserialization mechanism!
			They can only be serialized as plain ccHObject instances (CC_TYPES::HIERARCHY_OBJECT).
			Hierarchical custom objects (CC_TYPES::CUSTOM_H_OBJECT) will be deserialized as ccCustomHObject
			instances. Leaf custom objects (CC_TYPES::CUSTOM_LEAF_OBJECT) will be deserialized as
			ccCustomLeafObject instances.
		**/
		CUSTOM_H_OBJECT		=	HIERARCHY_OBJECT | CC_CUSTOM_BIT,
		CUSTOM_LEAF_OBJECT	=	CUSTOM_H_OBJECT | CC_LEAF_BIT,
	};
}

//! Unique ID generator (should be unique for the whole application instance - with plugins, etc.)
class QCC_DB_LIB_API ccUniqueIDGenerator
{
public:

	static constexpr unsigned InvalidUniqueID = 0xFFFFFFFF;
	static constexpr unsigned MinUniqueID = 0x00000100;

	//! Shared type
	using Shared = QSharedPointer<ccUniqueIDGenerator>;

	//! Default constructor
	ccUniqueIDGenerator() : m_lastUniqueID(MinUniqueID) {}

	//! Resets the unique ID
	void reset() { m_lastUniqueID = MinUniqueID; }
	//! Returns a (new) unique ID
	unsigned fetchOne() { return ++m_lastUniqueID; }
	//! Returns the value of the last generated unique ID
	unsigned getLast() const { return m_lastUniqueID; }
	//! Updates the value of the last generated unique ID with the current one
	void update(unsigned ID) { if (ID > m_lastUniqueID) m_lastUniqueID = ID; }

protected:
	unsigned m_lastUniqueID;
};

//! Generic "CloudCompare Object" template
class QCC_DB_LIB_API ccObject : public ccSerializableObject
{
public:

	//! Default constructor
	/** \param name object name (optional)
	    \param uniqueID unique ID (handle with care! Will be auto generated if equal to ccUniqueIDGenerator::InvalidUniqueID)
	**/
	ccObject(const QString& name = QString(), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Copy constructor
	ccObject(const ccObject& object);

	//! Returns current database version
	static unsigned GetCurrentDBVersion();
	//! Sets the unique ID generator
	static void SetUniqueIDGenerator(ccUniqueIDGenerator::Shared generator);
	//! Returns the unique ID generator
	static ccUniqueIDGenerator::Shared GetUniqueIDGenerator();

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const = 0;

	//! Returns object name
	virtual inline QString getName() const { return m_name; }

	//! Sets object name
	virtual inline void setName(const QString& name) { m_name = name; }

	//! Returns object unique ID
	virtual inline unsigned getUniqueID() const { return m_uniqueID; }

	//! Changes unique ID
	/** \warning HANDLE WITH CARE!
		Updates persistent settings (last unique ID) if necessary.
	**/
	virtual void setUniqueID(unsigned ID);

	//! Returns whether the object is enabled or not
	/** Shortcut to access flag CC_ENABLED
	**/
	virtual inline bool isEnabled() const { return getFlagState(CC_ENABLED); }

	//! Sets the "enabled" property
	/** Shortcut to modify flag CC_ENABLED
	**/
	virtual inline void setEnabled(bool state) { setFlagState(CC_ENABLED,state); }

	//! Toggles the "enabled" property
	virtual inline void toggleActivation() { setEnabled(!isEnabled()); }

	//! Returns whether the object is locked  or not
	/** Shortcut to access flag CC_LOCKED
	**/
	virtual inline bool isLocked() const { return getFlagState(CC_LOCKED); }

	//! Sets the "enabled" property
	/** Shortcut to modify flag CC_LOCKED
	**/
	virtual inline void setLocked(bool state) { setFlagState(CC_LOCKED,state); }

	//shortcuts
	inline bool isLeaf() const {return (getClassID() & CC_LEAF_BIT) != 0; }
	inline bool isCustom() const {return (getClassID() & CC_CUSTOM_BIT) != 0; }
	inline bool isHierarchy() const { return (getClassID() & CC_HIERARCH_BIT) != 0; }

	inline bool isKindOf(CC_CLASS_ENUM type) const { return (getClassID() & type) == type; }
	inline bool isA(CC_CLASS_ENUM type) const { return (getClassID() == type); }

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
	static CC_CLASS_ENUM ReadClassIDFromFile(QFile& in, short dataVersion);

	//! Returns a given associated meta data
	/** \param key meta data unique identifier (case sensitive)
		\return meta data (if any) or an invalid QVariant
	**/
	QVariant getMetaData(const QString& key) const;

	//! Removes a given associated meta-data
	/** \param key meta-data unique identifier (case sensitive)
		\return success
	**/
	bool removeMetaData(const QString& key);

	//! Sets a meta-data element
	/** \param key meta-data unique identifier (case sensitive)
		\param data data
	**/
	void setMetaData(const QString& key, const QVariant& data);

	//! Sets several meta-data elements at a time
	/** \param dataset meta-data set
		\param overwrite whether existing meta-data elements should be replaced by the input ones (with the same key) or not
	**/
	void setMetaData(const QVariantMap& dataset, bool overwrite = false);

	//! Returns whether a meta-data element with the given key exists or not
	/** \param key meta-data unique identifier (case sensitive)
		\return whether the element exists or not
	**/
	bool hasMetaData(const QString& key) const;

	//! Returns meta-data map (const only)
	const QVariantMap& metaData() const { return m_metaData; }

protected:

	//! Returns flag state
	virtual inline bool getFlagState(CC_OBJECT_FLAG flag) const { return (m_flags & flag); }

	//! Sets flag state
	/** \param flag object flag to set
		\param state flag state
	**/
	virtual void setFlagState(CC_OBJECT_FLAG flag, bool state);

	//inherited from ccSerializableObject
	bool toFile(QFile& out, short dataVersion) const override;
	short minimumFileVersion() const override;

	//! Reimplemented from ccSerializableObject::fromFile
	/** Be sure to call ccObject::ReadClassIDFromFile (once)
		before calling this method, as the classID is voluntarily
		skipped (in order to let the user instantiate the object first)
	**/
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//! Object name
	QString m_name;

	//! Object flags
	unsigned m_flags;

	//! Associated meta-data
	QVariantMap m_metaData;

private:

	//! Object unique ID
	unsigned m_uniqueID;
};

#endif //CC_OBJECT_HEADER
