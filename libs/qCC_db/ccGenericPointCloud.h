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

#ifndef CC_GENERIC_POINT_CLOUD_HEADER
#define CC_GENERIC_POINT_CLOUD_HEADER

//CCLib
#include <GenericIndexedCloudPersist.h>
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>

//Local
#include "qCC_db.h"
#include "ccHObject.h"
#include "ccGLMatrix.h"
#include "ccAdvancedTypes.h"

class ccOctree;

/***************************************************
				ccGenericPointCloud
***************************************************/

//default material for clouds (with normals)
#define CC_DEFAULT_CLOUD_AMBIENT_COLOR		ccColor::bright
#define CC_DEFAULT_CLOUD_SPECULAR_COLOR		ccColor::bright
#define CC_DEFAULT_CLOUD_DIFFUSE_COLOR		ccColor::bright
#define CC_DEFAULT_CLOUD_EMISSION_COLOR		ccColor::night
#define CC_DEFAULT_CLOUD_SHININESS			50.0f

//! A 3D cloud interface with associated features (color, normals, octree, etc.)
/** A generic point cloud can have multiples features:
	- colors (RGB)
	- normals (compressed)
	- an octree strucutre
	- visibility information per point (to hide/display subsets of points)
**/
class QCC_DB_LIB_API ccGenericPointCloud : public ccHObject,  virtual public CCLib::GenericIndexedCloudPersist
{
	friend class ccMesh;

public:

	//! Default constructor
	ccGenericPointCloud(QString name = QString());

	//! Default destructor
	virtual ~ccGenericPointCloud();


	/***************************************************
						Clone/Copy
	***************************************************/

	//! Clones this entity
	/** All the main features of the entity are cloned, except from the octree and
		the points visibility information.
		\param destCloud destination cloud can be provided here (must be of the exact same type as the cloned cloud!)
		\param ignoreChildren [optional] whether to ignore the cloud's children or not (in which case they will be cloned as well)
		\return a copy of this entity
	**/
	virtual ccGenericPointCloud* clone(ccGenericPointCloud* destCloud = 0, bool ignoreChildren = false) = 0;


	/***************************************************
				Features deletion/clearing
	***************************************************/

	//! Clears the entity from all its points and features
	/** Display parameters are also reseted to their default values.
	**/
	virtual void clear();


	/***************************************************
				Octree management
	***************************************************/

	//! Computes the cloud octree
	/** The octree bounding-box is automatically defined as the smallest
		3D cube that encloses totally the cloud.
		WARNING: any previously attached octree will be deleted,
				 even if new octree computation failed.
		\param progressCb the caller can get some notification of the process progress through this callback mechanism (see CCLib documentation)
		\return the computed octree
	**/
	virtual ccOctree* computeOctree(CCLib::GenericProgressCallback* progressCb=NULL);

	//! Returns associated octree
	virtual ccOctree* getOctree();

	//! Erases the octree
	virtual void deleteOctree();


	/***************************************************
					Features getters
	***************************************************/

	//! Returns color corresponding to a given scalar value
	/** The returned value depends on the current scalar field display parameters.
		It may even be 0 if the value shouldn't be displayed.
		WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual const colorType* geScalarValueColor(ScalarType d) const = 0;

	//! Returns color corresponding to a given point associated scalar value
	/** The returned value depends on the current scalar field display parameters.
		It may even be 0 if the value shouldn't be displayed.
		WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual const colorType* getPointScalarValueColor(unsigned pointIndex) const = 0;

	//! Returns scalar value associated to a given point
	/** The returned value is taken from the current displayed scalar field
		WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual ScalarType getPointDisplayedDistance(unsigned pointIndex) const = 0;

	//! Returns color corresponding to a given point
	/** WARNING: color array must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual const colorType* getPointColor(unsigned pointIndex) const = 0;

	//! Returns compressed normal corresponding to a given point
	/** WARNING: normals array must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual const normsType& getPointNormalIndex(unsigned pointIndex) const = 0;

	//! Returns normal corresponding to a given point
	/** WARNING: normals array must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual const CCVector3& getPointNormal(unsigned pointIndex) const = 0;


	/***************************************************
					Visibility array
	***************************************************/

	//! Array of "visibility" information for each point
	/** See <CCConst.h>
	**/
	typedef GenericChunkedArray<1,uchar> VisibilityTableType;

	//! Returns associated visiblity array
	virtual VisibilityTableType* getTheVisibilityArray();

	//! Returns a ReferenceCloud equivalent to the visiblity array
	virtual CCLib::ReferenceCloud* getTheVisiblePoints() const;

	//! Returns whether the visiblity array is allocated or not
	virtual bool isVisibilityTableInstantiated() const;

	//! Resets the associated visiblity array
	/** Warning: allocates the array if it was not done yet!
	**/
	virtual bool resetVisibilityArray();

	//! Erases the points visibility information
	virtual void unallocateVisibilityArray();

	/***************************************************
					Other methods
	***************************************************/

	//Inherited from GenericCloud
	virtual uchar testVisibility(const CCVector3& P) const;

	//Inherited from ccHObject
	virtual ccBBox getMyOwnBB();

	//! Forces bounding-box update
	virtual void refreshBB() = 0;

	//! Creates a new point cloud with only the 'visible' points (as defined by the visibility array)
	/** \param removeSelectedPoints if true, exported point are also removed from the current point cloud
		\return new point cloud with selected points
	**/
	virtual ccGenericPointCloud* createNewCloudFromVisibilitySelection(bool removeSelectedPoints=false) = 0;

	//! Applies a rigid transformation (rotation + translation)
	virtual void applyRigidTransformation(const ccGLMatrix& trans) = 0;

	//! Sets shift applied to original coordinates (information storage only)
	/** Such a shift can typically be applied at loading time.
	**/
	void setGlobalShift(double x, double y, double z);

	//! Sets shift applied to original coordinates (information storage only)
	/** Such a shift can typically be applied at loading time.
		Original coordinates are equal to '(P/scale - shift)'
	**/
	void setGlobalShift(const CCVector3d& shift);

	//! Returns the shift applied to original coordinates
	/** See ccGenericPointCloud::setOriginalShift
	**/
	const CCVector3d& getGlobalShift() const { return m_globalShift; }

	//! Sets the scale applied to original coordinates (information storage only)
	void setGlobalScale(double scale);

	//! Returns whether the cloud is shifted or not
	inline bool isShifted() const
	{
		return (	m_globalShift.x != 0
				||	m_globalShift.y != 0
				||	m_globalShift.z != 0
				||	m_globalScale != 1.0 );
	}

	//! Returns the scale applied to original coordinates
	/** See ccGenericPointCloud::setOriginalScale
	**/
	double getGlobalScale() const { return m_globalScale; }

	//! Returns the point back-projected into the original coordinates system
	template<typename T> inline CCVector3d toGlobal3d(const Vector3Tpl<T>& Plocal) const
	{
		// Pglobal = Plocal/scale - shift
		return CCVector3d::fromArray(Plocal.u) / m_globalScale - m_globalShift;
	}

	//! Returns the point projected into the local (shifted) coordinates system
	template<typename T> inline CCVector3d toLocal3d(const Vector3Tpl<T>& Pglobal) const
	{
		// Plocal = (Pglobal + shift) * scale
		return (CCVector3d::fromArray(Pglobal.u) + m_globalShift) * m_globalScale;
	}
	//! Returns the point projected into the local (shifted) coordinates system
	template<typename T> inline CCVector3 toLocal3pc(const Vector3Tpl<T>& Pglobal) const
	{
		CCVector3d Plocal = CCVector3d::fromArray(Pglobal.u) * m_globalScale + m_globalShift;
		return CCVector3::fromArray(Plocal.u);
	}

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }

	//! Sets point size
	/** Overrides default value one if superior than 0
		(see glPointSize).
	**/
	void setPointSize(unsigned size = 0) { m_pointSize = static_cast<unsigned char>(size); }

	//! Returns current point size
	/** 0 means that the cloud will use current OpenGL value
		(see glPointSize).
	**/
	unsigned char getPointSize() const { return m_pointSize; }

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);

	//! Per-point visibility table
	/** If this table is allocated, only values set to POINT_VISIBLE
		will be considered as visible/selected.
	**/
	VisibilityTableType* m_pointsVisibility;

	//! Global shift (typically applied at loading time)
	CCVector3d m_globalShift;

	//! Global scale (typically applied at loading time)
	double m_globalScale;

	//! Point size (won't be applied if 0)
	unsigned char m_pointSize;

};

#endif //CC_GENERIC_POINT_CLOUD_HEADER
