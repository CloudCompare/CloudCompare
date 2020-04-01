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

#ifndef CC_GENERIC_POINT_CLOUD_HEADER
#define CC_GENERIC_POINT_CLOUD_HEADER

//Local
#include "ccAdvancedTypes.h"
#include "ccOctree.h"
#include "ccShiftedObject.h"

//System
#include <vector>

namespace CCLib
{
	class GenericProgressCallback;
	class ReferenceCloud;
}
	
class ccOctreeProxy;

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
class QCC_DB_LIB_API ccGenericPointCloud : public ccShiftedObject,  public CCLib::GenericIndexedCloudPersist
{
	friend class ccMesh;

public:

	//! Default constructor
	/** \param name cloud name (optional)
		\param uniqueID unique ID (handle with care)
	**/
	ccGenericPointCloud(QString name = QString(), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Copy constructor
	ccGenericPointCloud(const ccGenericPointCloud& cloud);

	//! Default destructor
	~ccGenericPointCloud() override;
	
	
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
	virtual ccGenericPointCloud* clone(ccGenericPointCloud* destCloud = nullptr, bool ignoreChildren = false) = 0;
	
	
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
		3D cube that totally encloses the cloud.
		WARNING: any previously attached octree will be deleted,
				 even if the new octree computation failed.
		\param progressCb the caller can get some notification of the process progress through this callback mechanism (see CCLib documentation)
		\param autoAddChild whether to automatically add the computed octree as child of this cloud or not
		\return the computed octree
	**/
	virtual ccOctree::Shared computeOctree(CCLib::GenericProgressCallback* progressCb = nullptr, bool autoAddChild = true);
	
	//! Returns the associated octree (if any)
	virtual ccOctree::Shared getOctree() const;
	//! Sets the associated octree
	virtual void setOctree(ccOctree::Shared octree, bool autoAddChild = true);
	//! Returns the associated octree proxy (if any)
	virtual ccOctreeProxy* getOctreeProxy() const;

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
	virtual const ccColor::Rgb* geScalarValueColor(ScalarType d) const = 0;

	//! Returns color corresponding to a given point associated scalar value
	/** The returned value depends on the current scalar field display parameters.
		It may even be 0 if the value shouldn't be displayed.
		WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual const ccColor::Rgb* getPointScalarValueColor(unsigned pointIndex) const = 0;

	//! Returns scalar value associated to a given point
	/** The returned value is taken from the current displayed scalar field
		WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
	**/
	virtual ScalarType getPointDisplayedDistance(unsigned pointIndex) const = 0;

	//! Returns color corresponding to a given point
	/** WARNING: color array must be enabled! (see ccDrawableObject::hasColors)
	**/
	virtual const ccColor::Rgba& getPointColor(unsigned pointIndex) const = 0;

	//! Returns compressed normal corresponding to a given point
	/** WARNING: normals array must be enabled! (see ccDrawableObject::hasNormals)
	**/
	virtual const CompressedNormType& getPointNormalIndex(unsigned pointIndex) const = 0;

	//! Returns normal corresponding to a given point
	/** WARNING: normals array must be enabled! (see ccDrawableObject::hasNormals)
	**/
	virtual const CCVector3& getPointNormal(unsigned pointIndex) const = 0;


	/***************************************************
					Visibility array
	***************************************************/

	//! Array of "visibility" information for each point
	/** See <CCConst.h>
	**/
	using VisibilityTableType = std::vector<unsigned char>;
	
	//! Returns associated visiblity array
	virtual inline VisibilityTableType& getTheVisibilityArray() { return m_pointsVisibility; }

	//! Returns associated visiblity array (const version)
	virtual inline const VisibilityTableType& getTheVisibilityArray() const { return m_pointsVisibility; }

	//! Returns a ReferenceCloud equivalent to the visiblity array
	/** \param visTable visibility table (optional, otherwise the cloud's default one will be used)
		\param silent don't issue warnings if no visible point is present
		\return the visible points as a ReferenceCloud
	**/
	virtual CCLib::ReferenceCloud* getTheVisiblePoints(const VisibilityTableType* visTable = nullptr, bool silent = false) const;
	
	//! Returns whether the visiblity array is allocated or not
	virtual bool isVisibilityTableInstantiated() const;

	//! Resets the associated visiblity array
	/** Warning: allocates the array if it was not done yet!
	**/
	virtual bool resetVisibilityArray();

	//! Inverts the visiblity array
	virtual void invertVisibilityArray();

	//! Erases the points visibility information
	virtual void unallocateVisibilityArray();


	/***************************************************
					Other methods
	***************************************************/

	//Inherited from ccHObject
	ccBBox getOwnBB(bool withGLFeatures = false) override;
	
	//! Forces bounding-box update
	virtual void refreshBB() = 0;

	//! Creates a new point cloud with only the 'visible' points (as defined by the visibility array)
	/** \param removeSelectedPoints if true, exported point are also removed from the current point cloud
		\param visTable visibility table (optional, otherwise the cloud's default one will be used)
		\param silent don't issue a warning message if there's no point to keep
		\return new point cloud with selected points
	**/
	virtual ccGenericPointCloud* createNewCloudFromVisibilitySelection(bool removeSelectedPoints = false, VisibilityTableType* visTable = nullptr, bool silent = false) = 0;
	
	//! Applies a rigid transformation (rotation + translation)
	virtual void applyRigidTransformation(const ccGLMatrix& trans) = 0;

	//! Crops the cloud inside (or outside) a boundig box
	/** \warning Always returns a selection (potentially empty) if successful.
		\param box croping box
		\param inside whether selected points are inside or outside the box
		\return points falling inside (or outside) as a selection
	**/
	virtual CCLib::ReferenceCloud* crop(const ccBBox& box, bool inside = true) = 0;

	//! Multiplies all coordinates by constant factors (one per dimension)
	/** WARNING: attached octree may be deleted.
		\param fx multiplication factor along the X dimension
		\param fy multiplication factor along the Y dimension
		\param fz multiplication factor along the Z dimension
		\param center scaling center (0,0,0) by default
	**/
	virtual void scale(PointCoordinateType fx, PointCoordinateType fy, PointCoordinateType fz, CCVector3 center = CCVector3(0, 0, 0)) = 0;

	//inherited from ccSerializableObject
	bool isSerializable() const override { return true; }
	
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

	//! Imports the parameters from another cloud
	/** Only the specific parameters are imported.
	**/
	void importParametersFrom(const ccGenericPointCloud* cloud);

	//! Point picking (brute force or octree-driven)
	/** \warning the octree-driven method only works if pickWidth == pickHeight
	**/
	bool pointPicking(	const CCVector2d& clickPos,
						const ccGLCameraParameters& camera,
						int& nearestPointIndex,
						double& nearestSquareDist,
						double pickWidth = 2.0,
						double pickHeight = 2.0,
						bool autoComputeOctree = false);

protected:
	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	
	//! Per-point visibility table
	/** If this table is allocated, only values set to POINT_VISIBLE
		will be considered as visible/selected.
	**/
	VisibilityTableType m_pointsVisibility;

	//! Point size (won't be applied if 0)
	unsigned char m_pointSize;

};

#endif //CC_GENERIC_POINT_CLOUD_HEADER
