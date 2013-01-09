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
//$Rev:: 2225                                                              $
//$LastChangedDate:: 2012-07-25 23:26:33 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//

#ifndef CC_GENERIC_POINT_CLOUD_HEADER
#define CC_GENERIC_POINT_CLOUD_HEADER

//CCLib
#include <GenericIndexedCloudPersist.h>
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>

#include "ccHObject.h"
#include "ccGLMatrix.h"
#include "ccAdvancedTypes.h"

class ccOctree;
class ccPlane;

/***************************************************
				ccGenericPointCloud
***************************************************/

//! A 3D cloud interface with associated features (color, normals, octree, etc.)
/** A generic point cloud can have multiples features:
	- colors (RGB)
	- normals (compressed)
	- an octree strucutre
	- visibility information per point (to hide/display subsets of points)
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccGenericPointCloud : public ccHObject,  virtual public CCLib::GenericIndexedCloudPersist
#else
class ccGenericPointCloud : public ccHObject,  virtual public CCLib::GenericIndexedCloudPersist
#endif
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
		\return a copy of this entity
	**/
	virtual ccGenericPointCloud* clone(ccGenericPointCloud* destCloud = 0)=0;


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
		WARNING: any precedently attached octree will be deleted,
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

	//! Returns whether the currently displayed SF is 'positive' or 'standard'
	virtual bool isDisplayedSFPositive()=0;

    //! Returns color corresponding to a given scalar value
    /** The returned value depends on the current scalar field display parameters.
        It may even be 0 if the value shouldn't be displayed.
        WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
    **/
	virtual const colorType* getDistanceColor(DistanceType d) const=0;

    //! Returns color corresponding to a given point associated scalar value
    /** The returned value depends on the current scalar field display parameters.
        It may even be 0 if the value shouldn't be displayed.
        WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
    **/
	virtual const colorType* getPointDistanceColor(unsigned pointIndex) const=0;

	//! Returns scalar value associated to a given point
    /** The returned value is taken from the current displayed scalar field
        WARNING: scalar field must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
    **/
	virtual DistanceType getPointDisplayedDistance(unsigned pointIndex) const=0;

    //! Returns color corresponding to a given point
    /** WARNING: color array must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
    **/
	virtual const colorType* getPointColor(unsigned pointIndex) const=0;

    //! Returns compressed normal corresponding to a given point
    /** WARNING: normals array must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
    **/
	virtual const normsType getPointNormalIndex(unsigned pointIndex) const=0;

    //! Returns normal corresponding to a given point
    /** WARNING: normals array must be enabled! (see ccDrawableObject::hasDisplayedScalarField)
    **/
	virtual const PointCoordinateType* getPointNormal(unsigned pointIndex) const=0;


	/***************************************************
                    Visibility array
	***************************************************/

	//! Array of "visibility" information for each point
	typedef GenericChunkedArray<1,uchar> VisibilityTableType;

    //! Returns associated visiblity array
	virtual VisibilityTableType* getTheVisibilityArray();

    //! Returns a ReferenceCloud equivalent to the visiblity array
	virtual CCLib::ReferenceCloud* getTheVisiblePoints();

	//! Returns whether the visiblity array is allocated or not
    virtual bool isVisibilityTableInstantiated() const;

    //! Resets the associated visiblity array
    /** Warning: allocates the array if it was not done yet!
    **/
	virtual bool razVisibilityArray();

	//! Erases the points visibility information
	virtual void unallocateVisibilityArray();


	/***************************************************
                    Other methods
	***************************************************/

    //Inherited from GenericCloud
	virtual CC_VISIBILITY_TYPE testVisibility(const CCVector3& P);

    //Inherited from ccHObject
    virtual ccBBox getMyOwnBB();

    //! Forces bounding-box update
    virtual void refreshBB()=0;

	//! Creates a new point cloud with only the 'visible' points (as defined by the visibility array)
	/** \param removeSelectedPoints if true, exported point are also removed from the current point cloud
        \return new point cloud with selected points
    **/
	virtual ccGenericPointCloud* createNewCloudFromVisibilitySelection(bool removeSelectedPoints=false)=0;

    //! Applies a rigid transformation (rotation + translation)
    virtual void applyRigidTransformation(const ccGLMatrix& trans)=0;

	//! Fits a plane primitive on cloud
	/** \param rms to retrieve plane fitting rms
		\return plane primitive (if successful)
	**/
	ccPlane* fitPlane(double* rms = 0);

	//! Sets shift to cloud original coordinates (information storage only)
	void setOriginalShift(double x, double y, double z);

	//! Returns shift to cloud original coordinates
	const double* getOriginalShift() const { return m_originalShift; }

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }

protected:

    //inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

	//! Visibility array
	/** If this array is allocated, values set to 0 indicates that
        the corresponding points are not visible/selected.
    **/
	VisibilityTableType* m_visibilityArray;

	//! Original shift (information backup)
	double m_originalShift[3];

};

#endif
