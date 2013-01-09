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

#ifndef CC_POINT_CLOUD_HEADER
#define CC_POINT_CLOUD_HEADER

#ifdef _MSC_VER
//To get rid of the warnings about dominant inheritance
#pragma warning( disable: 4250 )
#endif

//CCLib
#include <ReferenceCloud.h>
#include <ChunkedPointCloud.h>
#include <GenericProgressCallback.h>

#include "ccGenericPointCloud.h"

class ccPointCloud;
class ccScalarField;

/***************************************************
				ccPointCloud
***************************************************/

//! Max number of points per cloud (point cloud will be chunked above this limit)
const unsigned CC_MAX_NUMBER_OF_POINTS_PER_CLOUD = 128000000;

//! Max number of displayed point (per entity) in "low detail" display
const unsigned MAX_LOD_POINTS_NUMBER = 1000000;

//! A 3D cloud and its associated features (color, normals, scalar fields, etc.)
/** A point cloud can have multiple features:
	- colors (RGB)
	- normals (compressed)
	- scalar fields
	- an octree strucutre
	- per-point visibility information (to hide/display subsets of points)
	- other children objects (meshes, calibrated pictures, etc.)
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccPointCloud : public CCLib::ChunkedPointCloud, public ccGenericPointCloud
#else
class ccPointCloud : public CCLib::ChunkedPointCloud, public ccGenericPointCloud
#endif
{
public:

    //! Default constructor
	/** Creates an empty cloud without any feature. Each of them shoud be
        specifically instantiated/created (once the points have been
		added to this cloud, at least partially).
		\param name cloud name (optional)
    **/
    ccPointCloud(QString name = QString());

    //! Creates a new point cloud object from a ReferenceCloud
	/** "Reference clouds" are a set of indexes referring to a real point cloud.
        See CClib documentation for more information about ReferenceClouds.
		The real cloud must be explicitly known (and of type ccPointCloud). It
		must be passed as a second argument. It will allow the importation of
		points but also other properties such as colors, normals, etc.
		\param selection a ReferenceCloud (pointing to source)
		\param source a ccPointCloud (reference)
    **/
	ccPointCloud(CCLib::ReferenceCloud* selection, ccPointCloud* source);

    //! Creates a new point cloud object from a ReferenceCloud
	/** The real cloud must be explicitly known (and of type ccGenericPointCloud here).
        It must be passed as a second argument. It will allow the importation of
		points but also other properties such as colors, normals, etc. However, only
		the active scalar field can be imported
		\param selection a ReferenceCloud (pointing to source)
		\param source a ccGenericPointCloud (reference)
    **/
	ccPointCloud(CCLib::ReferenceCloud* selection, ccGenericPointCloud* source);

    //! Creates a new point cloud object from a GenericIndexedCloud
	/** "GenericIndexedCloud" is an extension of GenericCloud (from CCLib)
		which provides a const random accessor to points.
		See CClib documentation for more information about GenericIndexedCloud.
		As the GenericIndexedCloud interface is very simple, only points are imported.
		\param cloud a GenericIndexedCloud structure
    **/
	ccPointCloud(const CCLib::GenericIndexedCloud* cloud);

    //! Creates a new point cloud object from a GenericCloud
	/** "GenericCloud" is a very simple and light interface from CCLib. It is
        meant to give access to points coordinates of any cloud (on the
		condition it implements the GenericCloud interface of course).
		See CClib documentation for more information about GenericClouds.
		As the GenericCloud interface is very simple, only points are imported.
		\param cloud a GenericCloud structure
    **/
	ccPointCloud(CCLib::GenericCloud* cloud);

	//! Default destructor
	virtual ~ccPointCloud();

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_POINT_CLOUD;};

    /***************************************************
						Clone/Copy
	***************************************************/

	//! Clones this entity
	/** All the main features of the entity are cloned, except from the octree and
		the points visibility information.
		\param destCloud destination cloud can be provided here
		\return a copy of this entity
	**/
	virtual ccPointCloud* cloneThis(ccPointCloud* destCloud = 0);

	//inherited from ccGenericPointCloud
	virtual ccGenericPointCloud* clone(ccGenericPointCloud* destCloud = 0);

    //! Fuses another 3D entity with this one
	/** All the main features of the given entity are added, except from the octree and
        the points visibility information. Those features are deleted on this cloud.
    **/
	const ccPointCloud& operator +=(ccPointCloud*);

	/***************************************************
				Features deletion/clearing
	***************************************************/

	//! Clears the entity from all its points and features
	/** Display parameters are also reseted to their default values.
	**/
	virtual void clear();

	//! Erases the cloud colors
	void unallocateColors();

	//! Erases the cloud normals
	void unallocateNorms();

	/***************************************************
				Features allocation/resize
	***************************************************/

	//! Reserves memory to store the points coordinates
	/** Before adding points to the cloud (with addPoint())
		be sure to reserve the necessary amount of memory
		with this method. It the number of new elements is
		smaller than the actual one, nothing will happen.
		\param _numberOfPoints number of points to reserve the memory for
		\return true if ok, false if there's not enough memory
	**/
	bool reserveThePointsTable(unsigned _numberOfPoints);

	//! Reserves memory to store the RGB colors
	/** Before adding colors to the cloud (with addRGBColor())
		be sure to reserve the necessary amount of memory
		with this method. This method reserves memory for as
		many colors as the number of points in the cloud
		(effictively stored or reserved).
		\return true if ok, false if there's not enough memory
	**/
	bool reserveTheRGBTable();

	//! Resizes the RGB colors array
	/** If possible, the colors array is resized to fit exactly the number
		of points in the cloud (effictively stored or reserved). If the
		new size is inferior to the actual one, the last elements will be
		deleted. Otherwise, the array is filled with zeros (default behavior)
		or "white" colors (is fillWithWhite).
		WARNING: don't try to "add" any element on a resized array...
		\param fillWithWhite whether to fill new array elements with zeros (false) or white color (true)
		\return true if ok, false if there's not enough memory
	**/
	bool resizeTheRGBTable(bool fillWithWhite=false);

	//! Reserves memory to store the compressed normals
	/** Before adding normals to the cloud (with addNorm())
		be sure to reserve the necessary amount of memory
		with this method. This method reserves memory for as
		many colors as the number of points in the cloud
		(effictively stored or reserved).
		\return true if ok, false if there's not enough memory
	**/
	bool reserveTheNormsTable();

	//! Resizes the compressed normals array
	/** If possible, the normals array is resized to fit exactly the number
		of points in the cloud (effictively stored or reserved). If the
		new size is inferior to the actual one, the last elements will be
		deleted. Otherwise, the array is filled with blank elements.
		WARNING: don't try to "add" any element on a resized array...
		\return true if ok, false if there's not enough memory
	**/
	bool resizeTheNormsTable();

	//! Reserves memory for all the active features
	/** This method is meant to be called before increasing the cloud
		population. Only the already allocated features will be re-reserved.
		\return true if ok, false if there's not enough memory
	**/
	bool reserve(unsigned numberOfPoints);

	//! Resizes all the active features arrays
	/** This method is meant to be called after having increased the cloud
		population (if the final number of insterted point is lower than the
		reserved size). Otherwise, it fills all new elements with blank values.
		\return true if ok, false if there's not enough memory
	**/
	bool resize(unsigned numberOfPoints);


	/***************************************************
                    Scalar fields handling
	***************************************************/

	//! Returns the currently displayed scalar (or 0 if none)
	ccScalarField* getCurrentDisplayedScalarField() const;
	//! Returns the currently displayed scalar field index (or -1 if none)
	int getCurrentDisplayedScalarFieldIndex() const;
	//! Sets the currently displayed scalar field
	/** Warning: this scalar field will automatically be set as the OUTPUT one!
	**/
	virtual void setCurrentDisplayedScalarField(int index);

	virtual void deleteScalarField(int index); //redefinition
	virtual void deleteAllScalarFields();  //redefinition

	void setColorWithDistances(bool mixWithExistingColor);
	bool areNanScalarValuesInGrey() const;
	void setGreyForNanScalarValues(bool state);
    void showSFColorsScale(bool state);
    bool sfColorScaleShown() const;


	/***************************************************
                    Other methods
	***************************************************/

    //! Returns the cloud gravity center
    /** \return gravity center
    **/
    CCVector3 computeGravityCenter();

    //inherited from ccHObject
	virtual void getDrawingParameters(glDrawParams& params) const;

    //inherited from ccDrawableObject
    virtual bool hasColors() const;
    virtual bool hasNormals() const;
    virtual bool hasScalarFields() const;
    virtual bool hasDisplayedScalarField() const;

    //inherited from ccGenericPointCloud
	virtual bool isDisplayedSFPositive();
	virtual const colorType* getPointDistanceColor(unsigned pointIndex) const;
	virtual const colorType* getDistanceColor(DistanceType d) const;
	virtual DistanceType getPointDisplayedDistance(unsigned pointIndex) const;
	virtual const colorType* getPointColor(unsigned pointIndex) const;
	virtual const normsType getPointNormalIndex(unsigned pointIndex) const;
	virtual const PointCoordinateType* getPointNormal(unsigned pointIndex) const;
	/** WARNING: if removeSelectedPoints is true, any attached octree will be deleted.
	**/
	virtual ccGenericPointCloud* createNewCloudFromVisibilitySelection(bool removeSelectedPoints=false);
    virtual void applyRigidTransformation(const ccGLMatrix& trans);
    //virtual bool isScalarFieldEnabled() const;
    virtual void refreshBB();

    //! Sets a particular point color
    /** WARNING: colors must be enabled.
    **/
	void setPointColor(unsigned pointIndex, const colorType* col);

    //! Sets a particular point compressed normal
    /** WARNING: normals must be enabled.
    **/
	void setPointNormalIndex(unsigned pointIndex, normsType norm);

    //! Sets a particular point normal (shortcut)
    /** WARNING: normals must be enabled.
        Normal is automatically compressed before storage.
    **/
	void setPointNormal(unsigned pointIndex, const PointCoordinateType* N);

	//! Pushes a compressed normal vector
	/** \param index compressed normal vector
	**/
	void addNormIndex(normsType index);

	//! Pushes a normal vector on stack (shortcut)
	/** \param Nx normal vector (X)
        \param Ny normal vector (Y)
        \param Nz normal vector (Z)
    **/
	void addNorm(PointCoordinateType Nx, PointCoordinateType Ny, PointCoordinateType Nz);

	//! Pushes a normal vector on stack (shortcut)
	/** \param N normal vector (size: 3)
    **/
	void addNorm(const PointCoordinateType* N);

	//! Adds a normal vector to the one at a specific index
	/** The resulting sum is automatically normalized and compressed.
        \param N normal vector to add (size: 3)
        \param index normal index to modify
	**/
	void addNormAtIndex(const PointCoordinateType* N, unsigned index);

	//! Sets the (compressed) normals table
	void setNormsTable(NormsIndexesTableType* norms);

	//! Converts normals to RGB colors
	/** See ccNormalVectors::ConvertNormalToRGB
		\return success
	**/
	bool convertNormalToRGB();

	//! Pushes an RGB color on stack
    /** \param r red component
        \param g green component
        \param b blue component
    **/
	void addRGBColor(colorType r, colorType g, colorType b);

	//! Pushes an RGB color on stack
	/** \param C RGB color (size: 3)
	**/
	void addRGBColor(const colorType* C);

	//! Pushes a grey color on stack
	/** Shortcut: color is converted to RGB=(g,g,g).
        \param g grey component
	**/
	void addGreyColor(colorType g);

    //! Multiplies all color components of all points by coefficients
    /** If the cloud has no color, all points are considered white and
        the color array is automatically allocated.
        \param r red component
        \param g green component
        \param b blue component
    **/
	void colorize(float r, float g, float b);

	//! Assigns color to points proportionnaly to their 'height'
	/** Height is the coordinate along the specified dimension
        (heightDim). Color array is automatically allocated if
        necessary. The default color ramp is used.
        \param heightDim ramp dimension (0->X, 1->Y, 2->Z)
    **/
	void colorizeWithDefaultRamp(unsigned char heightDim);

	//! Assigns color to points proportionnaly to their 'height'
	/** Height is the coordinate along the specified dimension
        (heightDim: 0->X, 1->Y, 2->Z). Color array is automatically
        allocated if necessary. Linear color ramp is specified
        by starting and ending color.
        \param heightDim ramp dimension (0->X, 1->Y, 2->Z)
        \param minColor ramp starting RGB color (size: 3)
        \param maxColor ramp ending RGB color (size: 3)
    **/
	void colorizeByHeight(unsigned char heightDim, const colorType* minColor, const colorType* maxColor);

	//! Set a unique color for the whole cloud (shortcut)
	/** Color array is automatically allocated if necessary.
        \param r red component
        \param g green component
        \param b blue component
    **/
	void setRGBColor(colorType r, colorType g, colorType b);

	//! Set a unique color for the whole cloud
	/** Color array is automatically allocated if necessary.
        \param col RGB color (size: 3)
	**/
	void setRGBColor(const colorType* col);

    //! Inverts normals (if any)
	void invertNormals();

    //! Translates cloud
    /** \param T translation vector
    **/
	void translate(const CCVector3& T);

	//! Multiplies all coordinates by a constant factor (per dimension)
	/** WARNING: any attached octree will be deleted.
		\param fx multipliation factor along the X dimension
        \param fy multipliation factor along the Y dimension
        \param fz multipliation factor along the Z dimension
    **/
	void multiply(PointCoordinateType fx, PointCoordinateType fy, PointCoordinateType fz);

    //! Filters out points whose scalar values falls into an interval
    /** Threshold values should be expressed relatively to the current displayed scalar field.
        \param minVal minimum value (below, points are excluded)
        \param maxVal maximum value (above, points are excluded)
        \return resulting cloud (remaining points)
    **/
    ccPointCloud* filterPointsByScalarValue(DistanceType minVal, DistanceType maxVal);

    //! Hides points whose scalar values falls into an interval
    /** Values are taken from the current OUTPUT scalar field.
        \param minVal minimum value (below, points are hidden)
        \param maxVal maximum value (above, points are hidden)
    **/
    void hidePointsByScalarValue(DistanceType minVal, DistanceType maxVal);

	//! Unrolls the cloud and its normals on a cylinder
	/** This method is redundant with the "developCloudOnCylinder" method of CCLib,
		appart that it can also handle the cloud normals.
		\param radius unrolling cylinder radius
		\param center a point belonging to the cylinder axis (automatically computed if not specified)
		\param dim dimension along which the cylinder axis is aligned (X=0, Y=1, Z=2)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism
	**/
	void unrollOnCylinder(double radius, CCVector3* center=0, int dim=2, CCLib::GenericProgressCallback* progressCb=NULL);

	//! Unrolls the cloud and its normals on a cone
	/** This method is redundant with the "developCloudOnCone" method of CCLib,
		appart that it can also handle the cloud normals.
		\param baseRadius unrolling cone base radius
		\param alpha cone angle (between 0 and 180 degrees)
		\param apex cone apex
		\param dim dimension along which the cone axis is aligned (X=0, Y=1, Z=2)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism
	**/
	void unrollOnCone(double baseRadius, double alpha, const CCVector3& apex, int dim=2, CCLib::GenericProgressCallback* progressCb=NULL);

	//! Adds associated SF color ramp info to current GL context
	virtual void addColorRampInfo(CC_DRAW_CONTEXT& context);

    //inherited from ChunkedPointCloud
	virtual int addScalarField(const char* uniqueName, bool positive);

	//! Adds an existing scalar field to this cloud
	/** Warning: the cloud takes ownership of it!
		\param sf existing scalar field
		\return index of added scalar field (or -1 if an error occured)
	**/
	int addScalarField(ccScalarField* sf);

	//! Returns pointer on RGB colors table
	ColorsTableType* rgbColors() const {return m_rgbColors;}

	//! Returns pointer on compressed normals indexes table
	NormsIndexesTableType* normals() const {return m_normals;}

protected:

	//! Appends a cloud to this one
	const ccPointCloud& append(ccPointCloud* cloud, unsigned pointCountBefore);

    //inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);
    virtual void applyGLTransformation(const ccGLMatrix& trans);
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

    //inherited from ChunkedPointCloud
	virtual void swapPoints(unsigned firstIndex, unsigned secondIndex);

    //! Colors
	ColorsTableType* m_rgbColors;

	//! Normals (compressed)
	NormsIndexesTableType* m_normals;

    //! Specifies whether filtered out scalar values should be displayed (in gey) or not displayed at all
    bool m_greyForNanScalarValues;
	//! Specifies whether current scalar field color scale should be displayed or not
    bool m_sfColorScaleDisplayed;

	//! Currently displayed scalar field
	ccScalarField* m_currentDisplayedScalarField;
	//! Currently displayed scalar field index
	int m_currentDisplayedScalarFieldIndex;

private:

    //! Inits default parameters
	void init();

};

#endif
