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
#include <CCPlatform.h>

//Local
#include "qCC_db.h"
#include "ccGenericPointCloud.h"
#include "ccColorScale.h"

//Qt
#include <QGLBuffer>
#include <QMutex>

class ccPointCloud;
class ccScalarField;
class ccPolyline;
class QGLBuffer;
class LodStructThread;

//! Maximum number of points (per cloud) displayed in a single LOD iteration
/** \warning MUST BE GREATER THAN 'MAX_NUMBER_OF_ELEMENTS_PER_CHUNK'
**/
static const unsigned MAX_POINT_COUNT_PER_LOD_RENDER_PASS = MAX_NUMBER_OF_ELEMENTS_PER_CHUNK * 10; //~ 650K

/***************************************************
				ccPointCloud
***************************************************/

//! Max number of points per cloud (point cloud will be chunked above this limit)
#if defined(CC_ENV_32)
const unsigned CC_MAX_NUMBER_OF_POINTS_PER_CLOUD =  128000000;
#else //CC_ENV_64 (but maybe CC_ENV_128 one day ;)
const unsigned CC_MAX_NUMBER_OF_POINTS_PER_CLOUD = 2000000000; //we must keep it below MAX_INT to avoid probable issues ;)
#endif

//! A 3D cloud and its associated features (color, normals, scalar fields, etc.)
/** A point cloud can have multiple features:
	- colors (RGB)
	- normals (compressed)
	- scalar fields
	- an octree strucutre
	- per-point visibility information (to hide/display subsets of points)
	- other children objects (meshes, calibrated pictures, etc.)
**/
class QCC_DB_LIB_API ccPointCloud : public CCLib::ChunkedPointCloud, public ccGenericPointCloud
{
public:

	//! Default constructor
	/** Creates an empty cloud without any feature. Each of them shoud be
		specifically instantiated/created (once the points have been
		added to this cloud, at least partially).
		\param name cloud name (optional)
	**/
	ccPointCloud(QString name = QString()) throw();

	//! Default destructor
	virtual ~ccPointCloud();

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::POINT_CLOUD; }

	//inherited from ChunkedPointCloud
	virtual void invalidateBoundingBox();

	/***************************************************
						Clone/Copy
	***************************************************/

	//! Creates a new point cloud object from a GenericIndexedCloud
	/** "GenericIndexedCloud" is an extension of GenericCloud (from CCLib)
		which provides a const random accessor to points.
		See CClib documentation for more information about GenericIndexedCloud.
		As the GenericIndexedCloud interface is very simple, only points are imported.
		Note: throws an 'int' exception in case of error (see CTOR_ERRORS)
		\param cloud a GenericIndexedCloud structure
		\param sourceCloud cloud from which main parameters will be imported (optional)
	**/
	static ccPointCloud* From(const CCLib::GenericIndexedCloud* cloud, const ccGenericPointCloud* sourceCloud = 0);

	//! Creates a new point cloud object from a GenericCloud
	/** "GenericCloud" is a very simple and light interface from CCLib. It is
		meant to give access to points coordinates of any cloud (on the
		condition it implements the GenericCloud interface of course).
		See CClib documentation for more information about GenericClouds.
		As the GenericCloud interface is very simple, only points are imported.
		Note: throws an 'int' exception in case of error (see CTOR_ERRORS)
		\param cloud a GenericCloud structure
		\param sourceCloud cloud from which main parameters will be imported (optional)
	**/
	static ccPointCloud* From(CCLib::GenericCloud* cloud, const ccGenericPointCloud* sourceCloud = 0);

	//! Warnings for the partialClone method (bit flags)
	enum CLONE_WARNINGS {	WRN_OUT_OF_MEM_FOR_COLORS		= 1,
							WRN_OUT_OF_MEM_FOR_NORMALS		= 2,
							WRN_OUT_OF_MEM_FOR_SFS			= 4
	};

	//! Creates a new point cloud object from a ReferenceCloud (selection)
	/** "Reference clouds" are a set of indexes referring to a real point cloud.
		See CClib documentation for more information about ReferenceClouds.
		Warning: the ReferenceCloud structure must refer to this cloud. 
		\param selection a ReferenceCloud structure (pointing to source)
		\param[out] warnings [optional] to determine if warnings (CTOR_ERRORS) occurred during the duplication process
	**/
	ccPointCloud* partialClone(const CCLib::ReferenceCloud* selection, int* warnings = 0) const;

	//! Clones this entity
	/** All the main features of the entity are cloned, except from the octree and
		the points visibility information.
		\param destCloud [optional] the destination cloud can be provided here
		\param ignoreChildren [optional] whether to ignore the cloud's children or not (in which case they will be cloned as well)
		\return a copy of this entity
	**/
	virtual ccPointCloud* cloneThis(ccPointCloud* destCloud = 0, bool ignoreChildren = false);

	//inherited from ccGenericPointCloud
	virtual ccGenericPointCloud* clone(ccGenericPointCloud* destCloud = 0, bool ignoreChildren = false);

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

	//! Erases the cloud points
	/** Prefer ccPointCloud::clear by default.
		\warning DANGEROUS
	**/
	void unalloactePoints();

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
	bool resizeTheRGBTable(bool fillWithWhite = false);

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
	virtual bool reserve(unsigned numberOfPoints);

	//! Resizes all the active features arrays
	/** This method is meant to be called after having increased the cloud
		population (if the final number of insterted point is lower than the
		reserved size). Otherwise, it fills all new elements with blank values.
		\return true if ok, false if there's not enough memory
	**/
	virtual bool resize(unsigned numberOfPoints);


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

	//inherited from ChunkedPointCloud
	virtual void deleteScalarField(int index);
	virtual void deleteAllScalarFields();
	virtual int addScalarField(const char* uniqueName);

	//! Returns whether color scale should be displayed or not
	bool sfColorScaleShown() const;
	//! Sets whether color scale should be displayed or not
	void showSFColorsScale(bool state);


	/***************************************************
						Other methods
	***************************************************/

	//! Returns the cloud gravity center
	/** \return gravity center
	**/
	CCVector3 computeGravityCenter();

	//inherited from ccHObject
	virtual void getDrawingParameters(glDrawParams& params) const;
	virtual unsigned getUniqueIDForDisplay() const;

	//inherited from ccDrawableObject
	virtual bool hasColors() const;
	virtual bool hasNormals() const;
	virtual bool hasScalarFields() const;
	virtual bool hasDisplayedScalarField() const;
	virtual void removeFromDisplay(const ccGenericGLDisplay* win); //for proper VBO release

	//inherited from ccGenericPointCloud
	virtual const colorType* getPointScalarValueColor(unsigned pointIndex) const;
	virtual const colorType* geScalarValueColor(ScalarType d) const;
	virtual ScalarType getPointDisplayedDistance(unsigned pointIndex) const;
	virtual const colorType* getPointColor(unsigned pointIndex) const;
	virtual const normsType& getPointNormalIndex(unsigned pointIndex) const;
	virtual const CCVector3& getPointNormal(unsigned pointIndex) const;
	CCLib::ReferenceCloud* crop(const ccBBox& box, bool inside = true);
	virtual void scale(PointCoordinateType fx, PointCoordinateType fy, PointCoordinateType fz, CCVector3 center = CCVector3(0,0,0));
	/** \warning if removeSelectedPoints is true, any attached octree will be deleted. **/
	virtual ccGenericPointCloud* createNewCloudFromVisibilitySelection(bool removeSelectedPoints = false);
	virtual void applyRigidTransformation(const ccGLMatrix& trans);
	//virtual bool isScalarFieldEnabled() const;
	inline virtual void refreshBB() { invalidateBoundingBox(); }

	//! Interpolate colors from another cloud
	bool interpolateColorsFrom(	ccGenericPointCloud* cloud,
								CCLib::GenericProgressCallback* progressCb = NULL,
								unsigned char octreeLevel = 7);

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
	void setPointNormal(unsigned pointIndex, const CCVector3& N);

	//! Pushes a compressed normal vector
	/** \param index compressed normal vector
	**/
	void addNormIndex(normsType index);

	//! Pushes a normal vector on stack (shortcut)
	/** \param N normal vector
	**/
	void addNorm(const CCVector3& N);

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

	//! Converts normals to two scalar fields: 'dip' and 'dip direction'
	/**	One input scalar field may be empty if the corresponding value is not required
		\param[out] dipSF dip values
		\param[out] dipDirSF dip direction values
		\return success
	**/
	bool convertNormalToDipDirSFs(ccScalarField* dipSF, ccScalarField* dipDirSF);

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
		\return success
    **/
	bool colorize(float r, float g, float b);

	//! Assigns color to points proportionnaly to their 'height'
	/** Height is defined wrt to the specified dimension (heightDim).
		Color array is automatically allocated if necessary.
        \param heightDim ramp dimension (0:X, 1:Y, 2:Z)
		\param colorScale color scale to use
		\return success
    **/
	bool setRGBColorByHeight(unsigned char heightDim, ccColorScale::Shared colorScale);

	//! Assigns color to points by 'banding'
	/** Banding is performed along the specified dimension
		Color array is automatically allocated if necessary.
        \param dim banding dimension (0:X, 1:Y, 2:Z)
		\param freq banding frequency
		\return success
    **/
	bool setRGBColorByBanding(unsigned char dim, int freq);

	//! Sets RGB colors with current scalar field (values & parameters)
	/** \return success
	**/
	bool setRGBColorWithCurrentScalarField(bool mixWithExistingColor = false);
	
	//! Set a unique color for the whole cloud (shortcut)
	/** Color array is automatically allocated if necessary.
        \param r red component
        \param g green component
        \param b blue component
		\return success
    **/
	bool setRGBColor(colorType r, colorType g, colorType b);

	//! Set a unique color for the whole cloud
	/** Color array is automatically allocated if necessary.
        \param col RGB color (size: 3)
		\return success
	**/
	bool setRGBColor(const ccColor::Rgb& col);

    //! Inverts normals (if any)
	void invertNormals();

    //! Translates cloud
    /** \param T translation vector
    **/
	void translate(const CCVector3& T);

    //! Filters out points whose scalar values falls into an interval
    /** Threshold values should be expressed relatively to the current displayed scalar field.
        \param minVal minimum value (below, points are excluded)
        \param maxVal maximum value (above, points are excluded)
        \return resulting cloud (remaining points)
    **/
    ccPointCloud* filterPointsByScalarValue(ScalarType minVal, ScalarType maxVal);

    //! Hides points whose scalar values falls into an interval
    /** Values are taken from the current OUTPUT scalar field.
        \param minVal minimum value (below, points are hidden)
        \param maxVal maximum value (above, points are hidden)
    **/
    void hidePointsByScalarValue(ScalarType minVal, ScalarType maxVal);

	//! Unrolls the cloud and its normals on a cylinder
	/** This method is redundant with the "developCloudOnCylinder" method of CCLib,
		appart that it can also handle the cloud normals.
		\param radius unrolling cylinder radius
		\param center a point belonging to the cylinder axis (automatically computed if not specified)
		\param dim dimension along which the cylinder axis is aligned (X=0, Y=1, Z=2)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism
	**/
	void unrollOnCylinder(	PointCoordinateType radius,
							CCVector3* center = 0,
							unsigned char dim = 2,
							CCLib::GenericProgressCallback* progressCb = NULL);

	//! Unrolls the cloud and its normals on a cone
	/** This method is redundant with the "developCloudOnCone" method of CCLib,
		appart that it can also handle the cloud normals.
		\param baseRadius unrolling cone base radius
		\param alpha_deg cone angle (between 0 and 180 degrees)
		\param apex cone apex
		\param dim dimension along which the cone axis is aligned (X=0, Y=1, Z=2)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism
	**/
	void unrollOnCone(	PointCoordinateType baseRadius,
						double alpha_deg,
						const CCVector3& apex,
						unsigned char dim = 2,
						CCLib::GenericProgressCallback* progressCb = NULL);

	//! Adds associated SF color ramp info to current GL context
	virtual void addColorRampInfo(CC_DRAW_CONTEXT& context);

	//! Adds an existing scalar field to this cloud
	/** Warning: the cloud takes ownership of it!
		\param sf existing scalar field
		\return index of added scalar field (or -1 if an error occurred)
	**/
	int addScalarField(ccScalarField* sf);

	//! Returns pointer on RGB colors table
	ColorsTableType* rgbColors() const { return m_rgbColors; }

	//! Returns pointer on compressed normals indexes table
	NormsIndexesTableType* normals() const { return m_normals; }

	//! Crops the cloud inside (or outside) a 2D polyline
	/** \warning Always returns a selection (potentially empty) if successful.
		\param poly croping polyline
		\param orthoDim dimension orthogonal to the plane in which the segmentation should occur (X=0, Y=1, Z=2)
		\param inside whether selected points are inside or outside the polyline
		\return points falling inside (or outside) as a selection
	**/
	CCLib::ReferenceCloud* crop2D(const ccPolyline* poly, unsigned char orthoDim, bool inside = true);

	//! Appends a cloud to this one
	/** Same as the += operator with pointCountBefore == size()
		\param cloud cloud to be added
		\parm pointCountBefore the number of points previously contained in this cloud
		\param whether to copy clouds's children or not
		\return the resulting point cloud
	**/
	const ccPointCloud& append(ccPointCloud* cloud, unsigned pointCountBefore, bool ignoreChildren = false);

protected:

	//inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);
	virtual void applyGLTransformation(const ccGLMatrix& trans);
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);
	virtual void notifyGeometryUpdate();

	//inherited from ChunkedPointCloud
	virtual void swapPoints(unsigned firstIndex, unsigned secondIndex);

	//! Colors
	ColorsTableType* m_rgbColors;

	//! Normals (compressed)
	NormsIndexesTableType* m_normals;

	//! Specifies whether current scalar field color scale should be displayed or not
	bool m_sfColorScaleDisplayed;

	//! Currently displayed scalar field
	ccScalarField* m_currentDisplayedScalarField;
	//! Currently displayed scalar field index
	int m_currentDisplayedScalarFieldIndex;

protected: // VBO

	//! Init/updates VBOs
	bool updateVBOs(const glDrawParams& glParams);

	//! Release VBOs
	void releaseVBOs();

	class VBO : public QGLBuffer
	{
	public:
		int rgbShift;
		int normalShift;

		//! Inits the VBO
		/** \return the number of allocated bytes (or -1 if an error occurred)
		**/
		int init(int count, bool withColors, bool withNormals, bool* reallocated = 0);

		VBO()
			: QGLBuffer(QGLBuffer::VertexBuffer)
			, rgbShift(0)
			, normalShift(0)
		{}
	};

	//! VBO set
	struct vboSet
	{
		//! States of th VBO(s)
		enum STATES { NEW, INITIALIZED, FAILED };

		vboSet()
			: hasColors(false)
			, colorIsSF(false)
			, sourceSF(0)
			, hasNormals(false)
			, totalMemSizeBytes(0)
			, state(NEW)
		{}

		std::vector<VBO*> vbos;
		bool hasColors;
		bool colorIsSF;
		ccScalarField* sourceSF;
		bool hasNormals;
		int totalMemSizeBytes;

		//! Current state
		STATES state;
	};

	//! Set of VBOs attached to this cloud
	vboSet m_vboManager;

	//per-block data transfer to the GPU (VBO or standard mode)
	void glChunkVertexPointer(unsigned chunkIndex, unsigned decimStep, bool useVBOs);
	void glChunkColorPointer (unsigned chunkIndex, unsigned decimStep, bool useVBOs);
	void glChunkSFPointer    (unsigned chunkIndex, unsigned decimStep, bool useVBOs);
	void glChunkNormalPointer(unsigned chunkIndex, unsigned decimStep, bool useVBOs);

public: //Level of Detail (LOD)

public: //Level of Detail (LOD)

	//! L.O.D. (Level of Detail) structure
	class LodStruct
	{
	public:
		//! Structure initialization state
		enum State { NOT_INITIALIZED, UNDER_CONSTRUCTION, INITIALIZED, BROKEN };

		//! Default constructor
		LodStruct() : m_indexes(0), m_thread(0), m_state(NOT_INITIALIZED) {}
		//! Destructor
		~LodStruct() { clear(); }

		//! Initializes the construction process (asynchronous)
		bool init(ccPointCloud& cloud);

		//! Locks the structure
		inline void lock() { m_mutex.lock(); }
		//! Unlocks the structure
		inline void unlock() { m_mutex.unlock(); }

		//! Returns the current state
		inline State getState() { lock(); State state = m_state; unlock(); return state; }

		//! Sets the current state
		inline void setState(State state) { lock(); m_state = state; unlock(); }

		//! Clears the structure
		inline void clear() { clearExtended(true, NOT_INITIALIZED); }
		//! Clears the structure (extended version)
		void clearExtended(bool autoStopThread, State newState);

		//! Reserves memory for the indexes
		bool reserve(unsigned pointCount, int levelCount);

		//! Returns whether the structure is null (i.e. not under construction or initialized) or not
		inline bool isNull() { return getState() == NOT_INITIALIZED; }

		//! Returns whether the structure is initialized or not
		inline bool isInitialized() { return getState() == INITIALIZED; }

		//! Returns whether the structure is initialized or not
		inline bool isUnderConstruction() { return getState() == UNDER_CONSTRUCTION; }

		//! Returns whether the structure is broken or not
		inline bool isBroken() { return getState() == BROKEN; }

		//! L.O.D. indexes set
		typedef GenericChunkedArray<1, unsigned> IndexSet;

		//! Returns the indexes (if any)
		inline IndexSet* indexes() { return m_indexes; }
		//! Returns the indexes (if any) - const version
		inline const IndexSet* indexes() const { return m_indexes; }

		//! Level descriptor
		struct LevelDesc
		{
			//! Default constructor
			LevelDesc() : startIndex(0), count(0) {}
			//! Constructor from a start index and a count value
			LevelDesc(unsigned _startIndex, unsigned _count) : startIndex(_startIndex), count(_count) {}
			//! Start index (refers to the 'indexes' table)
			unsigned startIndex;
			//! Index count for this level
			unsigned count;
		};

		//! Adds a level descriptor
		inline void addLevel(const LevelDesc& desc) { lock(); m_levels.push_back(desc); unlock(); }
		//! Shrinks the level descriptor set to its minimal size
		inline void shrink() { lock(); m_levels.resize(m_levels.capacity()); unlock(); }

		//! Returns the maximum level
		inline unsigned char maxLevel() { lock(); size_t count = m_levels.size(); unlock(); return static_cast<unsigned char>(std::min<size_t>(count,256)); }
		//! Returns a given level descriptor
		inline LevelDesc level(unsigned char index) { lock(); LevelDesc desc = m_levels[index]; unlock(); return desc; }

	protected:

		//! L.O.D. indexes
		/** Point indexes that should be displayed at each level of detail.
		**/
		IndexSet* m_indexes;

		//! Actual levels
		std::vector<LevelDesc> m_levels;

		//! Computing thread
		LodStructThread* m_thread;

		//! For concurrent access
		QMutex m_mutex;

		//! State
		State m_state;
	};

	//! Intializes the LOD structure
	/** \return success
	**/
	bool initLOD(CCLib::GenericProgressCallback* progressCallback = 0);

	//! Clears the LOD structure
	inline void clearLOD() { m_lod.clear(); }

	//! Returns the LOD structure
	inline LodStruct& getLOD() { return m_lod; }

protected:

	//per-block data transfer to the GPU (LOD)
	void glLODChunkVertexPointer(const LodStruct::IndexSet& indexMap, unsigned startIndex, unsigned stopIndex);
	void glLODChunkColorPointer (const LodStruct::IndexSet& indexMap, unsigned startIndex, unsigned stopIndex);
	void glLODChunkSFPointer    (const LodStruct::IndexSet& indexMap, unsigned startIndex, unsigned stopIndex);
	void glLODChunkNormalPointer(const LodStruct::IndexSet& indexMap, unsigned startIndex, unsigned stopIndex);

	//! L.O.D. structure
	LodStruct m_lod;
};

#endif //CC_POINT_CLOUD_HEADER
