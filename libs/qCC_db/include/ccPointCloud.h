#pragma once

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

#ifdef _MSC_VER
//To get rid of the warnings about dominant inheritance
#pragma warning( disable: 4250 )
#endif

//CCCoreLib
#include <PointCloudTpl.h>

//Local
#include "ccColorScale.h"
#include "ccNormalVectors.h"
#include "ccWaveform.h"

//Qt
#include <QOpenGLBuffer>

class ccScalarField;
class ccPolyline;
class ccMesh;
class QOpenGLBuffer;
class ccProgressDialog;
class ccPointCloudLOD;

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
	- an octree structure
	- per-point visibility information (to hide/display subsets of points)
	- other children objects (meshes, calibrated pictures, etc.)
**/
class QCC_DB_LIB_API ccPointCloud : public CCCoreLib::PointCloudTpl<ccGenericPointCloud, QString>
{
public:
	//! Base class (shortcut)
	using BaseClass = CCCoreLib::PointCloudTpl<ccGenericPointCloud, QString>;

	//! Default constructor
	/** Creates an empty cloud without any feature. Each of them should be
		specifically instantiated/created (once the points have been
		added to this cloud, at least partially).
		\param name cloud name (optional)
		\param uniqueID unique ID (handle with care)
	**/
	ccPointCloud(QString name = QString(), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID) throw();

	//! Default destructor
	~ccPointCloud() override;

	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POINT_CLOUD; }

public: //clone, copy, etc.

	//! Creates a new point cloud object from a GenericIndexedCloud
	/** "GenericIndexedCloud" is an extension of GenericCloud (from CCCoreLib)
		which provides a const random accessor to points.
		See CClib documentation for more information about GenericIndexedCloud.
		As the GenericIndexedCloud interface is very simple, only points are imported.
		Note: throws an 'int' exception in case of error (see CTOR_ERRORS)
		\param cloud a GenericIndexedCloud structure
		\param sourceCloud cloud from which main parameters will be imported (optional)
	**/
	static ccPointCloud* From(const CCCoreLib::GenericIndexedCloud* cloud, const ccGenericPointCloud* sourceCloud = nullptr);

	//! Creates a new point cloud object from a GenericCloud
	/** "GenericCloud" is a very simple and light interface from CCCoreLib. It is
		meant to give access to points coordinates of any cloud (on the
		condition it implements the GenericCloud interface of course).
		See CClib documentation for more information about GenericClouds.
		As the GenericCloud interface is very simple, only points are imported.
		Note: throws an 'int' exception in case of error (see CTOR_ERRORS)
		\param cloud a GenericCloud structure
		\param sourceCloud cloud from which main parameters will be imported (optional)
	**/
	static ccPointCloud* From(CCCoreLib::GenericCloud* cloud, const ccGenericPointCloud* sourceCloud = nullptr);

	//! Warnings for the partialClone method (bit flags)
	enum CLONE_WARNINGS {	WRN_OUT_OF_MEM_FOR_COLORS		= 1,
							WRN_OUT_OF_MEM_FOR_NORMALS		= 2,
							WRN_OUT_OF_MEM_FOR_SFS			= 4,
							WRN_OUT_OF_MEM_FOR_FWF			= 8
	};

	//! Creates a new point cloud object from a ReferenceCloud (selection)
	/** "Reference clouds" are a set of indexes referring to a real point cloud.
		See CClib documentation for more information about ReferenceClouds.
		Warning: the ReferenceCloud structure must refer to this cloud.
		\param[in]  selection			a ReferenceCloud structure (pointing to source)
		\param[out] warnings			[optional] to determine if warnings (CTOR_ERRORS) occurred during the duplication process
		\param[in]  withChildEntities	whether child entities should be transferred as well (see ccHObjectCaster::CloneChildren)
	**/
	ccPointCloud* partialClone(const CCCoreLib::ReferenceCloud* selection, int* warnings = nullptr, bool withChildEntities = true) const;

	//! Clones this entity
	/** All the main features of the entity are cloned, except from the octree and
		the points visibility information.
		\param destCloud [optional] the destination cloud can be provided here
		\param ignoreChildren [optional] whether to ignore the cloud's children or not (in which case they will be cloned as well)
		\return a copy of this entity
	**/
	ccPointCloud* cloneThis(ccPointCloud* destCloud = nullptr, bool ignoreChildren = false);

	//inherited from ccGenericPointCloud
	ccGenericPointCloud* clone(ccGenericPointCloud* destCloud = nullptr, bool ignoreChildren = false) override;

	//! Fuses another 3D entity with this one
	/** All the main features of the given entity are added, except from the octree and
		the points visibility information. Those features are deleted on this cloud.
	**/
	const ccPointCloud& operator +=(ccPointCloud*);

public: //features deletion/clearing

	//! Clears the entity from all its points and features
	/** Display parameters are also reset to their default values.
	**/
	void clear() override;

	//! Erases the cloud points
	/** Prefer ccPointCloud::clear by default.
		\warning DANGEROUS
	**/
	void unallocatePoints();

	//! Erases the cloud colors
	void unallocateColors();

	//! Erases the cloud normals
	void unallocateNorms();

	//! Notify a modification of color / scalar field display parameters or contents
	inline void colorsHaveChanged() { m_vboManager.updateFlags |= vboSet::UPDATE_COLORS; }
	//! Notify a modification of normals display parameters or contents
	inline void normalsHaveChanged() { m_vboManager.updateFlags |= vboSet::UPDATE_NORMALS; decompressNormals();}
	//! Notify a modification of points display parameters or contents
	inline void pointsHaveChanged() { m_vboManager.updateFlags |= vboSet::UPDATE_POINTS; }

public: //features allocation/resize

	//! Reserves memory to store the points coordinates
	/** Before adding points to the cloud (with addPoint())
		be sure to reserve the necessary amount of memory
		with this method. If the number of new elements is
		smaller than the actual one, nothing will happen.
		\param _numberOfPoints number of points to reserve the memory for
		\return true if ok, false if there's not enough memory
	**/
	bool reserveThePointsTable(unsigned _numberOfPoints);

	//! Reserves memory to store the RGB colors
	/** Before adding colors to the cloud (with addColor())
		be sure to reserve the necessary amount of memory
		with this method. This method reserves memory for as
		many colors as the number of points in the cloud
		(effectively stored or reserved).
		\return true if ok, false if there's not enough memory
	**/
	bool reserveTheRGBTable();

	//! Resizes the RGB colors array
	/** If possible, the colors array is resized to fit exactly the number
		of points in the cloud (effectively stored or reserved). If the
		new size is inferior to the actual one, the last elements will be
		deleted. Otherwise, the array is filled with zeros (default behavior)
		or "white" colors (is fillWithWhite).
		\warning don't try to "add" any element on a resized array...
		\param fillWithWhite whether to fill new array elements with zeros (false) or white color (true)
		\return true if ok, false if there's not enough memory
	**/
	bool resizeTheRGBTable(bool fillWithWhite = false);

	//! Reserves memory to store the compressed normals
	/** Before adding normals to the cloud (with addNorm())
		be sure to reserve the necessary amount of memory
		with this method. This method reserves memory for as
		many normals as the number of points in the cloud
		(effectively stored or reserved).
		\return true if ok, false if there's not enough memory
	**/
	bool reserveTheNormsTable();

	//! Resizes the compressed normals array
	/** If possible, the normals array is resized to fit exactly the number
		of points in the cloud (effectively stored or reserved). If the
		new size is inferior to the actual one, the last elements will be
		deleted. Otherwise, the array is filled with blank elements.
		\warning don't try to "add" any element on a resized array...
		\return true if ok, false if there's not enough memory
	**/
	bool resizeTheNormsTable();

	//! Reserves memory for all the active features
	/** This method is meant to be called before increasing the cloud
		population. Only the already allocated features will be re-reserved.
		\return true if ok, false if there's not enough memory
	**/
	bool reserve(unsigned numberOfPoints) override;

	//! Resizes all the active features arrays
	/** This method is meant to be called after having increased the cloud
		population (if the final number of inserted point is lower than the
		reserved size). Otherwise, it fills all new elements with blank values.
		\return true if ok, false if there's not enough memory
	**/
	bool resize(unsigned numberOfPoints) override;

	//! Removes unused capacity
	inline void shrinkToFit() { if (size() < capacity()) resize(size()); }

public: //scalar-fields management

	//! Returns the currently displayed scalar (or 0 if none)
	ccScalarField* getCurrentDisplayedScalarField() const;
	//! Returns the currently displayed scalar field index (or -1 if none)
	int getCurrentDisplayedScalarFieldIndex() const;
	//! Sets the currently displayed scalar field
	/** Warning: this scalar field will automatically be set as the OUTPUT one!
	**/
	void setCurrentDisplayedScalarField(int index);

	//inherited from base class
	void deleteScalarField(int index) override;
	void deleteAllScalarFields() override;
	int addScalarField(const std::string& uniqueName) override;

	//! Returns whether color scale should be displayed or not
	bool sfColorScaleShown() const;
	//! Sets whether color scale should be displayed or not
	void showSFColorsScale(bool state);

public: //associated (scan) grid structure

	//! Grid structure
	struct QCC_DB_LIB_API Grid : public ccSerializableObject
	{
		//! Shared type
		using Shared = QSharedPointer<Grid>;

		//! Default constructor
		Grid();

		//! Copy constructor
		/** \warning May throw a bad_alloc exception
		**/
		Grid(const Grid& grid) = default;

		//! Inits the grid
		bool init(unsigned rowCount, unsigned colCount, bool withRGB = false);

		//! Sets an index at a given position inside the grid
		void setIndex(unsigned row, unsigned col, int index);

		//! Sets a color at a given position inside the grid
		void setColor(unsigned row, unsigned col, const ccColor::Rgb& rgb);

		//! Updates the min and max valid indexes
		void updateMinAndMaxValidIndexes();

		//! Converts the grid to an RGB image (needs colors)
		QImage toImage() const;

		//inherited from ccSerializableObject
		inline bool isSerializable() const override { return true; }
		bool toFile(QFile& out, short dataVersion) const override;
		bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
		short minimumFileVersion() const override;

		//! Grid width
		unsigned w;
		//! Grid height
		unsigned h;

		//! Number of valid indexes
		unsigned validCount;
		//! Minimum valid index
		unsigned minValidIndex;
		//! Maximum valid index
		unsigned maxValidIndex;

		//! Grid indexes (size: w x h)
		std::vector<int> indexes;
		//! Grid colors (size: w x h, or 0 = no color)
		std::vector<ccColor::Rgb> colors;

		//! Sensor position (expressed relatively to the cloud points)
		ccGLMatrixd sensorPosition;
	};

	//! Returns the number of associated grids
	size_t gridCount() const { return m_grids.size(); }
	//! Returns an associated grid
	inline Grid::Shared& grid(size_t gridIndex) { return m_grids[gridIndex]; }
	//! Returns an associated grid (const version)
	inline const Grid::Shared& grid(size_t gridIndex) const { return m_grids[gridIndex]; }
	//! Adds an associated grid
	inline bool addGrid(Grid::Shared grid) { try{ m_grids.push_back(grid); } catch (const std::bad_alloc&) { return false; } return true; }
	//! Remove all associated grids
	inline void removeGrids() { m_grids.resize(0); }

	//! Meshes a scan grid
	/** \warning The mesh vertices will be this cloud instance!
	**/
	ccMesh* triangulateGrid(const Grid& grid, double minTriangleAngle_deg = 0.0) const;

public: //normals computation/orientation

	//! Compute the normals with the associated grid structure(s)
	/** Can also orient the normals in the same run.
	**/
	bool computeNormalsWithGrids(	double minTriangleAngle_deg = 1.0,
									ccProgressDialog* pDlg = nullptr );

	//! Orient the normals with the associated grid structure(s)
	bool orientNormalsWithGrids(    ccProgressDialog* pDlg = nullptr );

	//! Normals are forced to point to O
	bool orientNormalsTowardViewPoint( CCVector3 & VP, ccProgressDialog* pDlg = nullptr);

	//! Compute the normals by approximating the local surface around each point
	bool computeNormalsWithOctree(	CCCoreLib::LOCAL_MODEL_TYPES model,
									ccNormalVectors::Orientation preferredOrientation,
									PointCoordinateType defaultRadius,
									ccProgressDialog* pDlg = nullptr );

	//! Orient the normals with a Minimum Spanning Tree
	bool orientNormalsWithMST(		unsigned kNN = 6,
									ccProgressDialog* pDlg = nullptr );

	//! Orient normals with Fast Marching
	bool orientNormalsWithFM(		unsigned char level,
									ccProgressDialog* pDlg = nullptr);

	//! Toggle the drawing of normals as small lines
	void showNormalsAsLines(bool state);

	//! Are normals drawn as vectors?
	bool normalsAreDrawn() const;

	//! Set the length of the normals
	void setNormalLength(const float& value);

	//! Get the length of the normals
	const float& getNormalLength() {return m_normalLineParameters.length;}

	//! Colors of the normals when they are displayed
	enum NormalLineColors{
		YELLOW,
		RED,
		GREEN,
		BLUE,
		BLACK
	};

	//! Set the color of the normals
	void setNormalLineColor(int colorIdx);

	//! Get the color of the normals
	const int& getNormalLineColor() {return m_normalLineParameters.colorIdx;}

	//! Do the drawing of normals
	void drawNormalsAsLines(CC_DRAW_CONTEXT& context);

	//! Update the decompressed normals which are used by drawNormalsAsLines
	void decompressNormals();

public: //waveform (e.g. from airborne scanners)

	//! Returns whether the cloud has associated Full WaveForm data
	bool hasFWF() const;

	//! Returns a proxy on a given waveform
	ccWaveformProxy waveformProxy(unsigned index) const;

	//! Waveform descriptors set
	using FWFDescriptorSet = QMap<uint8_t, WaveformDescriptor>;

	//! Waveform data container
	using FWFDataContainer = std::vector<uint8_t>;
	using SharedFWFDataContainer = QSharedPointer<const FWFDataContainer>;

	//! Gives access to the FWF descriptors
	FWFDescriptorSet& fwfDescriptors() { return m_fwfDescriptors; }
	//! Gives access to the FWF descriptors (const version)
	const FWFDescriptorSet& fwfDescriptors() const { return m_fwfDescriptors; }

	//! Gives access to the associated FWF data
	std::vector<ccWaveform>& waveforms() { return m_fwfWaveforms; }
	//! Gives access to the associated FWF data (const version)
	const std::vector<ccWaveform>& waveforms() const { return m_fwfWaveforms; }

	//! Reserves the FWF table
	bool reserveTheFWFTable();
	//! Resizes the FWF table
	bool resizeTheFWFTable();

	//! Gives access to the associated FWF data container
	SharedFWFDataContainer& fwfData() { return m_fwfData; }
	//! Gives access to the associated FWF data container (const version)
	const SharedFWFDataContainer& fwfData() const { return m_fwfData; }

	//! Compresses the associated FWF data container
	/** As the container is shared, the compressed version will be potentially added to the memory
		resulting in a decrease of the available memory...
	**/
	bool compressFWFData();

	//! Computes the maximum amplitude of all associated waveforms
	bool computeFWFAmplitude(double& minVal, double& maxVal, ccProgressDialog* pDlg = nullptr) const;

	//! Clears all associated FWF data
	void clearFWFData();

public: //other methods

	//! Returns the cloud gravity center
	/** \return gravity center
	**/
	CCVector3 computeGravityCenter();

	//inherited from base class
	void invalidateBoundingBox() override;

	//inherited from ccHObject
	void getDrawingParameters(glDrawParams& params) const override;

	//inherited from ccDrawableObject
	bool hasColors() const override;
	bool hasNormals() const override;
	bool hasScalarFields() const override;
	bool hasDisplayedScalarField() const override;
	void removeFromDisplay(const ccGenericGLDisplay* win) override; //for proper VBO release
	void setDisplay(ccGenericGLDisplay* win) override;

	//inherited from CCCoreLib::GenericCloud
	unsigned char testVisibility(const CCVector3& P) const override;

	//inherited from CCCoreLib::GenericIndexedCloud
	bool normalsAvailable() const override { return hasNormals(); }
	const CCVector3* getNormal(unsigned pointIndex) const override; //equivalent to getPointNormal, but for CCCoreLib

	//inherited from ccGenericPointCloud
	const ccColor::Rgb* geScalarValueColor(ScalarType d) const override;
	const ccColor::Rgb* getPointScalarValueColor(unsigned pointIndex) const override;
	ScalarType getPointDisplayedDistance(unsigned pointIndex) const override;
	const ccColor::Rgba& getPointColor(unsigned pointIndex) const override;
	const CompressedNormType& getPointNormalIndex(unsigned pointIndex) const override;
	const CCVector3& getPointNormal(unsigned pointIndex) const override;
	CCCoreLib::ReferenceCloud* crop(const ccBBox& box, bool inside = true) override;
	void scale(PointCoordinateType fx, PointCoordinateType fy, PointCoordinateType fz, CCVector3 center = CCVector3(0,0,0)) override;
	/** \warning if removeSelectedPoints is true, any attached octree will be deleted, as well as the visibility table. **/
	ccGenericPointCloud* createNewCloudFromVisibilitySelection(	bool removeSelectedPoints = false,
																VisibilityTableType* visTable = nullptr,
																std::vector<int>* newIndexesOfRemainingPoints = nullptr,
																bool silent = false,
																CCCoreLib::ReferenceCloud* selection = nullptr) override;
	bool removeVisiblePoints(VisibilityTableType* visTable = nullptr, std::vector<int>* newIndexes = nullptr) override;
	void applyRigidTransformation(const ccGLMatrix& trans) override;
	inline void refreshBB() override { invalidateBoundingBox(); }

	//! Sets whether visibility check is enabled or not (e.g. during distances computation)
	/** See ccPointCloud::testVisibility.
	**/
	inline void enableVisibilityCheck(bool state) { m_visibilityCheckEnabled = state; }

	//! Returns whether the mesh as an associated sensor or not
	bool hasSensor() const;

	//! Comptes the closest point of this cloud relatively to another cloud
	/** The output (reference) clouds will have as many points as this cloud
		(with the indexes pointing on the closest point in the other cloud)
	**/
	QSharedPointer<CCCoreLib::ReferenceCloud> computeCPSet(	ccGenericPointCloud& otherCloud,
														CCCoreLib::GenericProgressCallback* progressCb = nullptr,
														unsigned char octreeLevel = 0);

	//! Interpolate colors from another cloud (nearest neighbor only)
	bool interpolateColorsFrom(	ccGenericPointCloud* cloud,
								CCCoreLib::GenericProgressCallback* progressCb = nullptr,
								unsigned char octreeLevel = 0);

	//! Sets a particular point color
	/** \warning colors must be enabled.
	**/
	void setPointColor(unsigned pointIndex, const ccColor::Rgba& col);

	//! Sets a particular point color
	/** \warning colors must be enabled.
	**/
	inline void setPointColor(unsigned pointIndex, const ccColor::Rgb& col) { setPointColor(pointIndex, ccColor::Rgba(col, ccColor::MAX)); }

	//! Sets a particular point compressed normal
	/** \warning normals must be enabled.
	**/
	void setPointNormalIndex(unsigned pointIndex, CompressedNormType norm);

	//! Sets a particular point normal (shortcut)
	/** \warning normals must be enabled.
		Normal is automatically compressed before storage.
	**/
	void setPointNormal(unsigned pointIndex, const CCVector3& N);

	//! Pushes a compressed normal vector
	/** \param index compressed normal vector
	**/
	void addNormIndex(CompressedNormType index);

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

	//! Pushes an RGBA color on stack
	/** \param C RGBA color
	**/
	void addColor(const ccColor::Rgba& C);

	//! Pushes an RGB color on stack
	/** Will be converted to an RGBA color automatically.
		\param C RGB color
	**/
	inline void addColor(const ccColor::Rgb& C) { addColor(ccColor::Rgba(C, ccColor::MAX)); }

	//! Pushes an RGB color on stack (shortcut)
	/** \param r red component
	    \param g green component
	    \param b blue component
	    \param a alpha component
	**/
	inline void addColor(ColorCompType r, ColorCompType g, ColorCompType b, ColorCompType a = ccColor::MAX) { addColor(ccColor::Rgba(r, g, b, a)); }

	//! Pushes a grey color on stack (shortcut)
	/** Shortcut: color is converted to RGB(g, g, g)
		\param g grey component
	**/
	inline void addGreyColor(ColorCompType g) { addColor(ccColor::Rgba(g, g, g, ccColor::MAX)); }

	//! Converts RGB to grey scale colors
	/** \return success
	**/
	bool convertRGBToGreyScale();

	//! Multiplies all color components of all points by coefficients
	/** If the cloud has no color, all points are considered white and
		the color array is automatically allocated.
		\param r red component
		\param g green component
		\param b blue component
		\param a alpha component
		\return success
	**/
	bool colorize(float r, float g, float b, float a = 1.0f);

	enum RGB_FILTER_TYPES
	{
		NONE,
		BILATERAL,
		GAUSSIAN,
		MEAN,
		MEDIAN
	};

	struct RgbFilterOptions
	{
		bool applyToSFduringRGB = false;
		RGB_FILTER_TYPES filterType = RGB_FILTER_TYPES::NONE;
		unsigned char burntOutColorThreshold = 0;
		bool commandLine = false;
		double sigmaSF = -1;
		double spatialSigma = -1;
		bool blendGrayscale = false;
		unsigned char blendGrayscaleThreshold = 0;
		double blendGrayscalePercent = 0.5;
	};

	//! Applies a spatial Gaussian filter on RGB colors
	/** The "amplitutde" of the Gaussian filter must be specified (sigma).
		As 99% of the Gaussian distribution is between -3*sigma and +3*sigma around the mean value,
		this filter will only look for neighbors within a sphere of radius 3*sigma.
		One can also use the filter as a Bilateral filter. In this case the weights are computed considering the
		difference of the neighbors SF values with the current point SF value (also following a Gaussian distribution).
		Warning: this method assumes the output scalar field is set.
		\param sigma filter variance
		\param sigmaRGB if strictly positive, the variance for the Bilateral filter
		\param applyToSF if true, it will apply filter to the displayed scalar field as well
		\param burntOutColorThreshold if >0 color values which are outside of the [threshold:255-threshold] range will be filtered out
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return success
	**/
	bool applyFilterToRGB(	PointCoordinateType sigma,
									PointCoordinateType sigmaSF,
									RgbFilterOptions filterParams,
									CCCoreLib::GenericProgressCallback* progressCb = nullptr);

	//! Assigns color to points proportionally to their 'height'
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
	bool setRGBColorByBanding(unsigned char dim, double freq);

	//! Converts current scalar field (values & display parameters) to RGB colors
	/** \return success
	**/
	bool convertCurrentScalarFieldToColors(bool mixWithExistingColor = false);

	//! Set a unique color for the whole cloud (shortcut)
	/** Color array is automatically allocated if necessary.
		\param r red component
		\param g green component
		\param b blue component
		\param a alpha component
		\return success
	**/
	inline bool setColor(ColorCompType r, ColorCompType g, ColorCompType b, ColorCompType a = ccColor::MAX) { return setColor(ccColor::Rgba(r, g, b, a)); }

	//! Set a unique color for the whole cloud (RGB version)
	/** Color array is automatically allocated if necessary.
		\param col RGB color
		\return success
	**/
	inline bool setColor(const ccColor::Rgb& col) { return setColor(ccColor::Rgba(col, ccColor::MAX)); }

	//! Set a unique color for the whole cloud (RGBA version)
	/** Color array is automatically allocated if necessary.
		\param col RGBA color
		\return success
	**/
	bool setColor(const ccColor::Rgba& col);

	//! Inverts normals (if any)
	void invertNormals();

	//! Translates cloud
	/** \param T translation vector
	**/
	void translate(const CCVector3& T);

	//! Filters out points whose scalar values falls into an interval
	/** Threshold values should be expressed relatively to the current displayed scalar field.
		\param minVal minimum value
		\param maxVal maximum value
		\param outside whether to select the points inside or outside of the specified interval
		\return resulting cloud (remaining points) or the cloud itself if all points fall inside the input range
	**/
	ccPointCloud* filterPointsByScalarValue(ScalarType minVal, ScalarType maxVal, bool outside = false);

	//! Hides points whose scalar values falls into an interval
	/** Values are taken from the current OUTPUT scalar field.
		\param minVal minimum value (below, points are hidden)
		\param maxVal maximum value (above, points are hidden)
	**/
	void hidePointsByScalarValue(ScalarType minVal, ScalarType maxVal);

	//! Unrolling mode (see ccPointCloud::unroll)
	enum UnrollMode { CYLINDER = 0, CONE_CONICAL = 1, CONE_CYLINDRICAL_FIXED_RADIUS = 2, CONE_CYLINDRICAL_ADAPTIVE_RADIUS = 3 };

	//! Base unrolling parameters
	struct UnrollBaseParams
	{
		PointCoordinateType radius = 0;	//!< Unrolling cylinder radius (or cone base radius)
		CCVector3 axisDir;				//!< Unrolling cylinder/cone axis direction
	};

	//! Cylinder unrolling parameters
	struct UnrollCylinderParams : public UnrollBaseParams
	{
		CCVector3 center;			//!< A point belonging to the cylinder axis
	};

	//! Cone unrolling parameters
	struct UnrollConeParams : public UnrollBaseParams
	{
		CCVector3 apex;				//!< Cone apex
		double coneAngle_deg = 0.0;	//!< Cone aperture angle (in degrees)
		double spanRatio = 0.5;		//!< Conical projection span ratio
	};

	//! Unrolls the cloud and its normals on a cylinder or a cone
	/** This method is redundant with the "developCloudOnCylinder" method of CCCoreLib,
		apart that it can also handle the cloud normals.
		\param mode unrolling mode
		\param params unrolling parameters (must match the unrolling mode)
		\param exportDeviationSF to export the deviations from the ideal shape as a scalar field
		\param startAngle_deg start angle (in degrees) - 0 corresponds to +X (east)
		\param stopAngle_deg stop angle (in degrees)
		\param arbitraryOutputCS whether the output cloud should be exported in an arbitrary coordinate system or not
		\param progressCb for progress notification
		\return the unrolled point cloud
	**/
	ccPointCloud* unroll(	UnrollMode mode,
							UnrollBaseParams* params,
							bool exportDeviationSF = false,
							double startAngle_deg = 0.0,
							double stopAngle_deg = 360.0,
							bool arbitraryOutputCS = false,
							CCCoreLib::GenericProgressCallback* progressCb = nullptr) const;

	//! Adds associated SF color ramp info to current GL context
	void addColorRampInfo(CC_DRAW_CONTEXT& context);

	//! Adds an existing scalar field to this cloud
	/** Warning: the cloud takes ownership of it!
		\param sf existing scalar field
		\return index of added scalar field (or -1 if an error occurred)
	**/
	int addScalarField(ccScalarField* sf);

	//! Returns pointer on RGBA colors table
	RGBAColorsTableType* rgbaColors() const { return m_rgbaColors; }

	//! Returns pointer on compressed normals indexes table
	NormsIndexesTableType* normals() const { return m_normals; }

	//! Crops the cloud inside (or outside) a 2D polyline
	/** \warning Always returns a selection (potentially empty) if successful.
		\param poly cropping polyline
		\param orthoDim dimension orthogonal to the plane in which the segmentation should occur (X=0, Y=1, Z=2)
		\param inside whether selected points are inside or outside the polyline
		\return points falling inside (or outside) as a selection
	**/
	CCCoreLib::ReferenceCloud* crop2D(const ccPolyline* poly, unsigned char orthoDim, bool inside = true);

	//! Appends a cloud to this one
	/** Same as the += operator with pointCountBefore == size()
		\param cloud cloud to be added
		\param pointCountBefore the number of points previously contained in this cloud
		\param ignoreChildren whether to copy input cloud's children or not
		\param recomputeMinAndMax whether to compute min and max of scalar fields after append or not
		\return the resulting point cloud
	**/
	const ccPointCloud& append(ccPointCloud* cloud, unsigned pointCountBefore, bool ignoreChildren = false, bool recomputeMinAndMax = true);

	//! Enhances the RGB colors with the current scalar field (assuming it's intensities)
	bool enhanceRGBWithIntensitySF(int sfIdx, bool useCustomIntensityRange = false, double minI = 0.0, double maxI = 1.0);

	//! Exports the specified coordinate dimension(s) to scalar field(s)
	bool exportCoordToSF(bool exportDims[3]);

	//! Sets coordinate(s) from a scalar field
	bool setCoordFromSF(bool importDims[3], CCCoreLib::ScalarField* sf, PointCoordinateType defaultValueForNaN);

	//! Exports the specified normal dimension(s) to scalar field(s)
	bool exportNormalToSF(bool exportDims[3]);

	//! Release VBOs
	void releaseVBOs();

	//! Returns the VBOs size (if any)
	size_t vboSize() const;

	//! Removes the duplicate points and return the corresponding cloud (if any, or the same cloud if there's no duplicate point)
	ccPointCloud* removeDuplicatePoints(double minDistanceBetweenPoints, ccProgressDialog* pDlg = nullptr);

	//! Shift the points of a given quantity along their normals
	bool shiftPointsAlongNormals(PointCoordinateType shift);

	//! Set the path for shaders
	static void SetShaderPath(const QString &path);

	//! Release shaders (if any)
	/** Must be called before the OpenGL context is released.
	**/
	static void ReleaseShaders();

protected:

	//inherited from ccHObject
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	void applyGLTransformation(const ccGLMatrix& trans) override;
	bool toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;
	void notifyGeometryUpdate() override;

	//inherited from PointCloud
	/** \warning Doesn't handle scan grids!
	**/
	void swapPoints(unsigned firstIndex, unsigned secondIndex) override;

protected: // variable members

	//! Colors
	RGBAColorsTableType* m_rgbaColors;

	//! Normals (compressed)
	NormsIndexesTableType* m_normals;

	//! Used for drawing normals if needed
	std::vector<CCVector3> m_decompressedNormals;

	//! Specifies whether current scalar field color scale should be displayed or not
	bool m_sfColorScaleDisplayed;

	//! Currently displayed scalar field
	ccScalarField* m_currentDisplayedScalarField;
	//! Currently displayed scalar field index
	int m_currentDisplayedScalarFieldIndex;

	//! Associated grid structure
	std::vector<Grid::Shared> m_grids;

	//! Whether visibility check is available or not (during comparison)
	/** See ccPointCloud::testVisibility
	**/
	bool m_visibilityCheckEnabled;

protected: // VBO

	//! Init/updates VBOs
	bool updateVBOs(const CC_DRAW_CONTEXT& context, const glDrawParams& glParams);

	class VBO : public QOpenGLBuffer
	{
	public:
		int rgbShift;
		int normalShift;

		//! Inits the VBO
		/** \return the number of allocated bytes (or -1 if an error occurred)
		**/
		int init(int count, bool withColors, bool withNormals, bool* reallocated = nullptr);

		VBO()
			: QOpenGLBuffer(QOpenGLBuffer::VertexBuffer)
			, rgbShift(0)
			, normalShift(0)
		{}
	};

	//! VBO set
	struct vboSet
	{
		//! States of the VBO(s)
		enum STATES { NEW, INITIALIZED, FAILED };

		//! Update flags
		enum UPDATE_FLAGS {
			UPDATE_POINTS = 1,
			UPDATE_COLORS = 2,
			UPDATE_NORMALS = 4,
			UPDATE_ALL = UPDATE_POINTS | UPDATE_COLORS | UPDATE_NORMALS
		};

		vboSet()
			: hasColors(false)
			, colorIsSF(false)
			, sourceSF(nullptr)
			, hasNormals(false)
			, totalMemSizeBytes(0)
			, updateFlags(0)
			, state(NEW)
		{}

		std::vector<VBO*> vbos;
		bool hasColors;
		bool colorIsSF;
		ccScalarField* sourceSF;
		bool hasNormals;
		size_t totalMemSizeBytes;
		int updateFlags;

		//! Current state
		STATES state;
	};

	//! Set of VBOs attached to this cloud
	vboSet m_vboManager;

	//per-block data transfer to the GPU (VBO or standard mode)
	void glChunkVertexPointer(const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs);
	void glChunkColorPointer (const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs);
	void glChunkSFPointer    (const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs);
	void glChunkNormalPointer(const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs);

public: //Level of Detail (LOD)

	//! Initializes the LOD structure
	/** \return success
	**/
	bool initLOD();

	//! Clears the LOD structure
	void clearLOD();

	//! Returns if the cloud has a valuable LOD
	bool hasUsableLOD() const;

	//! Getter for the m_useLODRendering member
	bool useLODRendering() const;

	//! Mutates the m_useLODRendering member (setter)
	void setLODRendering(bool);

protected: //Level of Detail (LOD)

	//! L.O.D. structure
	ccPointCloudLOD* m_lod;

	//! Boolean flag indicating whether this specific cloud should
	//! be rendered using the LOD mechanism. (see its usage in DrawMeOnly)
	bool m_useLODRendering;

protected: //waveform (e.g. from airborne scanners)

	//! General waveform descriptors
	FWFDescriptorSet m_fwfDescriptors;

	//! Per-point waveform accessors
	std::vector<ccWaveform> m_fwfWaveforms;

	//! Waveforms raw data storage
	SharedFWFDataContainer m_fwfData;

protected: //Normals drawing
	bool m_normalsDrawnAsLines;

	struct NormalLineParameters
	{
		float length = 1.0f;
		ccColor::Rgba color = ccColor::yellow;
		int colorIdx = YELLOW;
	} m_normalLineParameters;
};
