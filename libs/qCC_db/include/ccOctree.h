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

#ifndef CC_OCTREE_HEADER
#define CC_OCTREE_HEADER

//Local
#include "ccGenericGLDisplay.h"
#include "ccHObject.h"

//CCCoreLib
#include <DgmOctree.h>
#include <ReferenceCloud.h>

//Qt
#include <QObject>

class ccGenericPointCloud;
class ccOctreeFrustumIntersector;
class ccCameraSensor;

//! Octree structure
/** Extends the CCCoreLib::DgmOctree class.
**/
class QCC_DB_LIB_API ccOctree : public QObject, public CCCoreLib::DgmOctree
{
	Q_OBJECT

public: //GENERAL METHODS

	//! Shared pointer
	typedef QSharedPointer<ccOctree> Shared;

	//! Default constructor
	/** \param cloud a point cloud
	**/
	explicit ccOctree(ccGenericPointCloud* cloud);

	//! Destructor
	virtual ~ccOctree();

	//! Multiplies the bounding-box of the octree
	/** If the cloud coordinates are simply multiplied by the same factor,
		there is no use in recomputing the octree structure. It's sufficient
		to update its bounding-box.
		\param  multFactor multiplication factor
	**/
	void multiplyBoundingBox(const PointCoordinateType multFactor);

	//! Translates the bounding-box of the octree
	/** If the cloud has been simply translated, there is no use to recompute
		the octree structure. It's sufficient to update its bounding-box.
		\param T translation vector
	**/
	void translateBoundingBox(const CCVector3& T);

	//! Returns the octree (square) bounding-box
	ccBBox getSquareBB() const;
	//! Returns the points bounding-box
	ccBBox getPointsBB() const;

	//inherited from DgmOctree
	virtual void clear() override;

public: //RENDERING
	
	//! Returns the currently displayed octree level
	int getDisplayedLevel() const { return m_displayedLevel; }
	//! Sets the currently displayed octree level
	void setDisplayedLevel(int level);

	//! Octree displaying methods
	enum DisplayMode {
		WIRE = 0,			/**< The octree is displayed as wired boxes (one box per cell) */
		MEAN_POINTS = 1,		/**< The octree is displayed as points (one point per cell = the center of gravity of the points lying in it) */
		MEAN_CUBES = 2			/**< The octree is displayed as plain 3D cubes (one cube per cell) */
	};
	//! Returns the currently display mode
	DisplayMode getDisplayMode() const { return m_displayMode; }
	//! Sets the currently display mode
	void setDisplayMode(DisplayMode mode);

	//! Draws the octree
	void draw(CC_DRAW_CONTEXT& context, ccColor::Rgb* pickingColor = nullptr);

	//! Intersects octree with a camera sensor
	bool intersectWithFrustum(	ccCameraSensor* sensor,
								std::vector<unsigned>& inCameraFrustum);

	//! Octree-driven point picking algorithm
	bool pointPicking(	const CCVector2d& clickPos,
						const ccGLCameraParameters& camera,
						PointDescriptor& output,
						double pickWidth_pix = 3.0) const;

public: //HELPERS
	
	//! Computes the average color of a set of points
	static ccColor::Rgb ComputeAverageColor(	CCCoreLib::ReferenceCloud* subset,
												ccGenericPointCloud* sourceCloud);

	//! Computes the average normal of a set of points
	static CCVector3 ComputeAverageNorm(CCCoreLib::ReferenceCloud* subset,
										ccGenericPointCloud* sourceCloud);

	//! Tries to guess a very naive 'local radius' for octree-based computation
	/** \param cloud	point cloud on which to process the normals.
		\return naive radius (one percent of the cloud largest dimension by default, unless the cloud have very few points)
	**/
	static PointCoordinateType GuessNaiveRadius(ccGenericPointCloud* cloud);

	//! Parameters for the GuessBestRadius method
	struct BestRadiusParams
	{
		int aimedPopulationPerCell = 16;	//!< Aimed poulation per octree cell
		int aimedPopulationRange = 4;		//!< Aimed poulation range per octree cell
		int minCellPopulation = 6;			//!< Minimum cell poulation
		double minAboveMinRatio = 0.97;		//!< Ratio of cells above the 'minCellPopulation' thershold
	};

	//! Tries to guess the best 'local radius' for octree-based computation
	/**	The ideal radius is determined by randomly sampling up to 200 points and looking at
		their neighborhood.
		\param cloud		point cloud on which to process the normals.
		\param params		parameters
		\param cloudOctree	input cloud octree (optional)
		\param progressCb	progress notification (optional)
		\return the best radius (strictly positive value) or 0 if an error occurred
	**/
	static PointCoordinateType GuessBestRadius(	ccGenericPointCloud* cloud,
												const BestRadiusParams& params,
												CCCoreLib::DgmOctree* cloudOctree = nullptr,
												CCCoreLib::GenericProgressCallback* progressCb = nullptr);

	//! Tries to guess the best 'local radius' for octree-based computation (auto-computes the octree if necessary)
	/**	The ideal radius is determined by randomly sampling up to 200 points and looking at
		their neighborhood.
		\param cloud		point cloud on which to process the normals.
		\param params		parameters
		\param parentWidget	parent widget (for the progress dialog, if any has to be shown)
		\return the best radius (strictly positive value) or 0 if an error occurred
	**/
	static PointCoordinateType GuessBestRadiusAutoComputeOctree(ccGenericPointCloud* cloud,
																const BestRadiusParams& params,
																QWidget* parentWidget = nullptr);

Q_SIGNALS:

	//! Signal sent when the octree organization is modified (cleared, etc.)
	void updated();

protected: ////RENDERING

	static bool DrawCellAsABox(	const CCCoreLib::DgmOctree::octreeCell& cell,
								void** additionalParameters,
								CCCoreLib::NormalizedProgress* nProgress = 0);

	static bool DrawCellAsAPoint(	const CCCoreLib::DgmOctree::octreeCell& cell,
									void** additionalParameters,
									CCCoreLib::NormalizedProgress* nProgress = 0);

	static bool DrawCellAsAPrimitive(	const CCCoreLib::DgmOctree::octreeCell& cell,
										void** additionalParameters,
										CCCoreLib::NormalizedProgress* nProgress = 0);

protected: //MEMBERS

	//! Associated cloud (as a ccGenericPointCloud)
	ccGenericPointCloud* m_theAssociatedCloudAsGPC;

	//! Displayed level
	int m_displayedLevel;

	//! Display mode
	DisplayMode m_displayMode;

	//! OpenGL display list
	GLuint m_glListID;
	//! Whether the display (list) should be refreshed or not
	bool m_glListIsDeprecated;

	//! For frustum intersection
	ccOctreeFrustumIntersector* m_frustumIntersector;

};

#endif //CC_OCTREE_HEADER
