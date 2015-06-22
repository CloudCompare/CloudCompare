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

#ifndef CC_OCTREE_HEADER
#define CC_OCTREE_HEADER

//Local
#include "qCC_db.h"
#include "ccHObject.h"

//CCLib
#include <DgmOctree.h>
#include <ReferenceCloud.h>

//Qt
#include <QSpinBox>

//system
#include <vector>

class ccGenericPointCloud;
class ccOctreeFrustrumIntersector;
class ccCameraSensor;

//! Octree displaying methods
enum CC_OCTREE_DISPLAY_TYPE {	WIRE				=	0,					/**< The octree is displayed as wired boxes (one box per cell) */
								MEAN_POINTS			=	1,					/**< The octree is displayed as points (one point per cell = the center of gravity of the points lying in it) */
								MEAN_CUBES			=	2					/**< The octree is displayed as plain 3D cubes (one cube per cell) */
};
const unsigned char OCTREE_DISPLAY_TYPE_NUMBERS										=	3;
const CC_OCTREE_DISPLAY_TYPE DEFAULT_OCTREE_DISPLAY_TYPE							=	WIRE;
const CC_OCTREE_DISPLAY_TYPE OCTREE_DISPLAY_TYPE_ENUMS[OCTREE_DISPLAY_TYPE_NUMBERS] =	{WIRE, MEAN_POINTS, MEAN_CUBES};
const char COCTREE_DISPLAY_TYPE_TITLES[OCTREE_DISPLAY_TYPE_NUMBERS][18]				=	{"Wire","Points","Plain cubes"};

//! Octree level editor dialog
class QCC_DB_LIB_API ccOctreeSpinBox : public QSpinBox
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccOctreeSpinBox(QWidget* parent = 0);

	//! Sets associated cloud on which the octree will be computed
	/** Alternative to ccOctreeSpinBox::setOctree
	**/
	void setCloud(ccGenericPointCloud* cloud);

	//! Sets associated octree
	/** Alternative to ccOctreeSpinBox::setCloud
	**/
	void setOctree(CCLib::DgmOctree* octree);

protected slots:

	//! Called each time the spinbox value changes
	void onValueChange(int);

protected:

	//! Corresponding octree base size
	double m_octreeBoxWidth;

};

//! Octree structure
/** Extends the CCLib::DgmOctree class.
**/
class QCC_DB_LIB_API ccOctree : public CCLib::DgmOctree, public ccHObject
{
public: //GENERAL METHODS

	//! Default constructor
	/** \param aCloud a point cloud
	**/
	explicit ccOctree(ccGenericPointCloud* aCloud);

	//! Destructor
	virtual ~ccOctree();

	//! Multiplies the bounding-box of the octree
	/** If the cloud coordinates are simply multiplied by the same factor,
		there is no use to recompute the octree structure. It's sufficient
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

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::POINT_OCTREE; }

	int getDisplayedLevel() const { return m_displayedLevel; }
	void setDisplayedLevel(int level);

	CC_OCTREE_DISPLAY_TYPE getDisplayType() const { return m_displayType; }
	void setDisplayType(CC_OCTREE_DISPLAY_TYPE type);

	//inherited from DgmOctree
	virtual void clear();

	//Inherited from ccHObject
	virtual ccBBox getOwnBB(bool withGLFeatures = false);

public: //RENDERING METHODS

	static void RenderOctreeAs(	CC_OCTREE_DISPLAY_TYPE octreeDisplayType,
								ccOctree* theOctree,
								unsigned char level,
								ccGenericPointCloud* theAssociatedCloud,
								int &octreeGLListID,
								bool updateOctreeGLDisplay = true);

	static void ComputeAverageColor(CCLib::ReferenceCloud* subset,
									ccGenericPointCloud* sourceCloud,
									colorType meanCol[]);

	static CCVector3 ComputeAverageNorm(CCLib::ReferenceCloud* subset,
										ccGenericPointCloud* sourceCloud);

	//! Intersects octree with a camera sensor
	bool intersectWithFrustrum(	ccCameraSensor* sensor,
								std::vector<unsigned>& inCameraFrustrum);

protected:

	//Inherited from ccHObject
	void drawMeOnly(CC_DRAW_CONTEXT& context);

	/*** RENDERING METHODS ***/

	static bool DrawCellAsABox(	const CCLib::DgmOctree::octreeCell& cell,
								void** additionalParameters,
								CCLib::NormalizedProgress* nProgress = 0);

	static bool DrawCellAsAPoint(	const CCLib::DgmOctree::octreeCell& cell,
									void** additionalParameters,
									CCLib::NormalizedProgress* nProgress = 0);

	static bool DrawCellAsAPrimitive(	const CCLib::DgmOctree::octreeCell& cell,
										void** additionalParameters,
										CCLib::NormalizedProgress* nProgress = 0);

	ccGenericPointCloud* m_theAssociatedCloudAsGPC;
	CC_OCTREE_DISPLAY_TYPE m_displayType;
	int m_displayedLevel;
	int m_glListID;
	bool m_shouldBeRefreshed;

	ccOctreeFrustrumIntersector* m_frustrumIntersector;
};

#endif //CC_OCTREE_HEADER
