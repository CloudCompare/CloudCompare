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
//$Rev:: 1959                                                              $
//$LastChangedDate:: 2011-12-04 22:21:14 +0100 (dim., 04 déc. 2011)       $
//**************************************************************************
//

#ifndef CC_OCTREE_HEADER
#define CC_OCTREE_HEADER

//CCLib
#include <DgmOctree.h>
#include <ReferenceCloud.h>

#include "ccHObject.h"

class ccGenericPointCloud;

//! Octree displaying methods
enum CC_OCTREE_DISPLAY_TYPE {WIRE					=	0,					/**< The octree is displayed as wired boxes (one box per cell) */
								MEAN_POINTS			=	1,					/**< The octree is displayed as points (one point per cell = the center of gravity of the points lying in it) */
								MEAN_CUBES			=	2					/**< The octree is displayed as plain 3D cubes (one cube per cell) */
};
const unsigned char OCTREE_DISPLAY_TYPE_NUMBERS										=		3;
const CC_OCTREE_DISPLAY_TYPE DEFAULT_OCTREE_DISPLAY_TYPE							=		WIRE;
const CC_OCTREE_DISPLAY_TYPE OCTREE_DISPLAY_TYPE_ENUMS[OCTREE_DISPLAY_TYPE_NUMBERS] =		{WIRE, MEAN_POINTS, MEAN_CUBES};
const char COCTREE_DISPLAY_TYPE_TITLES[OCTREE_DISPLAY_TYPE_NUMBERS][18]				=		{"Wire","Points","Plain cubes"};

//! Octree structure
/** Extends the DgmOctree class (Cf. CCLib).
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccOctree : public CCLib::DgmOctree, public ccHObject
#else
class ccOctree : public CCLib::DgmOctree, public ccHObject
#endif
{
public:

	//! Default constructor
	/** \param aCloud a point cloud
	**/
	ccOctree(ccGenericPointCloud* aCloud);

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
    virtual CC_CLASS_ENUM getClassID() const {return CC_POINT_OCTREE;};

	int getDisplayedLevel();
	void setDisplayedLevel(int level);

	CC_OCTREE_DISPLAY_TYPE getDisplayType();
	void setDisplayType(CC_OCTREE_DISPLAY_TYPE type);

	//inherited
	virtual void clear();

    //Inherited from ccHObject
    virtual ccBBox getMyOwnBB();
    virtual ccBBox getDisplayBB();

	/*** RENDERING METHODS ***/

	static void RenderOctreeAs(CC_OCTREE_DISPLAY_TYPE octreeDisplayType,
                                CCLib::DgmOctree* theOctree,
                                unsigned char level,
                                ccGenericPointCloud* theAssociatedCloud,
                                int &octreeGLListID,
                                bool updateOctreeGLDisplay=true);

	static void ComputeAverageColor(CCLib::ReferenceCloud* subset,
                                    ccGenericPointCloud* sourceList,
                                    colorType meanCol[]);

	static void ComputeAverageNorm(CCLib::ReferenceCloud* subset,
                                    ccGenericPointCloud* cloud,
                                    float norm[]);

	static void ComputeRobustAverageNorm(CCLib::ReferenceCloud* subset,
                                            ccGenericPointCloud* cloud,
                                            PointCoordinateType norm[]);

protected:

    //Inherited from ccHObject
    void drawMeOnly(CC_DRAW_CONTEXT& context);

    CC_OCTREE_DISPLAY_TYPE displayType;
    bool shouldBeRefreshed;
    int displayedLevel;
    int glID;

    ccGenericPointCloud* _associatedCloud;

	/*** RENDERING METHODS ***/

	static bool DrawCellAsABox(const CCLib::DgmOctree::octreeCell& cell,
                                void** additionalParameters);

	static bool DrawCellAsAPoint(const CCLib::DgmOctree::octreeCell& cell,
                                    void** additionalParameters);

	static bool DrawCellAsAPlainCube(const CCLib::DgmOctree::octreeCell& cell,
                                        void** additionalParameters);

};

#endif
