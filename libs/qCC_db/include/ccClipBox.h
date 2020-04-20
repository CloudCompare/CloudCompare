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

#ifndef CC_CLIP_BOX_HEADER
#define CC_CLIP_BOX_HEADER

//Local
#include "ccBBox.h"
#include "ccGenericPointCloud.h"
#include "ccHObject.h"
#include "ccInteractor.h"

//Qt
#include <QObject>

//! Clipping box
class QCC_DB_LIB_API ccClipBox : public QObject, public ccHObject, public ccInteractor
{
	Q_OBJECT

public:

	//! Default constructor
	/** \parma name entity name (optional)
		\param uniqueID unique ID (handle with care)
	**/
	ccClipBox(QString name = QString("Clipping box"), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Destructor
	~ccClipBox() override;

	//! Adds an associated entity
	/** Warning: resets the current clipping box
	**/
	bool addAssociatedEntity(ccHObject* associatedEntity);

	//! Releases all associated entities
	/** Warning: resets the current clipping box
	**/
	void releaseAssociatedEntities();

	//inherited from ccHObject
	ccBBox getOwnBB(bool withGLFeatures = false) override;

	//inherited from ccInteractor
	bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;
	bool move3D(const CCVector3d& u) override;

	//! Sets last clicked point (on screen)
	void setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd& viewMatrix);

	//! Components
	enum Components {	NONE			= 0,
						X_MINUS_ARROW	= 1,
						X_PLUS_ARROW	= 2,
						Y_MINUS_ARROW	= 3,
						Y_PLUS_ARROW	= 4,
						Z_MINUS_ARROW	= 5,
						Z_PLUS_ARROW	= 6,
						CROSS			= 7,
						SPHERE			= 8,
						X_MINUS_TORUS	= 9,
						Y_MINUS_TORUS	= 10,
						Z_MINUS_TORUS	= 11,
						X_PLUS_TORUS	= 12,
						Y_PLUS_TORUS	= 13,
						Z_PLUS_TORUS	= 14,
	};

	//! Sets currently active component
	/** \param id component ID (see Components)
	**/
	void setActiveComponent(int id);

	//inherited from ccHObject
	inline CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CLIPPING_BOX; }

	//! Returns the box extents
	inline const ccBBox& getBox() const { return m_box; }

	//! Whether to show the box or not
	inline void showBox(bool state) { m_showBox = state; }

	//! Sets the box extents
	void setBox(const ccBBox& box);

	//! Shifts the current box
	void shift(const CCVector3& v);

	//! Flags the points of a given cloud depending on whether they are inside or outside of this clipping box
	/** \param cloud point cloud
		\param visTable visibility flags
		\param shrink Whether the box is shrinking (faster) or not
	**/
	void flagPointsInside(	ccGenericPointCloud* cloud,
							ccGenericPointCloud::VisibilityTableType* visTable,
							bool shrink = false) const;

	//! Resets box
	void reset();

	//! Manually sets the box parameters
	void set(const ccBBox& extents, const ccGLMatrix& transformation);

	//! Returns the box parameters
	void get(ccBBox& extents, ccGLMatrix& transformation);

	//! Associated entity container
	inline const ccHObject& getContainer() const { return m_entityContainer; }

signals:

	//! Signal sent each time the box is modified
	void boxModified(const ccBBox* box);

protected: //methods

	//! Updates the associated entity clipping planes
	void update();

	//inherited from ccHObject
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Computes arrows display scale
	PointCoordinateType computeArrowsScale() const;

protected: //members
	
	//! Associated entities container
	ccHObject m_entityContainer;

	//! Clipping box
	ccBBox m_box;

	//! Show box
	bool m_showBox;

	//! Active component
	Components m_activeComponent;

	//! Last "orientation" vector (corresponding to last clicked point)
	CCVector3d m_lastOrientation;

	//! View matrix
	ccGLMatrixd m_viewMatrix;
};

#endif //CC_CLIP_BOX_HEADER
