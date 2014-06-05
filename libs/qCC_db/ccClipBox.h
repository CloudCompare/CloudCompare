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

#ifndef CC_CLIP_BOX_HEADER
#define CC_CLIP_BOX_HEADER

//Local
#include "qCC_db.h"
#include "ccBBox.h"
#include "ccHObject.h"
#include "ccInteractor.h"
#include "ccGLMatrix.h"

//Qt
#include <QObject>

//! Clipping box
class QCC_DB_LIB_API ccClipBox : public QObject, public ccHObject, public ccInteractor
{
	Q_OBJECT

public:

	//! Default constructor
	ccClipBox(ccHObject* associatedEntity = 0, QString name = QString("Clipping box"));

	//! Destructor
	virtual ~ccClipBox();

	//! Sets associated entity
	/** Warning: resets the current clipping box
	**/
	void setAssociatedEntity(ccHObject* associatedEntity);

	//inherited from ccHObject
	virtual ccBBox getMyOwnBB();
	virtual ccBBox getDisplayBB();

	//inherited from ccInteractor
	virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight);
	virtual bool move3D(const CCVector3d& u);

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
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::CLIPPING_BOX; }
	//virtual bool isSerializable() const { return false; }

	//! Returns current box
	const ccBBox& getBox() const { return m_box; }

	//! Sets current box
	void setBox(const ccBBox& box);

	//! Shifts current box
	void shift(const CCVector3& v);

	//! Updates associated entity 'visibility'
	/** \param shrink Whether box is shrinking (faster) or not
	**/
	void update(bool shrink = false);

	//! Resets box
	void reset();

	//! Associated entity
	ccHObject* getAssociatedEntity() const { return m_associatedEntity; }

signals:

	//! Signal sent each time the box is modified
	void boxModified(const ccBBox* box);

protected:

	//inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! Computes arrows display scale
	PointCoordinateType computeArrowsScale() const;

	//! Associated entity
	ccHObject* m_associatedEntity;

	//! Clipping box
	ccBBox m_box;

	//! Active component
	Components m_activeComponent;

	//! Last "orientation" vector (corresponding to last clicked point)
	CCVector3d m_lastOrientation;

	//! View matrix
	ccGLMatrixd m_viewMatrix;
};

#endif //CC_CLIP_BOX_HEADER
