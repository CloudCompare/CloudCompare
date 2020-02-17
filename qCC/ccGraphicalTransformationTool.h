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

#ifndef CC_GRAPHICAL_TRANSFORMATION_TOOL_HEADER
#define CC_GRAPHICAL_TRANSFORMATION_TOOL_HEADER

//Local
#include <ccOverlayDialog.h>

#include <ui_graphicalTransformationDlg.h>

//qCC_db
#include <ccHObject.h>

class ccGLWindow;

//! Dialog + mechanism for graphical transformation of entities
/** Mouse driven rotation and translation of selected entities at screen.
**/
class ccGraphicalTransformationTool : public ccOverlayDialog, public Ui::GraphicalTransformationDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccGraphicalTransformationTool(QWidget* parent);
	//! Default destructor
	virtual ~ccGraphicalTransformationTool();

	//inherited from ccOverlayDialog
	virtual bool linkWith(ccGLWindow* win) override;
	virtual bool start() override;
	virtual void stop(bool state) override;

	//! unselect all advanced mode references
	void clearAdvModeEntities();

	//! Adds an entity to the 'selected' entities set
	/** Only the 'selected' entities are moved.
		\return success, if the entity is eligible for graphical transformation
	**/
	bool addEntity(ccHObject* anObject);

	//! Returns the number of valid entities (see addEntity)
	unsigned getNumberOfValidEntities() const;

	//! Returns the 'to be transformed' entities set (see addEntity)
	const ccHObject& getValidEntities() const { return m_toTransform; }

	//! Sets the rotation center
	void setRotationCenter(CCVector3d& center);

	//! Returns the transform for translating along an arbitrary vector
	ccGLMatrixd arbitraryVectorTranslation(const CCVector3& vec);

	//! Returns the transform for rotation around an arbitrary vector
	ccGLMatrixd arbitraryVectorRotation(double angle, const CCVector3d&);


protected slots:

	//! Applies transformation to selected entities
	void apply();

	//! Resets transformation
	void reset();

	//! Cancels (no transformation is applied)
	void cancel();

	//! Pauses the transformation mode
	void pause(bool);

	//! Togggles the visibility of the advanced mode ui
	void advModeToggle(bool state);

	//! Updates the transform for advanced mode rotation when translate ref changed
	void advTranslateRefUpdate(int index);

	//! Updates the transform for advanced mode rotation when rotate ref changed
	void advRotateRefUpdate(int index);

	//! Updates the axis center of rotation to the ref object in adv rotate/translate mode
	void advRefAxisRadioToggled(bool state);

	//! Updates the axis center of rotation to the object center in adv rotate/translate mode
	void advObjectAxisRadioToggled(bool state);

	//! Updates the top center display message according to the mode
	void updateDisplayMessage();

	//! Applies translation (graphically) to selected entities
	void glTranslate(const CCVector3d&);

	//! Applies rotation (graphically) to selected entities
	void glRotate(const ccGLMatrixd&);

	//! To capture overridden shortcuts (pause button, etc.)
	void onShortcutTriggered(int);

protected:

	//! Clear all variables and 'unlink' dialog
	void clear();

	//! Updates all selected entities GL transformation matrices
	void updateAllGLTransformations();

	//! Sets Advanced translate/rotation mode reference items
	void populateAdvModeItems();

	//! Sets the translation transform used in advanced translate/rotate mode
	bool setAdvTranslationTransform(ccHObject* translateRef);

	//! Sets the rotation transform used in advaced translate/rotate mode
	bool setAdvRotationAxis(ccHObject* rotateRef);

	//! Check if the entitry is in m_toTransform
	bool entityInTransformList(ccHObject* entity);

	//! Flag for advanced mode
	bool m_advMode;

	//! Flag if the rotation reference object is in m_toTransform
	bool m_advRotateRefIsChild;

	//! Flag if the translate reference object is in m_toTransform
	bool m_advTranslateRefIsChild;

	//! List of entities to be transformed
	ccHObject m_toTransform;

	//! Current advanced translate mode ref object
	ccHObject* m_advTranslateRef = nullptr;

	//! Current advanced rotate mode ref object
	ccHObject* m_advRotateRef = nullptr;

	//! Current rotation
	ccGLMatrixd m_rotation;

	//! Current translation
	CCVector3d m_translation;

	//! Current position
	ccGLMatrixd m_position;

	//! Transform used in advanced translate/rotate mode
	ccGLMatrixd m_advTranslationTransform;

	//! Current rotation axis vector for adv translate/rotate mode (not neccesarily rotation center)
	CCVector3d m_advRotationAxis;

	//! Current reference object for rotation center point
	CCVector3d m_advRotationRefObjCenter;

	//! Rotation center
	/** The rotation center is actually the center of gravity of the selected 'entities'
	**/
	CCVector3d m_rotationCenter;

	//! Planes and line segments found in the dbtree for adv transate/rotate
	ccHObject::Container m_planesAndLineSegments;

	//! rotComboBox enum
	enum rotComboBoxItems {XYZ, X, Y, Z, NONE};
};

#endif //CC_GRAPHICAL_TRANSFORMATION_TOOL_HEADER
