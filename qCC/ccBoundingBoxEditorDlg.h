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

#ifndef CC_BOUNDING_BOX_EDITOR_DLG_HEADER
#define CC_BOUNDING_BOX_EDITOR_DLG_HEADER

#include <ui_boundingBoxEditorDlg.h>

//qCC_db
#include <ccBBox.h>

//! Dialog to define the extents of a 3D box
class ccBoundingBoxEditorDlg : public QDialog, public Ui::BoundingBoxEditorDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccBoundingBoxEditorDlg(QWidget* parent = 0);

	//! Returns bounding box
	ccBBox getBox() const { return m_currentBBox; }

	//! Sets (minimal) base box
	/** \param box base box
		\param isMinimal set whether the user must define a bounding-box at least as large as this one
	**/
	void setBaseBBox(const ccBBox& box, bool isMinimal = true);

	//! Whether the warning about bounding box inclusion in the base one should be displayed or not
	/** True by default.
	**/
	void showInclusionWarning(bool state) { m_showInclusionWarning = state; }

	//! Forces the 'keep square' mode
	void forceKeepSquare(bool state);

	//! Returns whether 'keep square' mode is enabled or not
	bool keepSquare() const;

	//! Sets 2D mode ('dim' line will be hidden)
	void set2DMode(bool state, unsigned char dim);

public slots:

	//overloaded from QDialog
	virtual int	exec();

protected slots:

	void squareModeActivated(bool);
	void resetToDefault();
	void resetToLast();
	void cancel();
	void saveBoxAndAccept();

	void updateXWidth(double);
	void updateYWidth(double);
	void updateZWidth(double);

	//! Updates current box based on the dialog state
	void updateCurrentBBox(double dummy = 0.0);
	//! Reflects changes on bbox
	void reflectChanges(int dummy = 0);

protected:

	//! Checks if currentBox includes baseBox
	void checkBaseInclusion();

	//! Base box (invalid if none)
	ccBBox m_baseBBox;

	//! Whether base box is minimal or not
	bool m_baseBoxIsMinimal;

	//! Current box
	ccBBox m_currentBBox;

	//! Box state at dialog start
	ccBBox m_initBBox;

	//! Whether to show 'inclusion' warning or not
	bool m_showInclusionWarning;

};

#endif //CC_BOUNDING_BOX_EDITOR_DLG_HEADER
