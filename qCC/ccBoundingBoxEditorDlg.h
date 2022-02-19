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
	/** \param showBoxAxes			Whether to display the box axes or not
		\param showRasterGridImage	Whether to display the raster grid image or not
		\param parent				Parent widget
	**/
	explicit ccBoundingBoxEditorDlg(bool showBoxAxes, bool showRasterGridImage, QWidget* parent = nullptr);

	//! Returns bounding box
	inline const ccBBox& getBox() const { return m_currentBBox; }

	//! Overrides the current bounding box
	/** \param box bounding box
	**/
	void setBox(const ccBBox& box);

	//! Sets the (minimal) base box
	/** \param box base box
		\param isMinimal set whether the user must define a bounding-box at least as large as this one
	**/
	void setBaseBBox(const ccBBox& box, bool isMinimal = true);

	//! Sets the box axes
	void setBoxAxes(const CCVector3& X, const CCVector3& Y, const CCVector3& Z);

	//! Returns the box axes
	void getBoxAxes(CCVector3d& X, CCVector3d& Y, CCVector3d& Z);

	//! Whether the warning about bounding box inclusion in the base one should be displayed or not
	/** True by default.
	**/
	void showInclusionWarning(bool state) { m_showInclusionWarning = state; }

	//! Forces the 'keep square' mode
	void forceKeepSquare(bool state);

	//! Returns whether 'keep square' mode is enabled or not
	bool keepSquare() const;

	//! Sets 2D mode (the line 'dim' will be hidden)
	void set2DMode(bool state, unsigned char dim);

public:

	//overloaded from QDialog
	virtual int	exec();

protected:

	void squareModeActivated(bool);
	void resetToDefault();
	void resetToLast();
	void cancel();
	void saveBoxAndAccept();
	int computeBestDialogHeight(bool showBoxAxes, bool showRasterGridImage) const;

	void updateXWidth(double);
	void updateYWidth(double);
	void updateZWidth(double);

	//! Updates current box based on the dialog state
	void updateCurrentBBox(double dummy = 0.0);
	//! Reflects changes on bbox
	void reflectChanges(int dummy = 0);
	
	//! Slot called anytime a component of the box axes is modified
	void onAxisValueChanged(double);

	void fromClipboardClicked();
	void toClipboardClicked();

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
