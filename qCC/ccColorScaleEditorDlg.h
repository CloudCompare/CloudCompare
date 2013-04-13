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

#ifndef CC_COLOR_SCALE_EDITOR_DLG_HEADER
#define CC_COLOR_SCALE_EDITOR_DLG_HEADER

#include <ui_colorScaleEditorDlg.h>

//local
#include "ccColorScaleEditorWidget.h"

//qCC_db
#include <ccColorScale.h>

//Qt
#include <QDialog>

class ccScalarField;

//! Dialog to edit/create color scales
class ccColorScaleEditorDialog : public QDialog, public Ui::ColorScaleEditorDlg
{
    Q_OBJECT

public:

    //! Default constructor
	ccColorScaleEditorDialog(ccColorScale::Shared currentScale = ccColorScale::Shared(0), QWidget* parent = 0);

    //! Destructor
	virtual ~ccColorScaleEditorDialog();

	//! Sets associated scalar field (optional)
	void setAssociatedScalarField(ccScalarField* sf) { m_associatedSF = sf; }

	//! Sets active scale
	void setActiveScale(ccColorScale::Shared currentScale);

	//! Returns active scale
	ccColorScale::Shared getActiveScale() { return m_colorScale; }

protected slots:

    void colorScaleChanged(int);

	void onStepSelected(int);

	void onStepModified(int);

	void deletecSelectedStep();

	void changeSelectedStepColor();

	void changeSelectedStepValue(double);

	void copyCurrentScale();
	void saveCurrentScale();
	void deleteCurrentScale();
	void renameCurrentScale();

	void createNewScale();

	void onClose();

protected:

	//! Updates main combox box with color scales manager
	void updateMainComboBox();

	//! Sets modification flag state
	void setModified(bool state);

	// If the current scale has been modified, ask the user what to do
	/** \param return whether user allows change
	**/
	bool canChangeCurrentScale();

	//! Current active color scale
	ccColorScale::Shared m_colorScale;

	//! Color scale editor widget
	ccColorScaleEditorWidget* m_scaleWidget;

	//! Associated scalar field
	ccScalarField* m_associatedSF;

	//! Modification flag
	bool m_modified;
};

#endif //CC_COLOR_SCALE_EDITOR_DLG_HEADER
